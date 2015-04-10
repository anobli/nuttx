/*
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author: Perry Hung
 * @author: Jean Pihet
 */

#define DBG_COMP    DBG_SWITCH
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <errno.h>
#include <string.h>

#include <nuttx/util.h>
#include <unipro/connection.h>
#include "stm32.h"
#include "up_debug.h"
#include "tsb_switch.h"

#define DEFAULT_STACK_SIZE          2048

/* Switch power supply times (1V1, 1V8), in us */
#define POWER_SWITCH_ON_STABILISATION_TIME_US  (50000)
#define POWER_SWITCH_OFF_STABILISATION_TIME_US (10000)
#define SWITCH_SETTLE_TIME_S                   (1)

/*
 * CportID and peerCPortID for L4 access by the SVC
 *
 * L4 CPortID: 0 for SVC -> CC, 2 for CC -> SVC
 * Peer CPortID: 3 for SVC -> CC, 2 for CC -> SVC
 */
#define L4_CPORT_SVC_TO_CC          0
#define L4_CPORT_CC_TO_SVC          2
#define L4_PEERCPORT_SVC_TO_CC      3
#define L4_PEERCPORT_CC_TO_SVC      2

/* NCP_SwitchIDSetReq: DIS and IRT fields values */
#define CPORT_ENABLE                0
#define CPORT_DISABLE               1
#define IRT_DISABLE                 0
#define IRT_ENABLE                  1

/* Default link configuration values.
 *
 * Bring everything up in non-auto HS-G1. The links which carry
 * display traffic need this, so apply it everywhere.
 *
 * TODO: these should be replaced with auto PWM-G1 (i.e. USE_HS=0,
 *       FLAGS=UNIPRO_LINK_CFGF_AUTO) when the AP and SVC know enough
 *       control protocol to do the right thing on all links.
 */
#define LINK_DEFAULT_USE_HS_GEAR    1
#define LINK_DEFAULT_HS_GEAR        1
#define LINK_DEFAULT_PWM_GEAR       1
#define LINK_DEFAULT_HS_NLANES      2
#define LINK_DEFAULT_PWM_NLANES     LINK_DEFAULT_HS_NLANES
#define LINK_DEFAULT_FLAGS          0

/* MaskId entry update */
#define SET_VALID_ENTRY(entry) \
    id_mask[15 - ((entry) / 8)] |= (1 << ((entry)) % 8)

static inline void dev_ids_update(struct tsb_switch *sw,
                                  uint8_t port_id,
                                  uint8_t dev_id) {
    sw->dev_ids[port_id] = dev_id;
}

static inline uint8_t dev_ids_port_to_dev(struct tsb_switch *sw,
                                          uint8_t port_id) {
    if (port_id >= SWITCH_PORT_MAX)
         return INVALID_PORT;

    return sw->dev_ids[port_id];
}

static inline uint8_t dev_ids_dev_to_port(struct tsb_switch *sw,
                                          uint8_t dev_id) {
    int i;

    for (i = 0; i < SWITCH_PORT_MAX; i++) {
        if (sw->dev_ids[i] == dev_id) {
             return i;
        }
    }

    return INVALID_PORT;
}

static void dev_ids_destroy(struct tsb_switch *sw) {
    memset(sw->dev_ids, INVALID_PORT, sizeof(sw->dev_ids));
}

static void switch_power_on_reset(struct tsb_switch *sw) {
    /*
     * Hold all the lines low while we turn on the power rails.
     */
    stm32_configgpio(sw->vreg_1p1);
    stm32_configgpio(sw->vreg_1p8);
    stm32_configgpio(sw->reset);
    up_udelay(POWER_SWITCH_OFF_STABILISATION_TIME_US);

    /* First 1V1, wait for stabilisation */
    stm32_gpiowrite(sw->vreg_1p1, true);
    up_udelay(POWER_SWITCH_ON_STABILISATION_TIME_US);
    /* Then 1V8, wait for stabilisation */
    stm32_gpiowrite(sw->vreg_1p8, true);
    up_udelay(POWER_SWITCH_OFF_STABILISATION_TIME_US);

    /* release reset */
    stm32_gpiowrite(sw->reset, true);
    sleep(SWITCH_SETTLE_TIME_S);
}

static void switch_power_off(struct tsb_switch *sw) {
    stm32_gpiowrite(sw->vreg_1p1, false);
    stm32_gpiowrite(sw->vreg_1p8, false);
    stm32_gpiowrite(sw->reset, false);
}


/* Switch communication link init and de-init */
static int switch_init_comm(struct tsb_switch *sw)
{
    if (!sw->ops->init_comm) {
        return -EOPNOTSUPP;
    }
    return sw->ops->init_comm(sw);
}

/*
 * Unipro NCP commands
 */
int switch_dme_set(struct tsb_switch *sw,
                          uint8_t portid,
                          uint16_t attrid,
                          uint16_t select_index,
                          uint32_t attr_value) {
    if (!sw->ops->set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->set(sw, portid, attrid, select_index, attr_value);
}

int switch_dme_get(struct tsb_switch *sw,
                          uint8_t portid,
                          uint16_t attrid,
                          uint16_t select_index,
                          uint32_t *attr_value) {
    if (!sw->ops->get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->get(sw, portid, attrid, select_index, attr_value);
}

int switch_dme_peer_set(struct tsb_switch *sw,
                               uint8_t portid,
                               uint16_t attrid,
                               uint16_t select_index,
                               uint32_t attr_value) {
    if (!sw->ops->peer_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->peer_set(sw, portid, attrid, select_index, attr_value);
}

int switch_dme_peer_get(struct tsb_switch *sw,
                               uint8_t portid,
                               uint16_t attrid,
                               uint16_t select_index,
                               uint32_t *attr_value) {
    if (!sw->ops->peer_get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->peer_get(sw, portid, attrid, select_index, attr_value);
}

int switch_port_irq_enable(struct tsb_switch *sw,
                           uint8_t portid,
                           bool enable) {
    if (!sw->ops->port_irq_enable) {
        return -EOPNOTSUPP;
    }
    return sw->ops->port_irq_enable(sw, portid, enable);
}

/*
 * Routing table configuration commands
 */
int switch_lut_set(struct tsb_switch *sw,
                   uint8_t unipro_portid,
                   uint8_t addr,
                   uint8_t dst_portid) {
    if (!sw->ops->lut_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->lut_set(sw, unipro_portid, addr, dst_portid);
}

int switch_lut_get(struct tsb_switch *sw,
                   uint8_t unipro_portid,
                   uint8_t addr,
                   uint8_t *dst_portid) {
    if (!sw->ops->lut_get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->lut_get(sw, unipro_portid, addr, dst_portid);
}

int switch_dump_routing_table(struct tsb_switch *sw) {
    if (!sw->ops->dump_routing_table) {
        return -EOPNOTSUPP;
    }
    return sw->ops->dump_routing_table(sw);
}

int switch_sys_ctrl_set(struct tsb_switch *sw,
                        uint16_t sc_addr,
                        uint32_t val) {
    if (!sw->ops->sys_ctrl_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->sys_ctrl_set(sw, sc_addr, val);
}

int switch_sys_ctrl_get(struct tsb_switch *sw,
                        uint16_t sc_addr,
                        uint32_t *val) {
    if (!sw->ops->sys_ctrl_get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->sys_ctrl_get(sw, sc_addr, val);
}

int switch_dev_id_mask_get(struct tsb_switch *sw,
                           uint8_t unipro_portid,
                           uint8_t *dst) {
    if (!sw->ops->dev_id_mask_get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->dev_id_mask_get(sw, unipro_portid, dst);
}

int switch_dev_id_mask_set(struct tsb_switch *sw,
                           uint8_t unipro_portid,
                           uint8_t *mask) {
    if (!sw->ops->dev_id_mask_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->dev_id_mask_set(sw, unipro_portid, mask);
}

/*
 * Switch internal configuration commands
 */
int switch_internal_getattr(struct tsb_switch *sw,
                            uint16_t attrid,
                            uint32_t *val) {
    if (!sw->ops->switch_attr_get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_attr_get(sw, attrid, val);
}

int switch_internal_setattr(struct tsb_switch *sw,
                            uint16_t attrid,
                            uint32_t val) {
    if (!sw->ops->switch_attr_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_attr_set(sw, attrid, val);
}

static int switch_internal_set_id(struct tsb_switch *sw,
                                  uint8_t cportid,
                                  uint8_t peercportid,
                                  uint8_t dis,
                                  uint8_t irt) {
    if (!sw->ops->switch_id_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_id_set(sw, cportid, peercportid, dis, irt);
}

int switch_irq_enable(struct tsb_switch *sw,
                      bool enable) {
    if (!sw->ops->switch_irq_enable) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_irq_enable(sw, enable);
}

int switch_irq_handler(struct tsb_switch *sw) {
    if (!sw->ops->switch_irq_handler) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_irq_handler(sw);
}

static int switch_cport_connect(struct tsb_switch *sw,
                                struct unipro_connection *c) {


    int rc = 0;

    /*
     * Follow MIPI Specification for UniPro Version 1.6: Section 8.11.1 for setting
     * up a connection, plus guidelines from Toshiba bridge manual.
     */

    /* Disable connection */
    rc = switch_dme_peer_set(sw, c->port_id0, T_CONNECTIONSTATE, c->cport_id0, 0x0);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, T_CONNECTIONSTATE, c->cport_id1, 0x0);
    if (rc) {
        return rc;
    }

    /*
     * The T_PeerDeviceID Attribute of each CPort shall be set to the
     * N_DeviceID of the peer CPort.
     */
    rc = switch_dme_peer_set(sw, c->port_id0, T_PEERDEVICEID, c->cport_id0, c->device_id1);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, T_PEERDEVICEID, c->cport_id1, c->device_id0);
    if (rc) {
        return rc;
    }

    /*
     * The T_PeerCPortID Attribute of each CPort shall be set to the CPortId of
     * the peer CPort.
     */
    rc = switch_dme_peer_set(sw, c->port_id0, T_PEERCPORTID, c->cport_id0, c->cport_id1);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, T_PEERCPORTID, c->cport_id1, c->cport_id0);
    if (rc) {
        return rc;
    }

    /*
     * The T_TrafficClass Attribute shall be set to the same value for both
     * CPorts.
     */
    rc = switch_dme_peer_set(sw, c->port_id0, T_TRAFFICCLASS, c->cport_id0, c->tc);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, T_TRAFFICCLASS, c->cport_id1, c->tc);
    if (rc) {
        return rc;
    }

    /*
     * The T_ProtocolID Attribute shall be set to the same value for both
     * CPorts.
     *
     * This is unused for us.
     */
    rc = switch_dme_peer_set(sw, c->port_id0, T_PROTOCOLID, c->cport_id0, 0);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, T_PROTOCOLID, c->cport_id1, 0);
    if (rc) {
        return rc;
    }

    /*
     * TxTokenValue and RxTokenValue
     */
    rc = switch_dme_peer_set(sw, c->port_id0, T_TXTOKENVALUE, c->cport_id0, 32);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, T_TXTOKENVALUE, c->cport_id1, 32);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id0, T_RXTOKENVALUE, c->cport_id0, 32);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, T_RXTOKENVALUE, c->cport_id1, 32);
    if (rc) {
        return rc;
    }


    /*
     * -The T_CPortFlags.E2EFC shall be set to the same value for both CPorts.
     * -If E2E FC is enabled (T_CPortFlags.E2EFC = ‘1’), the T_TxTokenValue
     *  Attribute of each CPortand the T_RxTokenValue Attribute of the peer
     *  CPort shall be set to the same value.
     * -The T_CPortMode Attribute for each CPort shall be set to
     *  CPORT_APPLICATION.
     * - If E2E FC is enabled (T_CPortFlags.E2EFC = ‘1’) or the E2E FC is
     *   disabled and CSD is enabled(T_CPortFlags.E2EFC = ‘0’and
     *   T_CPortFlags.CSD_n = '0'), each of the two CPorts shall set the
     *   T_PeerBufferSpace Attribute of the local CPort to the same value as
     *   T_LocalBufferSpace Attribute of the peer CPort. These values represent
     *   the amount of data that the CPort where T_PeerBufferSpace is set can
     *   initially transmit, and, hence, its peer CPort can receive.
     */
    rc = switch_dme_peer_set(sw, c->port_id0, T_CPORTFLAGS, c->cport_id0, c->flags);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, T_CPORTFLAGS, c->cport_id1, c->flags);
    if (rc) {
        return rc;
    }

    /*
     * The T_CreditsToSend Attribute for each CPort shall be set to 0.
     */
    rc = switch_dme_peer_set(sw, c->port_id0, T_CREDITSTOSEND, c->cport_id0, 0);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, T_CREDITSTOSEND, c->cport_id1, 0);
    if (rc) {
        return rc;
    }

    /*
     * TSB_MaxSegmentConfig
     */
    rc = switch_dme_peer_set(sw, c->port_id0, TSB_MAXSEGMENTCONFIG, c->cport_id0, 0x118);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, TSB_MAXSEGMENTCONFIG, c->cport_id1, 0x118);
    if (rc) {
        return rc;
    }


    /*
     * HACK: Wait for bridges to boot and write CONNECTION_READY to SVC
     * mailbox. We have to wait for this so that we know both bridges have
     * disabled E2EFC.
     */
    rc = switch_dme_peer_set(sw, c->port_id0, T_CONNECTIONSTATE, c->cport_id0, 1);
    if (rc) {
        return rc;
    }
    rc = switch_dme_peer_set(sw, c->port_id1, T_CONNECTIONSTATE, c->cport_id1, 1);
    if (rc) {
        return rc;
    }

    return 0;
}

/*
 * TODO: fix this
 */
static int switch_cport_disconnect(struct tsb_switch *sw,
                                   uint8_t port_id1,
                                   uint8_t cport_id1,
                                   uint8_t port_id2,
                                   uint8_t cport_id2) {
    switch_dme_peer_set(sw, port_id1, T_CONNECTIONSTATE, cport_id1, 0x0);
    switch_dme_peer_set(sw, port_id1, T_CONNECTIONSTATE, cport_id1, 0x0);
    return 0;
}

/**
 * @brief Assign a device id to a given port id
 */
int switch_if_dev_id_set(struct tsb_switch *sw,
                         uint8_t port_id,
                         uint8_t dev_id) {
    int rc;

    if (port_id >= SWITCH_UNIPORT_MAX) {
        return -EINVAL;
    }

    rc = switch_dme_peer_set(sw, port_id, N_DEVICEID, NCP_SELINDEX_NULL, dev_id);
    if (rc) {
        return rc;
    }

    rc = switch_dme_peer_set(sw, port_id, N_DEVICEID_VALID, NCP_SELINDEX_NULL, 1);
    if (rc) {
        /* do what on failure? */
        return rc;
    }

    /* update the table */
    dev_ids_update(sw, port_id, dev_id);

    return 0;
}

/**
 * @brief Setup network routing table
 *
 * Setup of the deviceID Mask tables (if supported) and the Switch LUTs,
 * bidirectionally between the source and destination Switch ports.
 */
int switch_setup_routing_table(struct tsb_switch *sw,
                               uint8_t device_id_0,
                               uint8_t port_id_0,
                               uint8_t device_id_1,
                               uint8_t port_id_1) {

    int rc;
    uint8_t id_mask[16];

    dbg_verbose("Setup routing table [%u:%u]<->[%u:%u]\n",
                device_id_0, port_id_0, device_id_1, port_id_1);

    // Set MaskId for devices 0->1
    rc = switch_dev_id_mask_get(sw,
                                port_id_0,
                                id_mask);
    if (rc && (rc != -EOPNOTSUPP)) {
        dbg_error("Failed to get MaskId for port %d\n", port_id_0);
        return rc;
    }

    SET_VALID_ENTRY(device_id_1);

    rc = switch_dev_id_mask_set(sw,
                                port_id_0,
                                id_mask);
    if (rc && (rc != -EOPNOTSUPP)) {
        dbg_error("Failed to set MaskId for port %d\n", port_id_0);
        return rc;
    }

    // Set MaskId for devices 1->0
    rc = switch_dev_id_mask_get(sw,
                                port_id_1,
                                id_mask);
    if (rc && (rc != -EOPNOTSUPP)) {
        dbg_error("Failed to get MaskId for port %d\n", port_id_1);
        return rc;
    }

    SET_VALID_ENTRY(device_id_0);

    rc = switch_dev_id_mask_set(sw,
                                port_id_1,
                                id_mask);
    if (rc && (rc != -EOPNOTSUPP)) {
        dbg_error("Failed to set MaskId for port %d\n", port_id_1);
        return rc;
    }

    // Setup routing table for devices 0->1
    rc = switch_lut_set(sw, port_id_0, device_id_1, port_id_1);
    if (rc) {
        dbg_error("Failed to set Lut for source port %d, disabling\n",
                  port_id_0);
        /* Undo deviceid_valid on failure */
        switch_dme_peer_set(sw, port_id_0, N_DEVICEID_VALID,
                            NCP_SELINDEX_NULL, 0);
        return rc;
    }

    // Setup routing table for devices 1->0
    rc = switch_lut_set(sw, port_id_1, device_id_0, port_id_0);
    if (rc) {
        dbg_error("Failed to set Lut for source port %d, disabling\n",
                  port_id_1);
        /* Undo deviceid_valid on failure */
        switch_dme_peer_set(sw, port_id_1, N_DEVICEID_VALID,
                            NCP_SELINDEX_NULL, 0);
        return rc;
    }

    return 0;
}

/**
 * @brief Create a connection between two cports
 */
int switch_connection_create(struct tsb_switch *sw,
                             struct unipro_connection *c) {
    int rc;

    if (!c) {
        rc = -EINVAL;
        goto err0;
    }

    dbg_info("Creating connection: [%u:%u:%u]<->[%u:%u:%u] TC: %u Flags: %x\n",
             c->port_id0,
             c->device_id0,
             c->cport_id0,
             c->port_id1,
             c->device_id1,
             c->cport_id1,
             c->tc,
             c->flags);

    rc = switch_cport_connect(sw, c);
    if (rc) {
        /*
         * TODO: fix disconnect
         */
        switch_cport_disconnect(sw,
                                c->port_id0,
                                c->cport_id0,
                                c->port_id1,
                                c->cport_id1);
    }

    return 0;

err0:
    dbg_error("%s: Connection setup failed. [%u:%u:%u]<->[%u:%u:%u] TC: %u Flags: %x rc: %u\n",
               __func__,
               c->port_id0,
               c->device_id0,
               c->cport_id0,
               c->port_id1,
               c->device_id1,
               c->cport_id1,
               c->tc,
               c->flags,
               rc);
    return rc;
}

static int switch_detect_devices(struct tsb_switch *sw,
                                 uint32_t *link_status)
{
    uint32_t attr_value = 0;
    int i, j;
    uint32_t attr_to_read[] = {
        /* DME_DDBL1 */
        /*  Revision,       expected 0x0010 */
        DME_DDBL1_REVISION,
        /*  Level,          expected 0x0003 */
        DME_DDBL1_LEVEL,
        /*  deviceClass,    expected 0x0000 */
        DME_DDBL1_DEVICECLASS,
        /*  ManufactureID,  expected 0x0126 */
        DME_DDBL1_MANUFACTURERID,
        /*  productID,      expected 0x1000 */
        DME_DDBL1_PRODUCTID,
        /*  length,         expected 0x0008 */
        DME_DDBL1_LENGTH,
        /* DME_DDBL2 VID and PID */
        TSB_DME_DDBL2_A,
        TSB_DME_DDBL2_B,
        /* Data lines */
        PA_CONNECTEDTXDATALANES,
        PA_CONNECTEDRXDATALANES
    };

    /* Read switch link status */
    if (switch_internal_getattr(sw, SWSTA, link_status)) {
        dbg_error("Switch read status failed\n");
        return -1;
    }

    dbg_info("%s: Link status: 0x%x\n", __func__, *link_status);

    /* Get attributes from connected devices */
    for (i = 0; i < SWITCH_UNIPORT_MAX; i++) {
        if (*link_status & (1 << i)) {
            for (j = 0; j < ARRAY_SIZE(attr_to_read); j++) {
                if (switch_dme_peer_get(sw, i, attr_to_read[j],
                                        NCP_SELINDEX_NULL, &attr_value)) {
                    dbg_error("%s: Failed to read attr(0x%x) from portID %d\n",
                              __func__, attr_to_read[j], i);
                } else {
                    dbg_verbose("%s: portID %d: attr(0x%x)=0x%x\n",
                                __func__, i, attr_to_read[j], attr_value);
                }
            }
        }
   }

   return 0;
}

/*
 * Determine if a link power mode ought to be taken to slow or slow
 * auto mode before reconfiguration in an HS gear, series B.
 */
static bool switch_ok_for_series_change(enum unipro_pwr_mode mode) {
    switch (mode) {
    case UNIPRO_FAST_MODE:  /* fall through */
    case UNIPRO_FASTAUTO_MODE:
        return false;
    case UNIPRO_SLOW_MODE:
    case UNIPRO_SLOWAUTO_MODE:
        return true;
    default:
        dbg_warn("%s(): unexpected/invalid power mode: 0x%x\n",
                 __func__, mode);
        return false;
    }
}

/*
 * Prepare a link for a change to its M-PHY RATE series.
 */
static int switch_prep_for_series_change(struct tsb_switch *sw,
                                         uint8_t port_id,
                                         const struct unipro_link_cfg *cfg,
                                         uint32_t *pwr_mode) {
    int rc;
    const struct unipro_pwr_cfg *tx = &cfg->upro_tx_cfg;
    const struct unipro_pwr_cfg *rx = &cfg->upro_rx_cfg;
    bool tx_unchanged = (tx->upro_mode == UNIPRO_MODE_UNCHANGED);
    bool rx_unchanged = (rx->upro_mode == UNIPRO_MODE_UNCHANGED);
    uint32_t cur_pwr_mode;
    enum unipro_pwr_mode cur_tx_pwr_mode;
    enum unipro_pwr_mode cur_rx_pwr_mode;

    /* Sanity checks. */
    if (cfg->upro_hs_ser == UNIPRO_HS_SERIES_UNCHANGED) {
        dbg_error("%s(): invoked when no HS series change is required\n",
                  __func__);
        return -EINVAL;
    }
    if (!(tx->upro_mode == UNIPRO_FAST_MODE ||
          tx->upro_mode == UNIPRO_FASTAUTO_MODE ||
          rx->upro_mode == UNIPRO_FAST_MODE ||
          rx->upro_mode == UNIPRO_FASTAUTO_MODE)) {
        dbg_error("%s(): invoked for non-fast RX/TX power modes %d/%d\n",
                  __func__, tx->upro_mode, rx->upro_mode);
        return -EINVAL;
    }

    /* Grab the current power mode. */
    rc = switch_dme_get(sw, port_id, PA_PWRMODE, NCP_SELINDEX_NULL,
                        &cur_pwr_mode);
    if (rc) {
        dbg_error("%s(): can't check current power mode state\n", __func__);
        return rc;
    }

    /* It's illegal to leave power modes unchanged when setting
     * PA_HSSeries, but we can set the mode to the same value. Use
     * that to respect the caller's intent to leave a mode
     * unchanged. */
    if (tx_unchanged || rx_unchanged) {
        if (tx_unchanged) {
            *pwr_mode &= ~0x0f;
            *pwr_mode |= cur_pwr_mode & 0x0f;
        }
        if (rx_unchanged) {
            *pwr_mode &= ~0xf0;
            *pwr_mode |= cur_pwr_mode & 0xf0;
        }
    }

    /* If both directions of the current mode are ready for the
     * change; there's nothing more to do.
     *
     * Otherwise, the "least common denominator" preparation is to
     * bring the entire link to PWM-G1, with one active lane in each
     * direction. */
    cur_tx_pwr_mode = (enum unipro_pwr_mode)(cur_pwr_mode & 0xf);
    cur_rx_pwr_mode = (enum unipro_pwr_mode)((cur_pwr_mode >> 4) & 0xf);
    if (switch_ok_for_series_change(cur_tx_pwr_mode) &&
        switch_ok_for_series_change(cur_rx_pwr_mode)) {
        return 0;
    } else {
        return switch_configure_link_pwm(sw, port_id, 1, 1, 0);
    }
}

/*
 * Sanity check an HS or PWM configuration.
 */
static int switch_active_cfg_is_sane(const struct unipro_pwr_cfg *pcfg,
                                     unsigned max_nlanes) {
    if (pcfg->upro_nlanes > max_nlanes) {
        dbg_error("%s(): attempt to use %u lanes, support at most %u",
                  __func__, pcfg->upro_nlanes, max_nlanes);
        return 0;
    }
    switch (pcfg->upro_mode) {
    case UNIPRO_FAST_MODE:      /* fall through */
    case UNIPRO_FASTAUTO_MODE:
        if (pcfg->upro_gear == 0 || pcfg->upro_gear > 3) {
            dbg_error("%s(): invalid HS gear %u", __func__, pcfg->upro_gear);
            return 0;
        }
        break;
    case UNIPRO_SLOW_MODE:      /* fall through */
    case UNIPRO_SLOWAUTO_MODE:
        if (pcfg->upro_gear == 0 || pcfg->upro_gear > 7) {
            dbg_error("%s(): invalid PWM gear %u", __func__, pcfg->upro_gear);
            return 0;
        }
        break;
    default:
        dbg_error("%s(): unexpected mode %u", __func__, pcfg->upro_mode);
        return 0;
    }
    return 1;
}

static int switch_configure_link_tx(struct tsb_switch *sw,
                                    uint8_t port_id,
                                    const struct unipro_pwr_cfg *tx,
                                    uint32_t tx_term) {
    int rc = 0;
    /* If it needs changing, apply the TX side of the new link
     * configuration. */
    if (tx->upro_mode != UNIPRO_MODE_UNCHANGED) {
        rc = switch_dme_set(sw, port_id, PA_TXGEAR,
                            NCP_SELINDEX_NULL, tx->upro_gear) ||
            switch_dme_set(sw, port_id, PA_TXTERMINATION,
                           NCP_SELINDEX_NULL, tx_term) ||
            switch_dme_set(sw, port_id, PA_ACTIVETXDATALANES,
                           NCP_SELINDEX_NULL, tx->upro_nlanes);
    }
    return rc;
}

static int switch_configure_link_rx(struct tsb_switch *sw,
                                    uint8_t port_id,
                                    const struct unipro_pwr_cfg *rx,
                                    uint32_t rx_term) {
    int rc = 0;
    /* If it needs changing, apply the RX side of the new link
     * configuration.
     */
    if (rx->upro_mode != UNIPRO_MODE_UNCHANGED) {
        rc = switch_dme_set(sw, port_id, PA_RXGEAR,
                            NCP_SELINDEX_NULL, rx->upro_gear) ||
            switch_dme_set(sw, port_id, PA_RXTERMINATION,
                           NCP_SELINDEX_NULL, rx_term) ||
            switch_dme_set(sw, port_id, PA_ACTIVERXDATALANES,
                           NCP_SELINDEX_NULL, rx->upro_nlanes);
    }
    return rc;
}

static int switch_configure_link_user_data
        (struct tsb_switch *sw,
         uint8_t port_id,
         const struct unipro_pwr_user_data *udata) {
    int rc = 0;
    const uint32_t flags = udata->flags;
    if (flags & UPRO_PWRF_FC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA0,
                            NCP_SELINDEX_NULL,
                            udata->upro_pwr_fc0_protection_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & UPRO_PWRF_TC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA1,
                            NCP_SELINDEX_NULL,
                            udata->upro_pwr_tc0_replay_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & UPRO_PWRF_AFC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA2,
                            NCP_SELINDEX_NULL,
                            udata->upro_pwr_afc0_req_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & UPRO_PWRF_FC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA3,
                            NCP_SELINDEX_NULL,
                            udata->upro_pwr_fc1_protection_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & UPRO_PWRF_TC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA4,
                            NCP_SELINDEX_NULL,
                            udata->upro_pwr_tc1_replay_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & UPRO_PWRF_AFC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA5,
                            NCP_SELINDEX_NULL,
                            udata->upro_pwr_afc1_req_timeout);
    }
    return rc;
}

static int switch_configure_link_tsbdata
        (struct tsb_switch *sw,
         uint8_t port_id,
         const struct tsb_local_l2_timer_cfg *tcfg) {
    int rc = 0;
    const unsigned int flags = tcfg->tsb_flags;
    if (flags & TSB_LOCALL2F_FC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_FC0PROTECTIONTIMEOUTVAL,
                            NCP_SELINDEX_NULL,
                            tcfg->tsb_fc0_protection_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_TC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_TC0REPLAYTIMEOUTVAL,
                            NCP_SELINDEX_NULL,
                            tcfg->tsb_tc0_replay_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_AFC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_AFC0REQTIMEOUTVAL,
                            NCP_SELINDEX_NULL,
                            tcfg->tsb_afc0_req_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_FC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_FC1PROTECTIONTIMEOUTVAL,
                            NCP_SELINDEX_NULL,
                            tcfg->tsb_fc1_protection_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_TC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_TC1REPLAYTIMEOUTVAL,
                            NCP_SELINDEX_NULL,
                            tcfg->tsb_tc1_replay_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_AFC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_AFC1REQTIMEOUTVAL,
                            NCP_SELINDEX_NULL,
                            tcfg->tsb_afc1_req_timeout);
    }
    return rc;
}

static int switch_apply_power_mode(struct tsb_switch *sw,
                                   uint8_t port_id,
                                   uint32_t pwr_mode) {
    int rc;
    uint32_t val;
    dbg_insane("%s(): enter, port=%u, pwr_mode=0x%x\n", __func__, port_id,
               pwr_mode);
    rc = switch_dme_set(sw, port_id, PA_PWRMODE, NCP_SELINDEX_NULL,
                        pwr_mode);
    if (rc) {
        goto out;
    }
    do {
        /*
         * Wait until the power mode change completes.
         *
         * FIXME error out after too many retries.
         * FIXME other error handling (UniPro specification 5.7.12.5).
         */
        rc = switch_dme_get(sw, port_id, TSB_DME_POWERMODEIND,
                            NCP_SELINDEX_NULL, &val);
        if (rc) {
            dbg_error("%s: failed to read power mode indication: %d\n",
                      __func__,
                      rc);
            goto out;
        }
    } while (val != TSB_DME_POWERMODEIND_SUCCESS);

    dbg_insane("%s(): testing link state with peer DME access\n", __func__);
    rc = switch_dme_peer_get(sw,
                             port_id,
                             T_CONNECTIONSTATE,
                             NCP_SELINDEX_NULL,
                             &val);

 out:
    dbg_insane("%s(): exit, rc=%d\n", __func__, rc);
    return rc;
}

/**
 * @brief Low-level UniPro link configuration routine.
 *
 * This supports separate reconfiguration of each direction of the
 * UniPro link to different power modes, and allows for hibernation
 * and off modes.
 *
 * Higher level convenience functions to set both directions of a link
 * to the same HS or PWM gear are available.
 *
 * @param sw Switch handle
 * @param port_id Port whose link to reconfigure
 * @param cfg UniPro power configuration to apply
 * @param tcg Toshiba extensions to UniPro power config to apply,
 *            This may be NULL if no extensions are needed.
 *
 * @see switch_configure_link_hs()
 * @see switch_configure_link_pwm()
 */
int switch_configure_link(struct tsb_switch *sw,
                          uint8_t port_id,
                          const struct unipro_link_cfg *cfg,
                          const struct tsb_link_cfg *tcfg) {
    int rc = 0;
    const struct unipro_pwr_cfg *tx = &cfg->upro_tx_cfg;
    uint32_t tx_term = !!(cfg->flags & UPRO_LINKF_TX_TERMINATION);
    const struct unipro_pwr_cfg *rx = &cfg->upro_rx_cfg;
    uint32_t rx_term = !!(cfg->flags & UPRO_LINKF_RX_TERMINATION);
    uint32_t scrambling = !!(cfg->flags & UPRO_LINKF_SCRAMBLING);
    const struct unipro_pwr_user_data *udata = &cfg->upro_user;
    uint32_t pwr_mode = tx->upro_mode | (rx->upro_mode << 4);

    dbg_verbose("%s(): port=%d\n", __func__, port_id);

    /* FIXME ADD JIRA support hibernation and link off modes. */
    if (tx->upro_mode == UNIPRO_HIBERNATE_MODE ||
        tx->upro_mode == UNIPRO_OFF_MODE ||
        rx->upro_mode == UNIPRO_HIBERNATE_MODE ||
        rx->upro_mode == UNIPRO_OFF_MODE) {
        rc = -EOPNOTSUPP;
        goto out;
    }

    /* Changes to a link's HS series require special preparation, and
     * involve restrictions on the power mode to apply next. */
    if (cfg->upro_hs_ser != UNIPRO_HS_SERIES_UNCHANGED) {
        rc = switch_prep_for_series_change(sw, port_id, cfg, &pwr_mode);
        if (rc) {
            goto out;
        }
    }

    /* Sanity-check the configuration. */
    if (!(switch_active_cfg_is_sane(tx, PA_CONN_TX_DATA_LANES_NR) &&
          switch_active_cfg_is_sane(rx, PA_CONN_RX_DATA_LANES_NR))) {
        rc = -EINVAL;
        goto out;
    }

    /* Apply TX and RX link reconfiguration as needed. */
    rc = switch_configure_link_tx(sw, port_id, tx, tx_term) ||
        switch_configure_link_rx(sw, port_id, rx, rx_term);
    if (rc) {
        goto out;
    }

    /* Handle scrambling. */
    rc = switch_dme_set(sw, port_id, PA_SCRAMBLING,
                        NCP_SELINDEX_NULL, scrambling);
    if (rc) {
        goto out;
    }

    /* Set any DME user data we understand. */
    rc = switch_configure_link_user_data(sw, port_id, udata);
    if (rc) {
        goto out;
    }

    /* Handle Toshiba extensions to the link configuration procedure. */
    if (tcfg) {
        rc = switch_configure_link_tsbdata(sw, port_id, &tcfg->tsb_l2tim_cfg);
    }
    if (rc) {
        goto out;
    }

    /* Kick off the actual power mode change, and see what happens. */
    rc = switch_apply_power_mode(sw, port_id, pwr_mode);
 out:
    dbg_insane("%s(): exit, rc=%d\n", __func__, rc);
    return rc;
}

/* Post a message to the IRQ worker thread */
int switch_post_irq(struct tsb_switch *sw)
{
    sem_post(&sw->sw_irq_lock);

    return 0;
}

/* IRQ worker thread */
static void *switch_irq_pending_worker(void *data)
{
    struct tsb_switch *sw = data;

    while (!sw->sw_irq_thread_exit) {

        sem_wait(&sw->sw_irq_lock);
        if (sw->sw_irq_thread_exit)
            break;

        /* Calls the low level handler to clear the interrupt source */
        switch_irq_handler(sw);
    }

    return NULL;
}

/* IRQ worker thread creation */
static int create_switch_irq_thread(struct tsb_switch *sw)
{
    pthread_attr_t thread_attr;
    int ret;

    sem_init(&sw->sw_irq_lock, 0, 0);
    sw->sw_irq_thread_exit = false;

    ret = pthread_attr_init(&thread_attr);
    if (ret)
        goto error;

    ret = pthread_attr_setstacksize(&thread_attr, DEFAULT_STACK_SIZE);
    if (ret)
        goto error;

    ret = pthread_create(&sw->sw_irq_thread, &thread_attr,
                         switch_irq_pending_worker, sw);

error:
    if (ret)
        dbg_error("%s: Failed to create IRQ worker thread\n", __func__);

    pthread_attr_destroy(&thread_attr);

    return ret;
}

/* IRQ worker thread destruction */
static int destroy_switch_irq_thread(struct tsb_switch *sw)
{
    void *value;

    sw->sw_irq_thread_exit = true;
    sem_post(&sw->sw_irq_lock);
    pthread_join(sw->sw_irq_thread, &value);

    return 0;
}

/**
 * @brief Initialize the switch and set default SVC<->Switch route
 * @param sw switch context
 * @param vreg_1p1 gpio for 1p1 regulator
 * @param vreg_1p8 gpio for 1p8 regulator
 * @param reset gpio for reset line
 * @param irq gpio for switch irq
 */
struct tsb_switch *switch_init(struct tsb_switch *sw,
                               unsigned int vreg_1p1,
                               unsigned int vreg_1p8,
                               unsigned int reset,
                               unsigned int irq) {
    unsigned int attr_value, link_status;
    int rc;

    dbg_verbose("%s: Initializing switch\n", __func__);
    if (!sw) {
        goto error;
    }

    if (!vreg_1p1 || !vreg_1p8 || !reset || !irq) {
        goto error;
    }

    sw->vreg_1p1 = vreg_1p1;
    sw->vreg_1p8 = vreg_1p8;
    sw->reset = reset;
    sw->irq = irq;

    switch_power_on_reset(sw);

    rc = switch_init_comm(sw);
    if (rc && (rc != -EOPNOTSUPP)) {
        goto error;
    }

    /*
     * Sanity check
     */
    if (switch_internal_getattr(sw, SWVER, &attr_value)) {
        dbg_error("Switch probe failed\n");
        goto error;
    }

    // Init port <-> deviceID mapping table
    dev_ids_destroy(sw);
    dev_ids_update(sw, SWITCH_PORT_ID, SWITCH_DEVICE_ID);

    /*
     * Set initial SVC deviceId to SWITCH_DEVICE_ID and setup
     * routes for internal L4 access from the SVC
     */
    rc = switch_internal_set_id(sw,
                                L4_CPORT_SVC_TO_CC,
                                L4_PEERCPORT_SVC_TO_CC,
                                CPORT_ENABLE,
                                IRT_ENABLE);
    if (rc) {
        goto error;
    }
    rc = switch_internal_set_id(sw,
                                L4_CPORT_CC_TO_SVC,
                                L4_PEERCPORT_CC_TO_SVC,
                                CPORT_ENABLE,
                                IRT_DISABLE);
    if (rc) {
        goto error;
    }

    // Create Switch interrupt handling thread, enable the interrupt
    rc = create_switch_irq_thread(sw);
    if (rc) {
        dbg_error("%s: Failed to create Switch IRQ thread\n", __func__);
        goto error;
    }

    // Detect Unipro devices
    rc = switch_detect_devices(sw, &link_status);
    if (rc) {
        goto error;
    }

    return sw;

error:
    if (sw) {
        switch_exit(sw);
    }
    dbg_error("%s: Failed to initialize switch.\n", __func__);

    return NULL;
}


/**
 * @brief Power down and disable the switch
 */
void switch_exit(struct tsb_switch *sw) {
    dbg_verbose("%s: Disabling switch\n", __func__);
    switch_irq_enable(sw, false);
    destroy_switch_irq_thread(sw);
    dev_ids_destroy(sw);
    switch_power_off(sw);
}
