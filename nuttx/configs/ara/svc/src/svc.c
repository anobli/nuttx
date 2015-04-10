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

#define DBG_COMP DBG_SVC

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>

#include <arch/board/board.h>
#include <unipro/connection.h>

#include "string.h"
#include "ara_board.h"
#include "up_debug.h"
#include "interface.h"
#include "tsb_switch.h"
#include "svc.h"

static struct svc the_svc;
struct svc *ara_svc = &the_svc;

/*
 * Static connections table
 *
 * The routing and connections setup is made of two tables:
 * 1) The interface to deviceID mapping table. Every interface is given by
 *    its name (provided in the board file). The deviceID to associate to
 *    the interface is freely chosen.
 *    Cf. the console command 'power' for the interfaces naming.
 *    The physical port to the switch is retrieved from the board file
 *    information, there is no need to supply it in the connection tables.
 *
 * 2) The connections table, given by the deviceID and the CPort to setup, on
 *    both local and remote ends of the link. The routing will be setup in a
 *    bidirectional way.
 *
 * Spring interfaces placement on BDB1B/2A:
 *
 *                             8
 *                            (9)
 *          7     5  6
 *    3  2     4        1
 */
struct svc_interface_device_id {
    char *interface_name;       // Interface name
    uint8_t device_id;          // DeviceID
    uint8_t port_id;            // PortID
    bool found;
    uint32_t state;
};

/*
 * Default routes used on BDB1B demo
 */
#define DEV_ID_APB1             (1)
#define DEV_ID_APB2             (2)
#define DEV_ID_APB3             (3)
#define DEV_ID_GPB1             (4)
#define DEV_ID_GPB2             (5)

#define DEV_ID_SPRING6          (8)
#define DEMO_GPIO_APB1_CPORT    (0)
#define DEMO_GPIO_APB2_CPORT    (5)
#define DEMO_I2C_APB1_CPORT     (1)
#define DEMO_I2C_APB2_CPORT     (4)
#define DEMO_DSI_APB1_CPORT     (16)
#define DEMO_DSI_APB2_CPORT     (16)

/* Interface name to deviceID mapping table */
static struct svc_interface_device_id devid[] = {
    { "apb1", DEV_ID_APB1 },
    { "apb2", DEV_ID_APB2 },
    { "apb3", DEV_ID_APB3 },
    { "gpb1", DEV_ID_GPB1 },
    { "gpb2", DEV_ID_GPB2 },
    { "spring6", DEV_ID_SPRING6 },
};

/* Connections table */
static struct unipro_connection conn[] = {
    {
        .device_id0 = DEV_ID_APB1,
        .cport_id0  = 0,
        .device_id1 = DEV_ID_APB2,
        .cport_id1  = 0,
        .tc = CPORT_TC0,
        .flags = CPORT_FLAGS_E2EFC | CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N
    },
    {
        .device_id0 = DEV_ID_APB1,
        .cport_id0  = 1,
        .device_id1 = DEV_ID_APB3,
        .cport_id1  = 0,
        .tc = CPORT_TC0,
        .flags = CPORT_FLAGS_E2EFC | CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N
    },
    {
        .device_id0 = DEV_ID_APB1,
        .cport_id0  = 2,
        .device_id1 = DEV_ID_GPB1,
        .cport_id1  = 0,
        .tc = CPORT_TC0,
        .flags = CPORT_FLAGS_E2EFC | CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N
    },
    {
        .device_id0 = DEV_ID_APB1,
        .cport_id0  = 3,
        .device_id1 = DEV_ID_GPB2,
        .cport_id1  = 0,
        .tc = CPORT_TC0,
        .flags = CPORT_FLAGS_E2EFC | CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N
    },
};


static int connection_state_get(uint8_t id) {
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(devid); i++) {
        if (id == devid[i].device_id) {
            return devid[i].state;
        }
    }
    return 0;
}

static int setup_default_routes(struct tsb_switch *sw) {
    int rc;
    unsigned int i, j;
    struct interface *iface;

    /*
     * Setup hard-coded default routes from the routing and
     * connection tables
     */

    /* Setup Port <-> deviceID and configure the Switch routing table */
    for (i = 0; i < ARRAY_SIZE(devid); i++) {
        devid[i].found = false;
        /*
         * Retrieve the portID from the interface name and fill in the device id table.
         */
        interface_foreach(iface, j) {
            if (!strcmp(iface->name, devid[i].interface_name)) {
                devid[i].port_id = iface->switch_portid;

                dbg_info("Setting deviceID %d to interface %s (portID %d)\n",
                         devid[i].device_id, devid[i].interface_name,
                         devid[i].port_id);

                rc = switch_if_dev_id_set(sw, devid[i].port_id,
                                          devid[i].device_id);
                if (rc) {
                    dbg_error("Failed to assign deviceID %u to interface %s\n",
                              devid[i].device_id, devid[i].interface_name);
                    continue;
                }
                devid[i].found = true;
            }
        }
    }

    /*
     * Fill in the switch port id in the connections table if the
     * interfaces are present.
     */
    for (i = 0; i < ARRAY_SIZE(conn); i++) {
        for (j = 0; j < ARRAY_SIZE(devid); j++) {
            if (!devid[j].found)
                continue;

            if (devid[j].device_id == conn[i].device_id0) {
                conn[i].port_id0 = devid[j].port_id;
            }

            if (devid[j].device_id == conn[i].device_id1) {
                conn[i].port_id1 = devid[j].port_id;
            }
        }
    }

    /* Connections setup */
    int done = 0;
    while (!done) {
        /*
         * Loop through all found bridges and update state
         */
        for (i = 0; i < ARRAY_SIZE(devid); i++) {
            if (!devid[i].found) {
                continue;
            }
            rc = switch_dme_get(sw, devid[i].port_id, TSB_MAILBOX, 0, &devid[i].state);
            if (rc) {
                dbg_error("Failed to read mailbox on port %u\n", devid[i].port_id);
                continue;
            }
        }

        up_udelay(2 * 1000 * 1000);

        for (i = 0; i < ARRAY_SIZE(conn); i++) {
            /* If both are present, create the requested connection */
            if (conn[i].state == 0) {
                dbg_info("Creating connection: [%u:%u:%u]<->[%u:%u:%u] TC: %u Flags: %x\n",
                        conn[i].port_id0,
                        conn[i].device_id0,
                        conn[i].cport_id0,
                        conn[i].port_id1,
                        conn[i].device_id1,
                        conn[i].cport_id1,
                        conn[i].tc,
                        conn[i].flags);

                /* Update Switch routing table */
                rc = switch_setup_routing_table(sw,
                        conn[i].device_id0, conn[i].port_id0,
                        conn[i].device_id1, conn[i].port_id1);
                if (rc) {
                    dbg_error("Failed to setup routing table [%u:%u]<->[%u:%u]\n",
                            conn[i].device_id0, conn[i].port_id0,
                            conn[i].device_id1, conn[i].port_id1);
                    return -1;
                }

                rc = switch_connection_create(sw, &conn[i]);
                if (rc) {
                    return -1;
                }

                /*
                 * And finally tell the bridges that they can proceed. They will then turn on
                 * E2EFC and tokens will begin flowing.
                 */
#if 0
                dbg_info("Setting local mailbox...\n");
                rc = switch_dme_set(sw, conn[i].port_id0, TSB_MAILBOX, 0, 1);
                if (rc) {
                    return rc;
                }
                rc = switch_dme_set(sw, conn[i].port_id1, TSB_MAILBOX, 0, 1);
                if (rc) {
                    return rc;
                }

#endif
                uint32_t cp0 = (1 << 16) | conn[i].cport_id0;
                uint32_t cp1 = (2 << 16) | conn[i].cport_id1;
                dbg_info("Setting peer mailbox...: %x\n", cp0);
                rc = switch_dme_peer_set(sw, conn[i].port_id0, TSB_MAILBOX, 0, cp0);
                if (rc) {
                    return rc;
                }
                dbg_info("Setting peer mailbox...: %x\n", cp1);
                rc = switch_dme_peer_set(sw, conn[i].port_id1, TSB_MAILBOX, 0, cp1);
                if (rc) {
                    return rc;
                }

#if 0
                uint32_t mbox0, mbox1;
                do {
                    dbg_info("Waiting for mailbox clear\n");
                    rc = switch_dme_get(sw, conn[i].port_id0, TSB_MAILBOX, 0, &mbox0);
                    if (rc) {
                        return rc;
                    }
                    rc = switch_dme_get(sw, conn[i].port_id1, TSB_MAILBOX, 0, &mbox1);
                    if (rc) {
                        return rc;
                    }
                    usleep(50000);
                } while (mbox0 || mbox1);
#endif

                conn[i].state = 1;
                dbg_info("Created connection: [%u:%u:%u]<->[%u:%u:%u] TC: %u Flags: %x\n",
                        conn[i].port_id0,
                        conn[i].device_id0,
                        conn[i].cport_id0,
                        conn[i].port_id1,
                        conn[i].device_id1,
                        conn[i].cport_id1,
                        conn[i].tc,
                        conn[i].flags);
            }
        }

        for (i = 0; i < ARRAY_SIZE(conn); i++) {
            if (!conn[i].state) {
                break;
            }

            if (i == (ARRAY_SIZE(conn) - 1)) {
                done = 1;
            }
        }
        usleep(50000);
    }

    switch_dump_routing_table(sw);

    return 0;
}


int svc_init(void) {
    struct ara_board_info *info;
    struct tsb_switch *sw;
    int i, rc;

    dbg_set_config(DBG_SVC, DBG_VERBOSE);
    dbg_info("Initializing SVC\n");

    // Allocate and zero the sw struct
    sw = zalloc(sizeof(struct tsb_switch));
    if (!sw)
        return -ENOMEM;

    /*
     * Board-specific initialization, all boards must define this.
     */
    info = board_init(sw);
    if (!info) {
        dbg_error("%s: No board information provided.\n", __func__);
        goto error0;
    }
    the_svc.board_info = info;

    /* Power on all provided interfaces */
    if (!info->interfaces) {
        dbg_error("%s: No interface information provided\n", __func__);
        goto error1;
    }

    rc = interface_init(info->interfaces, info->nr_interfaces);
    if (rc < 0) {
        dbg_error("%s: Failed to initialize interfaces\n", __func__);
        goto error1;
    }

    /* Init Switch */
    sw = switch_init(sw,
                     info->sw_1p1,
                     info->sw_1p8,
                     info->sw_reset,
                     info->sw_irq);
    if (!sw) {
        dbg_error("%s: Failed to initialize switch.\n", __func__);
        goto error2;
    }
    the_svc.sw = sw;

    /* Set up default routes */
    rc = setup_default_routes(sw);
    if (rc) {
        dbg_error("%s: Failed to set default routes\n", __func__);
    }

    /*
     * Enable the switch IRQ
     *
     * Note: the IRQ must be enabled after all NCP commands have been sent
     * for the switch and Unipro devices initialization.
     */
    rc = switch_irq_enable(sw, true);
    if (rc) {
        goto error2;
    }

    /* Enable interrupts for all Unipro ports */
    for (i = 0; i < SWITCH_PORT_MAX; i++)
        switch_port_irq_enable(sw, i, true);

    return 0;

error2:
    switch_irq_enable(sw, false);
    interface_exit();
error1:
    board_exit(sw);
error0:
    free(sw);
    return -1;
}

void svc_exit(void) {
    if (the_svc.sw)
        switch_exit(the_svc.sw);

    interface_exit();

    if (the_svc.sw)
        board_exit(the_svc.sw);

    free(the_svc.sw);
    the_svc.sw = NULL;
    the_svc.board_info = NULL;
}

