#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/gpio.h>
#include <arch/board/board.h>
#include <arch/tsb/gpio.h>
#include <arch/tsb/unipro.h>
#include <debug.h>
#include <unipro/connection.h>
#include <arch/tsb/device_table.h>
#include <arch/tsb/driver.h>

#include "up_arch.h"
#include "chip.h"
#include "tsb_scm.h"
#include "tsb_unipro_es2.h"
#include "tsb_es2_mphy_fixups.h"

/* UniPro Attributes */
#define T_REGACCCTRL_TESTONLY       (0x007F)
#define PA_ACTIVETXDATALANES        (0x1560)
#define PA_TXGEAR                   (0x1568)
#define PA_TXTERMINATION            (0x1569)
#define PA_HSSERIES                 (0x156A)
#define PA_PWRMODE                  (0x1571)
#define PA_ACTIVERXDATALANES        (0x1580)
#define PA_RXGEAR                   (0x1583)
#define PA_RXTERMINATION            (0x1584)
#define PA_PWRMODEUSERDATA0         (0x15B0)
#define N_DEVICEID                  (0x3000)
#define N_DEVICEID_VALID            (0x3001)
#define T_CONNECTIONSTATE           (0x4020)
#define T_PEERDEVICEID              (0x4021)
#define T_PEERCPORTID               (0x4022)
#define T_TRAFFICCLASS              (0x4023)
#define T_CPORTFLAGS                (0x4025)
#define DME_DDBL1_REVISION          (0x5000)
#define DME_DDBL1_LEVEL             (0x5001)
#define DME_DDBL1_DEVICECLASS       (0x5002)
#define DME_DDBL1_MANUFACTUREID     (0x5003)
#define DME_DDBL1_PRODUCTID         (0x5004)
#define DME_DDBL1_LENGTH            (0x5005)
#define DME_DDBL2_VID               (0x6000)
#define DME_DDBL2_PID               (0x6001)
#define TSB_MAILBOX                 (0xA000)
#define TSB_MAXSEGMENTCONFIG        (0xD089)
#define DME_POWERMODEIND            (0xD040)
#define TSB_INTERRUPT_ENABLE        (0xd080)
#define TSB_INTERRUPT_STATUS        (0xd081)

/*
 * "Map" constants for M-PHY fixups.
 */
#define TSB_MPHY_MAP (0x7F)
#define     TSB_MPHY_MAP_TSB_REGISTER_1 (0x01)
#define     TSB_MPHY_MAP_NORMAL         (0x00)
#define     TSB_MPHY_MAP_TSB_REGISTER_2 (0x81)


#define unipro_attr_local_read(attr, val, sel, rc) \
    unipro_attr_read(attr, val, sel, 0, rc)

#define unipro_attr_local_write(attr, val, sel, rc) \
    unipro_attr_write(attr, val, sel, 0, rc)

#define unipro_attr_peer_read(attr, val, sel, rc) \
    unipro_attr_read(attr, val, sel, 1, rc)

#define unipro_attr_peer_write(attr, val, sel, rc) \
    unipro_attr_write(attr, val, sel, 1, rc)

static unsigned int count = 0;
static uint32_t unipro_read(uint32_t offset) {
    return getreg32((volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}

static void unipro_write(uint32_t offset, uint32_t v) {
    putreg32(v, (volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}


static uint32_t cport_get_status(unsigned int cport) {
    uint32_t val;

    val = unipro_read(CPORT_STATUS_0);
    val >>= (cport * 2);
    return (val & (0x3));
}

#if 0
#define MODE_OFFSET(cport)     (cport/16)
#define MODE_MASK(cport, mode) ((mode & 0x3) << (cport % 16))
static int set_transfer_mode(uint32_t cport, int mode) {
    if (mode != 2) {
        return -EOPNOTSUPP;
    }

    unipro_write(AHM_MODE_CTRL_0 + MODE_OFFSET(cport), MODE_MASK(mode));

    return 0;
};
#endif

int unipro_driver_register(struct unipro_driver *driver, unsigned int cportid) {
    return -1;

}

static unsigned int v = 0;

int unipro_send(unsigned int cportid, const void *buf, size_t len) {
    //    lldbg("Sending bytes\n");
    unsigned int i = 0;
    unsigned int j = 0;
    size_t space = 0;
    size_t buffer_space = 0;
    int rc;

    volatile unsigned int *tx_queue = (volatile unsigned int*)(0x50000000 + cportid * 0x20000);
    volatile unsigned char *eom = (volatile unsigned char*)(0x50000000 + (cportid * 0x20000) + 0x1FFFF);

#if 0
    for (i = 1; i <= 1024; i++) {
        lldbg("%u: %u\n", i*4, unipro_read(CPB_TX_BUFFER_SPACE_00));
        putreg32(v, (uint32_t*)&tx_queue[i]);
    }
    v++;
#endif
    //    for (j = 0; j < 16; j++) {
//    while (1) {
    space = unipro_read(CPB_TX_BUFFER_SPACE_00);
    if (space > 16) {
        //        unipro_attr_read(0x4029, &buffer_space, 0, 0, &rc);
        putreg32(0xDEADBEEF + v, &tx_queue[0]);
        putreg32(0xFEEDFACE + v, &tx_queue[1]);
        putreg32(0xCAFEBABE + v, &tx_queue[2]);
        putreg32(0xCAFEBABE + v, &tx_queue[2]);
        putreg32(0xCAFEBABE + v, &tx_queue[2]);
        putreg32(0xCAFEBABE + v, &tx_queue[2]);
        putreg32(0xCAFEBABE + v, &tx_queue[2]);
        putreg32(0xCAFEBABE + v, &tx_queue[2]);
        eom[0] = 1;
//        lldbg("sending %u space: %u\n", v, space);
        v++;
//        lldbg("v: %u fifo space: %u buf_space = %u\n", v, space, buffer_space);
    }
    return 0;
}


/**
 * @brief UniPro debug dump
 */
static void dump_regs(void) {
    struct cport *cport;
    uint32_t val;
    unsigned int rc;
    unsigned int i;

#define DBG_ATTR(attr) do {                  \
    unipro_attr_local_read(attr, &val, 0, &rc); \
    lldbg("    [%s]: 0x%x\n", #attr, val);   \
} while (0);

#define DBG_CPORT_ATTR(attr, cportid) do {         \
    unipro_attr_local_read(attr, &val, cportid, &rc); \
    lldbg("    [%s]: 0x%x\n", #attr, val);         \
} while (0);

#define REG_DBG(reg) do {                 \
    val = unipro_read(reg);               \
    lldbg("    [%s]: 0x%x\n", #reg, val); \
} while (0)

    lldbg("DME Attributes\n");
    lldbg("========================================\n");
    DBG_ATTR(PA_ACTIVETXDATALANES);
    DBG_ATTR(PA_ACTIVERXDATALANES);
    DBG_ATTR(PA_TXGEAR);
    DBG_ATTR(PA_TXTERMINATION);
    DBG_ATTR(PA_HSSERIES);
    DBG_ATTR(PA_PWRMODE);
    DBG_ATTR(PA_ACTIVERXDATALANES);
    DBG_ATTR(PA_RXGEAR);
    DBG_ATTR(PA_RXTERMINATION);
    DBG_ATTR(PA_PWRMODEUSERDATA0);
    DBG_ATTR(N_DEVICEID);
    DBG_ATTR(N_DEVICEID_VALID);
    DBG_ATTR(DME_DDBL1_REVISION);
    DBG_ATTR(DME_DDBL1_LEVEL);
    DBG_ATTR(DME_DDBL1_DEVICECLASS);
    DBG_ATTR(DME_DDBL1_MANUFACTUREID);
    DBG_ATTR(DME_DDBL1_PRODUCTID);
    DBG_ATTR(DME_DDBL1_LENGTH);
    DBG_ATTR(DME_DDBL2_VID);
    DBG_ATTR(DME_DDBL2_PID);
    DBG_ATTR(TSB_MAILBOX);
    DBG_ATTR(TSB_MAXSEGMENTCONFIG);
    DBG_ATTR(DME_POWERMODEIND);

#if 0
    lldbg("Active CPort Configuration:\n");
    lldbg("========================================\n");
    for (i = 0; i < CPORT_MAX; i++) {
        cport = cport_handle(i);
        if (!cport) {
            continue;
        }
        if (cport->connected) {
            lldbg("CP%d:\n", i);
            DBG_CPORT_ATTR(T_CONNECTIONSTATE, i);
            DBG_CPORT_ATTR(T_PEERDEVICEID, i);
            DBG_CPORT_ATTR(T_PEERCPORTID, i);
            DBG_CPORT_ATTR(T_TRAFFICCLASS, i);
            DBG_CPORT_ATTR(T_CPORTFLAGS, i);
        }
    }
#endif

    lldbg("Unipro Interrupt Info:\n");
    lldbg("========================================\n");
    REG_DBG(UNIPRO_INT_EN);
    REG_DBG(AHM_RX_EOM_INT_EN_0);
    REG_DBG(AHM_RX_EOM_INT_EN_1);


    REG_DBG(UNIPRO_INT_BEF);
    REG_DBG(AHS_TIMEOUT_INT_BEF_0);
    REG_DBG(AHS_TIMEOUT_INT_BEF_1);
    REG_DBG(AHM_HRESP_ERR_INT_BEF_0);
    REG_DBG(AHM_HRESP_ERR_INT_BEF_1);
    REG_DBG(CPB_RX_E2EFC_RSLT_ERR_INT_BEF_0);
    REG_DBG(CPB_RX_E2EFC_RSLT_ERR_INT_BEF_1);
    REG_DBG(CPB_TX_RSLTCODE_ERR_INT_BEF_0);
    REG_DBG(CPB_TX_RSLTCODE_ERR_INT_BEF_1);
    REG_DBG(CPB_RX_MSGST_ERR_INT_BEF_0);
    REG_DBG(CPB_RX_MSGST_ERR_INT_BEF_1);
    REG_DBG(LUP_INT_BEF);
    REG_DBG(A2D_ATTRACS_INT_BEF);
    REG_DBG(AHM_RX_EOM_INT_BEF_0);
    REG_DBG(AHM_RX_EOM_INT_BEF_1);
    REG_DBG(AHM_RX_EOM_INT_BEF_2);
    REG_DBG(AHM_RX_EOT_INT_BEF_0);
    REG_DBG(AHM_RX_EOT_INT_BEF_1);




    lldbg("Unipro Registers:\n");
    lldbg("========================================\n");
    REG_DBG(AHM_MODE_CTRL_0);
    REG_DBG(AHM_ADDRESS_00);
    REG_DBG(REG_RX_PAUSE_SIZE_00);
    REG_DBG(CPB_RX_TRANSFERRED_DATA_SIZE_00);
    REG_DBG(CPB_TX_BUFFER_SPACE_00);
    REG_DBG(CPB_TX_RESULTCODE_0);
    REG_DBG(AHS_HRESP_MODE_0);
    REG_DBG(AHS_TIMEOUT_00);
    REG_DBG(CPB_TX_E2EFC_EN_0);
    REG_DBG(CPB_TX_E2EFC_EN_1);
    REG_DBG(CPB_RX_E2EFC_EN_0);
    REG_DBG(CPB_RX_E2EFC_EN_1);
    REG_DBG(CPORT_STATUS_0);

    for (i = 0; i < 16; i++) {
        val = cport_get_status(i);
//        lldbg("cport: %u status; %x\n", i, val);
#if 1

#define T_TXTOKENVALUE 0x4026
#define T_RXTOKENVALUE 0x4027
#define T_LOCALBUFFERSPACE 0x4028
#define T_PEERBUFFERSPACE 0x4029
#define T_CREDITSTOSEND  0x402A
        if (val == 0) {
            lldbg("CPORT %u:\n", i);

            unipro_attr_read(T_PEERDEVICEID, &val, i, 0, &rc);
            lldbg("\t%s: %x\n", "T_PEERDEVICEID", val);

            unipro_attr_read(T_PEERCPORTID, &val, i, 0, &rc);
            lldbg("\t%s: %x\n", "T_PEERCPORTID", val);

            unipro_attr_read(T_TRAFFICCLASS, &val, i, 0, &rc);
            lldbg("\t%s: %x\n", "T_TRAFFICCLASS", val);

            unipro_attr_read(T_CPORTFLAGS, &val, i, 0, &rc);
            lldbg("\t%s: %x\n", "T_PEERCPORTFLAGS", val);

            unipro_attr_read(T_LOCALBUFFERSPACE, &val, i, 0, &rc);
            lldbg("\t%s: %x\n", "T_LOCALBUFFERSPACE", val);

            unipro_attr_read(T_PEERBUFFERSPACE, &val, i, 0, &rc);
            lldbg("\t%s: %x\n", "T_PEERBUFFERSPACE", val);

            unipro_attr_read(T_CREDITSTOSEND, &val, i, 0, &rc);
            lldbg("\t%s: %x\n", "T_CREDITSTOSEND", val);

            unipro_attr_read(T_RXTOKENVALUE, &val, i, 0, &rc);
            lldbg("\t%s: %x\n", "T_RXTOKENVALUE", val);

            unipro_attr_read(T_TXTOKENVALUE, &val, i, 0, &rc);
            lldbg("\t%s: %x\n", "T_TXTOKENVALUE", val);

            unipro_attr_read(T_CONNECTIONSTATE, &val, i, 0, &rc);
            lldbg("\t%s: %x\n", "T_CONNECTIONSTATE", val);

        }
#endif
    }


    lldbg("NVIC:\n");
    lldbg("========================================\n");
    tsb_dumpnvic();

    lldbg("count: %u\n", count);
}
void unipro_info(void) {
    dump_regs();
}

int unipro_attr_write(uint16_t attr,
                      uint32_t val,
                      uint16_t selector,
                      int peer,
                      uint32_t *result_code) {
    uint32_t ctrl;
    // 0x40020000
    ctrl = REG_PEERENA(peer)    |    // Peer or local access
           REG_SELECT(selector) |    // Selector index
           (1 << 28) |
           attr;
    unipro_write(A2D_ATTRACS_CTRL_00, ctrl);
    unipro_write(A2D_ATTRACS_DATA_CTRL_00, val);

    /* Start the access */
    unipro_write(A2D_ATTRACS_MSTR_CTRL,
                 REG_ATTRACS_CNT(1) | REG_ATTRACS_UPD);

    while (!unipro_read(A2D_ATTRACS_INT_BEF))
        ;

    /* Clear status bit */
    unipro_write(A2D_ATTRACS_INT_BEF, 0x1);

    if (result_code)
        *result_code = unipro_read(A2D_ATTRACS_STS_00);

    return 0;
}

static int state = 0;
void unipro_on(int which) {
    while (!state) {
        unipro_send(0, NULL, 1);
    }
}

void unipro_off(int which) {
    unipro_write(CPB_RX_E2EFC_EN_0, 0);
}



int unipro_attr_read(uint16_t attr,
                     uint32_t *val,
                     uint16_t selector,
                     int peer,
                     uint32_t *result_code) {
    uint32_t ctrl;
    // 0x40020000
    ctrl = REG_PEERENA(peer)    |    // Peer or local access
           REG_SELECT(selector) |    // Selector index
           attr;
    unipro_write(A2D_ATTRACS_CTRL_00, ctrl);

    /* Start the access */
    unipro_write(A2D_ATTRACS_MSTR_CTRL,
                 REG_ATTRACS_CNT(1) | REG_ATTRACS_UPD);

    while (!unipro_read(A2D_ATTRACS_INT_BEF))
        ;

    /* Clear status bit */
    unipro_write(A2D_ATTRACS_INT_BEF, 0x1);

    *result_code = unipro_read(A2D_ATTRACS_STS_00);
    *val         = unipro_read(A2D_ATTRACS_DATA_STS_00);

    return 0;
}

#define CPORT_RX_BUF_BASE         (0x20000000U)

static unsigned int devid = 0;

#define irqn_to_cport(irqn)          cport_handle((irqn - TSB_IRQ_UNIPRO_RX_EOM00))
#define cportid_to_irqn(cportid)     (TSB_IRQ_UNIPRO_RX_EOM00 + cportid)
static inline int irq_rx_eom(int irqn, void *context) {
    uint32_t cport;
    cport = irqn - TSB_IRQ_UNIPRO_RX_EOM00;
    unipro_write(AHM_RX_EOM_INT_BEF_0, 0x3 << (cport * 2));
    lldbg("%s: cport %u\n", __func__, cport);
    /*
     * Allow data to flow again
     */
    unipro_write(REG_RX_PAUSE_SIZE_00 + cport*4, (1 << 31) | CPORT_BUF_SIZE);
    return 0;
}

static int irq_unipro(int irqn, void *context) {
    uint32_t val = 0;
    uint32_t e2efc = 0;
    unipro_attr_local_read(TSB_MAILBOX, &val, 0, NULL);
    unipro_attr_local_read(TSB_INTERRUPT_STATUS, NULL, 0, NULL);
    unipro_attr_peer_write(TSB_MAILBOX, 0, 0, NULL);
    e2efc = unipro_read(CPB_RX_E2EFC_EN_0);
    lldbg("%s: irq received: %x e2efc: %x\n", __func__, val, e2efc);
    e2efc |= (1 << (val & 0xFF));
    unipro_write(CPB_RX_E2EFC_EN_0, e2efc);
    return 0;
}

static int es2_fixup_mphy() {
    uint32_t debug_0720 = tsb_get_debug_reg(0x0720);
    uint32_t urc;
    struct tsb_mphy_fixup *fu;

    /*
     * Apply the "register 1" map fixups.
     */
    unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_TSB_REGISTER_1, 0,
                            &urc);
    if (urc) {
        lldbg("%s: failed to switch to register 1 map: %u\n",
              __func__, urc);
        return urc;
    }
    fu = tsb_register_1_map_mphy_fixups;
    do {
        if (tsb_mphy_r1_fixup_is_magic(fu)) {
            /* The magic R1 fixups come from the mysterious and solemn
             * debug register 0x0720. */
            unipro_attr_local_write(0x8002, (debug_0720 >> 1) & 0x1f, 0, &urc);
        } else {
            unipro_attr_local_write(fu->attrid, fu->value, fu->select_index,
                                    &urc);
        }
        if (urc) {
            lldbg("%s: failed to apply register 1 map fixup: %u\n",
                  __func__, urc);
            return urc;
        }
    } while (!tsb_mphy_fixup_is_last(fu++));

    /*
     * Switch to "normal" map.
     */
    unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_NORMAL, 0,
                            &urc);
    if (urc) {
        lldbg("%s: failed to switch to normal map: %u\n",
              __func__, urc);
        return urc;
    }

    /*
     * Apply the "register 2" map fixups.
     */
    unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_TSB_REGISTER_2, 0,
                            &urc);
    if (urc) {
        lldbg("%s: failed to switch to register 2 map: %u\n",
              __func__, urc);
        return urc;
    }
    fu = tsb_register_2_map_mphy_fixups;
    do {
        unipro_attr_local_write(fu->attrid, fu->value, fu->select_index,
                                &urc);
        if (urc) {
            lldbg("%s: failed to apply register 1 map fixup: %u\n",
                  __func__, urc);
            return urc;
        }
    } while (!tsb_mphy_fixup_is_last(fu++));

    /*
     * Switch to "normal" map.
     */
    unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_NORMAL, 0,
                            &urc);
    if (urc) {
        lldbg("%s: failed to switch to normal map: %u\n",
              __func__, urc);
        return urc;
    }

    return 0;
}

void unipro_init(void) {
    if (es2_fixup_mphy()) {
        lldbg("Failed to apply M-PHY fixups (results in link instability at HS-G1).\n");
    }

    /*
     * 5.7.4.1 Pause function is disabled when RX_PAUSE_SIZE is 0
     */

    /*
     * Credits are sent when the following condition is true:
     *  (T_CONNECTIONSTATE == 1) && (CPB_RX_E2EFC_EN == 1)
     *
     * In order to ensure that the credits are received, CPB_RX_E2EFC_EN
     * must be enabled after the peer has T_CONNECTIONSTATE set to 1.
     *
     * With control protocol:
     *     1) SVC probes all interfaces to find AP.
     *     - Finds all AP bridges
     *     - sets route from switch:4 to ap_x:0
     *     - Sets cport 4 to CONNECTED
     *     - Sets bridge cport 0 to CONNECTED
     *     - Bridge polls on connection state
     *     - ???
     *     - Connects to AP cport System controller connection
     *     2) Set up control protocol connection from gpb to apb
     *     3) Tell APB1 that control protocol connection is set
     *     4) APB2 tells APB1 that control protocol connection is set
     *
     * Hack without:
     *      - SVC powers on bridges. Waits for linkup. Waits for bridges to boot
     *        and write READY to SVC mailbox.
     *      - B0 and B1 set boot. CPB_RX_E2EFC_EN_0 = 0. Write READY to peer mailbox.
     *      - SVC sets T_CONNECTIONSTATE to 1 on all relevant connections to
     *        both bridges. Writes CONNECTED to B0 and B1 mailbox
     *      - B0 and B1 set CPB_RX_E2EFC_EN_0 to 1 to trigger tokens
     *      - They can now communicate
     */
    unipro_write(CPB_RX_E2EFC_EN_0, 0);

    /*
     * Configure cport0 to transfer mode 2
     */
    uint32_t base = CPORT_RX_BUF_BASE;
    unipro_write(AHM_MODE_CTRL_0, 0xAA);
    unipro_write(AHM_ADDRESS_00, base);
    base += CPORT_BUF_SIZE;
    unipro_write(AHM_ADDRESS_01, base);
    base += CPORT_BUF_SIZE;
    unipro_write(AHM_ADDRESS_02, base);
    base += CPORT_BUF_SIZE;
    unipro_write(AHM_ADDRESS_03, base);
    base += CPORT_BUF_SIZE;

    /*
     * Set pause size to CPORT_BUF_SIZE
     */
    unipro_write(REG_RX_PAUSE_SIZE_00, (1 << 31) | CPORT_BUF_SIZE);
    unipro_write(REG_RX_PAUSE_SIZE_01, (1 << 31) | CPORT_BUF_SIZE);
    unipro_write(REG_RX_PAUSE_SIZE_02, (1 << 31) | CPORT_BUF_SIZE);
    unipro_write(REG_RX_PAUSE_SIZE_03, (1 << 31) | CPORT_BUF_SIZE);
//    unipro_write(AHS_HRESP_MODE_0, 1);
//    unipro_write(AHS_TIMEOUT_00, 0x10000);

    /*
     * Install IRQ handler
     */
    unipro_write(AHM_RX_EOM_INT_EN_0, 0xFF);
    irq_attach(TSB_IRQ_UNIPRO_RX_EOM00, &irq_rx_eom);
    irq_attach(TSB_IRQ_UNIPRO_RX_EOM01, &irq_rx_eom);
    irq_attach(TSB_IRQ_UNIPRO_RX_EOM02, &irq_rx_eom);
    irq_attach(TSB_IRQ_UNIPRO_RX_EOM03, &irq_rx_eom);
    up_enable_irq(TSB_IRQ_UNIPRO_RX_EOM00);
    up_enable_irq(TSB_IRQ_UNIPRO_RX_EOM01);
    up_enable_irq(TSB_IRQ_UNIPRO_RX_EOM02);
    up_enable_irq(TSB_IRQ_UNIPRO_RX_EOM03);
    unipro_write(UNIPRO_INT_EN, 1);

//    unipro_write(TX_SW_RESET_00, 0x0);
//    unipro_write(RX_SW_RESET_00, 0x0);
//    unipro_write(CPB_RX_E2EFC_EN_0, 1);
//    unipro_write(CPB_RX_E2EFC_EN_1, 0);
//    unipro_write(CPB_TX_E2EFC_EN_0, 1);
//    unipro_write(CPB_TX_E2EFC_EN_1, 0);

 //   unipro_write(CPB_RX_E2EFC_RETRY_TIMES_00, 0xffffffff);

//    unipro_attr_write(T_TRAFFICCLASS, 1, 0, 0, NULL);

    irq_attach(TSB_IRQ_UNIPRO, &irq_unipro);
    up_enable_irq(TSB_IRQ_UNIPRO);
    unipro_attr_local_write(TSB_INTERRUPT_ENABLE, 1 << 15, 0, NULL);

    unipro_info();

    /*
     * Tell switch we're booted
     */
    unipro_attr_peer_write(TSB_MAILBOX, CONNECTION_BOOTED, 0, NULL);

//    /*
//     * Now wait for the SVC to tell us the connection has been set up
//     * before enabling E2EFC and allowing credits to flow.
//     */
//    unsigned int rc;
//    unsigned int val;
//    while (1) {
//        rc = unipro_attr_local_read(TSB_MAILBOX, &val, 0, &rc);
//        if (val == CONNECTION_READY) {
//            unipro_write(CPB_RX_E2EFC_EN_0, 1);
//            break;
//        }
//    }

    /*
     * Turn on mailbox and wait for notifications of connected cports
     */

}

void start_the_world(void) {
    tsb_gpio_register();
    tsb_device_table_register();
    tsb_driver_register();
    unipro_init();
    nsh_main(0, NULL);
}



