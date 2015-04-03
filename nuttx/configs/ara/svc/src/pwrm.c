/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted.
 *
 * @brief ARA BDB Power Measurement Library
 */

#define DBG_COMP DBG_POWER

#include <pwrm.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <nuttx/i2c.h>
#include <tca6424.h>
#include <up_debug.h>

#define VSW_1P1_PLL_I2C_ADDR        0x42
#define VSW_1P1_CORE_I2C_ADDR       0x41
#define VSW_1P8_UNIPRO_I2C_ADDR     0x47
#define VSW_1P8_IO_I2C_ADDR         0x48

#define VAPB_1P1_CORE_I2C_ADDR      0x41
#define VAPB_1P1_PLL1_I2C_ADDR      0x42
#define VAPB_1P2_CDSI_PLL_I2C_ADDR  0x4A
#define VAPB_1P2_CDSI_I2C_ADDR      0x4B
#define VAPB_1P2_HSIC_I2C_ADDR      0x46
#define VAPB_1P8_UNIPRO_I2C_ADDR    0x47
#define VAPB_1P8_IO_I2C_ADDR        0x48
#define VAPB_1P1_PLL2_I2C_ADDR      0x43

#define VGPB_1P1_CORE_I2C_ADDR      0x41
#define VGPB_1P1_PLL1_I2C_ADDR      0x42
#define VGPB_SDIO_I2C_ADDR          0x49
#define VGPB_1P2_HSIC_I2C_ADDR      0x46
#define VGPB_1P8_UNIPRO_I2C_ADDR    0x47
#define VGPB_1P8_IO_I2C_ADDR        0x48
#define VGPB_1P1_PLL2_I2C_ADDR      0x43

#define INVALID_I2C_ADDR            0xFF
#define PWRM_I2C_BUS                2
#define U135_IO_EXPANDER_ADDR       0x23

#define I2C_INA230_SEL1_A           17
#define I2C_INA230_SEL1_B           18
#define I2C_INA230_SEL1_INH         19

#define I2C_INA230_SEL2_A           20
#define I2C_INA230_SEL2_B           21
#define I2C_INA230_SEL2_INH         22

#define INA230_SHUNT_VALUE          2 /* mohm */

static const char pwrm_dev_names[DEV_COUNT][DEV_NAME_MAX_LENGTH] = {
    "SWitch",
    "APB1",
    "APB2",
    "APB3",
    "GPB1",
    "GPB2",
};

static const char pwrm_rail_names[DEV_MAX_RAIL_COUNT][DEV_COUNT][RAIL_NAME_MAX_LENGTH] = {
    {"VSW_1P1_PLL", "VAPB1_1P1_CORE", "VAPB2_1P1_CORE", "VAPB3_1P1_CORE", "VGPB1_1P1_CORE", "VGPB2_1P1_CORE"},
    {"VSW_1P1_CORE", "VAPB1_1P1_PLL1", "VAPB2_1P1_PLL1", "VAPB3_1P1_PLL1", "VGPB1_1P1_PLL1", "VGPB2_1P1_PLL1"},
    {"VSW_1P8_UNIPRO", "VAPB1_1P2_CDSI_PLL", "VAPB2_1P2_CDSI_PLL", "VAPB3_1P2_CDSI_PLL", "VGPB1_SDIO", "VGPB2_SDIO"},
    {"VSW_1P8_IO", "VAPB1_1P2_CDSI", "VAPB2_1P2_CDSI", "VAPB3_1P2_CDSI", "VGPB1_1P2_HSIC", "VGPB2_1P2_HSIC"},
    {"VSW_ERROR", "VAPB1_1P2_HSIC", "VAPB2_1P2_HSIC", "VAPB3_1P2_HSIC", "VGPB1_1P8_UNIPRO", "VGPB2_1P8_UNIPRO"},
    {"VSW_ERROR", "VAPB1_1P8_UNIPRO", "VAPB2_1P8_UNIPRO", "VAPB3_1P8_UNIPRO", "VGPB1_1P8_IO", "VGPB2_1P8_IO"},
    {"VSW_ERROR", "VAPB1_1P8_IO", "VAPB2_1P8_IO", "VAPB3_1P8_IO", "VGPB1_1P1_PLL2", "VGPB2_1P1_PLL2"},
    {"VSW_ERROR", "VAPB1_1P1_PLL2", "VAPB2_1P1_PLL2", "VAPB3_1P1_PLL2", "VGPB1_ERROR", "VGPB1_ERROR"}
};

static const uint8_t pwrm_i2c_addr[DEV_MAX_RAIL_COUNT][DEV_COUNT] = {
    {VSW_1P1_PLL_I2C_ADDR, VAPB_1P1_CORE_I2C_ADDR, VAPB_1P1_CORE_I2C_ADDR, VAPB_1P1_CORE_I2C_ADDR, VGPB_1P1_CORE_I2C_ADDR, VGPB_1P1_CORE_I2C_ADDR},
    {VSW_1P1_CORE_I2C_ADDR, VAPB_1P1_PLL1_I2C_ADDR, VAPB_1P1_PLL1_I2C_ADDR, VAPB_1P1_PLL1_I2C_ADDR, VGPB_1P1_PLL1_I2C_ADDR, VGPB_1P1_PLL1_I2C_ADDR},
    {VSW_1P8_UNIPRO_I2C_ADDR, VAPB_1P2_CDSI_PLL_I2C_ADDR, VAPB_1P2_CDSI_PLL_I2C_ADDR, VAPB_1P2_CDSI_PLL_I2C_ADDR, VGPB_SDIO_I2C_ADDR, VGPB_SDIO_I2C_ADDR},
    {VSW_1P8_IO_I2C_ADDR, VAPB_1P2_CDSI_I2C_ADDR, VAPB_1P2_CDSI_I2C_ADDR, VAPB_1P2_CDSI_I2C_ADDR, VGPB_1P2_HSIC_I2C_ADDR, VGPB_1P2_HSIC_I2C_ADDR},
    {INVALID_I2C_ADDR, VAPB_1P2_HSIC_I2C_ADDR, VAPB_1P2_HSIC_I2C_ADDR, VAPB_1P2_HSIC_I2C_ADDR, VGPB_1P8_UNIPRO_I2C_ADDR, VGPB_1P8_UNIPRO_I2C_ADDR},
    {INVALID_I2C_ADDR, VAPB_1P8_UNIPRO_I2C_ADDR, VAPB_1P8_UNIPRO_I2C_ADDR, VAPB_1P8_UNIPRO_I2C_ADDR, VGPB_1P8_IO_I2C_ADDR, VGPB_1P8_IO_I2C_ADDR},
    {INVALID_I2C_ADDR, VAPB_1P8_IO_I2C_ADDR, VAPB_1P8_IO_I2C_ADDR, VAPB_1P8_IO_I2C_ADDR, VGPB_1P1_PLL2_I2C_ADDR, VGPB_1P1_PLL2_I2C_ADDR},
    {INVALID_I2C_ADDR, VAPB_1P1_PLL2_I2C_ADDR, VAPB_1P1_PLL2_I2C_ADDR, VAPB_1P1_PLL2_I2C_ADDR, INVALID_I2C_ADDR, INVALID_I2C_ADDR}
};

static struct i2c_dev_s *i2c_dev;
static tca6424_device *tca6424_dev;
static uint32_t pwrm_current_lsb;
static ina230_conversion_time pwrm_ct;
static ina230_avg_count pwrm_avg_count;

const char *pwrm_dev_name(uint8_t dev)
{
    if (dev >= DEV_COUNT) {
        return NULL;
    }

    return pwrm_dev_names[dev];
}

const char *pwrm_rail_name(uint8_t dev, uint8_t rail)
{
    switch (dev) {
    case DEV_SW:
        if (rail >= VSW_COUNT) {
            return NULL;
        }
        break;
    case DEV_APB1:
    case DEV_APB2:
    case DEV_APB3:
        if (rail >= VAPB_COUNT) {
            return NULL;
        }
        break;
    case DEV_GPB1:
    case DEV_GPB2:
        if (rail >= VGPB_COUNT) {
            return NULL;
        }
        break;
    default:
        return NULL;
    }

    return pwrm_rail_names[rail][dev];
}

int pwrm_rail_id(const char *name, uint8_t *dev, uint8_t *rail)
{
    int ret = 0;

    if ((name == NULL) || (dev == NULL) || (rail == NULL)) {
        ret = -ENODEV;
    } else {
        *dev = DEV_COUNT;
        *rail = VAPB_COUNT;
    }

    if (strcmp(name, "VSW_1P1_PLL") == 0) {
        *dev = DEV_SW;
        *rail = VSW_1P1_PLL;
    } else if (strcmp(name, "VSW_1P1_CORE") == 0) {
        *dev = DEV_SW;
        *rail = VSW_1P1_CORE;
    } else if (strcmp(name, "VSW_1P8_UNIPRO") == 0) {
        *dev = DEV_SW;
        *rail = VSW_1P8_UNIPRO;
    } else if (strcmp(name, "VSW_1P8_IO") == 0) {
        *dev = DEV_SW;
        *rail = VSW_1P8_IO;
    } else if (strcmp(name, "VAPB1_1P1_CORE") == 0) {
        *dev = DEV_APB1;
        *rail = VAPB_1P1_CORE;
    } else if (strcmp(name, "VAPB1_1P1_PLL1") == 0) {
        *dev = DEV_APB1;
        *rail = VAPB_1P1_PLL1;
    } else if (strcmp(name, "VAPB1_1P2_CDSI_PLL") == 0) {
        *dev = DEV_APB1;
        *rail = VAPB_1P2_CDSI_PLL;
    } else if (strcmp(name, "VAPB1_1P2_CDSI") == 0) {
        *dev = DEV_APB1;
        *rail = VAPB_1P2_CDSI;
    } else if (strcmp(name, "VAPB1_1P2_HSIC") == 0) {
        *dev = DEV_APB1;
        *rail = VAPB_1P2_HSIC;
    } else if (strcmp(name, "VAPB1_1P8_UNIPRO") == 0) {
        *dev = DEV_APB1;
        *rail = VAPB_1P8_UNIPRO;
    } else if (strcmp(name, "VAPB1_1P8_IO") == 0) {
        *dev = DEV_APB1;
        *rail = VAPB_1P8_IO;
    } else if (strcmp(name, "VAPB1_1P1_PLL2") == 0) {
        *dev = DEV_APB1;
        *rail = VAPB_1P1_PLL2;
    } else if (strcmp(name, "VAPB2_1P1_CORE") == 0) {
        *dev = DEV_APB2;
        *rail = VAPB_1P1_CORE;
    } else if (strcmp(name, "VAPB2_1P1_PLL1") == 0) {
        *dev = DEV_APB2;
        *rail = VAPB_1P1_PLL1;
    } else if (strcmp(name, "VAPB2_1P2_CDSI_PLL") == 0) {
        *dev = DEV_APB2;
        *rail = VAPB_1P2_CDSI_PLL;
    } else if (strcmp(name, "VAPB2_1P2_CDSI") == 0) {
        *dev = DEV_APB2;
        *rail = VAPB_1P2_CDSI;
    } else if (strcmp(name, "VAPB2_1P2_HSIC") == 0) {
        *dev = DEV_APB2;
        *rail = VAPB_1P2_HSIC;
    } else if (strcmp(name, "VAPB2_1P8_UNIPRO") == 0) {
        *dev = DEV_APB2;
        *rail = VAPB_1P8_UNIPRO;
    } else if (strcmp(name, "VAPB2_1P8_IO") == 0) {
        *dev = DEV_APB2;
        *rail = VAPB_1P8_IO;
    } else if (strcmp(name, "VAPB2_1P1_PLL2") == 0) {
        *dev = DEV_APB2;
        *rail = VAPB_1P1_PLL2;
    } else if (strcmp(name, "VAPB3_1P1_CORE") == 0) {
        *dev = DEV_APB3;
        *rail = VAPB_1P1_CORE;
    } else if (strcmp(name, "VAPB3_1P1_PLL1") == 0) {
        *dev = DEV_APB3;
        *rail = VAPB_1P1_PLL1;
    } else if (strcmp(name, "VAPB3_1P2_CDSI_PLL") == 0) {
        *dev = DEV_APB3;
        *rail = VAPB_1P2_CDSI_PLL;
    } else if (strcmp(name, "VAPB3_1P2_CDSI") == 0) {
        *dev = DEV_APB3;
        *rail = VAPB_1P2_CDSI;
    } else if (strcmp(name, "VAPB3_1P2_HSIC") == 0) {
        *dev = DEV_APB3;
        *rail = VAPB_1P2_HSIC;
    } else if (strcmp(name, "VAPB3_1P8_UNIPRO") == 0) {
        *dev = DEV_APB3;
        *rail = VAPB_1P8_UNIPRO;
    } else if (strcmp(name, "VAPB3_1P8_IO") == 0) {
        *dev = DEV_APB3;
        *rail = VAPB_1P8_IO;
    } else if (strcmp(name, "VAPB3_1P1_PLL2") == 0) {
        *dev = DEV_APB3;
        *rail = VAPB_1P1_PLL2;
    } else if (strcmp(name, "VGPB1_1P1_CORE") == 0) {
        *dev = DEV_GPB1;
        *rail = VGPB_1P1_CORE;
    } else if (strcmp(name, "VGPB1_1P1_PLL1") == 0) {
        *dev = DEV_GPB1;
        *rail = VGPB_1P1_PLL1;
    } else if (strcmp(name, "VGPB1_SDIO") == 0) {
        *dev = DEV_GPB1;
        *rail = VGPB_SDIO;
    } else if (strcmp(name, "VGPB1_1P2_HSIC") == 0) {
        *dev = DEV_GPB1;
        *rail = VGPB_1P2_HSIC;
    } else if (strcmp(name, "VGPB1_1P8_UNIPRO") == 0) {
        *dev = DEV_GPB1;
        *rail = VGPB_1P8_UNIPRO;
    } else if (strcmp(name, "VGPB1_1P8_IO") == 0) {
        *dev = DEV_GPB1;
        *rail = VGPB_1P8_IO;
    } else if (strcmp(name, "VGPB1_1P1_PLL2") == 0) {
        *dev = DEV_GPB1;
        *rail = VGPB_1P1_PLL2;
    } else if (strcmp(name, "VGPB2_1P1_CORE") == 0) {
        *dev = DEV_GPB2;
        *rail = VGPB_1P1_CORE;
    } else if (strcmp(name, "VGPB2_1P1_PLL1") == 0) {
        *dev = DEV_GPB2;
        *rail = VGPB_1P1_PLL1;
    } else if (strcmp(name, "VGPB2_SDIO") == 0) {
        *dev = DEV_GPB2;
        *rail = VGPB_SDIO;
    } else if (strcmp(name, "VGPB2_1P2_HSIC") == 0) {
        *dev = DEV_GPB2;
        *rail = VGPB_1P2_HSIC;
    } else if (strcmp(name, "VGPB2_1P8_UNIPRO") == 0) {
        *dev = DEV_GPB2;
        *rail = VGPB_1P8_UNIPRO;
    } else if (strcmp(name, "VGPB2_1P8_IO") == 0) {
        *dev = DEV_GPB2;
        *rail = VGPB_1P8_IO;
    } else if (strcmp(name, "VGPB2_1P1_PLL2") == 0) {
        *dev = DEV_GPB2;
        *rail = VGPB_1P1_PLL2;
    } else {
        ret = -ENODEV;
    }

    dbg_verbose("%s(): name=%s => device=%u rail=%u\n",
                __func__, name, *dev, *rail);
    return ret;
}

int pwrm_device_id(const char *name, uint8_t *dev)
{
    int ret = 0;

    if ((name == NULL) || (dev == NULL)) {
        ret = -ENODEV;
    } else {
        *dev = DEV_COUNT;
    }

    if (strcmp(name, "SW") == 0) {
        *dev = DEV_SW;
    } else if (strcmp(name, "APB1") == 0) {
        *dev = DEV_APB1;
    } else if (strcmp(name, "APB2") == 0) {
        *dev = DEV_APB2;
    } else if (strcmp(name, "APB3") == 0) {
        *dev = DEV_APB3;
    } else if (strcmp(name, "GPB1") == 0) {
        *dev = DEV_GPB1;
    } else if (strcmp(name, "GPB2") == 0) {
        *dev = DEV_GPB2;
    } else {
        *dev = DEV_COUNT;
        ret = -ENODEV;
    }

    dbg_verbose("%s(): name=%s => device=%u\n", __func__, name, *dev);

    return ret;
}

static int pwrm_ina230_select(uint8_t dev)
{
    int ret = 0;
    static int current_dev = -1;

    if (dev == current_dev) {
        dbg_verbose("%s(): device already selected (%u).\n", __func__, dev);
        return 0;
    }

    /* First inhibit all lines, to make sure there is no short/collision */
    ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_INH, 1);
    ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL2_INH, 1);

    switch (dev) {
    case DEV_SW:
        dbg_verbose("%s(): dev=%s select U97\n",
                    __func__, pwrm_dev_name(dev));
        ret = tca6424_set(tca6424_dev, I2C_INA230_SEL1_A, 0);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_B, 0);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_INH, 0);
        if (ret) {
            dbg_error("%s(): failed! (%d)\n", __func__, ret);
            return -EIO;
        }
        break;
    case DEV_APB1:
        dbg_verbose("%s(): dev=%s select U97\n",
                    __func__, pwrm_dev_name(dev));
        ret = tca6424_set(tca6424_dev, I2C_INA230_SEL1_A, 1);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_B, 0);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_INH, 0);
        if (ret) {
            dbg_error("%s(): failed! (%d)\n", __func__, ret);
            return -EIO;
        }
        break;
    case DEV_APB2:
        dbg_verbose("%s(): dev=%s select U97\n",
                    __func__, pwrm_dev_name(dev));
        ret = tca6424_set(tca6424_dev, I2C_INA230_SEL1_A, 0);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_B, 1);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_INH, 0);
        if (ret) {
            dbg_error("%s(): failed! (%d)\n", __func__, ret);
            return -EIO;
        }
        break;
    case DEV_APB3:
        dbg_verbose("%s(): dev=%s select U97\n",
                    __func__, pwrm_dev_name(dev));
        ret = tca6424_set(tca6424_dev, I2C_INA230_SEL1_A, 1);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_B, 1);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_INH, 0);
        if (ret) {
            dbg_error("%s(): failed! (%d)\n", __func__, ret);
            return -EIO;
        }
        break;
    case DEV_GPB1:
        dbg_verbose("%s(): dev=%s select U103\n",
                    __func__, pwrm_dev_name(dev));
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL2_A, 0);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL2_B, 0);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL2_INH, 0);
        if (ret) {
            dbg_error("%s(): failed! (%d)\n", __func__, ret);
            return -EIO;
        }
        break;
    case DEV_GPB2:
        dbg_verbose("%s(): dev=%s select U103\n",
                    __func__, pwrm_dev_name(dev));
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL2_A, 1);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL2_B, 0);
        ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL2_INH, 0);
        if (ret) {
            dbg_error("%s(): failed! (%d)\n", __func__, ret);
            return -EIO;
        }
        break;
    default:
        return -EINVAL;
    }

    /* Save current selected device */
    current_dev = dev;

    return 0;
}

static uint8_t pwrm_i2c_addr_get(uint8_t dev, uint8_t rail)
{
    switch (dev) {
    case DEV_SW:
        if (rail >= VSW_COUNT) {
            return INVALID_I2C_ADDR;
        }
        break;
    case DEV_APB1:
    case DEV_APB2:
    case DEV_APB3:
        if (rail >= VAPB_COUNT) {
            return INVALID_I2C_ADDR;
        }
        break;
    case DEV_GPB1:
    case DEV_GPB2:
        if (rail >= VGPB_COUNT) {
            return INVALID_I2C_ADDR;
        }
        break;
    default:
        return INVALID_I2C_ADDR;
    }

    return pwrm_i2c_addr[rail][dev];
}

int pwrm_dev_rail_count(uint8_t dev)
{
    int rcount;

    switch (dev) {
    case DEV_SW:
        rcount = VSW_COUNT;
        break;
    case DEV_APB1:
    case DEV_APB2:
    case DEV_APB3:
        rcount = VAPB_COUNT;
        break;
    case DEV_GPB1:
    case DEV_GPB2:
        rcount = VGPB_COUNT;
        break;
    default:
        rcount = -EINVAL;
    }

    return rcount;
}

int pwrm_init(uint32_t current_lsb_uA,
              ina230_conversion_time ct,
              ina230_avg_count avg_count)
{
    int ret;

    /* Initialize I2C internal structs */
    i2c_dev = up_i2cinitialize(PWRM_I2C_BUS);
    if (!i2c_dev) {
        dbg_error("%s(): Failed to get I2C bus %u\n", __func__, PWRM_I2C_BUS);
        return -ENXIO;
    }

    pwrm_current_lsb = current_lsb_uA;
    if (ct >= ina230_ct_count) {
        dbg_error("%s(): invalid conversion time! (%u)\n", __func__, ct);
        up_i2cuninitialize(i2c_dev);
        return -EINVAL;
    }
    if (avg_count >= ina230_avg_count_max) {
        dbg_error("%s(): invalid average count! (%u)\n", __func__, avg_count);
        up_i2cuninitialize(i2c_dev);
        return -EINVAL;
    }
    pwrm_ct = ct;
    pwrm_avg_count = avg_count;

    /* Setup U135 GPIO expander */
    tca6424_dev = tca6424_init(i2c_dev, U135_IO_EXPANDER_ADDR);
    if (!tca6424_dev) {
        dbg_error("%s(): failed to get tca6424 device!\n", __func__);
        up_i2cuninitialize(i2c_dev);
        return -ENOMEM;
    }
    ret = tca6424_set_direction_out(tca6424_dev, I2C_INA230_SEL1_A);
    ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_A, 1);

    ret |= tca6424_set_direction_out(tca6424_dev, I2C_INA230_SEL1_B);
    ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_B, 1);

    ret |= tca6424_set_direction_out(tca6424_dev, I2C_INA230_SEL1_INH);
    ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL1_INH, 1);

    ret |= tca6424_set_direction_out(tca6424_dev, I2C_INA230_SEL2_A);
    ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL2_A, 1);

    ret |= tca6424_set_direction_out(tca6424_dev, I2C_INA230_SEL2_B);
    ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL2_B, 1);

    ret |= tca6424_set_direction_out(tca6424_dev, I2C_INA230_SEL2_INH);
    ret |= tca6424_set(tca6424_dev, I2C_INA230_SEL2_INH, 1);

    if (ret) {
        dbg_error("%s(): failed with error %s!\n", __func__, ret);
        up_i2cuninitialize(i2c_dev);
    } else {
        dbg_verbose("%s(): done.\n", __func__);
    }

    return ret;
}

void pwrm_deinit(void)
{
    tca6424_set(tca6424_dev, I2C_INA230_SEL1_A, 1);
    tca6424_set(tca6424_dev, I2C_INA230_SEL1_B, 1);
    tca6424_set(tca6424_dev, I2C_INA230_SEL1_INH, 1);
    tca6424_set(tca6424_dev, I2C_INA230_SEL2_A, 1);
    tca6424_set(tca6424_dev, I2C_INA230_SEL2_B, 1);
    tca6424_set(tca6424_dev, I2C_INA230_SEL2_INH, 1);
    tca6424_deinit(tca6424_dev);

    /* Release I2C resource */
    up_i2cuninitialize(i2c_dev);

    return;
}

pwrm_rail *pwrm_init_rail(uint8_t dev, uint8_t rail)
{
    pwrm_rail *pwrm_r = NULL;
    ina230_device *ina230_dev = NULL;
    uint8_t addr;
    int ret;

    if (dev > DEV_COUNT) {
        dbg_error("%s(): invalid dev! (%hhu)\n", __func__, dev);
        goto pwrm_init_device_end;
    }

    if (pwrm_dev_rail_count(dev) == -EINVAL) {
        dbg_error("%s(): invalid rail! (%hhu)\n", __func__, rail);
        goto pwrm_init_device_end;
    }

    dbg_verbose("%s(%u, %u): initializing %s device...\n",
                __func__, dev, rail, pwrm_rail_name(dev, rail));
    /* Configure I2C Mux */
    ret = pwrm_ina230_select(dev);
    if (ret) {
        dbg_error("%s(): failed to configure i2c mux! (%d)\n", __func__, ret);
        goto pwrm_init_device_end;
    }
    /* Retrieve device I2C address */
    addr = pwrm_i2c_addr_get(dev, rail);
    if (addr == INVALID_I2C_ADDR) {
        dbg_error("%s(): failed to Retrieve i2c addr!\n", __func__);
        goto pwrm_init_device_end;
    }
    /* Init device */
    ina230_dev = ina230_init(i2c_dev, addr,
                           INA230_SHUNT_VALUE, pwrm_current_lsb,
                           pwrm_ct,
                           pwrm_avg_count,
                           ina230_shunt_bus_cont);
    if (ina230_dev == NULL) {
        dbg_error("%s(): failed to init device!\n", __func__);
        goto pwrm_init_device_end;
    }
    /* Allocating memory for structure */
    pwrm_r = malloc(sizeof(pwrm_rail));
    if (pwrm_r == NULL) {
        dbg_error("%s(): failed to alloc memory!\n", __func__);
        ina230_deinit(ina230_dev);
        goto pwrm_init_device_end;
    }
    pwrm_r->ina230_dev = ina230_dev;
    pwrm_r->dev = dev;
    pwrm_r->rail = rail;
    pwrm_r->dev_name = pwrm_dev_name(dev);
    pwrm_r->rail_name = pwrm_rail_name(dev, rail);
    dbg_verbose("%s(): %s device init done.\n",
                __func__, pwrm_rail_name(dev, rail));

pwrm_init_device_end:
    return pwrm_r;
}

void pwrm_deinit_rail(pwrm_rail *pwrm_r)
{
    int ret;

    if (pwrm_r == NULL) {
        dbg_error("%s(): invalid pwrm_r!\n", __func__);
        return;
    }

    dbg_verbose("%s(): deinitializing %s device...\n",
                __func__, pwrm_r->rail_name);
    /* Configure I2C Mux */
    ret = pwrm_ina230_select(pwrm_r->dev);
    if (ret) {
        dbg_error("%s(): failed to configure i2c mux! (%d)\n",
                  __func__, ret);
        return;
    }
    /* Deinit device */
    ina230_deinit(pwrm_r->ina230_dev);
    /* Free memory */
    free(pwrm_r);
    dbg_verbose("%s(): device deinit done.\n", __func__);
}

int pwrm_measure_rail(pwrm_rail *pwrm_r, pwr_measure *m)
{
    int ret;

    if ((pwrm_r == NULL) || (m == NULL)) {
        dbg_error("%s(): NULL pointer!\n", __func__);
        return -EINVAL;
    }
    dbg_verbose("%s(): measuring %s rail...\n", __func__, pwrm_r->rail_name);

    /* Configure I2C Mux */
    ret = pwrm_ina230_select(pwrm_r->dev);
    if (ret) {
        dbg_error("%s(): failed to configure i2c mux! (%d)\n",
                  __func__, ret);
        return ret;
    }
    /* Get measurement data */
    ret = ina230_get_data(pwrm_r->ina230_dev, m);
    if (ret) {
        dbg_error("%s(): failed to retrieve measurement data! (%d)\n",
                  __func__, ret);
    } else {
        dbg_verbose("%s(): %s measurement: %duV %duA %duW\n", __func__,
                    pwrm_r->rail_name, m->uV, m->uA, m->uW);
    }

    return ret;
}
