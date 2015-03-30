/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted.
 *
 * @brief TI INA230 Driver
 */

#define DBG_COMP DBG_POWER

#include <ina230.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <up_debug.h>

#define INA230_CONFIG               0x00
#define INA230_SHUNT_VOLTAGE        0x01
#define INA230_BUS_VOLTAGE          0x02
#define INA230_POWER                0x03
#define INA230_CURRENT              0x04
#define INA230_CALIBRATION          0x05

/* CONFIG register bitfields */
#define INA230_CONFIG_POWER_MODE_MASK       ((uint16_t) 0x0007)
#define INA230_CONFIG_POWER_MODE_SHIFT      ((uint8_t) 0)
#define INA230_CONFIG_VSHUNT_CT_MASK        ((uint16_t) 0x0038)
#define INA230_CONFIG_VSHUNT_CT_SHIFT       ((uint8_t) 3)
#define INA230_CONFIG_VBUS_CT_MASK          ((uint16_t) 0x01C0)
#define INA230_CONFIG_VBUS_CT_SHIFT         ((uint8_t) 6)
#define INA230_CONFIG_AVG_MASK              ((uint16_t) 0x0E00)
#define INA230_CONFIG_AVG_SHIFT             ((uint8_t) 9)
#define INA230_CONFIG_RST_MASK              ((uint16_t) 0x8000)
#define INA230_CONFIG_RST_SHIFT             ((uint8_t) 15)

#define INA230_CALIBRATION_VALUE_MAX        ((uint16_t) 0x7FFF)
#define INA230_CALIBRATION_MULT             5120000 /* 0.00512 / uA / mohm */
#define INA230_VOLTAGE_LSB                  ((int32_t) 1250) /* 1.25mV */
#define INA230_POWER_CURRENT_RATIO          ((int32_t) 25)

static int ina230_i2c_get(struct i2c_dev_s *dev,
                          uint8_t addr, uint8_t reg, uint16_t *val)
{
    int ret;
    uint8_t buf[2];

    struct i2c_msg_s msg[] = {
        {
            .addr = addr,
            .flags = 0,
            .buffer = &reg,
            .length = 1,
        }, {
            .addr = addr,
            .flags = I2C_M_READ,
            .buffer = buf,
            .length = 2,
        },
    };

    ret = I2C_TRANSFER(dev, msg, 2);
    if (ret == 0) {
        *val = (uint16_t) buf[1] + (((uint16_t) buf[0]) << (uint16_t) 8);
        dbg_verbose("%s(): addr=0x%02X, reg=0x%02hhX: buf[0]=0x%02hhX buf[1]=0x%02hhX read 0x%04hX\n",
                    __func__, addr, reg, buf[0], buf[1], *val);
    } else {
        *val = 0;
        dbg_error("%s(): addr=0x%02hhX, reg=0x%02hhX: failed!\n",
                  __func__, addr, reg);
    }

    return ret;
}

static int ina230_i2c_set(struct i2c_dev_s *dev,
                          uint8_t addr, uint8_t reg, uint16_t val)
{
    int ret;
    uint8_t val_lsb = (uint8_t) (val & 0x00FF);
    uint8_t val_msb = (uint8_t) ((val & 0xFF00) >> 8);
    uint8_t cmd[3] = {reg, val_msb, val_lsb};
    struct i2c_msg_s msg[] = {
        {
            .addr = addr,
            .flags = 0,
            .buffer = cmd,
            .length = 3,
        },
    };

    dbg_verbose("%s(): addr=0x%02hhX: reg=0x%02hhX, val=0x%04hX\n",
                __func__, addr, reg, val);
    ret = I2C_TRANSFER(dev, msg, 1);

    return ret;
}

static int ina230_update_reg(struct i2c_dev_s *dev, uint8_t addr, uint8_t reg,
                             uint16_t mask, uint8_t shift, uint16_t val)
{
    uint16_t content;
    int ret;

    dbg_verbose("%s(): addr=0x%02hhX: reg=0x%02hhX, mask=0x%02hhX, shift=%02hhu, val=0x%04hX\n",
                __func__, addr, reg, mask, shift, val);
    if (addr >= 0x7F) {
        return -EINVAL;
    }

    /* Get register content */
    ret = ina230_i2c_get(dev, addr, reg, &content);
    if (ret) {
        return ret;
    }
    /* Update register content */
    dbg_verbose("%s(): content=0x%04hX\n", __func__, content);
    content &= ~mask;
    content |= (val << shift);
    dbg_verbose("%s(): new content=0x%04hX\n", __func__, content);
    /* Write back new register content */
    return ina230_i2c_set(dev, addr, reg, content);
}

static int ina230_reset(ina230_device *dev)
{
    dbg_verbose("%s(): addr=0x%02hhX\n", __func__, dev->addr);
    return ina230_update_reg(dev->i2c_dev, dev->addr,
                             INA230_CONFIG,
                             INA230_CONFIG_RST_MASK,
                             INA230_CONFIG_RST_SHIFT,
                             1);
}

static int ina230_set_mode(ina230_device *dev)
{
    dbg_verbose("%s(): addr=0x%02hhX, mode=%02hhu\n",
                __func__, dev->addr, dev->mode);
    return ina230_update_reg(dev->i2c_dev, dev->addr,
                             INA230_CONFIG,
                             INA230_CONFIG_POWER_MODE_MASK,
                             INA230_CONFIG_POWER_MODE_SHIFT,
                             dev->mode);
}

static int ina230_set_conversion_time(ina230_device *dev)
{
    int ret;

    dbg_verbose("%s(): addr=0x%02hhX, ct=%02hhu\n",
                __func__, dev->addr, dev->ct);
    ret = ina230_update_reg(dev->i2c_dev, dev->addr,
                             INA230_CONFIG,
                             INA230_CONFIG_VSHUNT_CT_MASK,
                             INA230_CONFIG_VSHUNT_CT_SHIFT,
                             dev->ct);
    ret |= ina230_update_reg(dev->i2c_dev, dev->addr,
                             INA230_CONFIG,
                             INA230_CONFIG_VBUS_CT_MASK,
                             INA230_CONFIG_VBUS_CT_SHIFT,
                             dev->ct);

    return ret;
}

static int ina230_set_avg_sample_count(ina230_device *dev)
{
    dbg_verbose("%s(): addr=0x%02hhX, count=%02hhu\n",
                __func__, dev->addr, dev->count);

    return ina230_update_reg(dev->i2c_dev, dev->addr,
                             INA230_CONFIG,
                             INA230_CONFIG_AVG_MASK,
                             INA230_CONFIG_AVG_SHIFT,
                             dev->count);
}

static int ina230_set_current_lsb(ina230_device *dev)
{
    uint16_t cal;

    dbg_verbose("%s(): addr=0x%02hhX, mohm=%u, current_lsb=%u\n",
                __func__, dev->addr, dev->mohm, dev->current_lsb);
    /* Convert to CALIBRATION value (equation 1 in datasheet) */
    cal = INA230_CALIBRATION_MULT / (dev->mohm * dev->current_lsb);
    dbg_verbose("%s(): cal=%u\n", __func__, cal);
    return ina230_i2c_set(dev->i2c_dev, dev->addr, INA230_CALIBRATION, cal);
}

ina230_device *ina230_init(struct i2c_dev_s *i2c_dev, uint8_t addr,
                           uint32_t mohm, uint32_t current_lsb,
                           ina230_conversion_time ct,
                           ina230_avg_count count,
                           ina230_power_mode mode)
{
    ina230_device *ina230_dev;
    int ret = 0;

    if ((i2c_dev == NULL)
        || (addr >= 0x7F)
        || (ct >= ina230_ct_count)
        || (count >= ina230_avg_count_max)
        || (mode >= ina230_power_mode_count)) {
        return NULL;
    }
    ina230_dev = (void *) malloc(sizeof(ina230_device));
    if (!ina230_dev) {
        dbg_error("%s(): failed to alloc device struct!\n", __func__);
        return NULL;
    }
    ina230_dev->i2c_dev = i2c_dev;
    ina230_dev->addr = addr;
    ina230_dev->mohm = mohm;
    ina230_dev->current_lsb = current_lsb;
    ina230_dev->mode = mode;
    ina230_dev->ct = ct;
    ina230_dev->count = count;

    ret |= ina230_reset(ina230_dev);
    ret |= ina230_set_conversion_time(ina230_dev);
    ret |= ina230_set_avg_sample_count(ina230_dev);
    ret |= ina230_set_current_lsb(ina230_dev);
    ret |= ina230_set_mode(ina230_dev);
    if (ret) {
        dbg_error("%s(): Failed to init device! (%d)\n", __func__, ret);
        free(ina230_dev);
        return NULL;
    }

    dbg_verbose("%s(): ina230 device successfully created.\n", __func__);
    return ina230_dev;
}

int ina230_get_data(ina230_device *dev, pwr_measure *m)
{
    int ret;
    int16_t raw_vbus, raw_current, raw_power;

    if ((dev == NULL) || (m == NULL)) {
        return -EINVAL;
    }
    m->uV = 0;
    m->uA = 0;
    m->uW = 0;

    ret = ina230_i2c_get(dev->i2c_dev, dev->addr,
                         INA230_BUS_VOLTAGE, (uint16_t *) &raw_vbus);
    ret |= ina230_i2c_get(dev->i2c_dev, dev->addr,
                          INA230_CURRENT, (uint16_t *) &raw_current);
    ret |= ina230_i2c_get(dev->i2c_dev, dev->addr,
                          INA230_POWER, (uint16_t *) &raw_power);
    if (ret) {
        dbg_error("%s(): failed to read data registers! (%d)\n", __func__, ret);
        return -EIO;
    }
    dbg_verbose("%s(): addr=0x%02X raw_vbus=0x%04X raw_current=0x%04X raw_power=0x%04X\n",
                __func__, dev->addr, raw_vbus, raw_current, raw_power);

    /* VBUS LSB = 1.25 mV. */
    m->uV = (int32_t) raw_vbus * INA230_VOLTAGE_LSB;

    /* Current register LSB programmed during init. Get it from structure. */
    m->uA = (int32_t) raw_current * dev->current_lsb;

    /*
     * The Power register LSB is internally programmed to equal 25 times
     * the programmed value of the Current_LSB.
     */
    m->uW = (int32_t) raw_power * INA230_POWER_CURRENT_RATIO * dev->current_lsb;

    dbg_verbose("%s(): addr=0x%02X ret=%d => %duV %duA %duW\n",
                __func__, dev->addr, ret, m->uV, m->uA, m->uW);
    return ret;
}

void ina230_deinit(ina230_device *dev)
{
    if (dev == NULL) {
        return;
    }

    /* Reset INA230 device */
    ina230_reset(dev);
    /* Put device in power down mode */
    dev->mode = ina230_power_down;
    ina230_set_mode(dev);

    free(dev);
}
