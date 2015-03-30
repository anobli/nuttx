/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted.
 *
 * @brief TCA6424 GPIO Expander Driver
 */

#define DBG_COMP DBG_POWER

#include <nuttx/config.h>
#include <errno.h>
#include <unistd.h>
#include <tca6424.h>
#include <stdio.h>
#include <up_debug.h>


#define TCA6424_INPUT0_REG          0x00
#define TCA6424_INPUT1_REG          0x01
#define TCA6424_INPUT2_REG          0x02
#define TCA6424_OUTPUT0_REG         0x04
#define TCA6424_OUTPUT1_REG         0x05
#define TCA6424_OUTPUT2_REG         0x06
#define TCA6424_POLARITY0_REG       0x08
#define TCA6424_POLARITY1_REG       0x09
#define TCA6424_POLARITY2_REG       0x0A
#define TCA6424_CONFIG0_REG         0x0C
#define TCA6424_CONFIG1_REG         0x0C
#define TCA6424_CONFIG2_REG         0x0C

#define TCA6424_MAX_ADDR_SHIFT      2

static uint8_t tca6424_get_addr_shift(uint8_t which)
{
    uint8_t shift = which / 8;

    if (shift > TCA6424_MAX_ADDR_SHIFT){
        dbg_error("%s(): out of range! (%hhu)\n", __func__);
    } else {
        dbg_verbose("%s(): which=%hhu shift=%hhu\n", __func__, which, shift);
    }

    return shift;
}

static int tca6424_i2c_get(struct i2c_dev_s *dev, uint8_t addr, uint8_t reg,
                           uint8_t *val)
{
    int ret;
    struct i2c_msg_s msg[] = {
        {
            .addr = addr,
            .flags = 0,
            .buffer = &reg,
            .length = 1,
        }, {
            .addr = addr,
            .flags = I2C_M_READ,
            .buffer = val,
            .length = 1,
        },
    };

    ret = I2C_TRANSFER(dev, msg, 2);
    if (ret == 0) {
        dbg_verbose("%s(): addr=0x%02hhX, reg=0x%02hhX: read 0x%02hhX\n",
                    __func__, addr, reg, *val);
    } else {
        dbg_error("%s(): addr=0x%02hhX, reg=0x%02hhX: failed!\n",
                  __func__, addr, reg);
    }

    return ret;
}

static int tca6424_i2c_set(struct i2c_dev_s *dev, uint8_t addr, uint8_t reg,
                           uint8_t val)
{
    int ret;
    uint8_t cmd[2] = {reg, val};
    struct i2c_msg_s msg[] = {
        {
            .addr = addr,
            .flags = 0,
            .buffer = cmd,
            .length = 2,
        },
    };

    dbg_verbose("%s(): addr=0x%02hhX: reg=0x%02hhX, val=0x%02hhX\n",
                __func__, addr, reg, val);
    ret = I2C_TRANSFER(dev, msg, 1);
    if (ret != 0) {
        dbg_error("%s(): failed to write register! (%d)\n", __func__, ret);
    }

    return ret;
}

tca6424_device *tca6424_init(struct i2c_dev_s *i2c_dev, uint8_t addr)
{
    tca6424_device *tca6424_dev;

    if ((i2c_dev == NULL) || (addr >= 0x7F)) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return NULL;
    }

    tca6424_dev = (void *) malloc(sizeof(tca6424_device));
    if (!tca6424_dev) {
        dbg_error("%s(): failed to alloc device struct!\n", __func__);
        return NULL;
    }

    tca6424_dev->i2c_dev = i2c_dev;
    tca6424_dev->addr = addr;

    dbg_verbose("%s(): tca6424 device successfully created.\n", __func__);
    return tca6424_dev;
}

void tca6424_deinit(tca6424_device *dev)
{
    if (dev == NULL) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return;
    }

    free(dev);
}

int tca6424_set_direction_in(tca6424_device *dev, uint8_t which)
{
    uint8_t reg;
    int ret;
    uint8_t shift;

    if (dev == NULL) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu\n", __func__, which);
    /* Configure pin as input
     *
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_CONFIG0_REG + shift, &reg);
    if (ret != 0)
        return -EIO;
    dbg_verbose("%s(): current cfg=0x%02X\n", __func__, reg);
    which = which - (8 * shift);
    reg |= (1 << which);
    dbg_verbose("%s(): new cfg=0x%02X\n", __func__, reg);
    ret = tca6424_i2c_set(dev->i2c_dev, dev->addr,
                          TCA6424_CONFIG0_REG + shift, reg);
    if (ret != 0) {
         return -EIO;
    }

    return 0;
}

int tca6424_set_direction_out(tca6424_device *dev, uint8_t which)
{
    uint8_t reg;
    int ret;
    uint8_t shift;

    if (dev == NULL) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu\n", __func__, which);
    /* Configure pin as output
     *
     * The Configuration Register configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_CONFIG0_REG + shift, &reg);
    if (ret != 0)
        return -EIO;
    dbg_verbose("%s(): current cfg=0x%02X\n", __func__, reg);
    which = which - (8 * shift);
    reg &= ~(1 << which);
    dbg_verbose("%s(): new cfg=0x%02X\n", __func__, reg);
    ret = tca6424_i2c_set(dev->i2c_dev, dev->addr,
                          TCA6424_CONFIG0_REG + shift, reg);
    if (ret != 0)
        return -EIO;

    return 0;
}


int tca6424_get_direction(tca6424_device *dev, uint8_t which)
{
    uint8_t direction;
    int ret;
    uint8_t shift;

    if (dev == NULL) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu", __func__, which);
    /*
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_CONFIG0_REG + shift, &direction);
    if (ret != 0)
        return -EIO;
    which = which - (8 * shift);
    direction = (direction & (1 << which)) >> which;
    return direction;
}


int tca6424_set_polarity_inverted(tca6424_device *dev,
                                  uint8_t which, uint8_t inverted)
{
    uint8_t polarity;
    int ret;
    uint8_t shift;

    if (dev == NULL) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu inverted=%hhu\n", __func__, which, inverted);
    /* Configure pin polarity inversion
     *
     * The Polarity Inversion Register (register 2) allows
     * polarity inversion of pins defined as inputs by the Configuration
     * Register. If a bit in this register is set (written with 1),
     * the corresponding port pin's polarity is inverted. If a bit in
     * this register is cleared (written with a 0), the corresponding
     * port pin's original polarity is retained.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_POLARITY0_REG + shift, &polarity);
    if (ret != 0)
        return -EIO;
    dbg_verbose("%s(): current polarity reg=0x%02hhX\n", __func__, polarity);
    which = which - (8 * shift);
    if (inverted) {
        polarity |= (1 << which);
    } else {
        polarity &= ~(1 << which);
    }
    dbg_verbose("%s(): new polarity reg=0x%02hhX\n", __func__, polarity);
    ret = tca6424_i2c_set(dev->i2c_dev, dev->addr,
                          TCA6424_POLARITY0_REG + shift, polarity);
    if (ret != 0)
        return -EIO;

    return 0;
}


int tca6424_get_polarity_inverted(tca6424_device *dev, uint8_t which)
{
    uint8_t polarity;
    int ret;
    uint8_t shift;

    if (dev == NULL) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu\n", __func__, which);
    /*
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_POLARITY0_REG + shift, &polarity);
    if (ret != 0)
        return -EIO;
    dbg_verbose("%s(): polarity reg=0x%02hhX\n", __func__, polarity);
    which = which - (8 * shift);
    polarity = (polarity & (1 << which)) >> which;
    dbg_verbose("%s(): polarity=0x%hhu\n", __func__, polarity);

    return polarity;
}


int tca6424_set(tca6424_device *dev, uint8_t which, uint8_t val)
{
    uint8_t reg;
    int ret;
    uint8_t shift;

    if (dev == NULL) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu, val=%hhu\n", __func__, which, val);
    /* Set output pins default value (before configuring it as output
     *
     * The Output Port Register (register 1) shows the outgoing logic
     * levels of the pins defined as outputs by the Configuration Register.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_OUTPUT0_REG + shift, &reg);
    if (ret != 0)
        return -EIO;
    dbg_verbose("%s(): current reg=0x%02hhX\n", __func__, reg);
    which = which - (8 * shift);
    if (val) {
        reg |= (1 << which);
    } else {
        reg &= ~(1 << which);
    }
    dbg_verbose("%s(): new reg=0x%02hhX\n", __func__, reg);
    ret = tca6424_i2c_set(dev->i2c_dev, dev->addr,
                          TCA6424_OUTPUT0_REG + shift, reg);
    if (ret != 0)
        return -EIO;

    return 0;
}


int tca6424_get(tca6424_device *dev, uint8_t which)
{
    uint8_t in;
    int ret;
    uint8_t shift;

    if (dev == NULL) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu\n", __func__, which);
    /*
     * The Input Port Register (register 0) reflects the incoming logic
     * levels of the pins, regardless of whether the pin is defined as an
     * input or an output by the Configuration Register. They act only on
     * read operation.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_INPUT0_REG + shift, &in);
    if (ret != 0)
        return -EIO;
    dbg_verbose("%s(): input reg=0x%02hhX\n", in);
    which = which - (8 * shift);
    in &= (1 << which);
    in = !!in;
    dbg_verbose("%s(): input=%hhu\n", __func__, in);

    return in;
}
