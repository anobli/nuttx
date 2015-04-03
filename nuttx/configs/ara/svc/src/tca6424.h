/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted.
 *
 * @brief TCA6424 GPIO Expander Driver
 */

#ifndef _TSB_TCA6424_H_
#define _TSB_TCA6424_H_

#include <stdint.h>
#include <nuttx/i2c.h>

typedef struct
{
    struct i2c_dev_s *i2c_dev;  /* Nuttx I2C bus handler */
    uint8_t addr;               /* I2C device address */
} tca6424_device;


tca6424_device *tca6424_init(struct i2c_dev_s *i2c_dev, uint8_t addr);

int tca6424_set_direction_in(tca6424_device *dev, uint8_t which);
int tca6424_set_direction_out(tca6424_device *dev, uint8_t which);
int tca6424_get_direction(tca6424_device *dev, uint8_t which);
int tca6424_set_polarity_inverted(tca6424_device *dev,
                                  uint8_t which, uint8_t inverted);
int tca6424_get_polarity_inverted(tca6424_device *dev, uint8_t which);
int tca6424_set(tca6424_device *dev, uint8_t which, uint8_t val);
int tca6424_get(tca6424_device *dev, uint8_t which);

void tca6424_deinit(tca6424_device *dev);

#endif
