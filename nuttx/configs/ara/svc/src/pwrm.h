/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted.
 *
 * @brief ARA BDB Power Measurement Library
 */

#ifndef __PWRM_H__
#define __PWRM_H__

#include <stdint.h>
#include <ina230.h>
#include <pwr_measure.h>

#define APB_DEV_COUNT               3
#define GPB_DEV_COUNT               2
#define DEV_MAX_RAIL_COUNT          8

#define DEV_NAME_MAX_LENGTH         8
#define RAIL_NAME_MAX_LENGTH        20

typedef enum {
    DEV_SW,
    DEV_APB1,
    DEV_APB2,
    DEV_APB3,
    DEV_GPB1,
    DEV_GPB2,
    DEV_COUNT,
} device;

typedef enum {
    VSW_1P1_PLL,
    VSW_1P1_CORE,
    VSW_1P8_UNIPRO,
    VSW_1P8_IO,
    VSW_COUNT
} sw_pwr_rail;

typedef enum {
    VAPB_1P1_CORE,
    VAPB_1P1_PLL1,
    VAPB_1P2_CDSI_PLL,
    VAPB_1P2_CDSI,
    VAPB_1P2_HSIC,
    VAPB_1P8_UNIPRO,
    VAPB_1P8_IO,
    VAPB_1P1_PLL2,
    VAPB_COUNT
} apb_pwr_rail;

typedef enum {
    VGPB_1P1_CORE,
    VGPB_1P1_PLL1,
    VGPB_SDIO,
    VGPB_1P2_HSIC,
    VGPB_1P8_UNIPRO,
    VGPB_1P8_IO,
    VGPB_1P1_PLL2,
    VGPB_COUNT
} gpb_pwr_rail;

typedef struct {
    ina230_device *ina230_dev;
    uint8_t dev;
    uint8_t rail;
    const char *dev_name;
    const char *rail_name;
} pwrm_rail;

int pwrm_init(uint32_t current_lsb_uA,
              ina230_conversion_time ct,
              ina230_avg_count avg_count);
pwrm_rail *pwrm_init_rail(uint8_t dev, uint8_t rail);
int pwrm_measure_rail(pwrm_rail *pwrm_dev, pwr_measure *m);
const char *pwrm_dev_name(uint8_t dev);
const char *pwrm_rail_name(uint8_t dev, uint8_t rail);
int pwrm_device_id(const char *name, uint8_t *dev);
int pwrm_rail_id(const char *name, uint8_t *dev, uint8_t *rail);
int pwrm_dev_rail_count(uint8_t dev);
void pwrm_deinit_rail(pwrm_rail *pwrm_dev);
void pwrm_deinit(void);

#endif
