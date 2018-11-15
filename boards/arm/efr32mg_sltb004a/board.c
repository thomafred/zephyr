/*
 * Copyright (c) 2018 Thomas Li Fredriksen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include "board.h"
#include <gpio.h>
#include <misc/printk.h>

static int efr32mg_sltb004a_mod_enable(const char *port, int pin)
{
    struct device *gpio;

    gpio = device_get_binding(port);
    if(!gpio)
    {
        return -ENODEV;
    }

    gpio_pin_write(gpio, pin, 1);

    return 0;
}

static int efr32mg_sltb004a_init(struct device *dev)
{
	ARG_UNUSED(dev);

#if defined(CONFIG_ICM20648)
    if(efr32mg_sltb004a_mod_enable(EFR32MG_SLTB004A_IMU_ENABLE_PORT,
                EFR32MG_SLTB004A_IMU_ENABLE_PIN) < 0)
    {
        return -ENODEV;
    }
#endif

#if defined(CONFIG_SI1133) || defined(CONFIG_BMP280) || defined(CONFIG_SI7021)
    if(efr32mg_sltb004a_mod_enable(EFR32MG_SLTB004A_ENV_SENSE_ENABLE_PORT,
                EFR32MG_SLTB004A_ENV_SENSE_ENABLE_PIN) < 0)
    {
        return -ENODEV;
    }
#endif

#if defined(CONFIG_CCS811)
    if(efr32mg_sltb004a_mod_enable(EFR32MG_SLTB004A_CCS811_ENABLE_PORT,
                EFR32MG_SLTB004A_CCS811_ENABLE_PIN) < 0)
    {
        return -ENODEV;
    }
#endif

#if defined(CONFIG_SI7210)
    if(efr32mg_sltb004a_mod_enable(EFR32MG_SLTB004A_HALL_ENABLE_PORT,
                EFR32MG_SLTB004A_HALL_ENABLE_PIN) < 0)
    {
        return -ENODEV;
    }
#endif

	return 0;
}

/* needs to be done after GPIO driver init */
SYS_INIT(efr32mg_sltb004a_init, PRE_KERNEL_1, CONFIG_BOARD_INIT_PRIORITY);
