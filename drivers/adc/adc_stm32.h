/*
 * Copyright (c) 2018 Kokoon Technology Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef STM32_ADC_H_
#define STM32_ADC_H_

#include <zephyr/types.h>
#include <adc.h>
#include <kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

// set timeout to 1sec
#define ADC_STM32_ADC_TIMEOUT_US (USEC_PER_SEC * 1000)

//! type and enum for channel ids
typedef enum adc_channels {
	adc_channel_0 = ADC_CHANNEL_0,
	adc_channel_1 = ADC_CHANNEL_1,
	adc_channel_2 = ADC_CHANNEL_2,
	adc_channel_3 = ADC_CHANNEL_3,
	adc_channel_4 = ADC_CHANNEL_4,
	adc_channel_5 = ADC_CHANNEL_5,
	adc_channel_6 = ADC_CHANNEL_6,
	adc_channel_7 = ADC_CHANNEL_7,
	adc_channel_8 = ADC_CHANNEL_8,
	adc_channel_9 = ADC_CHANNEL_9,
	adc_channel_10 = ADC_CHANNEL_10,
	adc_channel_11 = ADC_CHANNEL_11,
	adc_channel_12 = ADC_CHANNEL_12,
	adc_channel_13 = ADC_CHANNEL_13,
	adc_channel_14 = ADC_CHANNEL_14,
	adc_channel_15 = ADC_CHANNEL_15,
	adc_channel_temp = ADC_CHANNEL_TEMPSENSOR, // 16
	adc_channel_vref = ADC_CHANNEL_VREFINT, // 17
#if defined(CONFIG_SOC_SERIES_STM32F4X)
	adc_channel_vbat = ADC_CHANNEL_VBAT, // 18
#endif
	adc_channel_max,
	adc_channel_unused,
} adc_channel_index_t;

enum stm32adc_errors {
	stm32adc_error_none = 0,
	stm32adc_error_hal_error = 10,
	stm32adc_error_adc_hal_init,
	stm32adc_error_adc_hal_config_channel,
	stm32adc_error_adc_hal_error,
	stm32adc_error_drv_error = 100,
	stm32adc_error_config_channel,
	stm32adc_error_adc_timeout,
	stm32adc_error_unknown_adc_unit,
};

//! definition of the config structure
typedef struct adc_config {
	uint32_t adc_dev_num; // number of the device
	uint32_t active_channels; // bit mask defining the channels
} adc_config_t;

//! defiuntion of the driver data
typedef struct adc_drv_data {
	// handle to adc defintion
	ADC_HandleTypeDef hadc;
} adc_drv_data_t;

//
// @brief ADC Initialization function.
//
// Inits device model for the ADC IP from Dataware.
//
// @param dev Pointer to the device structure descriptor that
// will be initialized.
//
// @return Integer: 0 for success, error otherwise.
//
static int adc_stm32_init(struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /*  STM32_ADC_H_ */
