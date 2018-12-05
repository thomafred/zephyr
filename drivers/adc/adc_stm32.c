/*
 * Copyright (c) 2018 Kokoon Technology Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

// #include <board.h>
#include <adc.h>
#include <device.h>
#include <kernel.h>
#include <init.h>

#include "adc_context.h"

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adc_stm32);

#include <clock_control/stm32_clock_control.h>

#ifdef CONFIG_SOC_SERIES_STM32L0X
#include <stm32l0xx.h>
#include <stm32l0xx_hal.h>
#include <stm32l0xx_hal_adc.h>
#include <stm32l0xx_hal_adc_ex.h>
#include <stm32l0xx_hal_cortex.h>
#elif defined(CONFIG_SOC_SERIES_STM32F4X)
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_adc.h>
#include <stm32f4xx_hal_adc_ex.h>
#include <stm32f4xx_hal_cortex.h>
#endif

struct adc_stm32_data {

	struct adc_context ctx;
	u16_t *buffer;
	u32_t active_channels;

	const struct adc_sequence *entries;
	u8_t seq_size;
	u8_t resolution;
	u32_t channels;

	volatile ADC_TypeDef *Instance;
};

struct adc_stm32_config {
	volatile ADC_TypeDef *reg_base;
};

static int adc_stm32_adc_enable(struct device *dev)
{
	struct adc_stm32_data *data = dev->driver_data;
	volatile ADC_TypeDef *adc = data->Instance;

	int retries = ADC_ENABLE_TIMEOUT;

	if (ADC_ENABLING_CONDITIONS(data) == RESET) {
		return -EINVAL;
	}

	__HAL_ADC_ENABLE(data);

	do {
		k_sleep(1);
		
		if (__HAL_ADC_GET_FLAG(data, ADC_FLAG_RDY) == RESET) {
			break;
		}
	} while(--retries);

	return retries ? 0 : -EFAULT;
}

static void adc_stm32_start_conversion(struct device *dev)
{
	struct adc_stm32_data *data = dev->driver_data;
	const struct adc_sequence *entry = data->ctx.sequence;
	u32_t interval_us = 0;

	if (entry->options) {
		interval_us = entry->options->interval_us;
	}

	/* Setup sequence */

	if (ADC_IS_CONVERSION_ONGOING_REGULAR(data) == RESET) {
		
		data->Instance->CR |= ADC_CR_ADSTART;
	}

	return 0;
}

static int adc_stm32_read_request(struct device *dev, const struct adc_sequence *seq_tbl)
{
	struct adc_stm32_data *data = dev->driver_data;
	int err;

	/* Ensure resolution is valid */

	data->channels = seq_tbl->channels & data->active_channels;
	if (data->channels != seq_tbl->channels) {
		return -EINVAL;
	}

	switch (seq_tbl->resolution) {
	case 6:
	case 8:
	case 10:
	case 12:
		break;
	default:
		return -EINVAL;
	}

	data->entries = seq_tbl;
	data->buffer = (u16_t*) seq_tbl->buffer;

	if (seq_tbl->options) {
		data->seq_size = seq_tbl->options->extra_samplings + 1;
	} else {
		data->seq_size = 1;
	}

	/* Assert FIFO length */

	/* Check if buffer has enough size */

	adc_context_start_read(&data->ctx, seq_tbl);
	err = adc_context_wait_for_completion(&data->ctx);
	adc_context_release(&data->ctx, err);

	return err;
}

#if 0
ISR_DIRECT_DECLARE(ADC_IRQHandler)
{
	HAL_ADC_IRQHandler(&adc_control.act_drv->hadc);
	ISR_DIRECT_PM(); /* PM done after servicing interrupt for best latency */
	return 1; /* We should check if scheduling decision should be made */
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint16_t v;
	v = (uint16_t)HAL_ADC_GetValue(hadc);
	k_msgq_put(&adc_control.adc_vals, &v, K_NO_WAIT);
}
#endif

/********************************************
 *********** ADC context-functions **********
 ********************************************/

static void adc_context_start_sampling(struct adc_context *ctx)
{

}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat)
{

}

/********************************************
 ******************* API ********************
 ********************************************/

static int adc_stm32_read(struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_stm32_data *data = dev->driver_data;

	adc_context_lock(&data->ctx, false, NULL);

	return adc_stm32_read_request(dev, sequence);
}
#if 0
	struct adc_drv_data *drv_data = dev->driver_data;
	const struct adc_config *config = dev->config->config_info;

	adc_channel_index_t i;
	uint32_t active_channels;
	struct adc_seq_entry *pe;
	uint16_t val;
	int rc;

	rc = stm32adc_error_none;
	LOG_DBG("adc%u conversion", (unsigned int)config->adc_dev_num);
	k_sem_take(&adc_control.adc_read_lock, K_FOREVER);
	adc_control.act_drv = drv_data;
	// ok let's read for once the values
	HAL_ADC_Start_IT(&drv_data->hadc);
	
	pe = seq_tbl->entries;
	active_channels = config->active_channels;
	i = adc_channel_0;
	while (active_channels) {
		if (active_channels & 0x1) {
			if (!k_msgq_get(&adc_control.adc_vals, &val,
					ADC_STM32_ADC_TIMEOUT_US)) {
				*((uint16_t *)pe->buffer) = val;
				pe++;
			} else {
				rc = stm32adc_error_adc_timeout;
				*((uint16_t *)pe->buffer) = 0;
				goto error;
			}
		}
		active_channels >>= 1;
		i++;
	}

	HAL_ADC_Stop_IT(&drv_data->hadc);
	k_sem_give(&adc_control.adc_read_lock);
	return rc;
error:
	HAL_ADC_Stop_IT(&drv_data->hadc);
	k_msgq_purge(&adc_control.adc_vals);
	k_sem_give(&adc_control.adc_read_lock);
	return rc;
}
#endif

#ifdef CONFIG_ADC_ASYNC
static int adc_stm32_read_async(struct device *dev,
		const struct adc_sequence *sequence, struct k_poll_signal *async)
{
	struct adc_stm32_data *data = dev->driver_data;

	adc_context_lock(&data->ctx, true, async);

	return adc_stm32_read_request(dev, sequence);
}
#endif

int adc_stm32_cfg_channel(struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{

}
#if 0
	ADC_ChannelConfTypeDef cfg;
	GPIO_InitTypeDef gpio_cfg;
	GPIO_TypeDef *gpio_port;

	u8_t channel_id = channel_cfg->channel_id;

	LOG_DBG("Try to config adc channel %u", channel_id);
	
	// Unsure of what this should be set to
	// cfg.Rank = r;
	cfg.Offset = 0;

#if defined(CONFIG_SOC_SERIES_STM32L0X)
	cfg.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
#elif defined(CONFIG_SOC_SERIES_STM32F4X)
	cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;
#endif

	gpio_cfg.Mode = GPIO_MODE_ANALOG;
	gpio_cfg.Pull = GPIO_NOPULL;

	// set the pins and special function registers
	gpio_port = NULL;

	switch (channel_id) {
	case adc_channel_0:
		gpio_port = GPIOA;
		gpio_cfg.Pin = GPIO_PIN_0;
		cfg.Channel = ADC_CHANNEL_0;
		break;
	case adc_channel_1:
		gpio_port = GPIOA;
		gpio_cfg.Pin = GPIO_PIN_1;
		cfg.Channel = ADC_CHANNEL_1;
		break;
	case adc_channel_2:
		gpio_port = GPIOA;
		gpio_cfg.Pin = GPIO_PIN_2;
		cfg.Channel = ADC_CHANNEL_2;
		break;
	case adc_channel_3:
		gpio_port = GPIOA;
		gpio_cfg.Pin = GPIO_PIN_3;
		cfg.Channel = ADC_CHANNEL_3;
		break;
	case adc_channel_4:
		gpio_port = GPIOA;
		gpio_cfg.Pin = GPIO_PIN_4;
		cfg.Channel = ADC_CHANNEL_4;
		break;
	case adc_channel_5:
		gpio_port = GPIOA;
		gpio_cfg.Pin = GPIO_PIN_5;
		cfg.Channel = ADC_CHANNEL_5;
		break;
	case adc_channel_6:
		gpio_port = GPIOA;
		gpio_cfg.Pin = GPIO_PIN_6;
		cfg.Channel = ADC_CHANNEL_6;
		break;
	case adc_channel_7:
		gpio_port = GPIOA;
		gpio_cfg.Pin = GPIO_PIN_7;
		cfg.Channel = ADC_CHANNEL_7;
		break;
	case adc_channel_8:
		gpio_port = GPIOB;
		gpio_cfg.Pin = GPIO_PIN_0;
		cfg.Channel = ADC_CHANNEL_8;
		break;
	case adc_channel_9:
		gpio_port = GPIOB;
		gpio_cfg.Pin = GPIO_PIN_1;
		cfg.Channel = ADC_CHANNEL_9;
		break;
	case adc_channel_10:
		gpio_port = GPIOC;
		gpio_cfg.Pin = GPIO_PIN_0;
		cfg.Channel = ADC_CHANNEL_10;
		break;
	case adc_channel_11:
		gpio_port = GPIOC;
		gpio_cfg.Pin = GPIO_PIN_1;
		cfg.Channel = ADC_CHANNEL_11;
		break;
	case adc_channel_12:
		gpio_port = GPIOC;
		gpio_cfg.Pin = GPIO_PIN_2;
		cfg.Channel = ADC_CHANNEL_12;
		break;
	case adc_channel_13:
		gpio_port = GPIOC;
		gpio_cfg.Pin = GPIO_PIN_3;
		cfg.Channel = ADC_CHANNEL_13;
		break;
	case adc_channel_14:
		gpio_port = GPIOC;
		gpio_cfg.Pin = GPIO_PIN_4;
		cfg.Channel = ADC_CHANNEL_14;
		break;
	case adc_channel_15:
		gpio_port = GPIOC;
		gpio_cfg.Pin = GPIO_PIN_5;
		cfg.Channel = ADC_CHANNEL_15;
		break;

	case adc_channel_vref:
		cfg.Channel = ADC_CHANNEL_VREFINT;
#if defined(CONFIG_SOC_SERIES_STM32F4X)
		ADC->CCR |= ADC_CCR_TSVREFE;
#elif defined(CONFIG_SOC_SERIES_STM32L0X)
		ADC->CCR |= ADC_CCR_VREFEN;
#endif
		break;

	case adc_channel_temp:
		cfg.Channel = ADC_CHANNEL_TEMPSENSOR;
		break;
#if defined (CONFIG_SERIES_STM32F4X)
	case adc_channel_vbat:
		cfg.Channel = ADC_CHANNEL_VBAT;
		ADC->CCR |= ADC_CCR_VBATE;
		break;
#endif
	default:
		// none
		break;
	}
	if (gpio_port)
		HAL_GPIO_Init(gpio_port, &gpio_cfg);

	if (HAL_ADC_ConfigChannel(phadc, &cfg) != HAL_OK) {
		LOG_ERR("config adc channel failed");
		return stm32adc_error_adc_hal_config_channel;
	}
	LOG_DBG("internal adc channel %lu has been configured", cfg.Channel);

	return stm32adc_error_none;
}
#endif

/********************************************
 ****************** INIT ********************
 ********************************************/

static int adc_stm32_init(struct device *dev)
{
#if 0
	const struct adc_config *config = dev->config->config_info;
	struct adc_drv_data *drv_data = dev->driver_data;
	uint32_t active_channels;

	LOG_INF("init adc%u", (unsigned int)config->adc_dev_num);

	// init adc control structure
	if (!adc_control.init_flag) {
		k_sem_init(&adc_control.adc_read_lock, 0, 1);
		k_sem_give(&adc_control.adc_read_lock);
		k_msgq_init(&adc_control.adc_vals, (char *)adc_control.adc_val_buff,
			    sizeof(uint16_t),
			    sizeof(adc_control.adc_val_buff) / sizeof(uint16_t));
		adc_control.act_drv = NULL;
		adc_control.isr_connected_flag = 0;
		adc_control.init_flag = 1;
	}

#ifdef CONFIG_SOC_SERIES_STM32F4X 
	if (!adc_control.isr_connected_flag) {
		IRQ_DIRECT_CONNECT(ADC_IRQn, 0, ADC_IRQHandler, 0);
		irq_enable(ADC_IRQn);
		adc_control.isr_connected_flag = 1;
	}
#endif

	switch (config->adc_dev_num) {
#ifdef CONFIG_ADC_0
	case 0:
		__HAL_RCC_ADC1_CLK_ENABLE();
		drv_data->hadc.Instance = ADC1;
		break;
#endif

#ifdef CONFIG_ADC_1
	case 1:
		__HAL_RCC_ADC2_CLK_ENABLE();
		drv_data->hadc.Instance = ADC2;
		break;
#endif
#ifdef CONFIG_ADC_2
	case 2:
		__HAL_RCC_ADC3_CLK_ENABLE();
		drv_data->hadc.Instance = ADC3;
		break;
#endif
	default:
		LOG_ERR("unknown ADC unit");
		return stm32adc_error_unknown_adc_unit;
	}

	drv_data->hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	drv_data->hadc.Init.Resolution = ADC_RESOLUTION_12B;
	drv_data->hadc.Init.ScanConvMode = ENABLE;
	drv_data->hadc.Init.ContinuousConvMode = DISABLE;
	drv_data->hadc.Init.DiscontinuousConvMode = DISABLE;
	drv_data->hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	drv_data->hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	drv_data->hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	drv_data->hadc.Init.DMAContinuousRequests = DISABLE;
	drv_data->hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	drv_data->hadc.Init.NbrOfConversion = 0;
	active_channels = config->active_channels;

	while (active_channels) {
		if (active_channels & 1)
			drv_data->hadc.Init.NbrOfConversion++;
		active_channels >>= 1;
	}
	LOG_INF("use %u multiplexed channels",
		    (unsigned int)drv_data->hadc.Init.NbrOfConversion);

	// start the adc
	if (HAL_ADC_Init(&drv_data->hadc) != HAL_OK) {
		LOG_ERR("HAL ADC init failed");
		return stm32adc_error_adc_hal_init;
	}

	// // setup the sequencer for the channels
	// i = 0;
	// r = 1;
	// active_channels = config->active_channels;
	// while (active_channels) {
	// 	if (active_channels & 1) {
	// 		if (adc_stm32_cfg_channel(&drv_data->hadc, (uint32_t)i,
	// 					 r)) {
	// 			LOG_ERR("activate adc channel %u failed",
	// 				    i);
	// 			return stm32adc_error_config_channel;
	// 		} else {
	// 			LOG_INF("activate adc channel %u", i);
	// 		}
	// 		r++;
	// 	}
	// 	active_channels >>= 1;
	// 	i++;
	// }

	return stm32adc_error_none;

#endif

	return 0;
}

static struct adc_driver_api api_stm32_driver_api = {
	.channel_setup = adc_stm32_cfg_channel,
	.read = adc_stm32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_stm32_read_async,
#endif
};

#ifdef CONFIG_ADC_0

struct adc_drv_data adc_drv_data_dev0 = {};

static struct adc_config adc_config_dev0 = {
	.adc_dev_num = 0,
	.active_channels = 0
#ifdef CONFIG_ADC0_CHAN0
			  + BIT(adc_channel_0)
#endif
#ifdef CONFIG_ADC0_CHAN1
			  + BIT(adc_channel_1)
#endif
#ifdef CONFIG_ADC0_CHAN2
			  + BIT(adc_channel_2)
#endif
#ifdef CONFIG_ADC0_CHAN3
			  + BIT(adc_channel_3)
#endif
#ifdef CONFIG_ADC0_CHAN4
			  + BIT(adc_channel_4)
#endif
#ifdef CONFIG_ADC0_CHAN5
			  + BIT(adc_channel_5)
#endif
#ifdef CONFIG_ADC0_CHAN6
			  + BIT(adc_channel_6)
#endif
#ifdef CONFIG_ADC0_CHAN7
			  + BIT(adc_channel_7)
#endif
#ifdef CONFIG_ADC0_CHAN8
			  + BIT(adc_channel_8)
#endif
#ifdef CONFIG_ADC0_CHAN9
			  + BIT(adc_channel_9)
#endif
#ifdef CONFIG_ADC0_CHAN10
			  + BIT(adc_channel_10)
#endif
#ifdef CONFIG_ADC0_CHAN11
			  + BIT(adc_channel_11)
#endif
#ifdef CONFIG_ADC0_CHAN12
			  + BIT(adc_channel_12)
#endif
#ifdef CONFIG_ADC0_CHAN13
			  + BIT(adc_channel_13)
#endif
#ifdef CONFIG_ADC0_CHAN14
			  + BIT(adc_channel_14)
#endif
#ifdef CONFIG_ADC0_CHAN15
			  + BIT(adc_channel_15)
#endif
#ifdef CONFIG_ADC0_CHAN_TEMP
			  + BIT(adc_channel_temp)
#endif
#ifdef CONFIG_ADC0_CHAN_VREFINT
			  + BIT(adc_channel_vref)
#endif
#ifdef CONFIG_ADC0_CHAN_VBAT && defined(CONFIG_SERIES_STM32F4X)
			  + BIT(adc_channel_vbat)
#endif
		,
};

DEVICE_AND_API_INIT(adc_stm32, CONFIG_ADC_0_NAME, &adc_stm32_init,
		    &adc_drv_data_dev0, &adc_config_dev0, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api_stm32_driver_api);
#endif // CONFIG_ADC_0

#ifdef CONFIG_ADC_1

struct adc_drv_data adc_drv_data_dev1 = {};

static struct adc_config adc_config_dev1 = {
	.adc_dev_num = 0,
	.active_channels = 0
#ifdef CONFIG_ADC1_CHAN0
			  + BIT(adc_channel_0)
#endif
#ifdef CONFIG_ADC1_CHAN1
			  + BIT(adc_channel_1)
#endif
#ifdef CONFIG_ADC1_CHAN2
			  + BIT(adc_channel_2)
#endif
#ifdef CONFIG_ADC1_CHAN3
			  + BIT(adc_channel_3)
#endif
#ifdef CONFIG_ADC1_CHAN4
			  + BIT(adc_channel_4)
#endif
#ifdef CONFIG_ADC1_CHAN5
			  + BIT(adc_channel_5)
#endif
#ifdef CONFIG_ADC1_CHAN6
			  + BIT(adc_channel_6)
#endif
#ifdef CONFIG_ADC1_CHAN7
			  + BIT(adc_channel_7)
#endif
#ifdef CONFIG_ADC1_CHAN8
			  + BIT(adc_channel_8)
#endif
#ifdef CONFIG_ADC1_CHAN9
			  + BIT(adc_channel_9)
#endif
#ifdef CONFIG_ADC1_CHAN10
			  + BIT(adc_channel_10)
#endif
#ifdef CONFIG_ADC1_CHAN11
			  + BIT(adc_channel_11)
#endif
#ifdef CONFIG_ADC1_CHAN12
			  + BIT(adc_channel_12)
#endif
#ifdef CONFIG_ADC1_CHAN13
			  + BIT(adc_channel_13)
#endif
#ifdef CONFIG_ADC1_CHAN14
			  + BIT(adc_channel_14)
#endif
#ifdef CONFIG_ADC1_CHAN15
			  + BIT(adc_channel_15)
#endif
#ifdef CONFIG_ADC1_CHAN_TEMP
			  + BIT(adc_channel_temp)
#endif
#ifdef CONFIG_ADC1_CHAN_VREFINT
			  + BIT(adc_channel_vref)
#endif
#if defined(CONFIG_ADC1_CHAN_VBAT) && defined(CONFIG_SERIES_STM32F4X)
			  + BIT(adc_channel_vbat)
#endif
		,
};

DEVICE_AND_API_INIT(adc_stm32, CONFIG_ADC_1_NAME, &adc_stm32_init,
		    &adc_drv_data_dev1, &adc_config_dev1, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api_stm32_driver_api);
#endif // CONFIG_ADC_1

#ifdef CONFIG_ADC_2

struct adc_drv_data adc_drv_data_dev2 = {};

static struct adc_config adc_config_dev2 = {
	.adc_dev_num = 0,
	.active_channels = 0
#ifdef CONFIG_ADC2_CHAN0
			  + BIT(adc_channel_0)
#endif
#ifdef CONFIG_ADC2_CHAN1
			  + BIT(adc_channel_1)
#endif
#ifdef CONFIG_ADC2_CHAN2
			  + BIT(adc_channel_2)
#endif
#ifdef CONFIG_ADC2_CHAN3
			  + BIT(adc_channel_3)
#endif
#ifdef CONFIG_ADC2_CHAN10
			  + BIT(adc_channel_10)
#endif
#ifdef CONFIG_ADC2_CHAN11
			  + BIT(adc_channel_11)
#endif
#ifdef CONFIG_ADC2_CHAN12
			  + BIT(adc_channel_12)
#endif
#ifdef CONFIG_ADC2_CHAN13
			  + BIT(adc_channel_13)
#endif
#ifdef CONFIG_ADC2_CHAN_TEMP
			  + BIT(adc_channel_temp)
#endif
#ifdef CONFIG_ADC2_CHAN_VREFINT
			  + BIT(adc_channel_vref)
#endif
#ifdef CONFIG_ADC2_CHAN_VABT
			  + BIT(adc_channel_vbat)
#endif
		,
};

DEVICE_AND_API_INIT(adc_stm32, CONFIG_ADC_2_NAME, &adc_stm32_init,
		    &adc_drv_data_dev2, &adc_config_dev2, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api_stm32_driver_api);
#endif // CONFIG_ADC_2
