/* ieee802154_efr32.c - Silabs EFR32 driver */

/*
 * Copyright (c) 2018 Evry ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME ieee802154_efr32
#define LOG_LEVEL CONFIG_IEEE802154_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <assert.h>
#include <init.h>
#include <irq.h>
#include <net/ieee802154_radio.h>
#include <net/net_if.h>
#include <net/net_pkt.h>
#include <misc/byteorder.h>
#include <random/rand32.h>

#include "rail_types.h"
#include "em_core.h"
#include "em_system.h"

#include "rail.h"
#include "rail_ieee802154.h"
#include "pa_conversions_efr32.h"

#if defined(CONFIG_NET_L2_OPENTHREAD)
#include <net/openthread.h>
#endif

#define RADIO_CONFIG_XTAL_FREQUENCY 38400000UL

RAIL_DECLARE_TX_POWER_VBAT_CURVES(piecewiseSegments, curvesSg, curves24Hp, curves24Lp);

#define ACK_TIMEOUT K_MSEC(10)

enum EfrState
{
    EFR32_STATE_RX,
    EFR32_STATE_TX,
    EFR32_STATE_DISABLED,
    EFR32_STATE_CCA,
    EFR32_STATE_SLEEP,
};

static enum EfrState efr_state;

enum
{
    IEEE802154_MAX_LENGTH = 256,
    EFR32_FCS_LENGTH = 2,
};

struct efr32_context
{
    struct net_if *iface;
    u8_t mac_addr[8];

    u8_t rx_buf[IEEE802154_MAX_LENGTH];
    u8_t tx_buf[IEEE802154_MAX_LENGTH];

	u16_t rx_buf_size;

	int err;

    K_THREAD_STACK_MEMBER(rx_stack,
                          CONFIG_IEEE802154_EFR32_RX_STACK_SIZE);
    struct k_thread rx_thread;


    /* CCA complete sempahore. Unlocked when CCA is complete. */
    struct k_sem cca_wait;
    /* RX synchronization semaphore. Unlocked when frame has been
	 * received.
	 */
    struct k_sem rx_wait;
    /* TX synchronization semaphore. Unlocked when frame has been
	 * sent or CCA failed.
	 */
    struct k_sem tx_wait;

    u8_t channel;

    RAIL_Handle_t rail_handle;
};

static struct efr32_context efr32_data;

static const RAIL_CsmaConfig_t rail_csma_config = RAIL_CSMA_CONFIG_802_15_4_2003_2p4_GHz_OQPSK_CSMA;

const RAIL_IEEE802154_Config_t rail_ieee802154_config = {
    .addresses = NULL,
    .ackConfig = {
        .enable = true,
        .ackTimeout = 894,
        .rxTransitions = {
            RAIL_RF_STATE_RX,
            RAIL_RF_STATE_RX,
        },
        .txTransitions = {
            RAIL_RF_STATE_RX,
            RAIL_RF_STATE_RX,
        }},
    .timings = {
        .idleToRx = 100,         
        .idleToRx = 192 - 10,    
        .idleToTx = 100,         
        .rxToTx = 192,           
        .rxSearchTimeout = 0,    
        .txToRxSearchTimeout = 0,
    },
    .framesMask = RAIL_IEEE802154_ACCEPT_STANDARD_FRAMES, 
    .promiscuousMode = false,                             
    .isPanCoordinator = false,                            
};

CORE_irqState_t CORE_EnterCritical(void)
{
    return irq_lock();
}

void CORE_ExitCritical(CORE_irqState_t irqState)
{
    (void)irqState;

    irq_unlock(irqState);
}

CORE_irqState_t CORE_EnterAtomic(void)
{
    return irq_lock();
}

void CORE_ExitAtomic(CORE_irqState_t irqState)
{
    (void)irqState;

    irq_unlock(irqState);
}

static void efr32_rail_cb(RAIL_Handle_t rail_handle, RAIL_Events_t a_events);

static RAIL_Config_t s_rail_config = {
    .eventsCallback = &efr32_rail_cb,
    .protocol = NULL,
    .scheduler = NULL,
};

static inline u8_t *get_mac(struct device *dev)
{
    uint64_t uniqueID = SYSTEM_GetUnique();
    uint8_t *mac = (uint8_t *)&uniqueID;
    return mac;
}

static void efr32_iface_init(struct net_if *iface)
{
    struct device *dev = net_if_get_device(iface);
    struct efr32_context *efr32 = dev->driver_data;
    u8_t *mac = get_mac(dev);

    net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);
    efr32->iface = iface;
    ieee802154_init(iface);
}

static enum ieee802154_hw_caps efr32_get_capabilities(struct device *dev)
{
    return IEEE802154_HW_FCS |
           IEEE802154_HW_2_4_GHZ |
           IEEE802154_HW_FILTER |
           IEEE802154_HW_TX_RX_ACK |
           IEEE802154_HW_CSMA;
}

/* RAIL handles this in tx() */
static int efr32_cca(struct device *dev)
{
    return 0;
}

static int efr32_set_channel(struct device *dev, u16_t channel)
{
    struct efr32_context *efr32 = dev->driver_data;

    RAIL_Status_t status;
    status = RAIL_PrepareChannel(efr32->rail_handle, channel);

    if (status != RAIL_STATUS_NO_ERROR)
    {
        return -EIO;
    }

    efr32->channel = channel;

    return 0;
}

static int efr32_set_pan_id(struct device *dev, u16_t pan_id)
{
    struct efr32_context *efr32 = dev->driver_data;

    RAIL_Status_t status;
    status = RAIL_IEEE802154_SetPanId(efr32->rail_handle, pan_id, 0);

    if (status != RAIL_STATUS_NO_ERROR)
    {
        return -EIO;
    }

    return 0;
}

static int efr32_set_short_addr(struct device *dev, u16_t short_addr)
{
    struct efr32_context *efr32 = dev->driver_data;

    RAIL_Status_t status;
    status = RAIL_IEEE802154_SetShortAddress(efr32->rail_handle, short_addr, 0);

    if (status != RAIL_STATUS_NO_ERROR)
    {
        return -EIO;
    }

    return 0;
}

static int efr32_set_ieee_addr(struct device *dev, const u8_t *ieee_addr)
{
    struct efr32_context *efr32 = dev->driver_data;

    RAIL_Status_t status;
    status = RAIL_IEEE802154_SetLongAddress(efr32->rail_handle, ieee_addr, 0);

    if (status != RAIL_STATUS_NO_ERROR)
    {
        LOG_ERR("Error setting address via RAIL");
        return -EIO;
    }

    return 0;
}

static int efr32_filter(struct device *dev,
                        bool set,
                        enum ieee802154_filter_type type,
                        const struct ieee802154_filter *filter)
{
    LOG_DBG("Applying filter %u", type);

    if (!set)
    {
        return -ENOTSUP;
    }

    if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR)
    {
        return efr32_set_ieee_addr(dev, filter->ieee_addr);
    }
    else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR)
    {
        return efr32_set_short_addr(dev, filter->short_addr);
    }
    else if (type == IEEE802154_FILTER_TYPE_PAN_ID)
    {
        return efr32_set_pan_id(dev, filter->pan_id);
    }

    return -ENOTSUP;
}

static int efr32_set_txpower(struct device *dev, s16_t dbm)
{
    struct efr32_context *efr32 = dev->driver_data;

    RAIL_Status_t status;
    status = RAIL_SetTxPower(efr32->rail_handle, (RAIL_TxPower_t)dbm * 10);

    if (status != RAIL_STATUS_NO_ERROR)
    {
        return -EIO;
    }

    return 0;
}

static int efr32_start(struct device *dev)
{
    struct efr32_context *efr32 = dev->driver_data;
    ARG_UNUSED(efr32);

    efr_state = EFR32_STATE_SLEEP;

    LOG_DBG("EFR32 802154 radio started");

    return 0;
}

static int efr32_stop(struct device *dev)
{
    struct efr32_context *efr32 = dev->driver_data;
    ARG_UNUSED(efr32);

    efr_state = EFR32_STATE_DISABLED;

    LOG_DBG("EFR32 802154 radio stopped");

    return 0;
}

static int efr32_tx(struct device *dev, struct net_pkt *pkt,
                    struct net_buf *frag)
{
    struct efr32_context *efr32 = dev->driver_data;

    u8_t payload_len = net_pkt_ll_reserve(pkt) + frag->len;
    u8_t *payload = frag->data - net_pkt_ll_reserve(pkt);
    u16_t written;

    RAIL_TxOptions_t tx_opts = RAIL_TX_OPTIONS_NONE;
    RAIL_Status_t status;

    LOG_DBG("TX %p (%u)", payload, payload_len);

    memcpy(efr32->tx_buf + 1, payload, payload_len);
    efr32->tx_buf[0] = payload_len + EFR32_FCS_LENGTH;

    /* Reset semaphore in case ACK was received after timeout */
    k_sem_reset(&efr32->tx_wait);

    RAIL_Idle(efr32->rail_handle, RAIL_IDLE_ABORT, true);

    written = RAIL_WriteTxFifo(efr32->rail_handle, efr32->tx_buf, payload_len, true);
    status = RAIL_StartCcaCsmaTx(efr32->rail_handle, efr32->channel,
			tx_opts, &rail_csma_config, NULL);
	//status = RAIL_StartTx(efr32->rail_handle, efr32->channel, tx_opts, NULL);

    if (status != RAIL_STATUS_NO_ERROR)
    {
        LOG_ERR("Cannot start tx, error %i", status);
        return -EIO;
    }

    LOG_DBG("Sending frame: channel=%d, written=%u", efr32->channel, written);

    if (k_sem_take(&efr32->tx_wait, ACK_TIMEOUT))
    {
        LOG_DBG("ACK not received");
        return -EIO;
    }

    LOG_DBG("Result: %d", efr32->err);

    return efr32->err;
}

/* 
    Continously runs and tries to fetch packets..
*/
static void efr32_rx(int arg)
{
	RAIL_RxPacketHandle_t packet_handle;
	RAIL_RxPacketInfo_t packet_info;
	RAIL_RxPacketDetails_t packet_details;
	RAIL_Status_t status;
	u16_t len;

    struct device *dev = INT_TO_POINTER(arg);
    struct efr32_context *efr32 = dev->driver_data;

    while (1)
    {
        LOG_DBG("Waiting for frame");
        k_sem_take(&efr32->rx_wait, K_FOREVER);

		if (efr32->err) {
			LOG_ERR("RX error: %d", efr32->err);

			continue;
		}

		packet_handle = RAIL_GetRxPacketInfo(efr32->rail_handle,
				RAIL_RX_PACKET_HANDLE_OLDEST, &packet_info);

		packet_details.timeReceived.timePosition = RAIL_PACKET_TIME_INVALID;
		packet_details.timeReceived.totalPacketBytes = 0;

		status = RAIL_GetRxPacketDetails(efr32->rail_handle, packet_handle, &packet_details);
		if (status != RAIL_STATUS_NO_ERROR) {
			LOG_ERR("Failed to get packet details (err = %d)", status);
			continue;
		}

		len = packet_info.packetBytes + 1;

		LOG_DBG("Received %u bytes", len);


		if(packet_details.isAck)
		{
			k_sem_give(&efr32->tx_wait);
		}


		if(packet_handle != RAIL_RX_PACKET_HANDLE_INVALID)
		{
			RAIL_ReleaseRxPacket(efr32->rail_handle, packet_handle);
		}
    }
}

static int efr32_init(struct device *dev)
{
    struct efr32_context *efr32 = dev->driver_data;

    RAIL_Status_t status;

    static const RAIL_DataConfig_t rail_data_config = {
        TX_PACKET_DATA,
        RX_PACKET_DATA,
        PACKET_MODE,
        PACKET_MODE,
    };

    RAIL_TxPowerCurvesConfig_t txPowerCurvesConfig = {curves24Hp, curvesSg, curves24Lp, piecewiseSegments};

    RAIL_TxPowerConfig_t txPowerConfig = {RAIL_TX_POWER_MODE_2P4_HP, 1800, 10};

    efr32->rail_handle = RAIL_Init(&s_rail_config, NULL);
	efr32->rx_buf_size = sizeof(efr32->rx_buf);
	efr32->err = 0;

	/* Initialize interrupts */

	IRQ_DIRECT_CONNECT(1, 2, FRC_PRI_IRQHandler, 0);
	IRQ_DIRECT_CONNECT(4, 2, FRC_IRQHandler, 0);
	IRQ_DIRECT_CONNECT(8, 2, BUFC_IRQHandler, 0);
	IRQ_DIRECT_CONNECT(29, 2, PROTIMER_IRQHandler, 0);

    k_sem_init(&efr32->tx_wait, 0, 1);
    k_sem_init(&efr32->rx_wait, 0, 1);

    if (efr32->rail_handle == NULL)
    {
        LOG_ERR("Unable to init");
        return -EIO;
    }

    status = RAIL_ConfigData(efr32->rail_handle, &rail_data_config);
    if (status != RAIL_STATUS_NO_ERROR)
    {
        LOG_ERR("Error with config data.");
        return -EIO;
    }

    status = RAIL_ConfigCal(efr32->rail_handle, RAIL_CAL_ALL);
    if (status != RAIL_STATUS_NO_ERROR)
    {
        LOG_ERR("Error with config cal.");
        return -EIO;
    }

    status = RAIL_IEEE802154_Config2p4GHzRadio(efr32->rail_handle);
    if (status != RAIL_STATUS_NO_ERROR)
    {
        return -EIO;
    }

    status = RAIL_IEEE802154_Init(efr32->rail_handle, &rail_ieee802154_config);
    if (status != RAIL_STATUS_NO_ERROR)
    {
        return -EIO;
    }

    RAIL_SetTxFifo(efr32->rail_handle, efr32->tx_buf, 0, sizeof(efr32->tx_buf));
	RAIL_SetRxFifo(efr32->rail_handle, efr32->rx_buf, &efr32->rx_buf_size);

	RAIL_SetTxFifoThreshold(efr32->rail_handle, (size_t) (0.9 * sizeof(efr32->tx_buf)));
	RAIL_SetRxFifoThreshold(efr32->rail_handle, (size_t) (0.9 * efr32->rx_buf_size));

    status = RAIL_ConfigEvents(efr32->rail_handle, RAIL_EVENTS_ALL,
		RAIL_EVENTS_RX_COMPLETION
		| RAIL_EVENTS_TX_COMPLETION
		| RAIL_EVENTS_TXACK_COMPLETION
		| RAIL_EVENT_RX_ACK_TIMEOUT
		| RAIL_EVENT_TX_FIFO_ALMOST_EMPTY
		| RAIL_EVENT_RX_FIFO_ALMOST_FULL
		| RAIL_EVENT_IEEE802154_DATA_REQUEST_COMMAND
		| RAIL_EVENT_CAL_NEEDED
	);
    if (status != RAIL_STATUS_NO_ERROR)
    {
        return -EIO;
    }

    status = RAIL_InitTxPowerCurves(&txPowerCurvesConfig);

    if (status != RAIL_STATUS_NO_ERROR)
    {
        return -EIO;
    }

    status = RAIL_ConfigTxPower(efr32->rail_handle, &txPowerConfig);

    efr32_set_txpower(dev, 0);

	k_thread_create(&efr32->rx_thread, efr32->rx_stack,
			CONFIG_IEEE802154_EFR32_RX_STACK_SIZE,
			(k_thread_entry_t)efr32_rx,
			dev, NULL, NULL, K_PRIO_COOP(2), 0, 0);

    LOG_DBG("Init done!");
    return 0;
}

static void efr32_rail_cb(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
	struct efr32_context *efr32 = &efr32_data; // FIXME: Not valid

    LOG_DBG("Processing events 0x%llX", events);

	if (events & RAIL_EVENTS_RX_COMPLETION) {
		if (events & RAIL_EVENT_RX_PACKET_RECEIVED) {

			LOG_DBG("Received packet frame");

			RAIL_HoldRxPacket(rail_handle);
			efr32->err = 0;
		}
		if (events & RAIL_EVENT_RX_PACKET_ABORTED) {

			LOG_DBG("RX packet aborted");
			efr32->err = -ECONNABORTED;
		}
		if (events & RAIL_EVENT_RX_FRAME_ERROR) {

			LOG_DBG("RX frame error");
			efr32->err = -EIO;
		}
		if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {

			LOG_DBG("RX FIFO overflow");
			efr32->err = -EIO;
		}
		if (events & RAIL_EVENT_RX_ADDRESS_FILTERED) {
			LOG_DBG("RX Address filtered");

			efr32->err = -EFAULT;
		}

		k_sem_give(&efr32->rx_wait);
	}

	if (events & RAIL_EVENT_TX_PACKET_SENT) {

		efr32->err = 0;
		LOG_DBG("TX packet sent");
	}
	if (events & RAIL_EVENT_TX_ABORTED) {

		LOG_DBG("TX was aborted");
		efr32->err = -ECONNABORTED;
	}
	if (events & RAIL_EVENT_TX_BLOCKED) {
		LOG_DBG("TX is blocked");
	}
	if (events & RAIL_EVENT_TX_UNDERFLOW) {

		LOG_DBG("TX underflow");
		efr32->err = -EIO;
	}
	if (events & RAIL_EVENT_TX_CHANNEL_BUSY) {

		LOG_DBG("TX channel busy");
		efr32->err = -EBUSY;
	}

	if (events & RAIL_EVENT_TXACK_PACKET_SENT) {
		LOG_DBG("TXACK packet sent");
	}
	if (events & RAIL_EVENT_TXACK_ABORTED) {
		LOG_DBG("TXACK aborted");
	}
	if (events & RAIL_EVENT_TXACK_BLOCKED) {
		LOG_DBG("TXACK blocked");
	}
	if (events & RAIL_EVENT_TXACK_UNDERFLOW) {
		LOG_DBG("TXACK underflow");
	}

	if (events & RAIL_EVENT_TX_FIFO_ALMOST_EMPTY) {
		LOG_DBG("TX FIFO almost empty");
	}
	if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
		LOG_DBG("RX FIFO almost full");
	}
	if (events & RAIL_EVENT_RX_ACK_TIMEOUT) {
		LOG_DBG("RX AutoAck occurred");
	}
	if (events & RAIL_EVENT_CAL_NEEDED) {
		LOG_DBG("Calibration needed");
		RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
	}
	if (events & RAIL_EVENT_IEEE802154_DATA_REQUEST_COMMAND) {
		LOG_DBG("IEEE802154 Data request command");

		//ieee802154DataRequestCommand(rail_handle);
	}
}

static struct ieee802154_radio_api efr32_radio_api = {
    .iface_api.init = efr32_iface_init,
    .iface_api.send = ieee802154_radio_send,

    .get_capabilities = efr32_get_capabilities,
    .cca = efr32_cca,
    .set_channel = efr32_set_channel,
    .filter = efr32_filter,
    .set_txpower = efr32_set_txpower,
    .start = efr32_start,
    .stop = efr32_stop,
    .tx = efr32_tx,
};

#if defined(CONFIG_NET_L2_IEEE802154)

#define L2 IEEE802154_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(IEEE802154_L2)
#define MTU 125

#elif defined(CONFIG_NET_L2_OPENTHREAD)
#define L2 OPENTHREAD_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
#define MTU 1280

#endif

NET_DEVICE_INIT(
    efr32,
    CONFIG_IEEE802154_EFR32_DRV_NAME,
    efr32_init,
    &efr32_data,
    NULL,
    CONFIG_IEEE802154_EFR32_INIT_PRIO,
    &efr32_radio_api,
    L2,
    L2_CTX_TYPE,
    MTU);
