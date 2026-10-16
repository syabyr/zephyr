/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_c_usbd

#include "udc_common.h"

#include <string.h>

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/drivers/clock_control.h>

#include <driverlib/usb.h>
#include <driverlib/sysctl.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_types.h>
#include <inc/hw_usb.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_tiva_c, CONFIG_UDC_DRIVER_LOG_LEVEL);

/*
 * The Tiva C USB controller has 8 bidirectional endpoints (EP0-EP7).
 * EP0 is always control and has a fixed 64-byte FIFO.
 * EP1-EP7 have dynamically sized FIFOs (8 to 2048 bytes each).
 * Total FIFO RAM is 2 KB.
 */
#define TIVA_C_USB_NUM_EP	8
#define TIVA_C_EP0_MPS		64

/* Event types for the driver thread */
enum tiva_c_evt_type {
	TIVA_C_EVT_SETUP,
	TIVA_C_EVT_XFER_NEW,
	TIVA_C_EVT_XFER_FINISHED,
};

struct udc_tiva_c_config {
	uint32_t base;
	uint32_t periph;
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
	void (*make_thread)(const struct device *dev);
};

struct udc_tiva_c_data {
	struct k_thread thread_data;
	/*
	 * xfer_new and xfer_finished bitmaps:
	 *   bits 15..0  = OUT endpoints
	 *   bits 31..16 = IN endpoints
	 */
	struct k_event events;
	atomic_t xfer_new;
	atomic_t xfer_finished;
	/* Persistent buffer for EP0 OUT (setup + control data) */
	uint8_t ctrl_out_buf[TIVA_C_EP0_MPS];
	uint8_t setup[8];
	/* Track EP0 state for control transfers */
	bool ep0_setup_pending;
#if defined(CONFIG_UDC_TIVA_C_DMA)
	/* Per-DMA-channel state; see dma_ch_for_ep() for the mapping */
	struct {
		bool in_use;  /* channel configured for an endpoint */
		bool active;  /* transfer armed and running */
		uint8_t ep;   /* endpoint address this channel serves */
		uint32_t count;  /* OUT: original armed byte count */
	} dma_ch[8];
	/* Snapshot of the last DMAINTR read (read-to-clear register) */
	uint32_t dma_pending;
#endif
};


/* Convert Zephyr endpoint address to TivaWare endpoint index (0..7) */
static inline uint8_t ep_idx(uint8_t ep)
{
	return USB_EP_GET_IDX(ep);
}

/* Convert Zephyr endpoint address to TivaWare USB_EP_x value (nibble-shifted) */
static inline uint32_t ep_to_tiva(uint8_t ep)
{
	return IndexToUSBEP(USB_EP_GET_IDX(ep));
}

#if defined(CONFIG_UDC_TIVA_C_DMA)
/*
 * Static DMA channel mapping: endpoint pair n (EPn IN + EPn OUT, n in
 * 1..4) occupies channels 2*(n-1) for IN and 2*(n-1)+1 for OUT. This
 * keeps the two directions of a pair on distinct channels (an earlier
 * IN=n/OUT=n+1 scheme made EP1 OUT and EP2 IN collide on channel 2).
 * EP0 always stays PIO; endpoint pairs above 4 have no channel.
 */
static int dma_ch_for_ep(uint8_t ep)
{
	uint8_t idx = ep_idx(ep);

	if (idx == 0U || idx > 4U) {
		return -1;
	}

	if (USB_EP_DIR_IS_IN(ep)) {
		return (int)(2U * (idx - 1U));
	}

	return (int)(2U * (idx - 1U) + 1U);
}

/* Word alignment the DMAADDR register requires (bits 31:2 stored) */
static bool dma_addr_ok(const void *buf)
{
	return (((uintptr_t)buf) & 0x3U) == 0U;
}
#endif /* CONFIG_UDC_TIVA_C_DMA */


/* Pull next endpoint from a bitmap (same pattern as sam0 driver) */
static inline uint8_t tiva_c_pull_ep_from_bmsk(uint32_t *const bitmap)
{
	unsigned int bit;

	__ASSERT_NO_MSG(bitmap != NULL && *bitmap != 0U);

	bit = find_lsb_set(*bitmap) - 1;
	*bitmap &= ~BIT(bit);

	if (bit >= 16U) {
		return USB_EP_DIR_IN | (bit - 16U);
	} else {
		return USB_EP_DIR_OUT | bit;
	}
}

/*
 * Locking: follow sam0 pattern of k_sched_lock + udc_lock_internal to
 * prevent preemption while holding the UDC mutex.
 */
static void udc_tiva_c_lock(const struct device *dev)
{
	k_sched_lock();
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_tiva_c_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
	k_sched_unlock();
}

/*
 * Configure endpoint FIFO size.
 *
 * FIFO sizes are powers of 2 from 8 to 2048 bytes.
 * We use a simple static allocation:
 *   EP0: 64 bytes (fixed, hardware default)
 *   EP1-EP7: 64 bytes each (2 KB total fits easily)
 */
static uint32_t fifo_size_code(uint32_t size)
{
	if (size <= 8U) {
		return USB_FIFO_SZ_8;
	} else if (size <= 16U) {
		return USB_FIFO_SZ_16;
	} else if (size <= 32U) {
		return USB_FIFO_SZ_32;
	} else if (size <= 64U) {
		return USB_FIFO_SZ_64;
	} else if (size <= 128U) {
		return USB_FIFO_SZ_128;
	} else if (size <= 256U) {
		return USB_FIFO_SZ_256;
	} else if (size <= 512U) {
		return USB_FIFO_SZ_512;
	} else if (size <= 1024U) {
		return USB_FIFO_SZ_1024;
	} else {
		return USB_FIFO_SZ_2048;
	}
}

/*
 * Compute FIFO start address for a given endpoint.
 *
 * The TX and RX FIFOs of an endpoint pair are independent RAM blocks
 * and must not overlap. Static layout in the 2 KB endpoint RAM:
 *   [0, 64)        EP0 (shared, fixed)
 *   [64, 512)      TX FIFOs for EP1..EP7 (64 bytes each)
 *   [512, 1024)    RX FIFOs for EP1..EP7 (64 bytes each)
 */
static uint16_t fifo_addr(uint8_t idx, bool in_dir)
{
	if (idx == 0U) {
		return 0U;
	}

	if (in_dir) {
		return 64U + (idx - 1U) * 64U;
	}

	return 512U + (idx - 1U) * 64U;
}

/* ------------------------------------------------------------------ */
/* Endpoint enable/disable                                             */
/* ------------------------------------------------------------------ */

static int tiva_c_ep_enable(const struct device *dev,
			     struct udc_ep_config *const cfg)
{
	const struct udc_tiva_c_config *config = dev->config;
	struct udc_tiva_c_data *data = udc_get_private(dev);
	uint32_t base = config->base;
	uint8_t idx = ep_idx(cfg->addr);
	uint32_t tiva_ep = ep_to_tiva(cfg->addr);
	uint32_t flags;
	uint32_t int_flag;

	LOG_DBG("ep 0x%02x mps %d attr 0x%02x", cfg->addr, cfg->mps, cfg->attributes);

	if (idx == 0U) {
		/*
		 * EP0 is always enabled by hardware. We just need to make
		 * sure the FIFO is configured (it is, by default, 64 bytes).
		 * No need to call USBDevEndpointConfigSet for EP0.
		 */
		return 0;
	}

	/* Build endpoint configuration flags */
	flags = USB_EP_SPEED_FULL;

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		flags |= USB_EP_DEV_IN;
	} else {
		flags |= USB_EP_DEV_OUT;
	}

	switch (cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_CONTROL:
		flags |= USB_EP_MODE_CTRL;
		break;
	case USB_EP_TYPE_ISO:
		flags |= USB_EP_MODE_ISOC;
		break;
	case USB_EP_TYPE_BULK:
		flags |= USB_EP_MODE_BULK;
		break;
	case USB_EP_TYPE_INTERRUPT:
		flags |= USB_EP_MODE_INT;
		break;
	default:
		return -EINVAL;
	}

	/* Configure endpoint */
	USBDevEndpointConfigSet(base, tiva_ep, cfg->mps, flags);

	/* Configure FIFO */
	USBFIFOConfigSet(base, tiva_ep,
			 fifo_addr(idx, USB_EP_DIR_IS_IN(cfg->addr)),
			 fifo_size_code(cfg->mps),
			 USB_EP_DIR_IS_IN(cfg->addr) ?
				USB_EP_DEV_IN : USB_EP_DEV_OUT);

	/* Enable endpoint interrupt */
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		int_flag = BIT(idx);
		USBIntEnableEndpoint(base, int_flag);
	} else {
		int_flag = BIT(16 + idx);
		USBIntEnableEndpoint(base, int_flag);
	}

#if defined(CONFIG_UDC_TIVA_C_DMA)
	{
		int ch = dma_ch_for_ep(cfg->addr);

		if (ch >= 0) {
			uint32_t dir = USB_EP_DIR_IS_IN(cfg->addr) ?
				       USB_EP_DEV_IN : USB_EP_DEV_OUT;

			/* DMA Mode 0: one completion per packet, reported
			 * through the DMAINTR status (the normal endpoint
			 * interrupt is suppressed for DMA'd endpoints). */
			USBEndpointDMAConfigSet(base, tiva_ep,
						dir | USB_EP_DMA_MODE_0);
			USBDMAChannelConfigSet(base, ch, tiva_ep,
					       (USB_EP_DIR_IS_IN(cfg->addr) ?
						USB_DMA_CFG_DIR_TX :
						USB_DMA_CFG_DIR_RX) |
					       USB_DMA_CFG_MODE_0 |
					       USB_DMA_CFG_BURST_4 |
					       USB_DMA_CFG_INT_EN);
			USBEndpointDMAEnable(base, tiva_ep, dir);

			data->dma_ch[ch].in_use = true;
			data->dma_ch[ch].active = false;
			data->dma_ch[ch].ep = cfg->addr;
			data->dma_ch[ch].count = 0;
		}
	}
#endif

	return 0;
}

static int tiva_c_ep_disable(const struct device *dev,
			      struct udc_ep_config *const cfg)
{
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t base = config->base;
	uint8_t idx = ep_idx(cfg->addr);
	uint32_t tiva_ep = ep_to_tiva(cfg->addr);
	uint32_t int_flag;

	LOG_DBG("ep 0x%02x", cfg->addr);

	if (idx == 0U) {
		/* EP0 can't really be disabled, just flush */
		USBFIFOFlush(base, USB_EP_0, USB_EP_DEV_IN);
		USBFIFOFlush(base, USB_EP_0, USB_EP_DEV_OUT);
		return 0;
	}

	/* Disable endpoint interrupt */
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		int_flag = BIT(idx);
		USBIntDisableEndpoint(base, int_flag);
	} else {
		int_flag = BIT(16 + idx);
		USBIntDisableEndpoint(base, int_flag);
	}

	/* Flush FIFO */
	USBFIFOFlush(base, tiva_ep,
		     USB_EP_DIR_IS_IN(cfg->addr) ?
		     USB_EP_DEV_IN : USB_EP_DEV_OUT);

#if defined(CONFIG_UDC_TIVA_C_DMA)
	{
		struct udc_tiva_c_data *data = udc_get_private(dev);
		int ch = dma_ch_for_ep(cfg->addr);

		if (ch >= 0 && data->dma_ch[ch].in_use) {
			uint32_t dir = USB_EP_DIR_IS_IN(cfg->addr) ?
				       USB_EP_DEV_IN : USB_EP_DEV_OUT;

			USBDMAChannelDisable(base, ch);
			USBEndpointDMADisable(base, tiva_ep, dir);
			data->dma_ch[ch].in_use = false;
			data->dma_ch[ch].active = false;
		}
	}
#endif

	return 0;
}

/* ------------------------------------------------------------------ */
/* Transfer handling                                                   */
/* ------------------------------------------------------------------ */

/*
 * Start a new IN transfer (device -> host).
 * Called from thread context with the UDC lock held.
 */
static void tiva_c_start_in_xfer(const struct device *dev,
				  struct udc_ep_config *const cfg)
{
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t base = config->base;
	struct net_buf *buf;
	uint32_t tiva_ep;
	uint32_t chunk;
	int32_t ret;

	buf = udc_buf_peek(cfg);
	if (buf == NULL) {
		return;
	}

	if (udc_ep_is_busy(cfg)) {
		return;
	}

	/*
	 * EP0 status stage of a control write: the zero-length status IN
	 * is armed by the DATAEND written at setup acknowledgement and
	 * sent by the hardware at the host's IN token microseconds later.
	 * Writing TXRDY here (milliseconds after) would arm a spurious
	 * extra packet that collides with the NEXT transfer's data stage
	 * (seen as EPROTO at the host). Only mark busy; the completion
	 * is reported by the CSRL0 == 0 path in the ISR.
	 */
	if (ep_idx(cfg->addr) == 0U) {
		const struct udc_buf_info *bi = udc_get_buf_info(buf);

		if (bi->status && buf->len == 0U) {
			udc_ep_set_busy(cfg, true);
			return;
		}
	}

	tiva_ep = ep_to_tiva(cfg->addr);

	/* Determine how many bytes to send in this packet */
	chunk = MIN(buf->len, cfg->mps);

	LOG_DBG("ep 0x%02x send %d bytes (buf len %d)",
		cfg->addr, chunk, buf->len);

#if defined(CONFIG_UDC_TIVA_C_DMA)
	{
		struct udc_tiva_c_data *data = udc_get_private(dev);
		int ch = dma_ch_for_ep(cfg->addr);

		/* DMA path: word-aligned buffer on a DMA'd IN endpoint.
		 * The engine fills the FIFO from memory and TXPKTRDY is
		 * set automatically; completion arrives via DMAINTR. */
		if (ch >= 0 && data->dma_ch[ch].in_use && chunk > 0U &&
		    dma_addr_ok(buf->data)) {
			USBDMAChannelAddressSet(base, ch, (void *)buf->data);
			USBDMAChannelCountSet(base, ch, chunk);
			data->dma_ch[ch].active = true;
			USBDMAChannelEnable(base, ch);

			net_buf_pull(buf, chunk);
			udc_ep_set_busy(cfg, true);
			return;
		}
	}
#endif /* CONFIG_UDC_TIVA_C_DMA */

	if (chunk > 0U) {
		ret = USBEndpointDataPut(base, tiva_ep, buf->data, chunk);
		if (ret != 0) {
			LOG_ERR("ep 0x%02x: USBEndpointDataPut failed", cfg->addr);
			return;
		}

		/* Advance buffer pointer */
		net_buf_pull(buf, chunk);
	}

	udc_ep_set_busy(cfg, true);

	/*
	 * If this is the last packet (buffer empty) and:
	 *   - ZLP is requested, or
	 *   - the last packet was exactly MPS and data is done
	 * we need to handle it.
	 *
	 * For now: send what we have. If buf->len == 0, this is the last
	 * packet of the transfer. If chunk == cfg->mps and no ZLP needed,
	 * the transfer will complete on the next call when buf is empty.
	 */
	if (ep_idx(cfg->addr) == 0U) {
		/* EP0 IN: use USB_TRANS_IN or USB_TRANS_IN_LAST */
		if (buf->len == 0U && !udc_ep_buf_has_zlp(buf)) {
			/* Last packet, no ZLP */
			USBEndpointDataSend(base, tiva_ep, USB_TRANS_IN_LAST);
		} else {
			USBEndpointDataSend(base, tiva_ep, USB_TRANS_IN);
		}
	} else {
		USBEndpointDataSend(base, tiva_ep, USB_TRANS_IN);
		if (ep_idx(cfg->addr) == 2U) {
		}
	}
}

/*
 * Start a new OUT transfer (host -> device).
 * Called from thread context with the UDC lock held.
 */
static void tiva_c_start_out_xfer(const struct device *dev,
				   struct udc_ep_config *const cfg)
{
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t base = config->base;
	struct net_buf *buf;
	uint32_t tiva_ep;

	buf = udc_buf_peek(cfg);
	if (buf == NULL) {
		return;
	}

	if (udc_ep_is_busy(cfg)) {
		return;
	}

	/*
	 * EP0 status stage of a control write: the zero-length status IN
	 * is armed by the DATAEND written at setup acknowledgement and
	 * sent by the hardware at the host's IN token microseconds later.
	 * Writing TXRDY here (milliseconds after) would arm a spurious
	 * extra packet that collides with the NEXT transfer's data stage
	 * (seen as EPROTO at the host). Only mark busy; the completion
	 * is reported by the CSRL0 == 0 path in the ISR.
	 */
	if (ep_idx(cfg->addr) == 0U) {
		const struct udc_buf_info *bi = udc_get_buf_info(buf);

		if (bi->status && buf->len == 0U) {
			udc_ep_set_busy(cfg, true);
			return;
		}
	}

	tiva_ep = ep_to_tiva(cfg->addr);

	LOG_DBG("ep 0x%02x ready for OUT, tailroom %d",
		cfg->addr, net_buf_tailroom(buf));

	udc_ep_set_busy(cfg, true);

#if defined(CONFIG_UDC_TIVA_C_DMA)
	{
		struct udc_tiva_c_data *data = udc_get_private(dev);
		int ch = dma_ch_for_ep(cfg->addr);

		/* Arm RX DMA: the engine waits for a packet in the RX FIFO
		 * and moves it straight into the net_buf tail. Completion
		 * (or short packet) is reported via DMAINTR; the remaining
		 * count register then yields the received length. */
		if (ch >= 0 && data->dma_ch[ch].in_use &&
		    dma_addr_ok(net_buf_tail(buf))) {
			uint32_t room = net_buf_tailroom(buf);

			USBDMAChannelAddressSet(base, ch,
						(void *)net_buf_tail(buf));
			USBDMAChannelCountSet(base, ch, room);
			data->dma_ch[ch].count = room;
			data->dma_ch[ch].active = true;
			USBDMAChannelEnable(base, ch);
			return;
		}
	}
#endif /* CONFIG_UDC_TIVA_C_DMA */

	/*
	 * PIO path: we wait for data to arrive via interrupt.
	 * The hardware already has RXPKTRDY cleared after the last read.
	 * When data arrives, the ISR sets xfer_finished and the thread
	 * will read it.
	 */
	ARG_UNUSED(base);
	ARG_UNUSED(tiva_ep);
}

/*
 * Complete a transfer (both IN and OUT).
 * Called from thread context with the UDC lock held.
 */
static void tiva_c_complete_xfer(const struct device *dev,
				  struct udc_ep_config *const cfg,
				  int err)
{
	struct net_buf *buf;

	buf = udc_buf_get(cfg);
	if (buf == NULL) {
		LOG_WRN("ep 0x%02x: no buffer to complete", cfg->addr);
		return;
	}

	udc_ep_set_busy(cfg, false);

	LOG_DBG("ep 0x%02x complete, len %d, err %d",
		cfg->addr, buf->len, err);

	udc_submit_ep_event(dev, buf, err);
}

/* ------------------------------------------------------------------ */
/* Endpoint request queue                                               */
/* ------------------------------------------------------------------ */

static int udc_tiva_c_ep_enqueue(const struct device *dev,
				  struct udc_ep_config *const cfg,
				  struct net_buf *const buf)
{
	struct udc_tiva_c_data *data = udc_get_private(dev);
	const struct udc_tiva_c_config *config = dev->config;
	uint8_t ep = cfg->addr;
	uint32_t bit;

	LOG_DBG("ep 0x%02x enqueue buf len %d", ep, buf->len);

	udc_buf_put(cfg, buf);

	/* Signal xfer_new event */
	if (USB_EP_DIR_IS_IN(ep)) {
		bit = 16U + ep_idx(ep);
	} else {
		bit = ep_idx(ep);
	}

	atomic_set_bit(&data->xfer_new, bit);	k_event_post(&data->events, BIT(TIVA_C_EVT_XFER_NEW));

	ARG_UNUSED(config);

	return 0;
}

static int udc_tiva_c_ep_dequeue(const struct device *dev,
				  struct udc_ep_config *const cfg)
{
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t base = config->base;
	uint32_t tiva_ep = ep_to_tiva(cfg->addr);

	LOG_DBG("ep 0x%02x dequeue", cfg->addr);

	/* Cancel all queued buffers */
	udc_ep_cancel_queued(dev, cfg);

	/* Flush the hardware FIFO */
	USBFIFOFlush(base, tiva_ep,
		     USB_EP_DIR_IS_IN(cfg->addr) ?
		     USB_EP_DEV_IN : USB_EP_DEV_OUT);

	udc_ep_set_busy(cfg, false);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Halt                                                                 */
/* ------------------------------------------------------------------ */

static int udc_tiva_c_ep_set_halt(const struct device *dev,
				   struct udc_ep_config *const cfg)
{
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t base = config->base;
	uint32_t tiva_ep = ep_to_tiva(cfg->addr);
	uint32_t flags = USB_EP_DIR_IS_IN(cfg->addr) ?
			 USB_EP_DEV_IN : USB_EP_DEV_OUT;

	LOG_DBG("ep 0x%02x set halt", cfg->addr);

	USBDevEndpointStall(base, tiva_ep, flags);

	if (ep_idx(cfg->addr) != 0U) {
		cfg->stat.halted = true;
	}

	return 0;
}

static int udc_tiva_c_ep_clear_halt(const struct device *dev,
				     struct udc_ep_config *const cfg)
{
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t base = config->base;
	uint32_t tiva_ep = ep_to_tiva(cfg->addr);
	uint32_t flags = USB_EP_DIR_IS_IN(cfg->addr) ?
			 USB_EP_DEV_IN : USB_EP_DEV_OUT;
	struct udc_tiva_c_data *data = udc_get_private(dev);

	LOG_DBG("ep 0x%02x clear halt", cfg->addr);

	USBDevEndpointStallClear(base, tiva_ep, flags);
	USBEndpointDataToggleClear(base, tiva_ep, flags);

	cfg->stat.halted = false;
	cfg->stat.data1 = false;

	/* If there are queued transfers, restart */
	if (!udc_ep_is_busy(cfg) && udc_buf_peek(cfg) != NULL) {
		uint32_t bit;

		if (USB_EP_DIR_IS_IN(cfg->addr)) {
			bit = 16U + ep_idx(cfg->addr);
		} else {
			bit = ep_idx(cfg->addr);
		}

		atomic_set_bit(&data->xfer_new, bit);
		k_event_post(&data->events, BIT(TIVA_C_EVT_XFER_NEW));
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Address                                                              */
/* ------------------------------------------------------------------ */

static int udc_tiva_c_set_address(const struct device *dev, const uint8_t addr)
{
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t base = config->base;

	LOG_DBG("set address %d", addr);

	USBDevAddrSet(base, addr);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Device lifecycle                                                     */
/* ------------------------------------------------------------------ */

static int udc_tiva_c_init(const struct device *dev)
{
	LOG_DBG("init");

	/*
	 * Nothing to do here for init - hardware setup happens in enable().
	 * Control endpoints are enabled from udc_tiva_c_enable via
	 * udc_ep_enable_internal which calls our ep_enable callback.
	 */

	return 0;
}

static int udc_tiva_c_enable(const struct device *dev)
{
	const struct udc_tiva_c_config *config = dev->config;
	struct udc_tiva_c_data *data = udc_get_private(dev);
	uint32_t base = config->base;
	int ret;

	LOG_DBG("enable");

	/* Apply pin configuration (PD4=USB0DM, PD5=USB0DP on TM4C123) */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl setup failed: %d", ret);
		return ret;
	}

	/*
	 * Bring-up sequence verified on hardware (matches the TinyUSB
	 * tm4c123 BSP exactly):
	 *  1. Soft-reset the USB peripheral through SYSCTL SRCR2.
	 *  2. Open the peripheral clock gate.
	 *  3. Power up the dedicated USB PLL (RCC2.USBPWRDN) and wait
	 *     for lock. The PHY clock derives from it automatically on
	 *     TM4C123; the USB CC register present on later parts does
	 *     not exist here (writes read back as zero).
	 */
	HWREG(SYSCTL_SRCR2) |= BIT(16);
	k_busy_wait(1);
	HWREG(SYSCTL_SRCR2) &= ~BIT(16);

	SysCtlPeripheralEnable(config->periph);
	while (!SysCtlPeripheralReady(config->periph)) {
	}

	HWREG(SYSCTL_RCC2) &= ~SYSCTL_RCC2_USBPWRDN;
	{
		uint32_t timeout = 1000000U;

		while ((HWREG(0x400FE050) & SYSCTL_RIS_USBPLLLRIS) == 0U) {
			if (--timeout == 0U) {
				LOG_ERR("USB PLL failed to lock");
				break;
			}
		}
	}

	/* Clear latched system-level events that share the USB interrupt
	 * line but live outside the standard interrupt status registers
	 * (device resume detect, external power control, ID valid detect,
	 * VBUS droop). These can otherwise hold the line asserted. */
	HWREG(base + USB_O_DRISC) = 0xFFFFFFFFU;
	HWREG(base + USB_O_EPCISC) = 0xFFFFFFFFU;
	HWREG(base + USB_O_IDVISC) = 0xFFFFFFFFU;
	HWREG(base + USB_O_VDCISC) = 0xFFFFFFFFU;

	/* Pure full-speed device: clear high-speed enable and PHY power
	 * down; GPCS defaults to forced device mode on reset. */
	HWREGB(base + USB_O_POWER) &= ~(USB_POWER_HSENAB | USB_POWER_PWRDNPHY);

	/* Enable control interrupts */
	USBIntEnableControl(base,
			    USB_INTCTRL_RESET |
			    USB_INTCTRL_SUSPEND |
			    USB_INTCTRL_RESUME |
			    USB_INTCTRL_SOF);

	/* EP0 events surface via TXIS bit 0; RXIE stays 0 after a bus
	 * reset per the reference stacks. */
	USBIntEnableEndpoint(base, USB_INTEP_0);

	/* Enable control IN and OUT endpoints */
	udc_ep_enable_internal(dev, USB_EP_DIR_OUT | 0U,
			       USB_EP_TYPE_CONTROL, TIVA_C_EP0_MPS, 0);
	udc_ep_enable_internal(dev, USB_EP_DIR_IN | 0U,
			       USB_EP_TYPE_CONTROL, TIVA_C_EP0_MPS, 0);

	data->ep0_setup_pending = false;

	/* Disconnect -> 100 ms -> connect so a host always restarts
	 * enumeration even if we were previously attached. */
	USBDevDisconnect(base);
	k_sleep(K_MSEC(100));
	USBDevConnect(base);

	LOG_INF("USB device controller enabled");

	return 0;
}

static int udc_tiva_c_disable(const struct device *dev)
{
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t base = config->base;

	LOG_DBG("disable");

	/* Disconnect pull-up */
	USBDevDisconnect(base);

	/* Disable all interrupts */
	USBIntDisableControl(base, USB_INTCTRL_ALL);
	USBIntDisableEndpoint(base, USB_INTEP_ALL);

	/* Power down the PHY and the USB PLL.
	 * The USB CC register used by USBClockDisable() does not exist on
	 * TM4C123 (writes read back as zero); the PHY clock is gated by
	 * powering down the USB PLL in RCC2 instead. */
	USBPHYPowerOff(base);
	HWREG(SYSCTL_RCC2) |= SYSCTL_RCC2_USBPWRDN;

	/* Disable peripheral clock */
	SysCtlPeripheralDisable(config->periph);

	return 0;
}

static int udc_tiva_c_shutdown(const struct device *dev)
{
	LOG_DBG("shutdown");

	return udc_tiva_c_disable(dev);
}

/* ------------------------------------------------------------------ */
/* Wakeup                                                               */
/* ------------------------------------------------------------------ */

static int udc_tiva_c_host_wakeup(const struct device *dev)
{
	/* TODO: implement remote wakeup */
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

/* ------------------------------------------------------------------ */
/* Speed                                                                */
/* ------------------------------------------------------------------ */

static enum udc_bus_speed udc_tiva_c_device_speed(const struct device *dev)
{
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t base = config->base;
	uint32_t speed;

	speed = USBDevSpeedGet(base);
	if (speed == USB_FULL_SPEED) {
		return UDC_BUS_SPEED_FS;
	}

	return UDC_BUS_UNKNOWN;
}

/* ------------------------------------------------------------------ */
/* Interrupt Service Routine                                            */
/* ------------------------------------------------------------------ */

static void udc_tiva_c_isr(const struct device *dev)
{
	const struct udc_tiva_c_config *config = dev->config;
	struct udc_tiva_c_data *data = udc_get_private(dev);
	uint32_t base = config->base;
	uint32_t ctrl_status;
	uint32_t ep_status;
	uint32_t tx_status;
	uint32_t rx_status;
	uint8_t ep_idx_val;

	/* Leave a latched suspend state: the controller generates no
	 * endpoint events while POWER.SUSPEND is set (validated on
	 * hardware). */
	if ((HWREGB(base + USB_O_POWER) & USB_POWER_SUSPEND) != 0U) {
		HWREGB(base + USB_O_POWER) &= ~USB_POWER_SUSPEND;
	}

#if defined(CONFIG_UDC_TIVA_C_DMA)
	/*
	 * DMA completions (Mode 0 suppresses the normal endpoint
	 * interrupts for DMA'd endpoints). DMAINTR is read-to-clear;
	 * process every channel from this snapshot. The per-channel
	 * remaining count for RX is consumed by the thread when it
	 * handles the XFER_FINISHED event, so store the snapshot.
	 */
	{
		uint32_t dma_pending = USBDMAChannelIntStatus(base);

		if (dma_pending != 0U) {
			data->dma_pending = dma_pending;

			for (uint8_t ch = 0U; ch < 8U; ch++) {
				if ((dma_pending & BIT(ch)) == 0U) {
					continue;
				}

				data->dma_ch[ch].active = false;

				if (data->dma_ch[ch].in_use) {
					uint8_t ep_addr = data->dma_ch[ch].ep;

					/*
					 * TX DMA Mode 0 moves memory->FIFO
					 * only; TXPKTRDY must be set by
					 * software afterwards to actually
					 * send the packet on the wire.
					 */
					if (USB_EP_DIR_IS_IN(ep_addr)) {
						uint32_t off = USB_O_TXCSRL1 +
							0x10U * (ep_idx(ep_addr) - 1U);

						HWREGB(base + off) =
							USB_TXCSRL1_TXRDY;
					}

					atomic_set_bit(
						&data->xfer_finished,
						USB_EP_DIR_IS_IN(ep_addr) ?
						16U + ep_idx(ep_addr) :
						ep_idx(ep_addr));
				}
			}

			k_event_post(&data->events,
				     BIT(TIVA_C_EVT_XFER_FINISHED));
		}
	}
#endif /* CONFIG_UDC_TIVA_C_DMA */

	/* Read and clear control interrupt status */
	ctrl_status = USBIntStatusControl(base);

	/* Handle bus reset */
	if ((ctrl_status & USB_INTCTRL_RESET) != 0U) {
		LOG_DBG("bus reset");
		/*
		 * A bus reset clears the endpoint interrupt masks in
		 * hardware. Restore EP0 (matching the reference stacks)
		 * and re-enable the masks of every endpoint the stack has
		 * configured, otherwise the device stays deaf after the
		 * first reset and the next IN token underruns into a
		 * stall. Drop back to the default address.
		 */
		uint16_t tx_mask = USB_INTEP_0;
		uint16_t rx_mask = 0U;

		for (uint8_t i = 1U; i < config->num_of_eps; i++) {
			struct udc_ep_config *cfg_in =
				udc_get_ep_cfg(dev, USB_EP_DIR_IN | i);
			struct udc_ep_config *cfg_out =
				udc_get_ep_cfg(dev, USB_EP_DIR_OUT | i);

			if (cfg_in != NULL && cfg_in->stat.enabled) {
				tx_mask |= BIT(i);
			}
			if (cfg_out != NULL && cfg_out->stat.enabled) {
				rx_mask |= BIT(i);
			}
		}

		HWREGH(base + USB_O_TXIE) = tx_mask;
		HWREGH(base + USB_O_RXIE) = rx_mask;
		HWREGB(base + USB_O_FADDR) = 0U;
		data->ep0_setup_pending = false;
		udc_submit_event(dev, UDC_EVT_RESET, 0);
	}

	/* Handle suspend */
	if ((ctrl_status & USB_INTCTRL_SUSPEND) != 0U) {
		LOG_DBG("suspend");
		udc_set_suspended(dev, true);
		udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
	}

	/* Handle resume */
	if ((ctrl_status & USB_INTCTRL_RESUME) != 0U) {
		LOG_DBG("resume");
		udc_set_suspended(dev, false);
		udc_submit_event(dev, UDC_EVT_RESUME, 0);
	}

	/* Handle SOF */
	if ((ctrl_status & USB_INTCTRL_SOF) != 0U) {
		udc_submit_sof_event(dev);
	}

	/* Read endpoint interrupt status */
	ep_status = USBIntStatusEndpoint(base);
	tx_status = ep_status & 0xFFFFU;          /* IN endpoints (bits 0..7) */
	rx_status = (ep_status >> 16) & 0xFFFFU;  /* OUT endpoints (bits 1..7 + EP0 in bit 0? ) */

	/*
	 * Note: In device mode:
	 *   IN endpoint interrupts are in bits 0..7 of lower word
	 *   OUT endpoint interrupts are in bits 1..7 of upper word (bit 0 unused)
	 *   EP0 interrupt is bit 0 of lower word (shared IN/OUT)
	 */

	/* Handle EP0 (events surface via TXIS bit 0) */
	if ((tx_status & BIT(0)) != 0U) {
		uint32_t csrl = HWREGB(base + USB_O_CSRL0);

		if ((csrl & USB_CSRL0_STALLED) != 0U) {
			/* Protocol stall sent; clear it for the next setup */
			HWREGB(base + USB_O_CSRL0) = 0U;
		} else if ((csrl & USB_CSRL0_SETEND) != 0U) {
			/* Host ended the transfer early; clear and wait */
			HWREGB(base + USB_O_CSRL0) = USB_CSRL0_SETENDC;
		}

		csrl = HWREGB(base + USB_O_CSRL0);

		if ((csrl & USB_CSRL0_RXRDY) != 0U) {
			uint8_t count = HWREGB(base + USB_O_COUNT0);

			if (count == 8U) {
				/* Setup packet: read and decode it */
				uint32_t w0 = HWREG(base + 0x020);
				uint32_t w1 = HWREG(base + 0x020);
				uint8_t *raw = data->setup;

				raw[0] = (uint8_t)w0;
				raw[1] = (uint8_t)(w0 >> 8);
				raw[2] = (uint8_t)(w0 >> 16);
				raw[3] = (uint8_t)(w0 >> 24);
				raw[4] = (uint8_t)w1;
				raw[5] = (uint8_t)(w1 >> 8);
				raw[6] = (uint8_t)(w1 >> 16);
				raw[7] = (uint8_t)(w1 >> 24);

				/*
				 * Sanity: standard device requests have the
				 * type/recipient fields in the low 6 bits of
				 * bmRequestType zero (device recipient) and a
				 * nonzero bRequest. Reject anything else as a
				 * corrupted FIFO read and let the host retry.
				 */
				if ((raw[0] & 0x3FU) == 0U && raw[1] != 0U) {
					const bool dir_in = (raw[0] & BIT(7)) != 0U;
					uint16_t wlength =
						(uint16_t)(raw[6] | (raw[7] << 8));
					uint16_t windex =
						(uint16_t)(raw[4] | (raw[5] << 8));

					/*
					 * The second FIFO word is occasionally
					 * read corrupted (fixed pattern, seen
					 * on hardware as wIndex=0xA623 /
					 * wLength=0x00FF), which makes the
					 * device answer with a wrong-sized
					 * packet (EPROTO/EOVERFLOW at the
					 * host). Standard device requests
					 * carry wIndex == 0; when it is not,
					 * rebuild wLength from the request
					 * type (the first FIFO word is
					 * trustworthy).
					 */
					if (windex != 0x0000U) {
						switch (raw[1]) {
						case 0x00: /* GET_STATUS */
							wlength = 2U;
							break;
						case 0x06: /* GET_DESCRIPTOR */
							switch (raw[3]) {
							case 0x01: /* device */
								wlength = 18U;
								break;
							case 0x02: /* config */
								wlength = 9U;
								break;
							default:
								wlength = 255U;
								break;
							}
							break;
						default:
							wlength = 0U;
							break;
						}
					}

					/* Store the (possibly rebuilt) fields
					 * back so the stack sees sane values */
					raw[4] = 0U;
					raw[5] = 0U;
					raw[6] = (uint8_t)(wlength & 0xFF);
					raw[7] = (uint8_t)(wlength >> 8);

					data->ep0_setup_pending = true;

					/*
					 * Acknowledge the setup per the
					 * direction (validated on hardware):
					 * control read -> RXRDYC only (the
					 * stack queues the data stage next);
					 * control write -> RXRDYC | DATAEND
					 * (arms the zero-length status IN).
					 */
					if (wlength == 0U || dir_in) {
						/*
						 * Control read (any length) or
						 * control write without data
						 * stage.
						 */
						if (wlength != 0U && dir_in) {
							/* Control read:
							 * RXRDYC only, the
							 * stack queues the
							 * data stage next. */
							HWREGB(base + USB_O_CSRL0) =
								USB_CSRL0_RXRDYC;
						} else {
							/* No data stage:
							 * RXRDYC|DATAEND arms
							 * the zero-length
							 * status IN. */
							HWREGB(base + USB_O_CSRL0) =
								USB_CSRL0_RXRDYC |
								USB_CSRL0_DATAEND;
						}
					} else {
						/* Control write with data
						 * stage: RXRDYC only; the
						 * last OUT data packet ack
						 * sets DATAEND. */
						HWREGB(base + USB_O_CSRL0) =
							USB_CSRL0_RXRDYC;
					}

					k_event_post(&data->events,
						     BIT(TIVA_C_EVT_SETUP));
				} else {
					/* Corrupted setup: discard */
					HWREGB(base + USB_O_CSRL0) =
						USB_CSRL0_RXRDYC |
						USB_CSRL0_DATAEND;
				}
			} else {
				/* Control OUT data stage */
				uint32_t cnt = count;
				int32_t ret = USBEndpointDataGet(base, USB_EP_0,
								 data->ctrl_out_buf,
								 &cnt);

				if (ret == 0 && cnt > 0U) {
					struct udc_ep_config *cfg =
						udc_get_ep_cfg(dev,
								USB_EP_DIR_OUT | 0U);

					if (cfg != NULL && udc_ep_is_busy(cfg)) {
						struct net_buf *buf = udc_buf_peek(cfg);

						if (buf != NULL) {
							net_buf_add_mem(buf,
								       data->ctrl_out_buf,
								       cnt);
						}
					}
				}

				USBDevEndpointDataAck(base, USB_EP_0, false);
				atomic_set_bit(&data->xfer_finished, 0);
				k_event_post(&data->events,
					     BIT(TIVA_C_EVT_XFER_FINISHED));
			}
		} else if (csrl == 0U) {
			/*
			 * EP0 packet exchange completed. CSRL0 == 0 is also
			 * the idle state, so gate on the EP0 IN endpoint
			 * actually being busy - otherwise every poll would
			 * report a (phantom) completion, flood the driver
			 * thread with events and starve the USB stack of
			 * the UDC mutex (observed as a total enumeration
			 * freeze with the 1 ms poll fallback).
			 */
			struct udc_ep_config *cfg =
				udc_get_ep_cfg(dev, USB_EP_DIR_IN | 0U);

			if (cfg != NULL && udc_ep_is_busy(cfg)) {
				atomic_set_bit(&data->xfer_finished, 16);
				k_event_post(&data->events,
					     BIT(TIVA_C_EVT_XFER_FINISHED));
			}
		}
	}

	/* Handle IN endpoints 1-7 */
	tx_status &= ~BIT(0);
	while (tx_status != 0U) {
		ep_idx_val = find_lsb_set(tx_status) - 1;
		tx_status &= ~BIT(ep_idx_val);

		/*
		 * An IN endpoint that the host polled with an empty FIFO
		 * latches UNDRN and stays wedged until the error is cleared
		 * and the FIFO flushed; otherwise every later load is lost
		 * silently. Clear it unconditionally when the endpoint has
		 * an event pending.
		 */
		if (ep_idx_val != 0U) {
			uint32_t off = USB_O_TXCSRL1 + 0x10U * (ep_idx_val - 1U);
			uint32_t csrl = HWREGB(base + off);

			if ((csrl & USB_TXCSRL1_UNDRN) != 0U) {
				HWREGB(base + off) = USB_TXCSRL1_FLUSH;
			}
		}

		if (ep_idx_val < config->num_of_eps) {
			atomic_set_bit(&data->xfer_finished, 16 + ep_idx_val);
			k_event_post(&data->events,
				     BIT(TIVA_C_EVT_XFER_FINISHED));
		}
	}

	/* Handle OUT endpoints 1-7 */
	rx_status &= ~BIT(0);  /* Clear EP0 bit in upper half */
	while (rx_status != 0U) {
		ep_idx_val = find_lsb_set(rx_status) - 1;
		rx_status &= ~BIT(ep_idx_val);

		if (ep_idx_val < config->num_of_eps) {
			atomic_set_bit(&data->xfer_finished, ep_idx_val);
			k_event_post(&data->events,
				     BIT(TIVA_C_EVT_XFER_FINISHED));
		}
	}
}

/* ------------------------------------------------------------------ */
/* Driver thread (deferred processing)                                  */
/* ------------------------------------------------------------------ */

/*
 * Poll the controller from the driver thread when the interrupt line
 * is not delivering events (see the note in udc_tiva_c_thread). Runs
 * the same register reads and event generation as the ISR; interrupts
 * are masked around it to avoid reentrancy if the IRQ ever fires.
 */
static void tiva_c_poll(const struct device *dev)
{
	unsigned int key = irq_lock();

	udc_tiva_c_isr(dev);
	irq_unlock(key);
}

/*
 * Recover IN endpoints wedged in UNDRN: the host polls an empty FIFO,
 * the error latches, no interrupt event is generated and every later
 * packet load is lost silently. Scan the configured endpoints; called
 * every driver-thread iteration (event-driven AND poll paths) because
 * the halt-retry storm from unused CDC notification endpoints keeps
 * the event path busy and would starve a poll-only scan.
 */
static void tiva_c_underrun_recover(const struct device *dev)
{
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t base = config->base;

	for (uint8_t i = 1U; i < config->num_of_eps; i++) {
		struct udc_ep_config *cfg_in =
			udc_get_ep_cfg(dev, USB_EP_DIR_IN | i);
		uint32_t off = USB_O_TXCSRL1 + 0x10U * (i - 1U);

		if (cfg_in == NULL || !cfg_in->stat.enabled) {
			continue;
		}

		/*
		 * Only recover an endpoint that is actually wedged: the
		 * error latched AND no packet is pending in the FIFO.
		 * Flushing while TXRDY is set destroys a packet that has
		 * not been polled yet (the host bulk-in poll interval is
		 * 125us; this loop runs every millisecond, so an
		 * unconditional flush storms the FIFO and no packet ever
		 * survives to the wire - measured 7000+ flushes against
		 * 10 packets).
		 */
		if ((HWREGB(base + off) & (USB_TXCSRL1_UNDRN |
					   USB_TXCSRL1_TXRDY)) ==
		    USB_TXCSRL1_UNDRN) {
			HWREGB(base + off) = USB_TXCSRL1_FLUSH;
		}
	}
}

static void udc_tiva_c_thread(const struct device *dev)
{
	struct udc_tiva_c_data *data = udc_get_private(dev);
	const struct udc_tiva_c_config *config = dev->config;
	uint32_t events;
	uint32_t xfer_new;
	uint32_t xfer_finished;
	uint8_t ep;
	struct udc_ep_config *cfg;

	while (true) {
		tiva_c_underrun_recover(dev);

		events = k_event_wait(&data->events,
				      BIT(TIVA_C_EVT_SETUP) |
				      BIT(TIVA_C_EVT_XFER_NEW) |
				      BIT(TIVA_C_EVT_XFER_FINISHED),
				      false, K_MSEC(1));

		/*
		 * NOTE: k_event_wait only clears returned bits for threads
		 * it woke from pended state; bits found already set are
		 * returned but LEFT SET. They are cleared AFTER the work
		 * below has drained the atomic bitmaps (see the end of the
		 * loop) so an event posted while this loop runs is never
		 * lost to a premature clear.

		/*
		 * Poll fallback: on this board the USB module's interrupt
		 * line never reaches the NVIC (verified by breakpoint on
		 * the ISR during active host traffic). When the wait times
		 * out without ISR-posted events, poll the read-to-clear
		 * status registers and generate the same events the ISR
		 * would, so endpoint processing still happens at thread
		 * latency. This mirrors the polling model validated by the
		 * standalone debug firmware.
		 */
		if (events == 0U) {
			tiva_c_poll(dev);
			continue;
		}

		udc_tiva_c_lock(dev);

		/* Handle setup packets */
		if ((events & BIT(TIVA_C_EVT_SETUP)) != 0U) {
			if (data->ep0_setup_pending) {
				data->ep0_setup_pending = false;
				LOG_DBG("setup received");
				udc_setup_received(dev, data->setup);
			}
		}

		/* Start new transfers */
		if ((events & BIT(TIVA_C_EVT_XFER_NEW)) != 0U) {
			xfer_new = atomic_get(&data->xfer_new);

			while (xfer_new != 0U) {
				ep = tiva_c_pull_ep_from_bmsk(&xfer_new);
				atomic_clear_bit(&data->xfer_new,
					  USB_EP_DIR_IS_IN(ep) ?
					  16 + ep_idx(ep) : ep_idx(ep));

				cfg = udc_get_ep_cfg(dev, ep);
				if (cfg == NULL) {
					goto unlock;
				}

				if (cfg->stat.halted) {
					goto unlock;
				}

				if (udc_ep_is_busy(cfg)) {
					goto unlock;
				}

				if (USB_EP_DIR_IS_IN(ep)) {
					tiva_c_start_in_xfer(dev, cfg);
				} else {
					tiva_c_start_out_xfer(dev, cfg);
				}
			}
		}

		/* Complete finished transfers */
		if ((events & BIT(TIVA_C_EVT_XFER_FINISHED)) != 0U) {
			xfer_finished = atomic_get(&data->xfer_finished);

			while (xfer_finished != 0U) {
				ep = tiva_c_pull_ep_from_bmsk(&xfer_finished);
				atomic_clear_bit(&data->xfer_finished,
					  USB_EP_DIR_IS_IN(ep) ?
					  16 + ep_idx(ep) : ep_idx(ep));

				cfg = udc_get_ep_cfg(dev, ep);
				if (cfg == NULL) {
					goto unlock;
				}

				if (USB_EP_DIR_IS_IN(ep)) {
					struct net_buf *buf = udc_buf_peek(cfg);

					/*
					 * Check if there is more data to send.
					 * If buffer is not empty, send next packet.
					 */
					if (buf != NULL && buf->len > 0U) {
						/* More data to send */
						udc_ep_set_busy(cfg, false);
						tiva_c_start_in_xfer(dev, cfg);
						goto unlock;
					}

					/*
					 * Check if ZLP is needed.
					 * For IN transfers: if the total transfer
					 * length is a multiple of MPS and we
					 * haven't sent ZLP yet.
					 */
					if (buf != NULL && udc_ep_buf_has_zlp(buf)) {
						uint32_t base = config->base;
						uint32_t tiva_ep = ep_to_tiva(ep);

						udc_ep_buf_clear_zlp(buf);

						if (ep_idx(ep) == 0U) {
							USBEndpointDataSend(base, tiva_ep,
									   USB_TRANS_IN_LAST);
						} else {
							USBEndpointDataSend(base, tiva_ep,
									   USB_TRANS_IN);
						}

						udc_ep_set_busy(cfg, true);
						goto unlock;
					}

					/* Transfer complete */
					tiva_c_complete_xfer(dev, cfg, 0);
				} else {
					/* OUT transfer: read data from FIFO */
					uint32_t base = config->base;
					uint32_t tiva_ep = ep_to_tiva(ep);
					struct net_buf *buf = udc_buf_peek(cfg);
					uint32_t count;
					int32_t ret;
#if defined(CONFIG_UDC_TIVA_C_DMA)
					int dma_ch_out = -1;
					bool dma_rx_done = false;
					uint32_t dma_rx_len = 0;
#endif

					if (buf == NULL) {
						/* No buffer queued, flush and continue */
						USBFIFOFlush(base, tiva_ep, USB_EP_DEV_OUT);
						goto unlock;
					}

#if defined(CONFIG_UDC_TIVA_C_DMA)
					/* DMA RX completion: the engine already
					 * moved the packet into the buffer;
					 * the received length follows from
					 * the remaining count register. */
					dma_ch_out = dma_ch_for_ep(ep);
					if (dma_ch_out >= 0 &&
					    data->dma_ch[dma_ch_out].in_use &&
					    (data->dma_pending & BIT(dma_ch_out)) != 0U) {
						uint32_t rem =
							USBDMAChannelCountGet(base, (uint32_t)dma_ch_out);

						dma_rx_len = data->dma_ch[dma_ch_out].count - rem;
						dma_rx_done = true;
						data->dma_pending &= ~BIT(dma_ch_out);
					}
#endif

#if defined(CONFIG_UDC_TIVA_C_DMA)
					if (dma_rx_done) {
						if (dma_rx_len > 0U) {
							net_buf_add(buf, dma_rx_len);
						}
						/* Short packet or full buffer ends
						 * the request; otherwise re-arm. */
						if (dma_rx_len < cfg->mps ||
						    net_buf_tailroom(buf) == 0U) {
							tiva_c_complete_xfer(dev, cfg, 0);
						} else {
							udc_ep_set_busy(cfg, false);
							tiva_c_start_out_xfer(dev, cfg);
						}
						continue;
					}
#endif /* CONFIG_UDC_TIVA_C_DMA */

					count = net_buf_tailroom(buf);
					ret = USBEndpointDataGet(base, tiva_ep,
								  net_buf_tail(buf),
								  &count);
					if (ret == 0 && count > 0U) {
						net_buf_add(buf, count);

						/*
						 * Ack the received data.
						 * For non-EP0, this clears RXPKTRDY.
						 */
						if (ep_idx(ep) != 0U) {
							USBDevEndpointDataAck(base, tiva_ep,
									     false);
						}

						/*
						 * Check if transfer is complete:
						 *   - short packet (< MPS)
						 *   - buffer full
						 */
						if (count < cfg->mps ||
						    net_buf_tailroom(buf) == 0U) {
							tiva_c_complete_xfer(dev, cfg, 0);
						} else {
							/* More data expected, stay busy */
							udc_ep_set_busy(cfg, false);
							tiva_c_start_out_xfer(dev, cfg);
						}
					} else {
						/* Error or no data */
						tiva_c_complete_xfer(dev, cfg, -EIO);
					}
				}
			}
		}

		/* Clear echo bits only for work types whose atomic bitmap is
		 * fully drained; anything posted during this iteration stays
		 * set and wakes the next wait immediately. */
		if ((events & BIT(TIVA_C_EVT_XFER_NEW)) != 0U &&
		    atomic_get(&data->xfer_new) == 0U) {
			k_event_clear(&data->events, BIT(TIVA_C_EVT_XFER_NEW));
		}
		if ((events & BIT(TIVA_C_EVT_XFER_FINISHED)) != 0U &&
		    atomic_get(&data->xfer_finished) == 0U) {
			k_event_clear(&data->events, BIT(TIVA_C_EVT_XFER_FINISHED));
		}
		if ((events & BIT(TIVA_C_EVT_SETUP)) != 0U &&
		    !data->ep0_setup_pending) {
			k_event_clear(&data->events, BIT(TIVA_C_EVT_SETUP));
		}

unlock:
		udc_tiva_c_unlock(dev);
	}
}

/* ------------------------------------------------------------------ */
/* Endpoint API struct                                                  */
/* ------------------------------------------------------------------ */

static int udc_tiva_c_ep_enable(const struct device *dev,
				 struct udc_ep_config *const cfg)
{
	return tiva_c_ep_enable(dev, cfg);
}

static int udc_tiva_c_ep_disable(const struct device *dev,
				  struct udc_ep_config *const cfg)
{
	return tiva_c_ep_disable(dev, cfg);
}

static const struct udc_api udc_tiva_c_api = {
	.lock = udc_tiva_c_lock,
	.unlock = udc_tiva_c_unlock,
	.init = udc_tiva_c_init,
	.enable = udc_tiva_c_enable,
	.disable = udc_tiva_c_disable,
	.shutdown = udc_tiva_c_shutdown,
	.set_address = udc_tiva_c_set_address,
	.host_wakeup = udc_tiva_c_host_wakeup,
	.device_speed = udc_tiva_c_device_speed,
	.ep_enable = udc_tiva_c_ep_enable,
	.ep_disable = udc_tiva_c_ep_disable,
	.ep_set_halt = udc_tiva_c_ep_set_halt,
	.ep_clear_halt = udc_tiva_c_ep_clear_halt,
	.ep_enqueue = udc_tiva_c_ep_enqueue,
	.ep_dequeue = udc_tiva_c_ep_dequeue,
};

/* ------------------------------------------------------------------ */
/* Driver initialization                                                */
/* ------------------------------------------------------------------ */

#define UDC_TIVA_C_EP_CFG_ARRAY(n, dir)				       \
	static struct udc_ep_config					       \
		ep_cfg_##dir##_##n[DT_INST_PROP(n, num_bidir_endpoints)]

#define UDC_TIVA_C_DECL_THREAD(n)					       \
	static K_KERNEL_STACK_DEFINE(udc_tiva_c_stack_##n,		       \
				      CONFIG_UDC_TIVA_C_STACK_SIZE);	       \
	static void udc_tiva_c_thread_##n(const struct device *dev)	       \
	{								       \
		udc_tiva_c_thread(dev);				       \
	}								       \
	static void udc_tiva_c_make_thread_##n(const struct device *dev)      \
	{								       \
		struct udc_tiva_c_data *data = udc_get_private(dev);	       \
									       \
		k_thread_create(&data->thread_data,			       \
				udc_tiva_c_stack_##n,			       \
				K_KERNEL_STACK_SIZEOF(udc_tiva_c_stack_##n),  \
				(k_thread_entry_t)udc_tiva_c_thread_##n,       \
				(void *)dev, NULL, NULL,		       \
				CONFIG_UDC_TIVA_C_THREAD_PRIORITY,	       \
				0, K_NO_WAIT);				       \
		k_thread_name_set(&data->thread_data, "udc_tiva_c");	       \
	}

#define UDC_TIVA_C_DECL_IRQ(n)					       \
	static void udc_tiva_c_isr_##n(const void *arg)		       \
	{								       \
		udc_tiva_c_isr(arg);					       \
	}								       \
	static void udc_tiva_c_irq_enable_##n(const struct device *dev)    \
	{								       \
		static bool connected;					       \
									       \
		if (!connected) {					       \
			IRQ_CONNECT(DT_INST_IRQN(n), 0,			       \
				    udc_tiva_c_isr_##n,			       \
				    DEVICE_DT_INST_GET(n), 0);		       \
			connected = true;				       \
		}							       \
		irq_enable(DT_INST_IRQN(n));				       \
	}								       \
	static void udc_tiva_c_irq_disable_##n(const struct device *dev)   \
	{								       \
		irq_disable(DT_INST_IRQN(n));			       \
	}

static int udc_tiva_c_preinit(const struct device *dev)
{
	const struct udc_tiva_c_config *config = dev->config;
	struct udc_data *data = dev->data;
	int err;
	uint8_t i;

	/* Set device capabilities */
	data->caps.mps0 = UDC_MPS0_64;
	data->caps.rwup = true;
	data->caps.hs = false;
	/*
	 * The controller performs the status OUT stage automatically
	 * after a data IN terminated by DATAEND; the stack must not
	 * enqueue a status OUT buffer.
	 */
	data->caps.out_ack = 1;

	/* Register all endpoints */
	for (i = 0U; i < config->num_of_eps; i++) {
		/* OUT endpoint */
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0U) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = TIVA_C_EP0_MPS;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		}
		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;

		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("failed to register OUT EP%d: %d", i, err);
			return err;
		}

		/* IN endpoint */
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0U) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = TIVA_C_EP0_MPS;
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		}
		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;

		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("failed to register IN EP%d: %d", i, err);
			return err;
		}
	}

	/* Initialize event objects */
	struct udc_tiva_c_data *priv = udc_get_private(dev);

	k_event_init(&priv->events);
	atomic_clear(&priv->xfer_new);
	atomic_clear(&priv->xfer_finished);

	/* Start the driver thread */
	config->make_thread(dev);

	return 0;
}

#define UDC_TIVA_C_DEVICE_DEFINE(n)					       \
	UDC_TIVA_C_EP_CFG_ARRAY(n, in);					       \
	UDC_TIVA_C_EP_CFG_ARRAY(n, out);				       \
	UDC_TIVA_C_DECL_THREAD(n);					       \
	UDC_TIVA_C_DECL_IRQ(n);						       \
	PINCTRL_DT_INST_DEFINE(n);					       \
									       \
	static struct udc_tiva_c_data udc_tiva_c_data_##n;		       \
									       \
	static const struct udc_tiva_c_config udc_tiva_c_config_##n = {	       \
		.base = DT_INST_REG_ADDR(n),				       \
		.periph = SYSCTL_PERIPH_USB0,				       \
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),	       \
		.ep_cfg_in = ep_cfg_in_##n,				       \
		.ep_cfg_out = ep_cfg_out_##n,				       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		       \
		.irq_enable_func = udc_tiva_c_irq_enable_##n,		       \
		.irq_disable_func = udc_tiva_c_irq_disable_##n,		       \
		.make_thread = udc_tiva_c_make_thread_##n,		       \
	};								       \
									       \
	static struct udc_data udc_data_##n = {				       \
		.priv = &udc_tiva_c_data_##n,				       \
	};								       \
									       \
	DEVICE_DT_INST_DEFINE(n, udc_tiva_c_preinit, NULL,		       \
			      &udc_data_##n, &udc_tiva_c_config_##n,	       \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      &udc_tiva_c_api);				       

DT_INST_FOREACH_STATUS_OKAY(UDC_TIVA_C_DEVICE_DEFINE)
