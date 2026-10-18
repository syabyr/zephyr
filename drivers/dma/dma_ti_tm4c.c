/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_c_udma

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include <driverlib/sysctl.h>
#include <driverlib/udma.h>
#include <driverlib/interrupt.h>
#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_udma.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_types.h>

LOG_MODULE_REGISTER(dma_ti_tm4c, CONFIG_DMA_LOG_LEVEL);

#define TI_TM4C_DMA_NUM_CHANNELS  32

/*
 * The uDMA channel control table: 64 entries (primary + alternate for
 * each of the 32 channels), 16 bytes each, and the base must be
 * 1024-byte aligned.
 */
static tDMAControlTable control_table[TI_TM4C_DMA_NUM_CHANNELS * 2]
	__aligned(1024);

enum channel_owner {
	CHANNEL_UNUSED,
	CHANNEL_SW,      /* software / memory-to-memory, driver owns it */
	CHANNEL_PERIPH,  /* hardwired peripheral request, client owns it */
};

struct dma_ti_channel {
	dma_callback_t callback;
	void *user_data;
	struct dma_config cfg;
	enum channel_owner owner;
};

struct dma_ti_data {
	struct dma_context dma_ctx;
	atomic_t chan_atomic;
	struct dma_ti_channel channels[TI_TM4C_DMA_NUM_CHANNELS];
};

static struct dma_ti_data dma_data_0 = {
	.dma_ctx = {
		.magic = DMA_MAGIC,
		.dma_channels = TI_TM4C_DMA_NUM_CHANNELS,
		.atomic = &dma_data_0.chan_atomic,
	},
};

/*
 * Software channels by preference: the dedicated software channel 30
 * plus channels not wired to a peripheral on this part (12, 13, 26-29,
 * 31) so memory-to-memory transfers leave the peripheral lines alone.
 */
static const uint8_t sw_channel_order[] = { 30, 12, 13, 26, 27, 28, 29, 31 };

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static int dma_ti_alloc_sw_channel(struct dma_ti_data *data)
{
	for (size_t i = 0; i < ARRAY_SIZE(sw_channel_order); i++) {
		uint8_t ch = sw_channel_order[i];

		if (!atomic_test_and_set_bit(&data->chan_atomic, ch)) {
			return ch;
		}
	}

	return -ENODEV;
}

static uint32_t dma_ti_width_bits(uint16_t width)
{
	switch (width) {
	case 1U:
		return UDMA_SIZE_8;
	case 2U:
		return UDMA_SIZE_16;
	case 4U:
		return UDMA_SIZE_32;
	default:
		return 0U;
	}
}

static uint32_t dma_ti_inc_bits(uint16_t adj, uint16_t width)
{
	bool no_change = (adj == 0b10U);

	if (no_change) {
		return UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE;
	}
	if (adj != 0b00U) {
		/* decrement is not supported */
		return 0U;
	}

	switch (width) {
	case 1U:
		return UDMA_SRC_INC_8 | UDMA_DST_INC_8;
	case 2U:
		return UDMA_SRC_INC_16 | UDMA_DST_INC_16;
	case 4U:
		return UDMA_SRC_INC_32 | UDMA_DST_INC_32;
	default:
		return 0U;
	}
}

/* Program one block into the channel control structures. */
static int dma_ti_setup_block(const struct device *dev, uint32_t channel,
			      const struct dma_config *cfg)
{
	struct dma_ti_data *data = dev->data;
	const struct dma_block_config *block = cfg->head_block;
	struct dma_ti_channel *ch = &data->channels[channel];
	uint32_t ctl = 0;
	uint32_t mode;
	uint32_t items;

	uint16_t src_width = cfg->source_data_size;
	uint16_t dst_width = cfg->dest_data_size;

	if (src_width != dst_width) {
		LOG_ERR("source/dest width mismatch (%u vs %u)",
			src_width, dst_width);
		return -EINVAL;
	}

	uint32_t size_bits = dma_ti_width_bits(src_width);
	uint32_t inc_bits = dma_ti_inc_bits(
		block->source_addr_adj, src_width);

	/*
	 * UDMA_SIZE_8 is 0x00000000, so a zero size_bits does not signal
	 * an error; validate the width itself instead.
	 */
	if (src_width == 0U || src_width > 4U || (src_width & (src_width - 1U)) != 0U) {
		LOG_ERR("unsupported width %u", src_width);
		return -ENOTSUP;
	}
	if (inc_bits == 0U) {
		LOG_ERR("unsupported address adjust");
		return -ENOTSUP;
	}

	if (block->source_gather_en || block->dest_scatter_en) {
		return -ENOTSUP;
	}

	if (block->next_block != NULL) {
		/* scatter-gather lists are not supported yet */
		return -ENOTSUP;
	}

	if (block->block_size % src_width != 0U) {
		LOG_ERR("block size %u not a multiple of width %u",
			block->block_size, src_width);
		return -EINVAL;
	}

	items = block->block_size / src_width;
	if (items == 0U || items > 1024U) {
		LOG_ERR("item count %u out of range (1..1024)", items);
		return -EINVAL;
	}

	if (cfg->channel_direction == MEMORY_TO_MEMORY) {
		mode = UDMA_MODE_AUTO;
	} else {
		mode = UDMA_MODE_BASIC;
	}

	ctl = inc_bits | size_bits | UDMA_ARB_4 |
	      ((cfg->channel_priority != 0U) ? 0U : 0U);

	uDMAChannelControlSet(channel | UDMA_PRI_SELECT, ctl);
	uDMAChannelTransferSet(channel | UDMA_PRI_SELECT, mode,
			       (void *)(uintptr_t)block->source_address,
			       (void *)(uintptr_t)block->dest_address,
			       items);

	/* Clear any completion status left over from a previous run */
	uDMAIntClear(BIT(channel));

	return 0;
}

/* ------------------------------------------------------------------ */
/* DMA driver API                                                      */
/* ------------------------------------------------------------------ */

static int dma_ti_config(const struct device *dev, uint32_t channel,
			 struct dma_config *cfg)
{
	struct dma_ti_data *data = dev->data;
	struct dma_ti_channel *ch;

	if (channel >= TI_TM4C_DMA_NUM_CHANNELS) {
		return -EINVAL;
	}

	if (cfg == NULL || cfg->head_block == NULL) {
		return -EINVAL;
	}

	if (cfg->source_data_size != cfg->dest_data_size) {
		return -EINVAL;
	}

	if (cfg->cyclic != 0U) {
		return -ENOTSUP;
	}

	switch (cfg->channel_direction) {
	case MEMORY_TO_MEMORY:
	case MEMORY_TO_PERIPHERAL:
	case PERIPHERAL_TO_MEMORY:
	case PERIPHERAL_TO_PERIPHERAL:
		break;
	default:
		return -ENOTSUP;
	}

	ch = &data->channels[channel];

	/*
	 * dma_slot selects the hardwired peripheral request line and must
	 * match the channel number it is wired to (they are identical on
	 * this controller). Memory to memory transfers ignore dma_slot.
	 */
	if (cfg->channel_direction != MEMORY_TO_MEMORY &&
	    cfg->dma_slot != channel) {
		LOG_ERR("channel %u does not carry request line %u",
			channel, cfg->dma_slot);
		return -EINVAL;
	}

	/* Claim the channel on first configuration. dma_request_channel()
	 * has already set the atomic bit for dynamically allocated
	 * channels, so only reject a claim when neither the atomic bit
	 * nor the owner marks the channel as ours. */
	if (ch->owner == CHANNEL_UNUSED) {
		if (atomic_test_and_set_bit(&data->chan_atomic, channel) &&
		    !atomic_test_bit(&data->chan_atomic, channel)) {
			LOG_ERR("channel %u already in use", channel);
			return -EBUSY;
		}
		ch->owner = (cfg->channel_direction == MEMORY_TO_MEMORY) ?
			    CHANNEL_SW : CHANNEL_PERIPH;
	}

	ch->cfg = *cfg;
	ch->callback = cfg->dma_callback;
	ch->user_data = cfg->user_data;

	return 0;
}

static int dma_ti_start(const struct device *dev, uint32_t channel)
{
	struct dma_ti_data *data = dev->data;
	struct dma_ti_channel *ch;
	int ret;

	if (channel >= TI_TM4C_DMA_NUM_CHANNELS) {
		return -EINVAL;
	}

	ch = &data->channels[channel];
	if (ch->owner == CHANNEL_UNUSED) {
		return -EINVAL;
	}

	if (uDMAChannelIsEnabled(channel)) {
		LOG_WRN("channel %u already enabled", channel);
		return -EBUSY;
	}

	ret = dma_ti_setup_block(dev, channel, &ch->cfg);
	if (ret != 0) {
		return ret;
	}

	uDMAChannelEnable(channel);

	if (ch->cfg.channel_direction == MEMORY_TO_MEMORY) {
		/* AUTO mode: software request kicks the transfer off */
		uDMAChannelRequest(channel);
	}

	return 0;
}

static int dma_ti_stop(const struct device *dev, uint32_t channel)
{
	if (channel >= TI_TM4C_DMA_NUM_CHANNELS) {
		return -EINVAL;
	}

	uDMAChannelDisable(channel);

	return 0;
}

static int dma_ti_reload(const struct device *dev, uint32_t channel,
			 uint32_t src, uint32_t dst, size_t size)
{
	struct dma_ti_data *data = dev->data;
	struct dma_ti_channel *ch;
	struct dma_config *cfg;
	struct dma_block_config *block;

	if (channel >= TI_TM4C_DMA_NUM_CHANNELS) {
		return -EINVAL;
	}

	ch = &data->channels[channel];
	cfg = &ch->cfg;
	block = cfg->head_block;

	if (block == NULL) {
		return -EINVAL;
	}

	block->source_address = src;
	block->dest_address = dst;
	block->block_size = size;

	uDMAChannelDisable(channel);

	return dma_ti_start(dev, channel);
}

static int dma_ti_get_status(const struct device *dev, uint32_t channel,
			     struct dma_status *status)
{
	struct dma_ti_data *data = dev->data;
	struct dma_ti_channel *ch;
	uint32_t remaining;
	uint32_t width;

	if (channel >= TI_TM4C_DMA_NUM_CHANNELS || status == NULL) {
		return -EINVAL;
	}

	ch = &data->channels[channel];
	width = ch->cfg.source_data_size;
	if (width == 0U) {
		width = 1U;
	}

	status->busy = uDMAChannelIsEnabled(channel);

	remaining = uDMAChannelSizeGet(channel | UDMA_PRI_SELECT);
	status->pending_length = remaining * width;

	return 0;
}

/* ------------------------------------------------------------------ */
/* Interrupt handling                                                  */
/* ------------------------------------------------------------------ */

static void dma_ti_sw_isr(const struct device *dev)
{
	struct dma_ti_data *data = dev->data;
	uint32_t pending = uDMAIntStatus();

	uDMAIntClear(pending);

	for (uint8_t ch = 0U; ch < TI_TM4C_DMA_NUM_CHANNELS; ch++) {
		struct dma_ti_channel *chdata = &data->channels[ch];

		if ((pending & BIT(ch)) == 0U) {
			continue;
		}

		if (chdata->owner == CHANNEL_SW && chdata->callback != NULL) {
			chdata->callback(dev, chdata->user_data, ch, 0);
		} else if (chdata->callback != NULL) {
			/*
			 * A peripheral channel completing here (its own
			 * interrupt is late or unused): report as well.
			 */
			chdata->callback(dev, chdata->user_data, ch, 0);
		}
	}
}

static void dma_ti_err_isr(const struct device *dev)
{
	struct dma_ti_data *data = dev->data;
	uint32_t err = uDMAErrorStatusGet();

	uDMAErrorStatusClear();

	LOG_ERR("uDMA bus error 0x%08x", err);

	/* Report the error on every owned channel so clients can retry */
	for (uint8_t ch = 0U; ch < TI_TM4C_DMA_NUM_CHANNELS; ch++) {
		struct dma_ti_channel *chdata = &data->channels[ch];

		if (chdata->owner != CHANNEL_UNUSED && chdata->callback != NULL) {
			uDMAChannelDisable(ch);
			chdata->callback(dev, chdata->user_data, ch, -EIO);
		}
	}
}

/* ------------------------------------------------------------------ */
/* Channel allocation (dma_context based)                              */
/* ------------------------------------------------------------------ */

static bool dma_ti_chan_filter(const struct device *dev, int ch_id,
			       void *param)
{
	/* Software channels only hand out the request lines that are not
	 * wired to a peripheral. */
	ARG_UNUSED(dev);
	ARG_UNUSED(param);

	return (ch_id == 30) || (ch_id == 12) || (ch_id == 13) ||
	       (ch_id >= 26);
}

static void dma_ti_chan_release(const struct device *dev, uint32_t channel)
{
	struct dma_ti_data *data = dev->data;

	if (channel >= TI_TM4C_DMA_NUM_CHANNELS) {
		return;
	}

	uDMAChannelDisable(channel);
	atomic_clear_bit(&data->chan_atomic, channel);
	data->channels[channel].owner = CHANNEL_UNUSED;
}

static const struct dma_driver_api dma_ti_api = {
	.config = dma_ti_config,
	.start = dma_ti_start,
	.stop = dma_ti_stop,
	.reload = dma_ti_reload,
	.get_status = dma_ti_get_status,
	.chan_filter = dma_ti_chan_filter,
	.chan_release = dma_ti_chan_release,
};

/* ------------------------------------------------------------------ */
/* Initialization                                                      */
/* ------------------------------------------------------------------ */

#define TI_TM4C_INIT_FN(n)						       \
	static void dma_ti_isr_##n(const void *arg)			       \
	{								       \
		dma_ti_sw_isr(arg);					       \
	}								       \
	static void dma_ti_err_isr_##n(const void *arg)			       \
	{								       \
		dma_ti_err_isr(arg);					       \
	}								       \
	static int dma_ti_init_##n(const struct device *dev)		       \
	{								       \
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);		       \
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA)) {	       \
		}							       									       		/* Reset the controller through SRCR2 first: without this the \
		 * UDMA_CFG write below is silently dropped (same behaviour as \
		 * the USB CC register on this part). */		       \
		HWREG(SYSCTL_SRCR2) |= SYSCTL_SRCR2_UDMA;		       		k_busy_wait(1);					       \
		HWREG(SYSCTL_SRCR2) &= ~SYSCTL_SRCR2_UDMA;		       \
									       \
		memset(control_table, 0, sizeof(control_table));	       \
		uDMAControlBaseSet(control_table);			       \
		uDMAEnable();						       \
									       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq),		       \
			    DT_INST_IRQ_BY_IDX(n, 0, priority),		       \
			    dma_ti_isr_##n, DEVICE_DT_INST_GET(n), 0);	       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 1, irq),		       \
			    DT_INST_IRQ_BY_IDX(n, 1, priority),		       \
			    dma_ti_err_isr_##n, DEVICE_DT_INST_GET(n), 0);     \
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));		       \
		irq_enable(DT_INST_IRQ_BY_IDX(n, 1, irq));		       \
									       \
		return 0;						       \
	}								       \
	DEVICE_DT_INST_DEFINE(n, dma_ti_init_##n, NULL,		       \
			      &dma_data_0, NULL, POST_KERNEL,		       \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	       \
			      &dma_ti_api);

DT_INST_FOREACH_STATUS_OKAY(TI_TM4C_INIT_FN)
