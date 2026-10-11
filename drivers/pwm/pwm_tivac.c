/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_c_gptm_pwm

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>

/* GPTM register offsets */
#define GPTM_TAMR	0x004	/* Timer A Mode */
#define GPTM_TBMR	0x008	/* Timer B Mode */
#define GPTM_CTL	0x00C	/* Control */
#define GPTM_CFG	0x000	/* Configuration */
#define GPTM_TAILR	0x028	/* Timer A Interval Load */
#define GPTM_TBILR	0x02C	/* Timer B Interval Load */
#define GPTM_TAMATCHR	0x030	/* Timer A Match */
#define GPTM_TBMATCHR	0x034	/* Timer B Match */
#define GPTM_TAPR	0x038	/* Timer A Prescale */
#define GPTM_TBPR	0x03C	/* Timer B Prescale */
#define GPTM_TAPMR	0x040	/* Timer A Prescale Match */
#define GPTM_TBPMR	0x044	/* Timer B Prescale Match */

/* GPTM Configuration register values */
#define GPTM_CFG_16BIT		0x4	/* 16-bit timer configuration */

/* GPTM TAMR/TBMR register bits */
#define GPTM_TnMR_TnMR_PERIOD	0x2	/* Periodic Timer mode */
#define GPTM_TnMR_TnAMS		0x8	/* Alternate Mode Select (PWM) */
#define GPTM_TnMR_TnPWMIE	0x200	/* PWM Interrupt Enable */

/* GPTM CTL register bits */
#define GPTM_CTL_TAEN		BIT(0)	/* Timer A Enable */
#define GPTM_CTL_TBEN		BIT(8)	/* Timer B Enable */
#define GPTM_CTL_TAPWML		BIT(6)	/* Timer A PWM Output Level */
#define GPTM_CTL_TBPWML		BIT(14)	/* Timer B PWM Output Level */

struct tiva_c_pwm_config {
	uint32_t base;
	uint32_t clock_frequency;
	const struct pinctrl_dev_config *pincfg;
};

struct tiva_c_pwm_data {
	uint32_t period_cycles;	/* current period in cycles (for both channels) */
	bool channel_enabled[2];
};

static void tiva_c_pwm_configure_channel(const struct device *dev, uint8_t channel)
{
	const struct tiva_c_pwm_config *cfg = dev->config;
	uint32_t base = cfg->base;
	uint32_t tnmr_reg = (channel == 0) ? GPTM_TAMR : GPTM_TBMR;
	uint32_t tnctl_bit = (channel == 0) ? GPTM_CTL_TAEN : GPTM_CTL_TBEN;
	uint32_t val;

	/* Select 16-bit timer configuration */
	sys_write32(GPTM_CFG_16BIT, base + GPTM_CFG);

	/* Configure for periodic PWM mode */
	val = sys_read32(base + tnmr_reg);
	val &= ~GPTM_TnMR_TnMR_PERIOD;	/* clear mode bits */
	val |= GPTM_TnMR_TnMR_PERIOD;	/* periodic mode */
	val |= GPTM_TnMR_TnAMS;		/* alternate mode (PWM) */
	sys_write32(val, base + tnmr_reg);

	/* Disable timer while we configure it */
	val = sys_read32(base + GPTM_CTL);
	val &= ~tnctl_bit;
	sys_write32(val, base + GPTM_CTL);
}

static int tiva_c_pwm_set_cycles(const struct device *dev, uint32_t channel,
				 uint32_t period_cycles,
				 uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct tiva_c_pwm_config *cfg = dev->config;
	struct tiva_c_pwm_data *data = dev->data;
	uint32_t base = cfg->base;
	uint32_t ilr_reg, match_reg, pr_reg, pmr_reg;
	uint32_t ctl_en_bit, ctl_pol_bit;
	uint32_t ilr_val, match_val, pr_val, pmr_val;
	uint32_t ctl;

	if (channel > 1) {
		return -EINVAL;
	}

	if (period_cycles == 0) {
		/* Disable the channel */
		ctl = sys_read32(base + GPTM_CTL);
		ctl &= ~((channel == 0) ? GPTM_CTL_TAEN : GPTM_CTL_TBEN);
		sys_write32(ctl, base + GPTM_CTL);
		data->channel_enabled[channel] = false;
		return 0;
	}

	if (pulse_cycles > period_cycles) {
		return -EINVAL;
	}

	/* Maximum resolution is 24 bits (16-bit counter + 8-bit prescaler) */
	if (period_cycles > 0x00FFFFFFUL) {
		return -ENOTSUP;
	}

	/* Set up register addresses based on channel */
	if (channel == 0) {
		ilr_reg = GPTM_TAILR;
		match_reg = GPTM_TAMATCHR;
		pr_reg = GPTM_TAPR;
		pmr_reg = GPTM_TAPMR;
		ctl_en_bit = GPTM_CTL_TAEN;
		ctl_pol_bit = GPTM_CTL_TAPWML;
	} else {
		ilr_reg = GPTM_TBILR;
		match_reg = GPTM_TBMATCHR;
		pr_reg = GPTM_TBPR;
		pmr_reg = GPTM_TBPMR;
		ctl_en_bit = GPTM_CTL_TBEN;
		ctl_pol_bit = GPTM_CTL_TBPWML;
	}

	/* Make sure channel is configured */
	if (!data->channel_enabled[channel]) {
		tiva_c_pwm_configure_channel(dev, channel);
	}

	/* Split period into prescaler (upper 8 bits) and interval load (lower 16 bits) */
	if (period_cycles <= 0x10000) {
		/* 16-bit mode, no prescaler needed */
		pr_val = 0;
		pmr_val = 0;
		ilr_val = period_cycles - 1;
		match_val = period_cycles - pulse_cycles;
	} else {
		/* Need prescaler for 17..24 bit periods */
		pr_val = (period_cycles >> 16) & 0xFF;
		ilr_val = (period_cycles & 0xFFFF) - 1;

		if (pulse_cycles >= (period_cycles & 0xFFFF)) {
			pmr_val = (pulse_cycles >> 16) & 0xFF;
			match_val = period_cycles - pulse_cycles;
		} else {
			pmr_val = pr_val;
			match_val = ilr_val + 1 - pulse_cycles;
		}
	}

	/* Set PWM polarity */
	ctl = sys_read32(base + GPTM_CTL);
	if ((flags & PWM_POLARITY_INVERTED) != 0) {
		ctl |= ctl_pol_bit;
	} else {
		ctl &= ~ctl_pol_bit;
	}

	/* Write period and match registers */
	sys_write8(pr_val, base + pr_reg);
	sys_write16(ilr_val, base + ilr_reg);
	sys_write8(pmr_val, base + pmr_reg);
	sys_write16(match_val, base + match_reg);

	/* Enable the timer (if not already enabled) */
	ctl |= ctl_en_bit;
	sys_write32(ctl, base + GPTM_CTL);

	data->channel_enabled[channel] = true;
	data->period_cycles = period_cycles;

	return 0;
}

static int tiva_c_pwm_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					 uint64_t *cycles)
{
	const struct tiva_c_pwm_config *cfg = dev->config;

	if (channel > 1) {
		return -EINVAL;
	}

	*cycles = cfg->clock_frequency;
	return 0;
}

static const struct pwm_driver_api tiva_c_pwm_driver_api = {
	.set_cycles = tiva_c_pwm_set_cycles,
	.get_cycles_per_sec = tiva_c_pwm_get_cycles_per_sec,
};

static int tiva_c_pwm_init(const struct device *dev)
{
	const struct tiva_c_pwm_config *cfg = dev->config;
	uint32_t base = cfg->base;
	int ret;

	/*
	 * Enable the timer peripheral clock.
	 *
	 * Note: The TM4C timer clock gating is handled by the SoC init code
	 * (similar to GPIO).  If timers are not already enabled, this driver
	 * will not work and the SoC code needs updating.
	 */

	/* Apply pin control configuration */
	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Set 16-bit timer configuration (required for PWM mode) */
	sys_write32(GPTM_CFG_16BIT, base + GPTM_CFG);

	return 0;
}

#define TIVA_C_PWM_INIT(n)							       \
	PINCTRL_DT_INST_DEFINE(n);						       \
	static struct tiva_c_pwm_data tiva_c_pwm_data_##n = {			       \
		.period_cycles = 0,						       \
		.channel_enabled = {false, false},				       \
	};									       \
										       \
	static const struct tiva_c_pwm_config tiva_c_pwm_config_##n = {		       \
		.base = DT_INST_REG_ADDR(n),					       \
		.clock_frequency = DT_INST_PROP(n, clock_frequency),		       \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			       \
	};									       \
										       \
	DEVICE_DT_INST_DEFINE(n, tiva_c_pwm_init, NULL,				       \
			      &tiva_c_pwm_data_##n, &tiva_c_pwm_config_##n,	       \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,		       \
			      &tiva_c_pwm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TIVA_C_PWM_INIT)
