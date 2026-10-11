/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_c_hib_rtc

#include <errno.h>
#include <time.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/timeutil.h>

/* Hibernation module register offsets */
#define HIB_RTCC	0x000	/* RTC Counter */
#define HIB_RTCM0	0x004	/* RTC Match 0 */
#define HIB_RTCLD	0x00C	/* RTC Load */
#define HIB_CTL		0x010	/* Control */
#define HIB_IM		0x014	/* Interrupt Mask */
#define HIB_RIS		0x018	/* Raw Interrupt Status */
#define HIB_MIS		0x01C	/* Masked Interrupt Status */
#define HIB_IC		0x020	/* Interrupt Clear */
#define HIB_RTCT	0x024	/* RTC Trim */
#define HIB_RTCSS	0x028	/* RTC Sub-Seconds */

/* HIB_CTL register bits */
#define HIB_CTL_WRC		BIT(31)	/* Write Complete/Capable */
#define HIB_CTL_OSCSEL		BIT(19)	/* Oscillator Select */
#define HIB_CTL_OSCDRV		BIT(17)	/* Oscillator Drive */
#define HIB_CTL_CLK32EN		BIT(6)	/* Clocking Enable */
#define HIB_CTL_RTCEN		BIT(0)	/* RTC Timer Enable */

struct tiva_c_rtc_config {
	uint32_t base;
	uint32_t clock_frequency;
};

static void hib_wait_wrc(uint32_t base)
{
	/* Wait for the previous write to complete */
	while ((sys_read32(base + HIB_CTL) & HIB_CTL_WRC) == 0) {
	}
}

static int tiva_c_rtc_set_time(const struct device *dev,
			       const struct rtc_time *timeptr)
{
	const struct tiva_c_rtc_config *cfg = dev->config;
	uint32_t base = cfg->base;
	struct tm tm;
	int64_t secs;

	/* Convert rtc_time to struct tm for timeutil */
	tm.tm_sec = timeptr->tm_sec;
	tm.tm_min = timeptr->tm_min;
	tm.tm_hour = timeptr->tm_hour;
	tm.tm_mday = timeptr->tm_mday;
	tm.tm_mon = timeptr->tm_mon;
	tm.tm_year = timeptr->tm_year;
	tm.tm_wday = timeptr->tm_wday;
	tm.tm_yday = timeptr->tm_yday;
	tm.tm_isdst = timeptr->tm_isdst;

	secs = timeutil_timegm64(&tm);
	if (secs < 0 || secs > UINT32_MAX) {
		return -EINVAL;
	}

	/* Write the load register to set the counter */
	hib_wait_wrc(base);
	sys_write32((uint32_t)secs, base + HIB_RTCLD);
	hib_wait_wrc(base);

	return 0;
}

static int tiva_c_rtc_get_time(const struct device *dev,
			       struct rtc_time *timeptr)
{
	const struct tiva_c_rtc_config *cfg = dev->config;
	uint32_t base = cfg->base;
	uint32_t secs;
	struct tm tm;
	time_t t;

	secs = sys_read32(base + HIB_RTCC);

	t = (time_t)secs;
	gmtime_r(&t, &tm);

	timeptr->tm_sec = tm.tm_sec;
	timeptr->tm_min = tm.tm_min;
	timeptr->tm_hour = tm.tm_hour;
	timeptr->tm_mday = tm.tm_mday;
	timeptr->tm_mon = tm.tm_mon;
	timeptr->tm_year = tm.tm_year;
	timeptr->tm_wday = tm.tm_wday;
	timeptr->tm_yday = tm.tm_yday;
	timeptr->tm_isdst = tm.tm_isdst;
	timeptr->tm_nsec = 0;

	return 0;
}

static const struct rtc_driver_api tiva_c_rtc_api = {
	.set_time = tiva_c_rtc_set_time,
	.get_time = tiva_c_rtc_get_time,
};

static int tiva_c_rtc_init(const struct device *dev)
{
	const struct tiva_c_rtc_config *cfg = dev->config;
	uint32_t base = cfg->base;
	uint32_t ctl;

	/*
	 * Configure and enable the 32 kHz clock source.
	 *
	 * Use the external low-frequency oscillator (LPOsc) with a
	 * 32.768 kHz crystal (default on the tm4c123gxl LaunchPad).
	 * OSCSEL=0 selects LPOsc, OSCDRV=1 sets 12 pF drive strength.
	 */
	ctl = sys_read32(base + HIB_CTL);
	ctl &= ~HIB_CTL_OSCSEL;		/* LPOsc (external 32.768 kHz) */
	ctl |= HIB_CTL_OSCDRV;		/* 12 pF drive strength */
	ctl |= HIB_CTL_CLK32EN;		/* Enable 32 kHz clocking */

	hib_wait_wrc(base);
	sys_write32(ctl, base + HIB_CTL);
	hib_wait_wrc(base);

	/* Enable the RTC counter */
	ctl = sys_read32(base + HIB_CTL);
	ctl |= HIB_CTL_RTCEN;

	hib_wait_wrc(base);
	sys_write32(ctl, base + HIB_CTL);
	hib_wait_wrc(base);

	return 0;
}

#define TIVA_C_RTC_INIT(n)						       \
	static const struct tiva_c_rtc_config tiva_c_rtc_config_##n = {	       \
		.base = DT_INST_REG_ADDR(n),				       \
		.clock_frequency = DT_INST_PROP(n, clock_frequency),	       \
	};								       \
									       \
	DEVICE_DT_INST_DEFINE(n, tiva_c_rtc_init, NULL, NULL,		       \
			      &tiva_c_rtc_config_##n, PRE_KERNEL_1,	       \
			      CONFIG_RTC_INIT_PRIORITY, &tiva_c_rtc_api);

DT_INST_FOREACH_STATUS_OKAY(TIVA_C_RTC_INIT)
