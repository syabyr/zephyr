/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2026 Linumiz
 * Author: Sri Surya <srisurya@linumiz.com>
 */

/*
 * SoC initialization for the TI Tiva C TM4C123x Series
 *
 * Configures the PLL to run at 80 MHz from the 16 MHz main oscillator
 * and enables the GPIO port peripherals (A-F) so that GPIO drivers can
 * access their registers without causing a bus fault.
 */

#include <zephyr/init.h>

#include "soc.h"

void soc_early_init_hook(void)
{
	/* Configure PLL: 80 MHz from 16 MHz crystal */
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
			SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	/*
	 * Enable all GPIO port peripherals (A-F).
	 *
	 * The Stellaris GPIO driver accesses registers directly and does
	 * not enable the peripheral clock itself, so we must turn them on
	 * here before any GPIO driver initialization runs (POST_KERNEL).
	 * Without this, the first register access causes a precise bus fault.
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	/* Wait for all ports to be ready (takes a few cycles after enable) */
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA) ||
	       !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB) ||
	       !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC) ||
	       !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD) ||
	       !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE) ||
	       !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {
	}
}
