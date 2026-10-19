.. zephyr:board:: ikea_tradfri

Overview
********

The IKEA TRÅDFRI smart-home line (light bulbs, dimmers, remotes) is built around
Silicon Labs EFR32MG1P Mighty Gecko SoCs, most commonly the
EFR32MG1P132F256GM32 (QFN32, 256 kB flash, 32 kB RAM) with a 2.4 GHz radio
driving a ZigBee mesh.

This board definition targets the bare TRÅDFRI module: a Cortex-M4F core with
the serial console on USART0 (TX on PB15, RX on PB14) and an indicator LED on
PB13. It serves as a base for bringing up Zephyr on the module before board
specific peripherals are configured per product.

Hardware
********

- EFR32MG1P132F256GM32 Gecko SoC
- CPU core: ARM Cortex-M4 with FPU
- Flash memory: 256 kB
- RAM: 32 kB
- Radio: 2.4 GHz (802.15.4 / Zigbee)
- Crystal for HFXO (38.4 MHz)

For more information about the EFR32MG1 SoC:

- `EFR32MG1 Website`_
- `EFR32MG1 Datasheet`_
- `EFR32xG1 Reference Manual`_

Supported Features
==================

.. zephyr:board-supported-hw::

System Clock
============

The EFR32MG1P SoC is configured to use the 38.4 MHz external oscillator on the
board, with the internal 32.768 kHz LFRCO for the low-frequency clock.

Serial Port
===========

USART0 is used for the console at 115200 baud. TX is routed to PB15 and RX to
PB14.

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Flashing
========

This board supports jlink and openocd. Solder wires to SWDIO (PF1), SWCLK (PF0),
nRST, GND and VCC (3.3V) and connect them to a debug adapter of your choice.

Here is an example for the :zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: ikea_tradfri
   :goals: flash

Open a serial terminal (minicom, putty, etc.) at 115200 8N1. Reset the board and
you should see the following message in the terminal:

.. code-block:: console

   Hello World! ikea_tradfri/efr32mg1p132f256gm32


.. _EFR32MG1 Website:
   https://www.silabs.com/wireless/zigbee/efr32mg1-series-1-socs

.. _EFR32MG1 Datasheet:
   https://www.silabs.com/documents/public/data-sheets/efr32mg1-datasheet.pdf

.. _EFR32xG1 Reference Manual:
   https://www.silabs.com/documents/public/reference-manuals/efr32xg1-rm.pdf