/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * Updates for ctxLink Copyright (C) 2025 Sid Price - sid@sidprice.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file provides the platform specific declarations for the ctxLink v2.x ESP32-S3 implementation. */

#ifndef PLATFORMS_CTXLINK_PLATFORM_ESP32S3_H
#define PLATFORMS_CTXLINK_PLATFORM_ESP32S3_H

//
//	Debug port definitions
//
#define JTAG_PORT    GPIOB
#define TDI_PORT     JTAG_PORT
#define TMS_PORT     JTAG_PORT
#define TCK_PORT     JTAG_PORT
#define TMS_DIR_PORT JTAG_PORT
#define TDO_PORT     GPIOA
#define TDI_PIN      GPIO6
#define TMS_PIN      GPIO5
#define TMS_DIR_PIN  GPIO7
#define TCK_PIN      GPIO5
#define TDO_PIN      GPIO15

#define SWDIO_PORT     JTAG_PORT
#define SWCLK_PORT     JTAG_PORT
#define SWDIO_DIR_PORT JTAG_PORT
#define SWDIO_PIN      TMS_PIN
#define SWCLK_PIN      TCK_PIN
#define SWDIO_DIR_PIN  TMS_DIR_PIN
// Port definitions for ESP32 wireless module
//
//		The ESP32 is attached to SPI_1
//
#define ESP32_SPI_CHANNEL SPI1
#define ESP32_RCC_SPI     RCC_SPI1

#define ESP32_SPI_NCS_PORT GPIOA
#define ESP32_SPI_NCS      GPIO4

#define ESP32_NATTN_PORT GPIOB
#define ESP32_nATTN      GPIO2

#define ESP32_nSPI_READY_PORT GPIOB
#define ESP32_nSPI_READY      GPIO10

#define ESP32_nREADY_PORT GPIOB
#define ESP32_nREADY      GPIO1

#define ESP32_nRESET_PORT GPIOB
#define ESP32_nRESET      GPIO0 // Reset output

#define ESP32_SPI_CLK_PORT GPIOA
#define ESP32_SPI_CLK      GPIO5

#define ESP32_SPI_DATA_PORT GPIOA
#define ESP32_SPI_MISO      GPIO6
#define ESP32_SPI_MOSI      GPIO7

#define TRST_PORT GPIOB
#define TRST_PIN  GPIO9

#define NRST_PORT GPIOB
#define NRST_PIN  GPIO9

#define NRST_SENSE_PORT GPIOB
#define NRST_SENSE_PIN  GPIO8

#define SWO_PORT GPIOA
#define SWO_PIN  GPIO15

#define LED_PORT     GPIOC
#define LED_IDLE_RUN GPIO14
#define LED_ERROR    GPIO15

#define LED_PORT_UART GPIOC
#define LED_UART      GPIO13

#define MODE_LED_PORT GPIOA
#define LED_MODE      GPIO3

#define SWITCH_PORT       GPIOB
#define SW_BOOTLOADER_PIN GPIO12

#define TPWR_PORT   GPIOA
#define TPWR_PIN    GPIO2
#define VBAT_PORT   GPIOA
#define VBAT_PIN    GPIO1
#define PWR_BR_PORT GPIOA
#define PWR_BR_PIN  GPIO0

/* USB pin definitions */
#define USB_PU_PORT GPIOA
#define USB_PORT    GPIOA
#define USB_PU_PIN  GPIO8
#define USB_DP_PIN  GPIO12
#define USB_DM_PIN  GPIO11

#endif
