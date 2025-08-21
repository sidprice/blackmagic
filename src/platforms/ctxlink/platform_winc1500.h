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

/* This file provides the platform specific declarations for the ctxLink v1.x WINC1500 implementation. */

#ifndef PLATFORMS_CTXLINK_PLATFORM_WINC1500_H
#define PLATFORMS_CTXLINK_PLATFORM_WINC1500_H
//
//	Debug port definitions
//
#define JTAG_PORT    GPIOA
#define TDI_PORT     JTAG_PORT
#define TMS_PORT     JTAG_PORT
#define TCK_PORT     JTAG_PORT
#define TMS_DIR_PORT JTAG_PORT
#define TDO_PORT     GPIOC
#define TDI_PIN      GPIO3
#define TMS_PIN      GPIO4
#define TMS_DIR_PIN  GPIO1
#define TCK_PIN      GPIO5
#define TDO_PIN      GPIO7

#define SWDIO_PORT     JTAG_PORT
#define SWCLK_PORT     JTAG_PORT
#define SWDIO_DIR_PORT JTAG_PORT
#define SWDIO_PIN      TMS_PIN
#define SWCLK_PIN      TCK_PIN
#define SWDIO_DIR_PIN  TMS_DIR_PIN

// Port definitions for WINC1500 wireless module
//
//		The WINC1500 is attached to SPI_2
//
#define WINC1500_SPI_CHANNEL SPI2
#define WINC1500_RCC_SPI     RCC_SPI2

#define WINC1500_PORT    GPIOB  // Port for CS and IRQ
#define WINC1500_SPI_NCS GPIO15 // Chip select
#define WINC1500_IRQ     GPIO9  // IRQ input
//
// Reset port and pin
//
#define WINC1500_RESET_PORT GPIOB
#define WINC1500_RESET      GPIO14 // Reset output

//
// Chip enable port and pin
//
#define WINC1500_CHIP_EN_PORT GPIOB
#define WINC1500_CHIP_EN      GPIO13

//
// SPI clock port
//
#define WINC1500_SPI_CLK_PORT GPIOB
#define WINC1500_SPI_CLK      GPIO10
//
// SPI Data port
//
#define WINC1500_SPI_DATA_PORT GPIOC
#define WINC1500_SPI_MISO      GPIO2
#define WINC1500_SPI_MOSI      GPIO3

#define TRST_PORT       GPIOA
#define TRST_PIN        GPIO2
#define NRST_PORT       GPIOA
#define NRST_PIN        GPIO2
#define NRST_SENSE_PORT GPIOA
#define NRST_SENSE_PIN  GPIO7

#define SWO_PORT GPIOC
#define SWO_PIN  GPIO7

#define LED_PORT GPIOC

#define LED_IDLE_RUN GPIO6
#define LED_ERROR    GPIO8
#define LED_MODE     GPIO9

#define LED_PORT_UART GPIOB
#define LED_UART      GPIO2

#define SWITCH_PORT       GPIOB
#define SW_BOOTLOADER_PIN GPIO12

#define TPWR_PORT   GPIOB
#define TPWR_PIN    GPIO0
#define VBAT_PORT   GPIOA
#define VBAT_PIN    GPIO0
#define PWR_BR_PORT GPIOB
#define PWR_BR_PIN  GPIO1

/* USB pin definitions */
#define USB_PU_PORT GPIOB
#define USB_PORT    GPIOA
#define USB_PU_PIN  GPIO13
#define USB_DP_PIN  GPIO12
#define USB_DM_PIN  GPIO11

#endif /* PLATFORMS_CTXLINK_PLATFORM_WINC1500_H */