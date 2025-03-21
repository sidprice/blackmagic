/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2025 Sid Price <sid@sidprice.com>
 * Written by Sid Price <sid@sidprice.com>
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

#include "general.h"
#include "morse.h"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>

#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/stm32/f4/spi.h>

#include "wifi_hardware.h"
#include "WiFi_Server.h"

#include "platform.h"

void wifi_hardware_init(void)
{
	//
	// Initialize the WINC1500 interface hardware
	//
	rcc_periph_clock_enable(WINC1500_RCC_SPI);
	//
	// Negate all outputs to WINC1500
	//
	gpio_set(WINC1500_RESET_PORT, WINC1500_RESET);
	gpio_set(WINC1500_PORT, WINC1500_SPI_NCS);
	gpio_clear(WINC1500_CHIP_EN_PORT, WINC1500_CHIP_EN);
	//
	// Set up the control outputs for the WINC1500
	//
	//		RESET output
	//
	gpio_mode_setup(WINC1500_RESET_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, WINC1500_RESET);
	//
	//		Chip select output
	//
	gpio_mode_setup(WINC1500_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, WINC1500_SPI_NCS);
	//
	//		CHIP_EN Output
	//
	gpio_mode_setup(WINC1500_CHIP_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, WINC1500_CHIP_EN);
	//
	// Need to make the irq pin an external interrupt on falling edge
	//
	//	First enable the SYSCFG clock
	//
	rcc_periph_clock_enable(RCC_SYSCFG);

	gpio_mode_setup(WINC1500_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, WINC1500_IRQ); // Input signal with pulldown

	exti_select_source(WINC1500_IRQ, WINC1500_PORT);
	//
	// Set the port pins of the SPI channel to high-speed I/O
	//
#ifndef CTXLINK_UBLOX_WIFI
	exti_set_trigger(WINC1500_IRQ, EXTI_TRIGGER_FALLING);
	gpio_set_output_options(WINC1500_SPI_DATA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
		WINC1500_SPI_CLK | WINC1500_SPI_MISO | WINC1500_SPI_MOSI | WINC1500_RESET);
#else
	exti_set_trigger(WINC1500_IRQ, EXTI_TRIGGER_RISING);
// #else
// 	gpio_set_output_options(WINC1500_SPI_DATA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
// 		WINC1500_SPI_CLK | WINC1500_SPI_MISO | WINC1500_SPI_MOSI | WINC1500_RESET);
#endif
	//
	// Enable alternate function for SPI2_CLK PB10 AF5
	//
	gpio_mode_setup(WINC1500_SPI_CLK_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, WINC1500_SPI_CLK);
	gpio_set_af(WINC1500_SPI_CLK_PORT, GPIO_AF5, WINC1500_SPI_CLK);
	//
	// Enable SPI alternate function pins - MISO and MOSI
	//
	gpio_mode_setup(WINC1500_SPI_DATA_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, WINC1500_SPI_MISO | WINC1500_SPI_MOSI);
	gpio_set_af(WINC1500_SPI_DATA_PORT, GPIO_AF5, WINC1500_SPI_MISO | WINC1500_SPI_MOSI);
	//
	// I think this is Mode_0, 8 bit data, MSB first, the clock rate is 42MHz with core of 84MHz
	//
#ifndef CTXLINK_UBLOX_WIFI
	// WINC1500 module has 21MHz SPI CLK
	spi_init_master(WINC1500_SPI_CHANNEL, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
#else
	// uBlox Wi-Fi has 2.62MHz SPI CLK, could be up to 4MHz, dividers don't work for that
	spi_init_master(WINC1500_SPI_CHANNEL, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
#endif
	//
	// Set NSS to software management and also ensure NSS is high, if not written high no data will be sent
	//
	spi_enable_software_slave_management(WINC1500_SPI_CHANNEL);
	spi_set_nss_high(WINC1500_SPI_CHANNEL);
	//
	// Enable the SPI channel
	//
	spi_enable(WINC1500_SPI_CHANNEL);
	exti_enable_request(WINC1500_IRQ);
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);
}
