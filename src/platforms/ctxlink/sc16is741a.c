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
#include <libopencm3/stm32/f4/gpio.h>

#include "platform.h"
#include "sc16is741a.h"

static uint32_t spi_interface;
static uint32_t spi_cs_port;
static uint32_t spi_cs_pin;

/*
    Control the CS line to the SPI device
*/
static void control_cs(bool state)
{
	if (state)
		gpio_clear(spi_cs_port, spi_cs_pin);
	else
		gpio_set(spi_cs_port, spi_cs_pin);
}

/*
    Write a byte to the SPI bus

    This function assumes the chip select for the SPI device has been asserted
*/
static void write_byte(uint8_t data)
{
	spi_write(spi_interface, data);
}

/*
    Read a byte from the SPI bus

    This function assumes the chip select for the SPI device has been asserted
*/
uint8_t read_byte(uint8_t addr)
{
	uint8_t read_register = addr << 3; // Move the register address to the correct position in command byte
	read_register += 0x80;             // Assert the read bit
	control_cs(true);                  // select the SPI device
	write_byte(read_register);         // Send the read command
	uint8_t data = spi_read(spi_interface);
	control_cs(false);
	return data;
}

// Initialize the SC16IS741A device
void sc16is741a_init(uint32_t spi, uint32_t port, uint32_t pin)
{
	spi_interface = spi;
	spi_cs_port = port;
	spi_cs_pin = pin;
	//
	// Do a test read of the IER
	//
	static volatile uint8_t ier_state;
	ier_state = read_byte(SC16IS741A_IER);
	if (ier_state == 0)
		ier_state = 0x01;
	else
		ier_state = 0x02;
	//
	// And the LCR
	//
	uint8_t lcr_state = read_byte(SC16IS741A_LCR);
}

// Set the baud rate of the SC16IS741A
void sc16is741a_set_baud_rate(uint32_t baudrate)
{
	(void)baudrate;
}

// Configure the line settings (word length, stop bits, parity)
void sc16is741a_set_line_config(uint8_t word_length, uint8_t stop_bits, uint8_t parity)
{
	(void)word_length;
	(void)stop_bits;
	(void)parity;
}

// Enable or disable FIFO
void sc16is741a_enable_fifo(uint8_t enable)
{
	(void)enable;
}

// Write a byte of data to the UART
void sc16is741a_write_byte(uint8_t data)
{
	(void)data;
}

// Read a byte of data from the UART
uint8_t sc16is741a_read_byte(void)
{
	return 0;
}

// Read the status register of the UART
uint8_t sc16is741a_read_status(void)
{
	return 0;
}

// Enable specific interrupts
void sc16is741a_enable_interrupts(uint8_t interrupts)
{
	(void)interrupts;
}

// Disable specific interrupts
void sc16is741a_disable_interrupts(uint8_t interrupts)
{
	(void)interrupts;
}

// Reset the Wi-Fi Module
void sc16is741a_reset(void)
{
	gpio_clear(WINC1500_RESET_PORT, WINC1500_RESET);
	platform_delay(1) ;
	gpio_set(WINC1500_RESET_PORT, WINC1500_RESET);
}