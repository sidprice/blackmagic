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

static _Atomic bool wifi_awake = false;

/*
	The uBlox module has initialized the SPI interface when
	MISO is low, so include that in the test for data ready
 */
void ublox_wait_for_drdy_and_miso(void)
{
	while (true) {
		int drdy = gpio_get(WINC1500_PORT, WINC1500_IRQ);
		int miso = gpio_get(WINC1500_SPI_DATA_PORT, WINC1500_SPI_MISO);
		if (miso == 0) {
			if (drdy == 1)
				break;
		}
		platform_delay(1);
	}
}

void ublox_wait_for_drdy(void)
{
	while (gpio_get(WINC1500_PORT, WINC1500_IRQ) == 0) {
		// int drdy = GPIO_IDR(WINC1500_PORT) & WINC1500_IRQ;
		// int miso = GPIO_IDR(WINC1500_SPI_DATA_PORT) & WINC1500_SPI_MISO;
		// if (miso == 0) {
		// 	if (drdy == 1)
		// 		break;
		// }
		platform_delay(1);
	}
}

void exti9_5_isr(void)
{
	//
	// Is it EXTI9?
	//
	if (exti_get_flag_status(EXTI9) == EXTI9) {
		// Reset the interrupt state
		exti_reset_request(EXTI9);
		//
		// The uBlox module is ready to initialize SPI
		// disable this interrupt
		//
		exti_disable_request(WINC1500_IRQ);
		nvic_disable_irq(NVIC_EXTI9_5_IRQ);
		__atomic_store_n(&wifi_awake, true, __ATOMIC_RELAXED);
	}
}

/*
	The format of a module -> host packet is:
		Header byte 1: 0xBA
		Header byte 2: 0x15
		Length byte 1: MSB of the length	<-- Bytes following this 16-bit length
		Length byte 2: LSB of the length
		Payload: The actual data
*/
bool ublox_read_packet(uint8_t *buffer, uint32_t length, bool first_contact)
{
	(void)length;
	uint8_t *buffer_start = buffer;
	uint16_t payload_size;
	if (first_contact == true) {
		/*
			Wait for the DRDY signal to go high
		*/
		ublox_wait_for_drdy();
	} else {
		/*
			Wait for DRDY to go high and miso low 
			
			miso being low indicates the uBlox module has initialized the SPI interface
		*/
		ublox_wait_for_drdy_and_miso();
	}
	/*
		Wait for the DRDY signal to go high
	*/
	ublox_wait_for_drdy();
	/*
		Asserting the chip select signals a packet read start
	*/
	gpio_clear(WINC1500_PORT, WINC1500_SPI_NCS);

	for (uint32_t index = 0; index < 4; index++) { // Read the header and byte count
		*buffer = spi_xfer(WINC1500_SPI_CHANNEL, *buffer);
		buffer++;
	}
	/*
		Check for valid header
	*/
	if (buffer_start[0] != 0xBA && buffer_start[1] != 0x15) {
		/*
			Invalid header, reset the chip select and return
		*/
		gpio_set(WINC1500_PORT, WINC1500_SPI_NCS);
		return false;
	}
	/*
		Now read the payload
	*/
	payload_size = (buffer_start[2] << 8) | buffer_start[3];
	if (payload_size == 0)
		return false;

	for (uint32_t index = 0; index < payload_size; index++) {
		*buffer = spi_xfer(WINC1500_SPI_CHANNEL, *buffer);
		buffer++;
	}
	gpio_set(WINC1500_PORT, WINC1500_SPI_NCS);
	return true;
}

void ublox_write_packet(uint8_t *buffer, uint32_t length)
{
	/*
		Asserting the chip select signals a packet start
	*/
	gpio_clear(WINC1500_PORT, WINC1500_SPI_NCS);

	for (uint32_t index = 0; index < length; index++)
		spi_xfer(WINC1500_SPI_CHANNEL, *buffer++);
	gpio_set(WINC1500_PORT, WINC1500_SPI_NCS);
}

/*
	To request the uBlox module uses the SPI interface, it
	requires SPI_CLK activity. This function activates SPI_CS
	and writes 8 bytes to the uBlox module.
 */
void ublox_spi_wakeup(void)
{
	gpio_clear(WINC1500_PORT, WINC1500_SPI_NCS);

	for (uint32_t index = 0; index < 8; index++) {
		spi_xfer(WINC1500_SPI_CHANNEL, 0);
	}
	gpio_set(WINC1500_PORT, WINC1500_SPI_NCS);
}

static inline void short_delay(void)
{
	for (volatile int i = 0; i < 1000; i++) {
		__asm__("nop");
	}
}

void app_initialize(void)
{
	wifi_hardware_init();
	//
	// Hold here wi-fi module to wake up
	//
	while (__atomic_load_n(&wifi_awake, __ATOMIC_RELAXED) == false) {
		platform_delay(1);
	}

	// The uBlox module requires SPI activity to enable the SPI interface
	// to be used for AT commands
	//
	// ublox_spi_wakeup();
	uint8_t input[32] = {0};

	while (true) {
		if (ublox_read_packet(input, 32, true)) {
			if (input[0] == 0xba && input[1] == 0x15) {
				// This is a valid packet
				break;
			}
		}
		memset(input, 0, 32);
		short_delay();
	}
	//
	// Let's make a  test by sending AT and looking for a reply
	//
	uint8_t output[32] = {0xba, 0x15, 0x00, 0x0c, 'A', 'T', 'S', '5', '?', '\r', '\n', 0xff, 0xff, 0xff, 0xff, 0xff};
	ublox_write_packet(output, 16);
	platform_delay(1);
	memset(input, 0xff, 32);
	while (ublox_read_packet(input, 32, true) == false)
		platform_delay(1);
	memset(input, 0xff, 32);
	platform_delay(1);
	while (ublox_read_packet(input, 32, true) == false)
		platform_delay(1);
	platform_delay(100);
}

void app_task(void)
{
}

//
void gdb_tcp_server(void)
{
}

bool is_gdb_client_connected(void)
{
	return false;
}

void data_tcp_server(void)
{
}

bool is_uart_client_connected(void)
{
	return false;
}

void send_uart_data(uint8_t *buffer, uint8_t length)
{
	(void)buffer;
	(void)length;
}

bool swo_trace_server_active(void)
{
	return false;
}

void wifi_setup_swo_trace_server(void)
{
}

bool is_swo_trace_client_connected(void)
{
	return false;
}

void send_swo_trace_data(uint8_t *buffer, uint8_t length)
{
	(void)buffer;
	(void)length;
}

void wifi_gdb_putchar(uint8_t ch, bool flush)
{
	(void)ch;
	(void)flush;
}

void wifi_gdb_flush(bool force)
{
	(void)force;
}

bool wifi_got_client(void)
{
	return false;
}

uint8_t wifi_get_next(void)
{
	return -1;
}

uint8_t wifi_get_next_to(uint32_t timeout)
{
	(void)timeout;
	return -1;
}
