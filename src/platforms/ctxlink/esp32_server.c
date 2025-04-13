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

void exti9_5_isr(void)
{
	//
	// Is it EXTI9?
	//
	if (exti_get_flag_status(EXTI9) == EXTI9) {
		// Reset the interrupt state
		exti_reset_request(EXTI9);
		__atomic_store_n(&wifi_awake, true, __ATOMIC_RELAXED);
	}
}

void esp32_transfer(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t length)
{
	//
	// Do the transfer
	//
	for (uint16_t i = 0; i < length; i++)
		rxBuffer[i] = spi_xfer(WINC1500_SPI_CHANNEL, txBuffer[i]);
}

//
// This function transfers a complete protocol packet
//
void esp32_transfer_header_and_packet(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t length)
{
	gpio_clear(WINC1500_PORT, WINC1500_SPI_NCS);
	//
	// Wait for ESP32 ready
	//
	while (gpio_get(WINC1500_CHIP_EN_PORT, WINC1500_CHIP_EN) != 0)
		;
	esp32_transfer(txBuffer, rxBuffer, length);
	gpio_set(WINC1500_PORT, WINC1500_SPI_NCS);
}

//
//	This function first transfers the protocol header, it contains the byte count.
//
//	It then transfers the balance of the protocol packet.
//
//		TODO Add protocol defs to ctxLink
void esp32_transfer_packet(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t length)
{
	uint32_t byte_count;
	gpio_clear(WINC1500_PORT, WINC1500_SPI_NCS);
	//
	// Wait for ESP32 ready
	//
	while (gpio_get(WINC1500_CHIP_EN_PORT, WINC1500_CHIP_EN) != 0)
		;
	//
	// Header size is 5 bytes.
	//
	esp32_transfer(txBuffer, rxBuffer, 5);
	//
	// Byte count is in bytes 3 & 4
	//
	byte_count = (*(rxBuffer + 3) >> 8) & 0xff;
	byte_count += *(rxBuffer + 4);
	//
	// Transfer the balance of the packet
	//
	esp32_transfer(txBuffer, rxBuffer + 5, byte_count);
	gpio_set(WINC1500_PORT, WINC1500_SPI_NCS);
}

// uint8_t buffer[1024] = {0};
// uint8_t inputBuffer[1024] = {0};

void app_initialize(void)
{
	wifi_hardware_init();
	//
	// Hold here wi-fi module to wake up
	//
	while (gpio_get(WINC1500_CHIP_EN_PORT, WINC1500_CHIP_EN) != 0)
		;

#if 0
	//
	// The ESP32 seems to not bring up GPIO in a clean way and some "glitches"
	// were observed on the ATTN input  when the ESP32 starts up.  This
	// causes the interrupt to be triggered and the system to wake up early.
	// So we need to wait for a while to make sure that the interrupt is not
	// triggered by a glitch.
	//
	// Hence the "strange" loop here.
	//
	while (__atomic_load_n(&wifi_awake, __ATOMIC_RELAXED) == false) {
		platform_delay(1);
		//
		// Check the ATTN input is still low before we continue
		//
		if (gpio_get(WINC1500_PORT, WINC1500_IRQ) == 0)
			break;
	}
	//
	// Send a greeting to test the ESP32 input
	//
	{
		uint8_t buffer[] = {'H', 'e', 'l', 'l', 'o', '\n'};
		uint8_t inputBuffer[32];
		esp32_transfer(buffer, inputBuffer, sizeof(buffer));
	}
	// platform_delay(100);
	while (gpio_get(WINC1500_PORT, WINC1500_IRQ))
		;
	while (!gpio_get(WINC1500_PORT, WINC1500_IRQ))
		;

	{
		uint8_t buffer[] = {'H', 'E', 'L', 'L', 'O', '\n'};
		uint8_t inputBuffer[32];
		esp32_transfer(buffer, inputBuffer, sizeof(buffer));
	}
#endif
	platform_delay(100);
	while (1) {
		uint8_t buffer[] = {0xDE, 0xAD, 0x05, 0x00, 0x07, 'H', 'E', 'L', 'L', 'O', 0, 0};
		uint8_t inputBuffer[32] = {0};
		// esp32_transfer_header_and_packet(buffer, inputBuffer, sizeof(buffer));
		// platform_delay(100);
		// while (gpio_get(WINC1500_PORT, WINC1500_IRQ) != 0)
		// 	;
		//		memset(buffer, 0, sizeof(buffer));
		esp32_transfer_header_and_packet(buffer, inputBuffer, 12);
		platform_delay(100);
	}
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

void app_task_wait_spin(void)
{
}

//
// Using the passed arguments, attempt to connect to a Wi-Fi AP
//
void wifi_connect(size_t argc, const char **argv, char *buffer, uint32_t size)
{
	(void)argc;
	(void)argv;
	(void)buffer;
	(void)size;
}

//
// Format the current connection data for display to user
//
//	SSID = 'name'
//	RSSI = xx
//	ip = xxx.xxx.xxx.xxx
//
void wifi_get_ip_address(char *buffer, uint32_t size)
{
	(void)buffer;
	(void)size;
}