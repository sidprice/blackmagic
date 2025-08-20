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
		rxBuffer[i] = spi_xfer(ESP32_SPI_CHANNEL, txBuffer[i]);
}

//
// This function transfers a complete protocol packet
//
void esp32_transfer_header_and_packet(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t length)
{
	gpio_clear(ESP32_PORT, ESP32_SPI_NCS);
	//
	// Wait for ESP32 ready
	//
	while (gpio_get(ESP32_nSPI_READY_PORT, ESP32_nSPI_READY) != 0)
		;
	esp32_transfer(txBuffer, rxBuffer, length);
	gpio_set(ESP32_PORT, ESP32_SPI_NCS);
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
	gpio_clear(ESP32_PORT, ESP32_SPI_NCS);
	//
	// Wait for ESP32 ready
	//
	while (gpio_get(ESP32_nSPI_READY_PORT, ESP32_nSPI_READY) != 0)
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
	gpio_set(ESP32_PORT, ESP32_SPI_NCS);
}

// uint8_t buffer[1024] = {0};
// uint8_t inputBuffer[1024] = {0};

void app_initialize(void)
{
	//
	// Initialize the ESP32 interface hardware
	//
	rcc_periph_clock_enable(ESP32_RCC_SPI);
	//
	// Negate all outputs to ESP32
	//
	gpio_set(ESP32_nSPI_READY_PORT, ESP32_nSPI_READY);
	gpio_set(ESP32_PORT, ESP32_SPI_NCS);
	gpio_set(ESP32_nREADY_PORT, ESP32_nREADY); // Will be used as input later

	//
	// Set up the control outputs for the ESP32
	//
	//		nSPI_READY input
	//
	gpio_mode_setup(ESP32_nREADY_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, ESP32_nREADY);
	//
	//		Chip select output
	//
	gpio_mode_setup(ESP32_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ESP32_SPI_NCS);
	//
	// ESP32 nREADY signal
	//
	gpio_mode_setup(ESP32_nREADY_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, ESP32_nREADY);
	//
	// Need to make the nATTN pin an external interrupt on falling edge
	//
	//	First enable the SYSCFG clock
	//
	rcc_periph_clock_enable(RCC_SYSCFG);
	gpio_mode_setup(ESP32_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, ESP32_nATTN); // Input signal with pulldown
	exti_select_source(ESP32_nATTN, ESP32_PORT);
	exti_set_trigger(ESP32_nATTN, EXTI_TRIGGER_FALLING);
	//
	// Set the port pins of the SPI channel to high-speed I/O
	//
	gpio_set_output_options(ESP32_SPI_DATA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
		ESP32_SPI_CLK | ESP32_SPI_MISO | ESP32_SPI_MOSI | ESP32_nSPI_READY);
	//
	// Enable alternate function for SPI2_CLK PB10 AF5
	//
	gpio_mode_setup(ESP32_SPI_CLK_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, ESP32_SPI_CLK);
	gpio_set_af(ESP32_SPI_CLK_PORT, GPIO_AF5, ESP32_SPI_CLK);
	//
	// Enable SPI alternate function pins - MISO and MOSI
	//
	gpio_mode_setup(ESP32_SPI_DATA_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, ESP32_SPI_MISO | ESP32_SPI_MOSI);
	gpio_set_af(ESP32_SPI_DATA_PORT, GPIO_AF5, ESP32_SPI_MISO | ESP32_SPI_MOSI);
	//
	// I think this is Mode_0, 8 bit data, MSB first, the clock rate is 42MHz with core of 84MHz
	//
	// ESP32 SPI as peripheral has 10MHz max SPI CLK
	spi_init_master(ESP32_SPI_CHANNEL, SPI_CR1_BAUDRATE_FPCLK_DIV_4, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	//
	// Set NSS to software management and also ensure NSS is high, if not written high no data will be sent
	//
	spi_enable_software_slave_management(ESP32_SPI_CHANNEL);
	spi_set_nss_high(ESP32_SPI_CHANNEL);
	//
	// Enable the SPI channel
	//
	spi_enable(ESP32_SPI_CHANNEL);
	exti_enable_request(ESP32_nATTN);
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);

	//
	// Hold here wi-fi module to wake up
	//
	while (gpio_get(ESP32_nREADY_PORT, ESP32_nREADY) != 0)
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
		uint8_t second_buffer[1024] = {0};
		uint8_t inputBuffer[1024] = {0};
		// esp32_transfer_header_and_packet(buffer, inputBuffer, sizeof(buffer));
		// platform_delay(100);
		memset(second_buffer, 0, sizeof(second_buffer));
		while (gpio_get(ESP32_PORT, ESP32_nATTN) != 0)
			;
		esp32_transfer_packet(buffer, inputBuffer, 12);
		//
		while (gpio_get(ESP32_nSPI_READY_PORT, ESP32_nSPI_READY) == 0)
			;

		for (size_t i = 0; i < 2000; i++)
			;
		esp32_transfer_header_and_packet(buffer, inputBuffer, sizeof(buffer));
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