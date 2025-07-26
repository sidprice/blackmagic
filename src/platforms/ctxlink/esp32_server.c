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

#include "protocol.h"

/**
 * @brief Minimum time in us between nCS being negated and asserted again.
 */
#define MINIMUM_CS_NEGATED_TIME 200

#define INPUT_BUFFER_SIZE 2048

static uint8_t input_buffer[INPUT_BUFFER_SIZE] = {0}; ///< The input buffer
static uint32_t input_index = 0;                      ///< Zero-based index of the input
static uint32_t output_index = 0;                     ///< Zero-based index of the output
static _Atomic uint32_t buffer_count = 0;             ///< Number of buffers

#define SEND_BUFFER_COUNT 4

static uint8_t send_buffer[INPUT_BUFFER_SIZE] = {0}; ///< The send buffer
static uint32_t send_count = 0;                      ///< Bytes to send and buffer input index

static network_connection_info_s network_information;

/**
 * @brief Reset and start TIM2
 * 
 * This function is called when the SPI nCS output is negated.
 */
void timer2_start(void)
{
	// Reset and start the timer
	timer_disable_counter(TIM2);
	timer_set_counter(TIM2, 0);
	timer_enable_counter(TIM2);
}

/**
 * @brief Return the current value of TIM2
 */

uint32_t timer2_get_count(void)
{
	return timer_get_counter(TIM2);
}

/**
 * @brief Wait for the minimum time between nCS negated and asserted
 */

void wait_minimum_cs_negated_time(void)
{
	uint32_t count = timer2_get_count();
	while (count < MINIMUM_CS_NEGATED_TIME) {
		count = timer2_get_count();
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
	wait_minimum_cs_negated_time();
	gpio_clear(ESP32_PORT, ESP32_SPI_NCS);
	//
	// Wait for ESP32 SPI ready
	//
	while (gpio_get(ESP32_nSPI_READY_PORT, ESP32_nSPI_READY) != 0)
		;
	esp32_transfer(txBuffer, rxBuffer, length);
	gpio_set(ESP32_PORT, ESP32_SPI_NCS);
	timer2_start(); // Reset and start TIM2
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
	wait_minimum_cs_negated_time();
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
	timer2_start(); // Reset and start TIM2
}

/**
 * @brief ESP32 SPI transfer buffers
 */
#define ESP32_SPI_BUFFER_SIZE 2048U
static uint8_t esp32_tx_buffer[ESP32_SPI_BUFFER_SIZE] = {0};
static uint8_t esp32_rx_buffer[ESP32_SPI_BUFFER_SIZE] = {0};

static uint32_t attn_count = 0; ///< Number of nATTN interrupts

/**
 * @brief nATTN interrupt handler
 * 
 * This interrupt is triggered when the ESP32 has data for ctxLink.
 * 
 */
void exti9_5_isr(void)
{
	//
	// Is it nATTN?
	//
	attn_count++;
	if (exti_get_flag_status(ESP32_nATTN) == ESP32_nATTN) {
		exti_reset_request(ESP32_nATTN);
		//
		// Read the data from ESP32
		//
		esp32_transfer_packet(esp32_tx_buffer, esp32_rx_buffer, ESP32_SPI_BUFFER_SIZE);
		//
		// Parse the packet
		//
		size_t packet_size;
		protocol_packet_type_e packet_type;
		uint8_t *packet_data;
		protocol_split(esp32_rx_buffer, &packet_size, &packet_type, &packet_data);
		//
		// Process input packet according to its type
		//
		switch (packet_type) {
		case PROTOCOL_PACKET_TYPE_FROM_GDB: {
			//
			// Copy the packet data to the local input buffer of ctxLink
			//
			__atomic_fetch_add(&buffer_count, packet_size, __ATOMIC_RELAXED);
			// buffer_count += packet_size;
			for (uint32_t i = 0; i < packet_size; i++, input_index = (input_index + 1) % INPUT_BUFFER_SIZE) {
				input_buffer[input_index] = packet_data[i];
			}
			break;
		}

		case PROTOCOL_PACKET_TYPE_NETWORK_INFO: {
			network_connection_info_s *network_info = (network_connection_info_s *)packet_data;
			memcpy(&network_information, network_info, sizeof(network_connection_info_s));
			break;
		}

		default: {
			//
			// Unknown packet type, assert
			//
			//__asm__("BKPT #0");
			break;
		}
		}
	}
}

/** @brief	Timer2 is used to set a minimum time between nCS being negated and asserted again.
 * 
 * This delay is required because the ESP32 SPI firmware/hardware requires a minimum time before
 * a new transaction may be started. There appears to be no way to determine this delay to control
 * nCS.
 * 
 * When CS is negated, the timer is reset and started.
 * When CS is required to asserted, the SPI driver checks that the minimum time has elapsed, if
 * not, it loops on the timer count until it exceeds the minimum delay.
 */
void timer_init(void)
{
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Reset TIM2 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM2);

	// Set timer mode:
	// internal clock,
	// edge-aligned
	// up-counting
	//one-pulse mode
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	// timer_one_shot_mode(TIM2);
	timer_continuous_mode(TIM2);

	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM2 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 1MHz
	 */
	timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 1000000));

	/* count to max value, then stop. */
	timer_set_period(TIM2, 1000);

	/* set the current count to max to avoid delaying initial CS assertion */
	timer_set_counter(TIM2, 65535);

	// Disable preload (optional, depending on your use case)
	timer_disable_preload(TIM2);
}

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

	timer_init();
	//
	// Hold here wi-fi module to wake up
	//
	while (gpio_get(ESP32_nREADY_PORT, ESP32_nREADY) != 0) // TODO Probably shouldn't do this without timeout
		;
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
	send_buffer[send_count++] = ch;
	if (flush) {
		//
		// Package the GDB response
		//
		send_count = package_data(&send_buffer[0], send_count, PROTOCOL_PACKET_TYPE_TO_GDB, INPUT_BUFFER_SIZE);
		esp32_transfer_header_and_packet(&send_buffer[0], input_buffer, send_count);
		send_count = 0;
	}
}

void wifi_gdb_flush(bool force)
{
	(void)force;
}

bool wifi_got_client(void)
{
	return false;
}

int wifi_have_input(void)
{
	int result;
	__atomic_load(&buffer_count, &result, __ATOMIC_RELAXED);
	return result;
}

uint8_t wifi_get_next(void)
{
	uint8_t result = 0x00;
	uint32_t local_buffer_count;
	//
	// The buffer count is also managed in an ISR so protect this code
	//
	__atomic_load(&buffer_count, &local_buffer_count, __ATOMIC_RELAXED);
	if (local_buffer_count != 0) {
		result = input_buffer[output_index];
		output_index = (output_index + 1) % INPUT_BUFFER_SIZE;
		__atomic_fetch_sub(&buffer_count, 1, __ATOMIC_RELAXED);
	}
	return result;
}

uint8_t wifi_get_next_to(uint32_t timeout)
{
	platform_timeout_s t;
	uint8_t count = 0;
	int input_count = 0;
	platform_timeout_set(&t, timeout);

	do {
		input_count = wifi_have_input();
		if (input_count != 0)
			break;
		//
		// We must run the platform tasks or incomming data will not be transferred
		// to the input buffers
		//

		platform_tasks();

	} while (!platform_timeout_is_expired(&t));

	if (input_count != 0)
		count = wifi_get_next();
	return count;
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