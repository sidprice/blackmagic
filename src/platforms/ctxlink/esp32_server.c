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

void app_initialize(void)
{
	wifi_hardware_init();
	//
	// Hold here wi-fi module to wake up
	//
	while (__atomic_load_n(&wifi_awake, __ATOMIC_RELAXED) == false)
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