/*
 * This file is part of the Black Magic Debug project.
 *
 * Based on work that is Copyright (C) 2017 Black Sphere Technologies Ltd.
 * Based on work that is Copyright (C) 2017 Dave Marples <dave@marples.net>
 * Copyrigh (C) Sid Price 2020 <sid@sidprice.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.	 If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements capture of the TRACESWO output using ASYNC signalling.
 *
 * ARM DDI 0403D - ARMv7M Architecture Reference Manual
 * ARM DDI 0337I - Cortex-M3 Technical Reference Manual
 * ARM DDI 0314H - CoreSight Components Technical Reference Manual
 */

/* TDO/TRACESWO signal comes into the SWOUSART RX pin.
 */

#include "general.h"
#include "cdcacm.h"
#include "traceswo.h"
#include "platform.h"

#include <libopencm3/cm3/common.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

/* For speed this is set to the USB transfer size */
#define FULL_SWO_PACKET	(64)
/* Default line rate....used as default for a request without baudrate */
#define DEFAULTSPEED	(2250000)

#define BUFFER_SIZE 1024

static volatile uint32_t inBuf = 0 ;	// input buffer index
static volatile uint32_t outBuf = 0 ;	// output buffer index
volatile uint32_t bufferSize = 0 ;	// Number of bytes in the buffer 

static uint8_t trace_rx_buf[BUFFER_SIZE] = {0} ;
#define NUM_PINGPONG_BUFFERS	2
static uint8_t pingpongBuffers[NUM_PINGPONG_BUFFERS * FULL_SWO_PACKET] = {0} ;
static uint32_t bufferSelect = 0 ;

void _trace_buf_drain(usbd_device *dev, uint8_t ep)
{
	uint32_t	outCount ;
	uint8_t	*bufferPointer, *bufferStart ;

	__atomic_load(&bufferSize, &outCount, __ATOMIC_RELAXED) ;
	if (outCount == 0)
	{
		return;
	}
	//
	// Set up the pointer to grab the data
	//
	bufferPointer = bufferStart = &pingpongBuffers[bufferSelect * FULL_SWO_PACKET] ;
	//
	// Copy the data
	//
	for ( uint32_t i = 0 ; i < outCount ;i++)
	{
		*bufferPointer++ = trace_rx_buf[outBuf++] ;
		if ( outBuf >= BUFFER_SIZE)
		{
			outBuf = 0 ;
		}
	}
	//
	// Bump the pingpong buffer selection
	//
	bufferSelect = (bufferSelect+1) % NUM_PINGPONG_BUFFERS ;

	if (usbd_ep_write_packet(dev, ep, bufferStart, outCount) == outCount)
	{
		__atomic_fetch_sub(&bufferSize, outCount, __ATOMIC_RELAXED);
	}
}

void trace_buf_drain(usbd_device *dev, uint8_t ep)
{
	_trace_buf_drain(usbdev, ep) ;
}

#define	TRACE_TIM_COMPARE_VALUE	2000

void SWO_UART_ISR(void)
{
	//
	// Make sure the flush timer is disabled 
	//
	timer_disable_counter(TRACE_TIM) ;
	uint32_t err = USART_SR(SWO_UART);
	char c = usart_recv(SWO_UART);

	if (err & (USART_FLAG_ORE | USART_FLAG_FE | USART_SR_NE))
	{
		return;
	}
	/* If the next increment of rx_in would put it at the same point
	 * as rx_out, the FIFO is considered full.
	 */
	uint32_t	copyOutBuf ;
	__atomic_load(&outBuf, &copyOutBuf, __ATOMIC_RELAXED) ;
	if (((inBuf + 1) % BUFFER_SIZE) != copyOutBuf)
	{
		/* insert into FIFO */
		trace_rx_buf[inBuf++] = c;
		__atomic_fetch_add(&bufferSize, 1, __ATOMIC_RELAXED) ;	// bufferSize++ ;

		/* wrap out pointer */
		if (inBuf >= BUFFER_SIZE)
		{
			inBuf = 0;
		}
		//
		// If we have a packet-sized amount of data send
		// it to USB
		//
		uint32_t outCount ;
		__atomic_load(&bufferSize, &outCount, __ATOMIC_RELAXED) ;
		// if (outCount >= FULL_SWO_PACKET)
		if (outCount >= FULL_SWO_PACKET)
		{
			_trace_buf_drain(usbdev, USB_TRACESWO_ENDPOINT) ;
		}
		//
		// Prepare the flush timer
		//
		//
		// Clear any compare interrupt flag.
		//
		// timer_clear_flag(TRACE_TIM, TIM_SR_CC1IF);
		// timer_set_counter(TRACE_TIM, 0) ;	// Reset the Counter
		// timer_enable_counter(TRACE_TIM) ;
		// timer_enable_irq(TRACE_TIM, TIM_DIER_CC1IE) ;
	}
	else
	{
		// Just drop data ????
	}
	
}

void TRACE_TIM_ISR(void)
{
	if (timer_get_flag(TRACE_TIM, TIM_SR_CC1IF)) {
		//
		// Clear compare interrupt flag.
		//
		timer_clear_flag(TRACE_TIM, TIM_SR_CC1IF);
		// Disable the  timer
		timer_disable_irq(TRACE_TIM, TIM_DIER_CC1IE) ;
		// Reset the Counter
		timer_set_counter(TRACE_TIM, 0) ;
		//
		// Now flush the data to the client
		//
		_trace_buf_drain(usbdev, USB_TRACESWO_ENDPOINT) ;
	}
}

void traceswo_init(uint32_t baudrate)
{
	rcc_periph_clock_enable(SWO_UART_CLK) ;
	TRACE_TIM_CLK_EN();

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(TRACE_TIM_IRQ);

	/* Reset TIM3 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM3);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(TRACE_TIM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM3 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 50kHz
	 */
	timer_set_prescaler(TRACE_TIM, ((rcc_apb1_frequency * 2) / 50000));

	/* Disable preload. */
	timer_disable_preload(TRACE_TIM);
	timer_continuous_mode(TRACE_TIM);

	/* count full range, as we'll update compare value continuously */
	timer_set_period(TRACE_TIM, 65535);

	/* Set the initual output compare value for OC1. */
	timer_set_oc_value(TRACE_TIM, TIM_OC1, TRACE_TIM_COMPARE_VALUE);

	/* Counter enable. */
	timer_enable_counter(TRACE_TIM);

	/* Enable Channel 1 compare interrupt to recalculate compare values */
	//timer_enable_irq(TRACE_TIM, TIM_DIER_CC1IE);
	//
	gpio_mode_setup(SWO_UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SWO_UART_RX_PIN);
	gpio_set_af(SWO_UART_PORT, GPIO_AF8, SWO_UART_RX_PIN); 
	//
	if (!baudrate)
		baudrate = DEFAULTSPEED;
	/* Setup input UART parameters. */
	usart_set_baudrate(SWO_UART, baudrate);
	usart_set_databits(SWO_UART, 8);
	usart_set_stopbits(SWO_UART, USART_STOPBITS_1);
	usart_set_mode(SWO_UART, USART_MODE_RX);
	usart_set_parity(SWO_UART, USART_PARITY_NONE);
	usart_set_flow_control(SWO_UART, USART_FLOWCONTROL_NONE);
	usart_enable(SWO_UART);
	// Enable interrupts
	SWO_UART_CR1 |= USART_CR1_RXNEIE;
	nvic_set_priority(SWO_UART_IRQ, IRQ_PRI_SWOUSART);
	nvic_enable_irq(SWO_UART_IRQ);
}
