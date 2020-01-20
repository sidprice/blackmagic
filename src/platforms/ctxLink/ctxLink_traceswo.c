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

#include <libopencm3/cm3/common.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
//#include <libopencm3/stm32/dma.h>

/* For speed this is set to the USB transfer size */
#define FULL_SWO_PACKET	(64)
/* Default line rate....used as default for a request without baudrate */
#define DEFAULTSPEED	(2250000)

#define SWOUART_TIMER_FREQ_HZ	1000000		// 1uS clock
#define	SWOUART_RUN_FREQ_HZ		500			// 200uS (100 characters @ 2Mbps)

#define FIFO_SIZE NUM_TRACE_PACKETS * FULL_SWO_PACKET

static volatile uint32_t w;	/* Packet currently received via UART */
static volatile uint32_t r;	/* Packet currently waiting to transmit to USB */
/* Packets arrived from the SWO interface */
static uint8_t trace_rx_buf[NUM_TRACE_PACKETS * FULL_SWO_PACKET];

void trace_buf_drain(usbd_device *dev, uint8_t ep)
{
	static volatile char inBufDrain;

	/* If we are already in this routine then we don't need to come in again */
	if (__atomic_test_and_set (&inBufDrain, __ATOMIC_RELAXED))
		return;
	/* Attempt to write everything we buffered */
	if ((w != r) && (usbd_ep_write_packet(dev, ep,
										  &trace_rx_buf[r * FULL_SWO_PACKET],
										  FULL_SWO_PACKET)))
		r =(r + 1) % NUM_TRACE_PACKETS;
	__atomic_clear (&inBufDrain, __ATOMIC_RELAXED);
}

static volatile char errors[32] = {0} ;
volatile uint32_t errorIndex = 0 ;

void SWO_UART_ISR(void)
{
	uint32_t err = USART_SR(SWO_UART);
	char c = usart_recv(SWO_UART);
#if !defined(USART_SR_NE) && defined(USART_ISR_NF)
# define USART_SR_NE USART_ISR_NF
#endif
	if (err & (USART_FLAG_ORE | USART_FLAG_FE | USART_SR_NE))
	{
		return;
	}
	if (((w + 1) % FIFO_SIZE) != r)
	{
		/* insert into FIFO */
		trace_rx_buf[w++] = c;
		/* wrap out pointer */
		if (w >= FIFO_SIZE)
		{
			w = 0;
		}
	}
}

#define	TRACE_TIM_COMPARE_VALUE	50

void TRACE_TIM_ISR(void)
{
	if (timer_get_flag(TRACE_TIM, TIM_SR_CC1IF)) {

		/* Clear compare interrupt flag. */
		timer_clear_flag(TRACE_TIM, TIM_SR_CC1IF);

		/*
		 * Get current timer value to calculate next
		 * compare register value.
		 */
		uint16_t compare_time = timer_get_counter(TRACE_TIM);

		/* Calculate and set the next compare value. */
		uint16_t frequency = TRACE_TIM_COMPARE_VALUE;
		uint16_t new_time = compare_time + frequency;

		timer_set_oc_value(TRACE_TIM, TIM_OC1, new_time);
		//
		// Fluch any Trace data to client
		//
		gpio_toggle(LED_PORT, LED_MODE) ;
	}
}

void traceswo_init(uint32_t baudrate)
{
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
	timer_enable_irq(TRACE_TIM, TIM_DIER_CC1IE);
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
	// SWO_UART_CR1 |= USART_CR1_RXNEIE;
	// nvic_set_priority(SWO_UART_IRQ, IRQ_PRI_SWOUSART);
	// nvic_enable_irq(SWO_UART_IRQ);
}
