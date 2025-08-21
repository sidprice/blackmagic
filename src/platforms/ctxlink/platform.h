/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * Updates for ctxLink Copyright (C) 2024 Sid Price - sid@sidprice.com
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

/* This file provides the platform specific declarations for the ctxLink implementation. */

#ifndef PLATFORMS_CTXLINK_PLATFORM_H
#define PLATFORMS_CTXLINK_PLATFORM_H

#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"

#define PLATFORM_HAS_TRACESWO
#define PLATFORM_HAS_POWER_SWITCH
#define PLATFORM_HAS_WIFI

#define PLATFORM_IDENT "(ctxLink) "

#define PLATFORM_HAS_BATTERY

#if ENABLE_DEBUG == 1
#define PLATFORM_HAS_DEBUG
extern bool debug_bmp;
#endif

/*
 * Important pin mappings for ctxLink implementation:
 *
 * +-----------------+----------------------+----------------------+---------------------+
 * | Signal          | Function             |  WINC1500            |  ESP32-S3           |
 * +-----------------+----------------------+----------------------+---------------------+
 * | LED0            |  Green LED - UART    |  PB2                 |   PC13              |
 * | LED1            |  Orange LED -  Idle  |  PC6                 |   PC14              |
 * | LED2            | 	Red LED - Error     |  PC8                 |   PC15              |
 * | LED3            |  Green LED - Mode    |  PC9                 |   PA3               |
 * +-----------------+----------------------+----------------------+---------------------+
 * | iTDI            |  TDI                 |  PA3                 |   PB6               |
 * | iTMS            |  TMS                 |  PA4                 |   PB4               |
 * | iTCK            |  TCK                 |  PA5                 |   PB5               |
 * | iTDO            |  TDO                 |  PC7                 |   PA15              |
 * | iTMS_DIR        |  TMS_DIR             |  PA1                 |   PB7               |
 * +-----------------+----------------------+----------------------+---------------------+
 * | iRST            |  nRST                |  PA2                 |   PB9               |
 * | iRST_SENSE      |  nRST_SENSE          |  PA7                 |   PB8               |
 * +-----------------+----------------------+----------------------+---------------------+
 * | PWR_BR          |  Target Power Ctrl   |  PB1                 |   PA0               |
 * | TPWR            |  Target Power Mon    |  PB0 - ADC1 IN8      |   PA2 - ADC1 IN2    |
 * | BATT_MON        |  Battery Monitor     |  PA0 - ADC1 IN0      |   PA1 - ADC1 IN1    |
 * +-----------------+----------------------+----------------------+---------------------+
 * | REN             |  USB_PU              |  PA8                 |   PB13              |
 * | USB_DM          |  USB D-              |  PA11                |   PA11              |
 * | USB_DP          |  USB D+              |  PA12                |   PA12              |
 * | VBUS            |  USB VBUS            |  PA9                 |   PB14              |
 * +-----------------+----------------------+----------------------+---------------------+
 * | SW_BOOTLOADER   | Bootloader (mode) SW |  PB12                |   PB12              |
 * +-----------------+----------------------+----------------------+---------------------+
 * |  SPI Peripheral |  SPI Peripheral       |  SPI2 - AF05        |   SPI1 - AF05       |
 * |  SPI_NCS        |  SPI Chip Select      |  PB15               |   PA4               |
 * |  nIRQ           |  WINC1500 IRQ         |  PB9                | ----------          |
 * |  SPI_CLK        |  SPI Clock            |  PB10               |   PA5               |
 * |  SPI_MISO       |  SPI MISO             |  PC2                |   PA6               |
 * |  SPI_MOSI       |  SPI MOSI             |  PC3                |   PA7               |
 * |  WINC1500_CE    |  WINC1500 Chip Enable |  PB13               | ----------          |
 * |  nESP32_RESET   |  ESP32 nReset         | -----------         |  PB0                |
 * |  nREADY         |  ESP32 nREADY         | -----------         |  PB1                |
 * |  nATTN          |  ESP32 nATTN          | -----------         |  PB2                |
 * |  nSPI_READY     |  ESP32 nSPI_READY     | -----------         |  PB10               |
 * +-----------------+-----------------------+---------------------+---------------------+
 * |  iTXD           |  UART TXD             |  PB6 - AF07 - UART1 |  PA9 - AF07 - UART1 |
 * |  iRXD           |  UART RXD             |  PB7 - AF07 - UART1 |  PA10 - AF07 - UART1|
 * +-----------------+-----------------------+---------------------+---------------------+
 *
 */

//
// Define the network name for the probe
//
//	TODO, use part or all of the MAC address to make this unique.
//

#define CTXLINK_NETWORK_NAME "ctxLink_0001"

#ifndef CTXLINK_ESP32_WIFI
#include "platform_winc1500.h"
#else
#include "platform_esp32s3.h"
#endif

//
// SWO UART definitions
//
#define SWO_UART        USART6
#define SWO_UART_CR1    USART6_CR1
#define SWO_UART_DR     USART6_DR
#define SWO_UART_CLK    RCC_USART6
#define SWO_UART_PORT   GPIOC
#define SWO_UART_RX_PIN GPIO7
#define SWO_UART_ISR    usart6_isr
#define SWO_UART_IRQ    NVIC_USART6_IRQ

/*
 * To use USART1 as USBUSART, DMA2 is selected from RM0368, page 170, table 29.
 * This table defines USART1_TX as stream 7, channel 4, and USART1_RX as stream 2, channel 4.
 * Because USART1 is on APB2 with max Pclk of 84 MHz,
 * reachable baudrates are up to 10.5M with OVER8 or 5.25M with default OVER16 (per DocID025644 Rev3, page 30, table 6)
 */
#define USBUSART               USART1
#define USBUSART_CR1           USART1_CR1
#define USBUSART_DR            USART1_DR
#define USBUSART_IRQ           NVIC_USART1_IRQ
#define USBUSART_CLK           RCC_USART1
#define USBUSART_PORT          GPIOB
#define USBUSART_TX_PIN        GPIO6
#define USBUSART_RX_PIN        GPIO7
#define USBUSART_ISR(x)        usart1_isr(x)
#define USBUSART_DMA_BUS       DMA2
#define USBUSART_DMA_CLK       RCC_DMA2
#define USBUSART_DMA_TX_CHAN   DMA_STREAM7
#define USBUSART_DMA_TX_IRQ    NVIC_DMA2_STREAM7_IRQ
#define USBUSART_DMA_TX_ISR(x) dma2_stream7_isr(x)
#define USBUSART_DMA_RX_CHAN   DMA_STREAM2
#define USBUSART_DMA_RX_IRQ    NVIC_DMA2_STREAM2_IRQ
#define USBUSART_DMA_RX_ISR(x) dma2_stream2_isr(x)
/* For STM32F4 DMA trigger source must be specified */
#define USBUSART_DMA_TRG DMA_SxCR_CHSEL_4

#define SWD_CR       GPIO_MODER(SWDIO_PORT)
#define SWD_CR_SHIFT (0x4U << 0x1U)

#define TMS_SET_MODE()                                                        \
	do {                                                                      \
		gpio_set(TMS_DIR_PORT, TMS_DIR_PIN);                                  \
		gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TMS_PIN); \
	} while (0)

#define SWDIO_MODE_FLOAT()                                \
	do {                                                  \
		uint32_t cr = SWD_CR;                             \
		cr &= ~(0x3U << SWD_CR_SHIFT);                    \
		GPIO_BSRR(SWDIO_DIR_PORT) = SWDIO_DIR_PIN << 16U; \
		SWD_CR = cr;                                      \
	} while (0)

#define SWDIO_MODE_DRIVE()                         \
	do {                                           \
		uint32_t cr = SWD_CR;                      \
		cr &= ~(0x3U << SWD_CR_SHIFT);             \
		cr |= (0x1U << SWD_CR_SHIFT);              \
		GPIO_BSRR(SWDIO_DIR_PORT) = SWDIO_DIR_PIN; \
		SWD_CR = cr;                               \
	} while (0)

#define UART_PIN_SETUP()                                                                            \
	do {                                                                                            \
		gpio_mode_setup(USBUSART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USBUSART_TX_PIN);              \
		gpio_set_output_options(USBUSART_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, USBUSART_TX_PIN); \
		gpio_set_af(USBUSART_PORT, GPIO_AF7, USBUSART_TX_PIN);                                      \
		gpio_mode_setup(USBUSART_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USBUSART_RX_PIN);            \
		gpio_set_output_options(USBUSART_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, USBUSART_RX_PIN); \
		gpio_set_af(USBUSART_PORT, GPIO_AF7, USBUSART_RX_PIN);                                      \
	} while (0)

#define USB_DRIVER stm32f107_usb_driver
#define USB_IRQ    NVIC_OTG_FS_IRQ
#define USB_ISR(x) otg_fs_isr(x)
/*
 * Interrupt priorities. Low numbers are high priority.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB          (1U << 4U)
#define IRQ_PRI_USBUSART     (2U << 4U)
#define IRQ_PRI_USBUSART_DMA (2U << 4U)
#define IRQ_PRI_SWO_TIM      (3U << 4U)
#define IRQ_PRI_SWO_DMA      (1U << 4U)

/* Use TIM3 Input 2 from PC7/TDO, AF2, trigger on rising edge */
#define SWO_TIM             TIM3
#define SWO_TIM_CLK_EN()    rcc_periph_clock_enable(RCC_TIM3)
#define SWO_TIM_IRQ         NVIC_TIM3_IRQ
#define SWO_TIM_ISR(x)      tim3_isr(x)
#define SWO_IC_IN           TIM_IC_IN_TI2
#define SWO_IC_RISING       TIM_IC2
#define SWO_CC_RISING       TIM3_CCR2
#define SWO_ITR_RISING      TIM_DIER_CC2IE
#define SWO_STATUS_RISING   TIM_SR_CC2IF
#define SWO_IC_FALLING      TIM_IC1
#define SWO_CC_FALLING      TIM3_CCR1
#define SWO_STATUS_FALLING  TIM_SR_CC1IF
#define SWO_STATUS_OVERFLOW (TIM_SR_CC1OF | TIM_SR_CC2OF)
#define SWO_TRIG_IN         TIM_SMCR_TS_TI2FP2
#define SWO_TIM_PIN_AF      GPIO_AF2

/* On ctxLink use USART6 RX mapped on PC7 for async capture */
#define SWO_UART        USART6
#define SWO_UART_CLK    RCC_USART6
#define SWO_UART_DR     USART6_DR
#define SWO_UART_PORT   GPIOC
#define SWO_UART_RX_PIN GPIO7
#define SWO_UART_PIN_AF GPIO_AF8

/* Bind to the same DMA Rx channel */
#define SWO_DMA_BUS  DMA2
#define SWO_DMA_CLK  RCC_DMA2
#define SWO_DMA_CHAN DMA_STREAM1
#define SWO_DMA_IRQ  NVIC_DMA2_STREAM1_IRQ
#define SWO_DMA_ISR  dma2_stream1_isr
#define SWO_DMA_TRG  DMA_SxCR_CHSEL_5

#define SET_RUN_STATE(state)      \
	{                             \
		running_status = (state); \
	}
#define SET_IDLE_STATE(state)                        \
	{                                                \
		gpio_set_val(LED_PORT, LED_IDLE_RUN, state); \
	}
#define SET_ERROR_STATE(state)                    \
	{                                             \
		gpio_set_val(LED_PORT, LED_ERROR, state); \
	}

extern bool systick_tick;
void platform_tasks(void);
const char *platform_battery_voltage(void);
bool platform_check_battery_voltage(void);
bool platform_configure_uart(char *configuration_string);
void platform_read_adc(void);
const char *platform_wifi_state(int argc, const char **argv);
void wifi_connect(size_t argc, const char **argv, char *buffer, uint32_t size);

#endif /* PLATFORMS_CTXLINK_PLATFORM_H */
