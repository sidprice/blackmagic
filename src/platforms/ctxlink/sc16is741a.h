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

#ifndef SC16IS741A_H
#define SC16IS741A_H

#include <stdint.h>

// Define SPI/I2C selection
#define SC16IS741A_USE_I2C 1
#define SC16IS741A_USE_SPI 0

// Register addresses
#define SC16IS741A_RHR 0x00 // Receive Holding Register (Read)
#define SC16IS741A_THR 0x00 // Transmit Holding Register (Write)
#define SC16IS741A_IER 0x01 // Interrupt Enable Register
#define SC16IS741A_FCR 0x02 // FIFO Control Register
#define SC16IS741A_LCR 0x03 // Line Control Register
#define SC16IS741A_MCR 0x04 // Modem Control Register
#define SC16IS741A_LSR 0x05 // Line Status Register
#define SC16IS741A_MSR 0x06 // Modem Status Register
#define SC16IS741A_SPR 0x07 // Scratchpad Register

#define SC16IS741A_TCR 0x06 // Transmission Control Register (When MCR[2] == 1 and EFR[4] == 1)
#define SC16IS741A_TLR 0x07 // Trigger Level Register (When MCR[2] == 1 and EFR[4] == 1)

#define SC16IS741A_TXLVL 0x08 // Transmit Fifo Level Register (Read only)
#define SC16IS741A_RXLVL 0x08 // Receive Fifo Level Register (Read only)

#define SC16IS741A_UART_RESET 0x0E // UART Reset Register
#define SC16IS741A_EFCR       0x0F // Extra Features Register

// Special Register Set (Accessible only when LCR[7] == 1 and LCR != 0xBF)
#define SC16IS741A_DLL 0x00 // Divisor Latch LSB Register
#define SC16IS741A_DLH 0x01 // Divisor Latch MSB Register

// Enhanced Register Set (Accessible only when LCR == 0xBF)
#define SC16IS741A_EFR   0x02 // Enhanced Feature Register
#define SC16IS741A_XON1  0x04 // XON Word 1 Register
#define SC16IS741A_XON2  0x05 // XON Word 2 Register
#define SC16IS741A_XOFF1 0x06 // XOFF Word 1 Register
#define SC16IS741A_XOFF2 0x07 // XOFF Word 2 Register

// Register bit definitions
// Interrupt Enable Register (IER) bits
#define SC16IS741A_IER_RHR       0x01 // Enable Received Data Available Interrupt
#define SC16IS741A_IER_THR       0x02 // Enable Transmitter Holding Register Empty Interrupt
#define SC16IS741A_IER_RLS       0x04 // Enable Receiver Line Status Interrupt
#define SC16IS741A_IER_MSI       0x08 // Enable Modem Status Interrupt
#define SC16IS741A_IER_MSI_SLEEP 0x10 // Enable Sleep Mode Interrupt
#define SC16IS741A_IER_NXOFF     0x20 // Enable XOFF Interrupt
#define SC16IS741A_IER_NRTS      0x40 // Enable RTS Interrupt
#define SC16IS741A_IER_NCTS      0x80 // Enable CTS Interrupt

// FIFO Control Register (FCR) bits
#define SC16IS741A_FCR_ENABLE   0x01 // Enable FIFOs (Write only)
#define SC16IS741A_FCR_RX_RESET 0x02 // Reset Receiver FIFO (Write only)
#define SC16IS741A_FCR_TX_RESET 0x04 // Reset Transmitter FIFO (Write only)

#define SC16IS741A_FCR_TX_TRIGGER_LSB 0x10 // TX Trigger Level LSB (Write only)
#define SC16IS741A_FCR_TX_TRIGGER_MSB 0x20 // TX Trigger Level MSB (Write only)
#define SC16IS741A_FCR_RX_TRIGGER_LSB 0x40 // RX Trigger Level LSB (Write only)
#define SC16IS741A_FCR_RX_TRIGGER_MSB 0x80 // RX Trigger Level LSB (Write only)

// Line Control Register (LCR) bits
#define SC16IS741A_LCR_WORD_LEN_5   0x00 // 5-bit word length
#define SC16IS741A_LCR_WORD_LEN_6   0x01 // 6-bit word length
#define SC16IS741A_LCR_WORD_LEN_7   0x02 // 7-bit word length
#define SC16IS741A_LCR_WORD_LEN_8   0x03 // 8-bit word length
#define SC16IS741A_LCR_STOP_BITS    0x04 // Number of stop bits
#define SC16IS741A_LCR_PARITY_EN    0x08 // Enable parity
#define SC16IS741A_LCR_PARITY_EVEN  0x10 // Even parity
#define SC16IS741A_LCR_BREAK_CTRL   0x40 // Set Break Control bit
#define SC16IS741A_LCR_DIV_LATCH_EN 0x80 // Divisor Latch Enable

// Modem Control Register (MCR) bits
#define SC16IS741A_MCR_RTS        0x02 // Request to Send
#define SC16IS741A_MCR_TCR_LCR_EN 0x04 // Enable TCR and LCR
#define SC16IS741A_MCR_LOOPBACK   0x10 // Enable Loopback Mode
#define SC16IS741A_MCR_XON_ANY    0x20 // TODO What is this?
#define SC16IS741A_MCR_IRDA_EN    0x40 // IrDA Enable
#define SC16IS741A_MCR_CLK_DIV    0x80 // Clock Divisor

// Line Status Register (LSR) bits
#define SC16IS741A_LSR_DATA_READY    0x01 // Data ready
#define SC16IS741A_LSR_OVERRUN_ERR   0x02 // Overrun error
#define SC16IS741A_LSR_PARITY_ERR    0x04 // Parity error
#define SC16IS741A_LSR_FRAMING_ERR   0x08 // Framing error
#define SC16IS741A_LSR_THR_EMPTY     0x20 // Transmitter Holding Register Empty
#define SC16IS741A_LSR_THR_TSR_EMPTY 0x40 // Transmitter Shift Register Empty
#define SC16IS741A_LSR_FIFO_ERR      0x80 // Fifo data error

// Modem Status Register (MSR) bits
#define SC16IS741A_MSR_CTS 0x10 // Clear to Send

// Extra Features Register bits
#define SC16IS741A_EFCR_9BIT_EN    0x01 // Enable 9-bit mode
#define SC16IS741A_EFCR_RX_DISABLE 0x02 // Disable Receiver
#define SC16IS741A_EFCR_TX_DISABLE 0x04 // Disable Transmitter
#define SC16IS741A_EFCR_RTS_DIR    0x10 // RTS Direction
#define SC16IS741A_EFCR_RTS_INV    0x20 // RTS Inversion
#define SC16IS741A_EFCR_IRDA_MODE  0x80 // IrDA mode

// Enhanced Feature Register bits
#define SC16IS741A_EFR_SFLOW0       0x00 // Software Flow Control 0
#define SC16IS741A_EFR_SFLOW1       0x01 // Software Flow Control 1
#define SC16IS741A_EFR_SFLOW2       0x02 // Software Flow Control 2
#define SC16IS741A_EFR_SFLOW3       0x03 // Software Flow Control 3
#define SC16IS741A_EFR_EN_ADV       0x10 // Enable Enhanced Functions
#define SC16IS741A_EFR_SPECIAL_CHAR 0x20 // Special Character Detect
#define SC16IS741A_EFR_AUTO_RTS     0x40 // Special Character Detect
#define SC16IS741A_EFR_AUTO_CTS     0x80 // Special Character Detect

// Function prototypes
void sc16is741a_init(uint32_t spi, uint32_t port, uint32_t pin);
void sc16is741a_set_baud_rate(uint32_t baudrate);
void sc16is741a_set_line_config(uint8_t word_length, uint8_t stop_bits, uint8_t parity);
void sc16is741a_enable_fifo(uint8_t enable);
void sc16is741a_write_byte(uint8_t data);
uint8_t sc16is741a_read_byte(void);
uint8_t sc16is741a_read_status(void);
void sc16is741a_enable_interrupts(uint8_t interrupts);
void sc16is741a_disable_interrupts(uint8_t interrupts);
void sc16is741a_reset(void);
uint8_t sc16is741a_tx_fifo_level(void);
#endif // SC16IS741A_H