/*
 * This file is part of the hoverboard-firmware-hack project.
 *
 * Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
 * Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
 * Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
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

#ifndef UART_H_
#define UART_H_

#include "config.h"

#define 	USART_IT_PE   	USART_INT_PERR
#define 	USART_IT_TXE   	USART_INT_TDE
#define 	USART_IT_TC   	USART_INT_TRAC
#define 	USART_IT_RXNE   USART_INT_RDNE
#define 	USART_IT_IDLE   USART_INT_IDLEF
#define 	USART_IT_LBD   	USART_INT_LBDF
#define 	USART_IT_CTS   	USART_INT_CTSF
#define 	USART_IT_ERR   	USART_INT_ERR
#define 	USART_IT_ORE   	USART_INT_ORERR
#define 	USART_IT_NE   	USART_INT_NERR
#define 	USART_IT_FE   	USART_INT_FERR

#define NL "\n\r"

#define UART_RX_BUF_SIZE 512
volatile char uart_rxBuff[UART_RX_BUF_SIZE];

#define UART_TX_BUF_SIZE 512
volatile char uart_txBuff[UART_TX_BUF_SIZE];

char uart_command[UART_RX_BUF_SIZE];

typedef struct {
	volatile char *const buffer;
	uint8_t head;
	uint8_t tail;
} circ_buffer_t;

void uart_initialize(void);
void uart_handle_command();
void uart_put_string(char *s);
int8_t uart_read_command();
int8_t uart_put_char(char data);
int8_t uart_get_char(char *data);
int8_t startswith(const char *a, const char *b);
#endif /* UART_H_ */
