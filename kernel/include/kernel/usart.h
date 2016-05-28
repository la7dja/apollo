/* This file is part of the OS kernel included with the Apollo firmware 
	 Copyright (C) 2016  Espen S. Johnsen, LA7DJA

	 This program is free software: you can redistribute it and/or modify
	 it under the terms of the GNU General Public License as published by
	 the Free Software Foundation, either version 3 of the License, or
	 (at your option) any later version.
	 
	 This program is distributed in the hope that it will be useful,
	 but WITHOUT ANY WARRANTY; without even the implied warranty of
	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	 GNU General Public License for more details.
	 
	 You should have received a copy of the GNU General Public License
	 along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef __USART_H__
#define __USART_H__

#include <stdio.h>
#include <kernel/kernel.h>

#if defined (__DOXYGEN__)
/**
 *  Parameters that can be specified when opening a USART device.
 */
typedef enum {
  USART_RX,                /*!< Enable receiver */
  USART_TX,                /*!< Enable transmitter */
  USART_ASYNC,             /*!< Enable asynchrnous mode (default) */
  USART_MASTER,            /*!< Enable synchronous master mode */
  USART_SLAVE,             /*!< Enable synchronous slave mode */
  USART_POLARITY_NORMAL,   /*!< TX on rising edge, RX on falling edge (default) */
  USART_POLARITY_INVERTED, /*!< TX on falling edge, RX on rising edge */
  USART_1_STOP_BIT,        /*!< Use one stop bit (default) */
  USART_2_STOP_BITS,       /*!< Use two stop bits */
  USART_NO_PARITY,         /*!< Use no parity (default) */
  USART_EVEN_PARITY,       /*!< Use even parity */
  USART_ODD_PARITY,        /*!< Use odd parity */
  USART_5_BITS,            /*!< Use five data bits */
  USART_6_BITS,            /*!< Use six data bits */
  USART_7_BITS,            /*!< Use seven data bits */
  USART_8_BITS,            /*!< Use eight data bits (default) */
#ifdef USART_9_BITS_SUPPORT
  USART_9_BITS,            /*!< Use nine data bits */
#endif
} usart_params_t;
#else
typedef enum {
  USART_RX = 1,
  USART_TX = 2,
  USART_1_STOP_BIT = 0,
  USART_2_STOP_BITS = 4,
  USART_NO_PARITY = 0,
  USART_EVEN_PARITY = 8,
  USART_ODD_PARITY = 16,
  USART_5_BITS = 32,
  USART_6_BITS = 64,
  USART_7_BITS = 96,
  USART_8_BITS = 0,
#ifdef USART_9_BITS_SUPPORT
  USART_9_BITS = 128,
#endif
  USART_ASYNC = 0,
  USART_MASTER = 256,
  USART_SLAVE = 512,
  USART_POLARITY_NORMAL = 0,
  USART_POLARITY_INVERTED = 1024
} usart_params_t;
#endif

#if defined(__DOXYGEN__)
/**
 *  Opens a stream to USART <n>
 *
 *  \param baudrate Baudrate to use (ignored in synchronous slave mode)
 *  \param params Parameters
 *
 *  Returns a pointer to a stream object
 */
FILE *usart<n>_open (uint32_t baudrate, usart_params_t params);
#else
FILE *usart0_open (uint32_t baudrate, usart_params_t params);
FILE *usart1_open (uint32_t baudrate, usart_params_t params);
FILE *usart2_open (uint32_t baudrate, usart_params_t params);
FILE *usart3_open (uint32_t baudrate, usart_params_t params);
#endif
	
void usart_set_timeout (FILE *stream, timeout_t timeout);

#endif /* __USART_H__ */
