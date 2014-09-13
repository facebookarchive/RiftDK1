/************************************************************************************

Filename    :   uart.h
Content     :   STM32 UART interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _UART_H_
#define _UART_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f10x.h"

#define UART_BUFFER_SIZE 64

typedef struct uart_struct {
    USART_TypeDef *port;
    uint8_t buf[UART_BUFFER_SIZE];
    uint32_t write_pos;
    uint32_t read_pos;
} uart_s, *uart_p;

void uart_init(uart_p uart, USART_TypeDef *port, uint32_t speed, GPIO_TypeDef *gpio, uint16_t tx, uint16_t rx);

void uart_write(uart_p uart, const uint8_t *buf, uint16_t len);

void uart_receive(USART_TypeDef *port);

bool uart_available(uart_p uart);

uint8_t uart_read(uart_p uart);

#endif /* _UART_H_ */
