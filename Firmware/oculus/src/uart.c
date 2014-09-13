/************************************************************************************

Filename    :   uart.c
Content     :   STM32 UART interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "uart.h"

uart_p uarts[3] = {0};

void uart_init(uart_p uart, USART_TypeDef *port, uint32_t speed, GPIO_TypeDef *gpio, uint16_t tx, uint16_t rx)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint8_t irq;

    uart->port = port;
    uart->write_pos = 0;
    uart->read_pos = 0;

    // Start the clocks
    if (port == USART1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        uarts[0] = uart;
        irq = USART1_IRQn;
    } else if (port == USART2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        uarts[1] = uart;
        irq = USART2_IRQn;
    } else if (port == USART3) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        uarts[2] = uart;
        irq = USART3_IRQn;
    } else {
        return;
    }

    // Configure the tx and rx pins
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = tx;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = rx;
    GPIO_Init(gpio, &GPIO_InitStructure);

    // Set up the data received interrupt
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Do the actual port setup
    USART_InitStructure.USART_BaudRate = speed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(port, &USART_InitStructure);
    USART_ITConfig(port, USART_IT_RXNE, ENABLE);
    USART_Cmd(port, ENABLE);
}

void uart_write(uart_p uart, const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        USART_SendData(uart->port, buf[i]);
        // Wait for the tx register to be ready again
        while(USART_GetFlagStatus(uart->port, USART_FLAG_TXE) == RESET);
    }
}

void uart_receive(USART_TypeDef *port)
{
    if (USART_GetITStatus(port, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(port);

        // Get the right ring buffer for the port
        uart_p uart = 0;
        if (port == USART1)
            uart = uarts[0];
        else if (port == USART2)
            uart = uarts[1];
        else if (port == USART3)
            uart = uarts[2];

        if (uart) {
            uart->buf[uart->write_pos % UART_BUFFER_SIZE] = data;
            uart->write_pos++;
        }
    }

    // clear overrun
    if (USART_GetFlagStatus(port, USART_FLAG_ORE) != RESET)
        USART_ReceiveData(port);
}

bool uart_available(uart_p uart)
{
    return uart->write_pos > uart->read_pos;
}

uint8_t uart_read(uart_p uart)
{
    uint8_t ret = 0;

    // TODO: handle wrapping, handle unavailable differently?
    if (uart_available(uart)) {
        ret = uart->buf[uart->read_pos % UART_BUFFER_SIZE];
        uart->read_pos++;
    }

    return ret;
}
