/************************************************************************************

Filename    :   spi.c
Content     :   STM32 SPI peripheral interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "spi.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"
#include <stdbool.h>

// To make sure we never get stuck forever
#define SPI_TIMEOUT  (0xFFFF)

void spi_init(spi_p spi, SPI_TypeDef *spi_port, uint16_t speed, uint32_t rcc, GPIO_TypeDef* gpio, uint32_t gpio_rcc, uint16_t miso, uint16_t mosi, uint16_t sck, uint16_t ss)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    spi->spi_port = spi_port;
    spi->gpio_port = gpio;
    spi->ss_pin = ss;
    spi->rcc = rcc;

    // Configure the clocks
    if (IS_SPI_23_PERIPH(spi_port)) {
        RCC_APB1PeriphClockCmd(spi->rcc, ENABLE);
        RCC_APB1PeriphClockCmd(gpio_rcc, ENABLE);
    } else {
        RCC_APB2PeriphClockCmd(spi->rcc, ENABLE);
        RCC_APB2PeriphClockCmd(gpio_rcc | RCC_APB2Periph_AFIO, ENABLE);
    }

    // Configure out pins for SPI Master
    GPIO_InitStructure.GPIO_Pin = sck | mosi;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(gpio, &GPIO_InitStructure);
    // Configure in pin for SPI Master
    GPIO_InitStructure.GPIO_Pin = miso;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(gpio, &GPIO_InitStructure);

    spi_port_config(spi, speed);

    GPIO_InitStructure.GPIO_Pin = ss;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(gpio, &GPIO_InitStructure);
    // Don't select the slave to start with
    GPIO_SetBits(gpio, ss);
}

void spi_deinit(spi_p spi)
{
    SPI_Cmd(spi->spi_port, DISABLE);
    SPI_I2S_DeInit(spi->spi_port);

    // Shut off the clocks
    if (IS_SPI_23_PERIPH(spi->spi_port)) {
        RCC_APB1PeriphClockCmd(spi->rcc, DISABLE);
    } else {
        RCC_APB2PeriphClockCmd(spi->rcc, DISABLE);
    }
}

void spi_port_config(spi_p spi, uint16_t speed)
{
    SPI_Cmd(spi->spi_port, DISABLE);
    SPI_InitTypeDef   SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = speed;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(spi->spi_port, &SPI_InitStructure);
    SPI_Cmd(spi->spi_port, ENABLE);
}

void spi_port_alt_config(spi_p spi, uint16_t speed)
{
    SPI_Cmd(spi->spi_port, DISABLE);
    SPI_InitTypeDef   SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = speed;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(spi->spi_port, &SPI_InitStructure);
    SPI_Cmd(spi->spi_port, ENABLE);
}

static bool spi_wait_write(spi_p spi)
{
    uint32_t timeout = SPI_TIMEOUT;

    while (SPI_I2S_GetFlagStatus(spi->spi_port, SPI_I2S_FLAG_TXE) == RESET) {
        if (!timeout--) {
            // unselect the slave before quitting if we time out
            GPIO_SetBits(spi->gpio_port, spi->ss_pin);
            return 0;
        }
    }

    return 1;
}

static bool spi_wait_read(spi_p spi)
{
    uint32_t timeout = SPI_TIMEOUT;

    while (SPI_I2S_GetFlagStatus(spi->spi_port, SPI_I2S_FLAG_RXNE) == RESET) {
        if (!timeout--) {
            GPIO_SetBits(spi->gpio_port, spi->ss_pin);
            return 0;
        }
    }

    return 1;
}

void spi_write_raw(spi_p spi, uint8_t data)
{
    GPIO_ResetBits(spi->gpio_port, spi->ss_pin);

    if (!spi_wait_write(spi)) return;
    SPI_I2S_SendData(spi->spi_port, data);

    GPIO_SetBits(spi->gpio_port, spi->ss_pin);
}

void spi_write(spi_p spi, uint8_t reg, uint8_t data)
{
    GPIO_ResetBits(spi->gpio_port, spi->ss_pin);

    if (!spi_wait_write(spi)) return;
    SPI_I2S_SendData(spi->spi_port, reg);
    // Read the zero that the slave is sending back
    if (!spi_wait_read(spi)) return;
    SPI_I2S_ReceiveData(spi->spi_port);

    if (!spi_wait_write(spi)) return;
    SPI_I2S_SendData(spi->spi_port, data);
    // Read the zero that the slave is sending back
    if (!spi_wait_read(spi)) return;
    SPI_I2S_ReceiveData(spi->spi_port);

    GPIO_SetBits(spi->gpio_port, spi->ss_pin);
}

void spi_rw(spi_p spi, uint8_t reg, uint8_t *write_buf, uint8_t *read_buf, uint16_t len)
{
    GPIO_ResetBits(spi->gpio_port, spi->ss_pin);

    if (!spi_wait_write(spi)) return;
    SPI_I2S_SendData(spi->spi_port, reg);
    // Read the zero that the slave is sending back
    if (!spi_wait_read(spi)) return;
    SPI_I2S_ReceiveData(spi->spi_port);

    // TODO: some kind of error handling so we don't get stuck
    uint16_t pos;
    for (pos = 0; pos < len; pos++) {
        if (!spi_wait_write(spi)) return;
        SPI_I2S_SendData(spi->spi_port, write_buf[pos]);
        if (!spi_wait_read(spi)) return;
        read_buf[pos] = SPI_I2S_ReceiveData(spi->spi_port);
    }

    GPIO_SetBits(spi->gpio_port, spi->ss_pin);
}

void spi_read(spi_p spi, uint8_t reg, uint8_t *buf, uint16_t len)
{
    GPIO_ResetBits(spi->gpio_port, spi->ss_pin);

    if (!spi_wait_write(spi)) return;
    SPI_I2S_SendData(spi->spi_port, reg);
    // Read the zero that the slave is sending back
    if (!spi_wait_read(spi)) return;
    SPI_I2S_ReceiveData(spi->spi_port);

    // TODO: some kind of error handling so we don't get stuck
    uint16_t pos;
    for (pos = 0; pos < len; pos++) {
        if (!spi_wait_write(spi)) return;
        SPI_I2S_SendData(spi->spi_port, 0);
        if (!spi_wait_read(spi)) return;
        buf[pos] = SPI_I2S_ReceiveData(spi->spi_port);
    }

    GPIO_SetBits(spi->gpio_port, spi->ss_pin);
}
