/**
 * \file halUart.c
 *
 * \brief ATSAMD20 UART implementation
 *
 * Copyright (C) 2012-2014, Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 * $Id: halUart.c 9168 2014-02-04 22:39:33Z ataradov $
 *
 */

/*- Includes ---------------------------------------------------------------*/
#include <stdbool.h>
#include "hal.h"
#include "halUart.h"
#include "halGpio.h"
#include "config.h"

/*- Definitions ------------------------------------------------------------*/
#ifndef HAL_UART_TX_FIFO_SIZE
#define HAL_UART_TX_FIFO_SIZE  10
#endif

#ifndef HAL_UART_RX_FIFO_SIZE
#define HAL_UART_RX_FIFO_SIZE  10
#endif

// Ignore HAL_UART_CHANNEL setting. We always use SERCOM3.
HAL_GPIO_PIN(UART_TXD, A, 24);
HAL_GPIO_PIN(UART_RXD, A, 25);

/*- Types ------------------------------------------------------------------*/
typedef struct
{
  uint16_t  head;
  uint16_t  tail;
  uint16_t  size;
  uint16_t  bytes;
  uint8_t   *data;
} FifoBuffer_t;

/*- Variables --------------------------------------------------------------*/
static FifoBuffer_t txFifo;
static uint8_t txData[HAL_UART_TX_FIFO_SIZE+1];

static volatile FifoBuffer_t rxFifo;
static uint8_t rxData[HAL_UART_RX_FIFO_SIZE+1];

static volatile bool udrEmpty;
static volatile bool newData;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
static void halUartSync(void)
{
  while (SERCOM3->USART.STATUS.bit.SYNCBUSY);
}

/*************************************************************************//**
*****************************************************************************/
void HAL_UartInit(uint32_t baudrate)
{
  uint64_t brr = (uint64_t)65536 * (F_CPU - 16 * baudrate) / F_CPU;

  HAL_GPIO_UART_TXD_out();
  HAL_GPIO_UART_TXD_pmuxen();
  HAL_GPIO_UART_RXD_in();
  HAL_GPIO_UART_RXD_pmuxen();

  PORT->Group[HAL_GPIO_PORTA].PMUX[12].bit.PMUXE = 2/*C*/; // TX
  PORT->Group[HAL_GPIO_PORTA].PMUX[12].bit.PMUXO = 2/*C*/; // RX

  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM3;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM3_GCLK_ID_CORE) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  SERCOM3->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
      SERCOM_USART_CTRLB_CHSIZE(0/*8 bits*/);
  halUartSync();

  SERCOM3->USART.BAUD.reg = (uint16_t)brr;
  halUartSync();

  SERCOM3->USART.CTRLA.reg = SERCOM_USART_CTRLA_ENABLE |
      SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
      SERCOM_USART_CTRLA_RXPO(3/*PAD3*/) | SERCOM_USART_CTRLA_TXPO/*PAD2*/;
  halUartSync();

  SERCOM3->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
  NVIC_EnableIRQ(SERCOM3_IRQn);

  txFifo.data = txData;
  txFifo.size = HAL_UART_TX_FIFO_SIZE;
  txFifo.bytes = 0;
  txFifo.head = 0;
  txFifo.tail = 0;

  rxFifo.data = rxData;
  rxFifo.size = HAL_UART_RX_FIFO_SIZE;
  rxFifo.bytes = 0;
  rxFifo.head = 0;
  rxFifo.tail = 0;

  udrEmpty = true;
  newData = false;
}

/*************************************************************************//**
*****************************************************************************/
void HAL_UartWriteByte(uint8_t byte)
{
  if (txFifo.bytes == txFifo.size)
    return;

  txFifo.data[txFifo.tail++] = byte;
  if (txFifo.tail == txFifo.size)
    txFifo.tail = 0;
  txFifo.bytes++;
}

/*************************************************************************//**
*****************************************************************************/
uint8_t HAL_UartReadByte(void)
{
  uint8_t byte;

  ATOMIC_SECTION_ENTER
    byte = rxFifo.data[rxFifo.head++];
    if (rxFifo.head == rxFifo.size)
      rxFifo.head = 0;
    rxFifo.bytes--;
  ATOMIC_SECTION_LEAVE

  return byte;
}

/*************************************************************************//**
*****************************************************************************/
void HAL_IrqHandlerSercom3(void)
{
  uint8_t flags = SERCOM3->USART.INTFLAG.reg;

  if (flags & SERCOM_USART_INTFLAG_DRE)
  {
    udrEmpty = true;
    SERCOM3->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
  }

  if (flags & SERCOM_USART_INTFLAG_RXC)
  {
    uint16_t status = SERCOM3->USART.STATUS.reg;
    uint8_t byte = SERCOM3->USART.DATA.reg;

    if (0 == (status & (SERCOM_USART_STATUS_BUFOVF | SERCOM_USART_STATUS_FERR |
        SERCOM_USART_STATUS_PERR)))
    {
      if (rxFifo.bytes == rxFifo.size)
        return;

      rxFifo.data[rxFifo.tail++] = byte;
      if (rxFifo.tail == rxFifo.size)
        rxFifo.tail = 0;
      rxFifo.bytes++;

      newData = true;
    }
  }
}

/*************************************************************************//**
*****************************************************************************/
void HAL_UartTaskHandler(void)
{
  if (txFifo.bytes && udrEmpty)
  {
    uint8_t byte;

    byte = txFifo.data[txFifo.head++];
    if (txFifo.head == txFifo.size)
      txFifo.head = 0;
    txFifo.bytes--;

    ATOMIC_SECTION_ENTER
      SERCOM3->USART.DATA.reg = byte;
      SERCOM3->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
      udrEmpty = false;
    ATOMIC_SECTION_LEAVE
  }

  {
    uint16_t bytes;
    bool new;

    ATOMIC_SECTION_ENTER
      new = newData;
      newData = false;
      bytes = rxFifo.bytes;
    ATOMIC_SECTION_LEAVE

    if (new)
      HAL_UartBytesReceived(bytes);
  }
}
