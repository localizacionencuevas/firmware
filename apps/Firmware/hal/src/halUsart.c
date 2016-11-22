/**************************************************************************//**
\file  halUsart.c

\brief Implementación del módulo usart.
*******************************************************************************/
/******************************************************************************
                   Includes section
******************************************************************************/

#include "usart.h"
#include <stddef.h>


/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Pone a 1 una tarea específica en la máscara de tareas usart.
\param[in]  taskId  el identificador de la tarea.
******************************************************************************/
void halPostUsartTask(HalUsartTaskId_t taskId);

/******************************************************************************
                   External global variables section
******************************************************************************/
/** \brief DTR wake up habilitado. */
extern volatile bool halEnableDtrWakeUp;
/** \brief Función callback para DTR wake up. */
extern void (* dtrWakeUpCallback)(void);

/******************************************************************************
                   Implementations section
******************************************************************************/
void halSetUsartConfig(HAL_UsartDescriptor_t *usartMode)
{
  UCSRnB(usartMode->tty) = 0x00; // disable usart
  UBRRn(usartMode->tty) = usartMode->baudrate; // usart speed

  if (USART_MODE_ASYNC == usartMode->mode)
  {
    UCSRnA(usartMode->tty) = (uint8_t)USART_DOUBLE_SPEED << U2X0; // Double the USART Transmition Speed
    UCSRnC(usartMode->tty) = 0x00;
  }
  else
  {
    UCSRnA(usartMode->tty) = 0;
    UCSRnC(usartMode->tty) = usartMode->edge; // edge select
  }

  UCSRnC(usartMode->tty) |= usartMode->mode;
  UCSRnC(usartMode->tty) |= usartMode->dataLength;     // character size
  UCSRnC(usartMode->tty) |= usartMode->parity;   // parity mode
  UCSRnC(usartMode->tty) |= usartMode->stopbits; // stop bit select
  UCSRnA(usartMode->tty) |= (1 << RXC0); // clear receive interrupt
  UCSRnB(usartMode->tty) |= (1 << RXEN1) | (1 << TXEN1); // usart enable
  UCSRnB(usartMode->tty) |= (1 << RXCIE0) ; // receive interrupt enable
}

/**************************************************************************//**
 \brief Interrupción para USART0 - data register is empty.
******************************************************************************/
ISR(USART0_UDRE_vect)
{
  // We must disable the interrupt because we must "break" context.
  halDisableUsartDremInterrupt(USART_CHANNEL_0);
  halPostUsartTask(HAL_USART_TASK_USART0_DRE);
}

/**************************************************************************//**
 \brief Interrupción para USART0 - transmission is completed.
******************************************************************************/
ISR(USART0_TX_vect)
{
  halDisableUsartTxcInterrupt(USART_CHANNEL_0);
  halPostUsartTask(HAL_USART_TASK_USART0_TXC);
}

/**************************************************************************//**
 \brief Interrupción para USART0 - reception is completed.
******************************************************************************/
ISR(USART0_RX_vect)
{
  uint8_t  status = UCSR0A;
  uint8_t  data = UDR0;

  if (!(status & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))))
  {
    halUsartRxBufferFiller(USART_CHANNEL_0, data);
    halPostUsartTask(HAL_USART_TASK_USART0_RXC);
  }
  #if defined(_USE_USART_ERROR_EVENT_)
    else // There is an error in the received byte.
    {
      halUsartSaveErrorReason(USART_CHANNEL_0, status);
      halPostUsartTask(HAL_USART_TASK_USART0_ERR);
    }
  #endif

}


#if defined(HAL_USE_USART_CHANNEL_1)
/**************************************************************************//**
 \brief Interrupción para USART1 - data register is empty.
******************************************************************************/
ISR(USART1_UDRE_vect)
{
  // We must disable the interrupt because we must "break" context.
  halDisableUsartDremInterrupt(USART_CHANNEL_1);
  halPostUsartTask(HAL_USART_TASK_USART1_DRE);
}

/**************************************************************************//**
 \brief Interrupción para USART1 - transmission is completed.
******************************************************************************/
ISR(USART1_TX_vect)
{
  halDisableUsartTxcInterrupt(USART_CHANNEL_1);
  halPostUsartTask(HAL_USART_TASK_USART1_TXC);
}

/**************************************************************************//**
 \brief Interrupción para USART1 - reception is completed.
******************************************************************************/
ISR(USART1_RX_vect)
{
  uint8_t  status = UCSR1A;
  uint8_t  data = UDR1;

  if (!(status & ((1 << FE1) | (1 << DOR1) | (1 << UPE1))))
  {
    halUsartRxBufferFiller(USART_CHANNEL_1, data);
    halPostUsartTask(HAL_USART_TASK_USART1_RXC);
  }
  #if defined(_USE_USART_ERROR_EVENT_)
    else // There is an error in the received byte.
    {
      halUsartSaveErrorReason(USART_CHANNEL_1, status);
      halPostUsartTask(HAL_USART_TASK_USART1_ERR);
    }
  #endif
}
#endif

/**************************************************************************//**
\brief Interrupción externa 4 (DTR)
******************************************************************************/
ISR(INT4_vect)
{
  if (halEnableDtrWakeUp)
  { /* enable DTR (irq 4) wake up */
	EIMSK &= ~(1 << INT4);
  } /* enable DTR (irq 4) wake up */

  if (NULL != dtrWakeUpCallback)
    dtrWakeUpCallback();
}
// eof halUsart.c

