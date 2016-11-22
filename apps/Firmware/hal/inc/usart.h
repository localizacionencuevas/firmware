/****************************************************************************//**
  \file  usart.h

  \brief Describe la interfaz usart.
*******************************************************************************/
#ifndef _USART_H
#define _USART_H

/******************************************************************************
                   Includes section
******************************************************************************/

#include "halUsart.h"

/******************************************************************************
                   Define(s) section
******************************************************************************/
/** \brief sin control de flujo */
#define USART_FLOW_CONTROL_NONE 0
/** \brief con control de flujo hardware */
#define USART_FLOW_CONTROL_HARDWARE (1 << 0)
/** \brief con control DTR */
#define USART_DTR_CONTROL (1 << 1)

/** \brief modo lectura */
#define USART_SPI_READ_MODE  (1 << 4)
/** \brief modo escritura */
#define USART_SPI_WRITE_MODE (1 << 3)


/******************************************************************************
                   Types section
******************************************************************************/
/** \brief Descriptor usart */
typedef struct
{
  /** \brief servicio HAL USART - contiene variables para el módulo HAL USART. */
  HalUsartService_t service;
  /** \brief tty - canal USART_CHANNEL_n a ser usado. El rango "n" depende de la plataforma. */
  UsartChannel_t tty;
  /** \brief Configura el modo síncrono o asíncrono. Valores posibles: \n
    USART_MODE_ASYNC \n
    USART_MODE_SYNC  \n */
  UsartMode_t mode;
  /** \brief baudrate - USART baud rate. Valores posibles: \n
    USART_BAUDRATE_1200  \n
    USART_BAUDRATE_2400  \n
    USART_BAUDRATE_4800  \n
    USART_BAUDRATE_9600 \n
    USART_BAUDRATE_19200 \n
    USART_BAUDRATE_38400 \n
    USART_SYNC_BAUDRATE_1200 \n
    USART_SYNC_BAUDRATE_2400 \n
    USART_SYNC_BAUDRATE_4800 \n
    USART_SYNC_BAUDRATE_9600 \n
    USART_SYNC_BAUDRATE_38400 \n
    USART_SYNC_BAUDRATE_57600 \n
    USART_SYNC_BAUDRATE_115200  \n */
  UsartBaudRate_t baudrate;
  /** \brief data -  longitud de datos. Valores posibles: \n
    USART_DATA5 \n
    USART_DATA6 \n
    USART_DATA7 \n
    USART_DATA8 \n */
  UsartData_t dataLength;
  /** \brief parity -  modo de paridad. Valores posibles: \n
    USART_PARITY_NONE \n
    USART_PARITY_EVEN \n
    USART_PARITY_ODD \n  */
  UsartParity_t parity;
  /** \brief stopbits - número de bits de parada. Valores posibles: \n
    USART_STOPBIT_1 \n
    USART_STOPBIT_2 \n */
  UsartStopBits_t stopbits;
  /** \brief edge - modo de flanco (solo para usart) de datos recibidos. Valores posibles: \n
    USART_EDGE_MODE_FALLING \n
    USART_EDGE_MODE_RISING  \n */
  UsartEdgeMode_t edge;
  /** \brief Maestro o esclavo en la usart (solo para usart). Valores posibles: \n
    USART_CLK_MODE_MASTER \n
    USART_CLK_MODE_SLAVE  \n */
  UsartClkMode_t syncMode;
  /** \brief Puntero al búffer de recepción. \n
   Si rxBuffer es NULL las transacciones son descartadas. \n
   El tamaño del búffer depende de la aplicación. */
  uint8_t *rxBuffer;
  /** \brief Longitud del búffer de recepción. */
  uint16_t rxBufferLength;
  /** \brief Puntero al búffer de envío. \n
   Si txBuffer es NULL se utiliza el método por callback. \n
   Si txBuffer no es NULL se utiliza el método por polling. */
  uint8_t *txBuffer;
  /** \brief Longitud del búffer de envío. */
  uint16_t txBufferLength;
  /** \brief Función callback de recepción de datos. \n
   Si rxCallback es NULLse utiliza el método por polling.  \n
   Si rxCallback no es NULL se utiliza el método por callback. */
  void (*rxCallback)(uint16_t);
  /** \brief Función callback de envío de datos. \n
   Si txBuffer no es NULL txCallback notifica cuando termina de enviar los bytes. */
  void (*txCallback)(void);
  #if defined(_USE_USART_ERROR_EVENT_)
    /** \brief Función callback de error. \n
    Razones válidas: \n
    FRAME_ERROR  \n
    DATA_OVERRUN \n
    PARITY_ERROR */
    void (*errCallback)(UsartErrorReason_t);
  #endif
  /** \brief Control de flujo de la usart. Se puede utilizar: \n
   USART_FLOW_CONTROL_NONE, \n
   USART_FLOW_CONTROL_HARDWARE, USART_DTR_CONTROL, es posible combinarlos con un OR. \n
  */
  uint8_t flowControl;
} HAL_UsartDescriptor_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Abre la usart. Realiza la configuración de los registros de la usart y 
la configuración de los pines RTS, CTS y DTR.

\param[in]	descriptor	puntero al descriptor HAL_UsartDescriptor_t.

\retval -1				mal uso del canal o no hay suficientes recursos.
\retval num_positivo	en otro caso.
******************************************************************************/
int HAL_OpenUsart(HAL_UsartDescriptor_t *descriptor);

/*************************************************************************//**
\brief Libera el cana lusart y los pines.

\param[in]	descriptor	puntero al descriptor HAL_UsartDescriptor_t.

\retval		-1	descriptor erróneo o el canal ya estaba cerrado.
\retval		0	éxito.
*****************************************************************************/
int HAL_CloseUsart(HAL_UsartDescriptor_t *descriptor);

/**************************************************************************//**
\brief Escribe un número de bytes en el canal usart.
La función txCallback se utilizará para notificar cuando la transmisión finalice. 
Si el control de flujo hardware se utiliza para la transmisión los pines RTS 
y DTR se testearán duarante la transmisión.

\param[in]	descriptor	puntero al descriptor HAL_UsartDescriptor_t.
\param[in]  buffer		puntero al búffer de datos.
\param[in]  length		número de bytes a transferir.

\retval		-1			descriptor erróneo.
\retval		otro		número de bytes escritos en el búffer usart, éxito.
******************************************************************************/
int HAL_WriteUsart(HAL_UsartDescriptor_t *descriptor, uint8_t *buffer, uint16_t length);

/*************************************************************************//**
\brief Lee un número de bytes de la usart y los almacena en el búffer.

\param[in]		descriptor	puntero al descriptor HAL_UsartDescriptor_t.
\param[in, out]	buffer		puntero al búffer donde se almacenan los datos leídos.
\param[in]		length		número de bytes a almacenar en el búffer.

\retval		-1				descriptor erróneo o número de bytes leídos erróneo.
\retval		otro			número de bytes leídos y almacenados en el búffer, éxito.
*****************************************************************************/
int HAL_ReadUsart(HAL_UsartDescriptor_t *descriptor, uint8_t *buffer, uint16_t length);

/**************************************************************************//**
\brief Prohibe al host transmitir datos. Solo USART_CHANNEL_1 puede ser usado 
para el control de flujo hardware en avr.

\param[in]	descriptor	puntero al descriptor HAL_UsartDescriptor_t.

\retval		-1			descriptor erróneo, usart errónea o modo no soportado.
\retval		0			éxito.
******************************************************************************/
int HAL_OnUsartCts(HAL_UsartDescriptor_t *descriptor);

/**************************************************************************//**
\brief Permite al host transmitir datos. Solo USART_CHANNEL_1 puede ser usado
para el control de flujo hardware en avr.

\param[in]	descriptor	puntero al descriptor HAL_UsartDescriptor_t.

\retval		-1			descriptor erróneo, usart errónea o modo no soportado.
\retval		0			éxito.
******************************************************************************/
int HAL_OffUsartCts(HAL_UsartDescriptor_t *descriptor);

/**************************************************************************//**
\brief Lee el estado del pin RTS. Solo USART_CHANNEL_1 puede ser usado
para el control de flujo hardware en avr.

\param[in]	descriptor	puntero al descriptor HAL_UsartDescriptor_t.

\retval		-1			descriptor erróneo, usart errónea o modo no soportado.
\retval		0			RTS está a nivel bajo.
\retval		1			RTS	está a nivel alto.
******************************************************************************/
int HAL_ReadUsartRts(HAL_UsartDescriptor_t *descriptor);

/**************************************************************************//**
\brief Lee el estado del pin DTR. Solo USART_CHANNEL_1 puede ser usado
para el control de flujo hardware en avr.

\param[in]	descriptor	puntero al descriptor HAL_UsartDescriptor_t.

\retval		-1			descriptor erróneo, usart errónea o modo no soportado.
\retval		0			DTR está a nivel bajo.
\retval		1			DTR	está a nivel alto.
******************************************************************************/
int HAL_ReadUsartDtr(HAL_UsartDescriptor_t *descriptor);

/**************************************************************************//**
\brief Comprueba el estado del buffer tx.
\param[in]	descriptor	puntero al descriptor HAL_UsartDescriptor_t.

\retval		-1			descriptor erróneo, no hay búffer tx.
\retval		0			El búffer tx no está vacío.
\retval		1			El búffer tx está vacío.
******************************************************************************/
int HAL_IsTxEmpty(HAL_UsartDescriptor_t *descriptor);

#endif /* _USART_H */
// eof usart.h

