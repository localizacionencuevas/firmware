/*****************************************************************************//**
\file  halUsart.h
\brief  Declaraciones del módulo usart.
**********************************************************************************/


#ifndef _HAL_USART_H
#define _HAL_USART_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <halGpio.h>
#include <stdbool.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
/** \brief Canal usart. */
#define HAL_USE_USART_CHANNEL_0
/** \brief Devuelve el byte de la dirección mem_addr. */
#define MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
/** \brief Devuelve la palabra de la dirección mem_addr. */
#define MMIO_WORD(mem_addr) (*(volatile uint16_t *)(mem_addr))
/* if USART_DOUBLE_SPEED is 1 the USART uses U2Xn bit (Double speed the usart transmition).
   if USART_DOUBLE_SPEED is 0 then U2Xn bit is not been used.
 */
#ifndef USART_DOUBLE_SPEED
/**  \brief Velociad usart. */
  #define USART_DOUBLE_SPEED 1ul
#endif
/** \brief Dirección de comienzo del canal 0 de la usart  para AtMega1281/2561. */
#define USART_CHANNEL_0  0xC0 // USART0 AtMega1281/2561 start address
/** \brief Dirección de comienzo del canal 1 de la usart  para AtMega1281/2561. */
#define USART_CHANNEL_1  0xC8 // USART1 AtMega1281/2561 start address

#if NUM_USART_CHANNELS == 2
/** \brief Devuelve el índice del canal */
  #define HAL_GET_INDEX_BY_CHANNEL(channel)  ((channel - USART_CHANNEL_0) >> 3)
#else
/** \brief Devuelve el índice del canal */
  #define HAL_GET_INDEX_BY_CHANNEL(channel)  (channel - channel)
#endif
/** \brief USART MSPIM Control and Status Register n A, ver pág 238 del datasheet de AtMega1281. */
#define UCSRnA(tty) MMIO_BYTE(tty + 0)
/** \brief USART MSPIM Control and Status Register n B, ver pág 238 del datasheet de AtMega1281. */
#define UCSRnB(tty) MMIO_BYTE(tty + 1)
/** \brief USART MSPIM Control and Status Register n C, ver pág 239 del datasheet de AtMega1281. */
#define UCSRnC(tty) MMIO_BYTE(tty + 2)
/** \brief UBRRnL and UBRRnH – USART MSPIM Baud Rate Registers, ver pág 240 del datasheet de AtMega1281. */
#define UBRRnL(tty) MMIO_BYTE(tty + 4)
/** \brief UBRRnL and UBRRnH – USART MSPIM Baud Rate Registers, ver pág 240 del datasheet de AtMega1281. */
#define UBRRnH(tty) MMIO_BYTE(tty + 5)
/** \brief UBRRn – USART MSPIM Baud Rate Registers, ver pág 240 del datasheet de AtMega1281. */
#define UBRRn(tty)  MMIO_WORD(tty + 4)
/** \brief UDRn – USART MSPIM I/O Data Register, ver pág 237 del datasheet de AtMega1281. */
#define UDRn(tty)   MMIO_BYTE(tty + 6)//

/******************************************************************************
                   Types section
******************************************************************************/
/** \brief Canal usart */
typedef uint8_t UsartChannel_t;

/** \brief Velocidad */
typedef enum
{
  USART_BAUDRATE_1200 =  (unsigned)((F_CPU * (USART_DOUBLE_SPEED + 1ul)) / (16ul * 1200ul) - 1ul), // 1200 baud rate
  USART_BAUDRATE_2400 =  (unsigned)((F_CPU * (USART_DOUBLE_SPEED + 1ul)) / (16ul * 2400ul) - 1ul), // 2400 baud rate
  USART_BAUDRATE_4800 =  (unsigned)((F_CPU * (USART_DOUBLE_SPEED + 1ul)) / (16ul * 4800ul) - 1ul), // 4800 baud rate
  USART_BAUDRATE_9600 =  (unsigned)((F_CPU * (USART_DOUBLE_SPEED + 1ul)) / (16ul * 9600ul) - 1ul), // 9600 baud rate
  USART_BAUDRATE_19200 =  (unsigned)((F_CPU * (USART_DOUBLE_SPEED + 1ul)) / (16ul * 19200ul) - 1ul), // 19200 baud rate
  USART_BAUDRATE_38400 = (unsigned)((F_CPU * (USART_DOUBLE_SPEED + 1ul)) / (16ul * 38400ul) - 1ul), // 38400 baud rate
  USART_BAUDRATE_115200 = (unsigned)((F_CPU * (USART_DOUBLE_SPEED + 1ul)) / (16ul * 115200ul)), // 115200 baud rate
  USART_SYNC_BAUDRATE_1200  =  (uint16_t)((F_CPU / (2ul * 1200ul)) - 1ul),
  USART_SYNC_BAUDRATE_2400  =  (uint16_t)((F_CPU / (2ul * 2400ul)) - 1ul),
  USART_SYNC_BAUDRATE_4800  =  (uint16_t)((F_CPU / (2ul * 4800ul)) - 1ul),
  USART_SYNC_BAUDRATE_9600  =  (uint16_t)((F_CPU / (2ul * 9600ul)) - 1ul),
  USART_SYNC_BAUDRATE_38400 =  (uint16_t)((F_CPU / (2ul * 38400ul)) - 1ul),
  USART_SYNC_BAUDRATE_57600 =  (uint16_t)((F_CPU / (2ul * 57600ul)) - 1ul),
  USART_SYNC_BAUDRATE_115200 = (uint16_t)((F_CPU / (2ul * 115200ul)) - 1ul)
} UsartBaudRate_t;

/** \brief Longitud de datos */
typedef enum
{
  USART_DATA5 = (0 << UCSZ12) | (0 << UCSZ11) | (0 << UCSZ10), // 5 bits data length
  USART_DATA6 = (0 << UCSZ12) | (0 << UCSZ11) | (1 << UCSZ10), // 6 bits data length
  USART_DATA7 = (0 << UCSZ12) | (1 << UCSZ11) | (0 << UCSZ10), // 7 bits data length
  USART_DATA8 = (0 << UCSZ12) | (1 << UCSZ11) | (1 << UCSZ10), // 8 bits data length
} UsartData_t;

/** \brief Modo de paridad */
typedef enum
{
  USART_PARITY_NONE = (0 << UPM11) | (0 << UPM10), // Non parity mode
  USART_PARITY_EVEN = (1 << UPM11) | (0 << UPM10), // Even parity mode
  USART_PARITY_ODD =  (1 << UPM11) | (1 << UPM10)  // Odd parity mode
} UsartParity_t;

/** \brief Número de bits de parada. */
typedef enum
{
  USART_STOPBIT_1 = (0 << USBS1), // 1 stop bits mode
  USART_STOPBIT_2 = (1 << USBS1)  // 2 stop bits mode
} UsartStopBits_t;

/** \brief Identificadores de tareas usart. */
typedef enum
{
  #if defined(HAL_USE_USART_CHANNEL_0)
    HAL_USART_TASK_USART0_DRE,
    HAL_USART_TASK_USART0_TXC,
    HAL_USART_TASK_USART0_RXC,
    #if defined(_USE_USART_ERROR_EVENT_)
      HAL_USART_TASK_USART0_ERR,
    #endif
  #endif

  #if defined(HAL_USE_USART_CHANNEL_1)
    HAL_USART_TASK_USART1_DRE,
    HAL_USART_TASK_USART1_TXC,
    HAL_USART_TASK_USART1_RXC,
    #if defined(_USE_USART_ERROR_EVENT_)
      HAL_USART_TASK_USART1_ERR,
    #endif
  #endif

  HAL_USART_TASKS_NUMBER
} HalUsartTaskId_t;

// Defines edge of clock to sample data.
/*
------------------------------------------------------------------------------------
|            |  Transmitted Data Changed (Output | Received Data Sampled (Input on |
|            |  of TxDn Pin)                     | RxDn Pin)                       |
|------------|-----------------------------------|----------------------------------
|FALLING_EDGE|  Rising XCKn Edge                 | Falling XCKn Edge               |
|RISING_EDGE |  Falling XCKn Edge                | Rising XCKn Edge                |
------------------------------------------------------------------------------------
*/
/** \brief Modo de transmisión de datos. */
typedef enum
{
  USART_EDGE_MODE_FALLING = 0,
  USART_EDGE_MODE_RISING  = 1
} UsartEdgeMode_t;

// USART synchronization mode.
/** \brief Modo de sincronización usart. */
typedef enum
{
  USART_MODE_ASYNC = ((0 << UMSEL01) | (0 << UMSEL00)),
  USART_MODE_SYNC  = ((0 << UMSEL01) | (1 << UMSEL00))
} UsartMode_t;

// clck is output in master mode else input
/** \brief clk es salida para maestro y entrada para esclavo. */
typedef enum
{
  USART_CLK_MODE_MASTER = 0,
  USART_CLK_MODE_SLAVE  = 1
} UsartClkMode_t;

#if defined(_USE_USART_ERROR_EVENT_)
  // usart receiver error reason
  /** \brief razón del error */
  typedef enum
  {
    FRAME_ERROR,
    DATA_OVERRUN,
    PARITY_ERROR
  } UsartErrorReason_t;
#endif

// usart control
/** \brief Servicio usart. Contiene variables para el uso interno de la usart. */
typedef struct
{
  /** \brief variable de uso interno. */
  volatile uint16_t txPointOfRead;
  /** \brief variable de uso interno. */
  volatile uint16_t txPointOfWrite;
  /** \brief variable de uso interno. */
  volatile uint16_t rxPointOfRead;
  /** \brief variable de uso interno. */
  volatile uint16_t rxPointOfWrite;
  /** \brief variable de uso interno. */
  volatile uint16_t rxBytesInBuffer;
  /** \brief variable de uso interno. */
  uint8_t  usartShiftRegisterEmpty;
#if defined(_USE_USART_ERROR_EVENT_)
/** \brief variable de uso interno. */
  uint8_t  errorReason;
#endif
} HalUsartService_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Guarda el byte recibido al búffer cíclico.
\param[in]	tty		número del canal usart.
\param[in]  data  byte a guardar.
******************************************************************************/
void halUsartRxBufferFiller(UsartChannel_t tty, uint8_t data);

/**************************************************************************//**
\brief Comprueba el número de canal usart.
\param[in]	channel	canal a verificar.
\retval		true	si es un canal válido.
\retval		false	en otro caso.
******************************************************************************/
bool halIsUsartChannelCorrect(UsartChannel_t channel);

#if defined(_USE_USART_ERROR_EVENT_)
/**************************************************************************//**
\brief Guarda el registro de estado para analizar la razón del error.
\param[in]	tty		número del canal usart.
\param[in]	status - contenido del registro de estado de la usart.
******************************************************************************/
void halUsartSaveErrorReason(UsartChannel_t tty, uint8_t status);
#endif

/******************************************************************************
                   Inline static functions section
******************************************************************************/
/**************************************************************************//**
\brief Deshabilita el canal usart.
\param[in]	tty		número del canal usart.
******************************************************************************/
INLINE void halCloseUsart(UsartChannel_t tty)
{
  UCSRnB(tty) = 0x00;
}

/**************************************************************************//**
\brief Habilita la interrupción de registro de datos vacío.
\param[in]	tty		número del canal usart.
******************************************************************************/
INLINE void halEnableUsartDremInterrupt(UsartChannel_t tty)
{
  UCSRnB(tty) |= (1 << UDRIE1);
}

/**************************************************************************//**
\brief Deshabilita la interrupción de registro de datos vacío.
\param[in]	tty		número del canal usart.
******************************************************************************/
INLINE void halDisableUsartDremInterrupt(UsartChannel_t tty)
{
  UCSRnB(tty) &= ~(1 << UDRIE1);
}

/**************************************************************************//**
\brief Habilita la interrupción de envío completado.
\param[in]	tty		número del canal usart.
******************************************************************************/
INLINE void halEnableUsartTxcInterrupt(UsartChannel_t tty)
{
  UCSRnB(tty) |= (1 << TXCIE1);
}

/**************************************************************************//**
\brief Deshabilita la interrupción de envío completado.
\param[in]	tty		número del canal usart.
******************************************************************************/
INLINE void halDisableUsartTxcInterrupt(UsartChannel_t tty)
{
  UCSRnB(tty) &= ~(1 << TXCIE1);
}

/**************************************************************************//**
\brief Habilita la interrupción de recepción completada.
\param[in]	tty		número del canal usart.
******************************************************************************/
INLINE void halEnableUsartRxcInterrupt(UsartChannel_t tty)
{
  UCSRnB(tty) |= (1 << RXCIE0);
}

/**************************************************************************//**
\brief Deshabilita la interrupción de recepción completada.
\param[in]	tty		número del canal usart.
******************************************************************************/
INLINE void halDisableUsartRxcInterrupt(UsartChannel_t tty)
{
  UCSRnB(tty) &= ~(1 << RXCIE0);
}

/**************************************************************************//**
\brief Escribe un byte al registro de datos de la usart (UCSRnA).
\param[in]	tty		número del canal usart.
\param[in]	data	byte a enviar.
******************************************************************************/
INLINE void halSendUsartByte(UsartChannel_t tty, uint8_t data)
{
  UCSRnA(tty) |= (1 << TXC1); // clear transmite complete flag
  UDRn(tty) = data;
}

#endif /* _HAL_USART_H */
//eof halUsart.h
