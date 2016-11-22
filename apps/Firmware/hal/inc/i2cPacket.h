/**************************************************************************//**
\file  i2cPacket.h

\brief Implementación de la interfaz i2c.
******************************************************************************/

#ifndef _I2CPACKET_H
#define _I2CPACKET_H

/******************************************************************************
                   Includes section
******************************************************************************/

#include "bcTypes.h"
#include <sysTypes.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
/** \brief Canal i2c. */
#define TWI_CHANNEL_0  0xBD

/** \brief Prescaler del bus i2c. */
#define HAL_I2C_PRESCALER 0ul

/** \brief Códigos de estado del i2c. */
enum
{
  TWS_BUSERROR      = 0x00,
  TWS_START         = 0x08,
  TWS_RSTART        = 0x10,
  TWS_MT_SLA_ACK    = 0x18,
  TWS_MT_SLA_NACK   = 0x20,
  TWS_MT_DATA_ACK   = 0x28,
  TWS_MT_DATA_NACK  = 0x30,
  TWS_M_ARB_LOST    = 0x38,
  TWS_MR_SLA_ACK    = 0x40,
  TWS_MR_SLA_NACK   = 0x48,
  TWS_MR_DATA_ACK   = 0x50,
  TWS_MR_DATA_NACK  = 0x58
};

/******************************************************************************
                   Types section
******************************************************************************/
/** \brief Canal i2c. */
typedef uint8_t I2cChannel_t;

/** \brief Velocidad (baud rate) i2c. */
typedef enum
{
  I2C_CLOCK_RATE_250 = ((F_CPU/250000ul) - 16ul)/(2ul * (1ul << HAL_I2C_PRESCALER) * (1ul << HAL_I2C_PRESCALER)), // 250 Kb/s clock rate  
  I2C_CLOCK_RATE_125 = ((F_CPU/125000ul) - 16ul)/(2ul * (1ul << HAL_I2C_PRESCALER) * (1ul << HAL_I2C_PRESCALER)), // 125 Kb/s clock rate
  I2C_CLOCK_RATE_100  = ((F_CPU/100000ul) - 16ul)/(2ul * (1ul << HAL_I2C_PRESCALER) * (1ul << HAL_I2C_PRESCALER)),// 100 Kb/s clock rate  
  I2C_CLOCK_RATE_62  = ((F_CPU/62500ul) - 16ul)/(2ul * (1ul << HAL_I2C_PRESCALER) * (1ul << HAL_I2C_PRESCALER))   // 62.5 Kb/s clock rate
} I2cClockRate_t;

/** \brief Tamaño interno de la dirección. */
typedef enum
{
  HAL_NO_INTERNAL_ADDRESS,
  HAL_ONE_BYTE_SIZE,
  HAL_TWO_BYTE_SIZE,
  HAL_THREE_BYTE_SIZE
} I2cInternalAddrSize_t;

/** \brief Contiene variables sobre el estado del módulo i2c. */
typedef struct
{
  /** \brief Índice. */	
  volatile uint16_t index;             // current index of read/write byte
  /** \brief Estado. */	
  volatile uint8_t state;
} HalI2cPacketService_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Inicializa el módulo i2c. Configura la velocidad del i2c.

\param[in]	rate	velocidad del i2c.
******************************************************************************/
void halInitI2c(I2cClockRate_t rate);

/**************************************************************************//**
\brief Notificación de comienzo enviada.
******************************************************************************/
void halSendStartDoneI2c(void);

/**************************************************************************//**
\brief El byte se ha escrito al i2c.
******************************************************************************/
void halWriteDoneI2c(void);

/**************************************************************************//**
\brief El byte ha sido leído del i2c.
\param[in]	data	contiene el byte que se ha leído.
******************************************************************************/
void halReadDoneI2c(uint8_t data);

/**************************************************************************//**
\brief El último byte se ha leído del i2c. Necesita enviar la condición de 
parada del bus.
\param[in]	data	contiene el byte que se ha leído.
******************************************************************************/
void halReadLastByteDoneI2c(uint8_t data);

/**************************************************************************//**
\brief El byte de dirección ha sido escrito al i2c y se ha leído el i2c. Comienza 
leyendo datos.
******************************************************************************/
void halMasterReadWriteAddressAckI2c(void);

/**************************************************************************//**
\brief Reinicia el bus i2c.
******************************************************************************/
void halI2cBusReset(void);

/******************************************************************************
                   Inline static functions section
******************************************************************************/
/**************************************************************************//**
\brief Espera a la condición de fin y parada del bus.
******************************************************************************/
INLINE void halWaitEndOfStopStation(void)
{
  loop_until_bit_is_clear(TWCR, TWSTO);
}

/**************************************************************************//**
\brief Habilita la interrupción i2c.
******************************************************************************/
INLINE void halInterruptEnableI2c(void)
{
  TWCR |= (1 << TWIE);
}

/**************************************************************************//**
\brief Deshabilita la interrupción i2c.
******************************************************************************/
INLINE void halInterruptDisableI2c(void)
{
  TWCR &= (~(1 << TWIE));
}

/*************************************************************************//**
\brief Devuelve el byte que ha sido leído del i2c.
\return El byte leído del i2c.
******************************************************************************/
INLINE uint8_t halReadByteI2c(void)
{
  return TWDR;
}

/*************************************************************************//**
\brief Reinicia el i2c.
******************************************************************************/
INLINE void halResetI2c(void)
{
  TWCR = ((1 << TWSTO) | (1 << TWINT));  // Reset TWI
}

/**************************************************************************//**
\brief Comienza a escribir un byte en el i2c.
\param[in]	data	el byte a escribir.
******************************************************************************/
INLINE void halWriteI2c(uint8_t data)
{
  TWDR =  data;
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
}

/**************************************************************************//**
\brief Comienza a leer un byte del i2c.
\param[in] ack	define la necesidad de enviar un ACK después de recibir el byte.
******************************************************************************/
INLINE void halReadI2c(bool ack)
{
  if (ack)
    TWCR |= (1 << TWEA);
  else
    TWCR &= ~(1 << TWEA);

  TWCR |= ((1 << TWINT) | (1 << TWIE) | (1 << TWEN));  // Trigger the TWI
}

/**************************************************************************//**
\brief Envía la condición de parada al i2c.
******************************************************************************/
INLINE void halSendStopI2c(void)
{
  TWCR = ((1 << TWSTO) | (1 << TWINT) | (1 << TWEN));
}

/**************************************************************************//**
\brief Envía la condición de comienzo al i2c.
******************************************************************************/
INLINE void halSendStartI2c(void)
{
  TWCR = ((1 << TWSTA) | (1 <<TWINT) | (1 << TWEN) | (1 << TWIE));
}

/******************************************************************************
                   Types section
******************************************************************************/

/**************************************************************************//**
\brief Estructura para el control y el acceso al i2c.
******************************************************************************/
typedef struct
{
  /** \brief campo del servicio i2c - contiene variables para el funcionamiento del módulo i2c. */
  HalI2cPacketService_t service;
  /** \brief tty - Canal i2c a utilizar. */
  I2cChannel_t tty;
  /** \brief Velocidad. */
  I2cClockRate_t clockRate;
  /** \brief Dirección del esclavo. */
  uint8_t id;
  /** \brief El número de bytes a ser escritos en el bus. */
  uint16_t length;
  /** \brief Puntero a los datos. */
  uint8_t *data;
  /** \brief Especifica el tamaño de la dirección contenida en internalAddr:
              HAL_NO_INTERNAL_ADDRESS \n
              HAL_ONE_BYTE_SIZE       \n
              HAL_TWO_BYTE_SIZE       \n
              HAL_THREE_BYTE_SIZE */
  I2cInternalAddrSize_t lengthAddr;
  /** \brief Contiene la dirección del espacio de direcciones interno del dispositivo. */
  uint32_t internalAddr;
  /** \brief Función callback. */
  void (* f)(bool result);
} HAL_I2cDescriptor_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Abre el recurso i2c. La velocidad y el canal deben ser configurados.

\param[in]	descriptor	puntero a una instancia del tipo HAL_I2cDescriptor_t.

\retval	-1	el recurso ya estaba abierto o el puntero es NULL.
\retval	0	éxito.
******************************************************************************/
int HAL_OpenI2cPacket(HAL_I2cDescriptor_t *descriptor);

/**************************************************************************//**
\brief Cierra el recurso i2c.

\param[in]	descriptor	puntero a una instancia del tipo HAL_I2cDescriptor_t.

\retval	-1	el recurso no estaba abierto.
\retval	0	éxito.
******************************************************************************/
int HAL_CloseI2cPacket(HAL_I2cDescriptor_t *descriptor);

/**************************************************************************//**
\brief Escribe una serie de bytes al bus i2c. El resultado de la operación se 
envía a la función de callback especificada en la instancia de HAL_I2cDescriptor_t.

\param[in]	descriptor	puntero a una instancia del tipo HAL_I2cDescriptor_t.

\retval	0	la petición ha sido aceptada y el bus está libre.
\retval	-1	en otro caso.
******************************************************************************/
int HAL_WriteI2cPacket(HAL_I2cDescriptor_t *descriptor);

/**************************************************************************//**
\brief Lee una serie de bytes del bus i2c. El resultado de la operación se 
envía a la función de callback especificada en la instancia de HAL_I2cDescriptor_t.

\param[in]	descriptor	puntero a una instacia del tipo HAL_I2cDescriptor_t.

\retval	0	la petición ha sido aceptada y el bus está libre.
\retval	-1	en otro caso.
******************************************************************************/
int HAL_ReadI2cPacket(HAL_I2cDescriptor_t *descriptor);

/**************************************************************************//**
\brief Espera a que termine de enviar y llama al callback.
******************************************************************************/
void halSig2WireSerialHandler(void);

#endif /* _I2CPACKET_H */

