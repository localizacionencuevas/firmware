/****************************************************************************//**
  \file spi.h

  \brief El archivo header describe el modo de funcionamiento USART para el 
  protocolo SPI para microcontroladores de la familia AVR.
******************************************************************************/

#ifndef _SPI_H
#define _SPI_H
/** \brief número de canales */
#define NUM_USART_CHANNELS 1
/******************************************************************************
                   Includes section
******************************************************************************/
#include "halUsart.h"
#include "usart.h"
#include <halGpio.h>
#include <stddef.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
/** \brief canal USART 0 */
#define SPI_CHANNEL_0 USART_CHANNEL_0  // USART0 AtMega1281/2561 start addresss
/** \brief canal USART 1 */
#define SPI_CHANNEL_1 USART_CHANNEL_1  // USART1 AtMega1281/2561 start addresss

/******************************************************************************
                   Types section
******************************************************************************/
/** \brief canal SPI */
typedef UsartChannel_t SpiChannel_t;

/**************************************************************************//**
\brief Tipos de modo de reloj del SPI.
******************************************************************************/
typedef enum
{
	// leading edge sample RX bit (rising), trailing edge setup TX bit (falling).
	SPI_CLOCK_MODE0,
	// leading edge setup TX bit (rising), trailing edge sample RX bit (falling).
	SPI_CLOCK_MODE1,
	// leading edge sample RX bit (falling), trailing edge setup TX bit (rising).
	SPI_CLOCK_MODE2,
	// leading edge setup TX bit (falling), trailing edge sample RX bit (rising).
	SPI_CLOCK_MODE3
} SpiClockMode_t;

/**************************************************************************//**
\brief Velocidad.
******************************************************************************/
typedef enum
{
	SPI_CLOCK_RATE_62 =  ((F_CPU / (2 * 62500ul)) - 1),
	SPI_CLOCK_RATE_125 =  ((F_CPU / (2 * 125000ul)) - 1),
	SPI_CLOCK_RATE_250 =  ((F_CPU / (2 * 250000ul)) - 1),
	SPI_CLOCK_RATE_500 =  ((F_CPU / (2 * 500000ul)) - 1),
	SPI_CLOCK_RATE_1000 = ((F_CPU / (2 * 1000000ul)) - 1),
	SPI_CLOCK_RATE_2000 = ((F_CPU / (2 * 2000000ul)) - 1)
} SpiBaudRate_t;

/**************************************************************************//**
\brief Orden de los datos
******************************************************************************/
typedef enum
{
	SPI_DATA_MSB_FIRST, // data with MSB first
	SPI_DATA_LSB_FIRST  // data with LSB first
} SpiDataOrder_t;

/** \brief macros para la manipulación de GPIO_USART_DTR */
HAL_GPIO_PIN(USART_DTR, E, 4);
/** \brief macros para la manipulación de GPIO_USART0_TXD */
HAL_GPIO_PIN(USART0_TXD, E, 0);
/** \brief macros para la manipulación de GPIO_USART0_RXD */
HAL_GPIO_PIN(USART0_RXD, E, 1);
/** \brief macros para la manipulación de GPIO_USART0_EXTCLK */
HAL_GPIO_PIN(USART0_EXTCLK, E, 2);
/** \brief macros para la manipulación de GPIO_IRQ_7 */
HAL_GPIO_PIN(IRQ_7, E, 7);
/** \brief macros para la manipulación de GPIO_IRQ_6 */
HAL_GPIO_PIN(IRQ_6, E, 6);


/** \brief descriptor spi */
typedef struct
{
  /** \brief tty - SPI_CHANNEL_n que se va a utilizar. "n" depende de la plataforma. */
  SpiChannel_t tty;
  /** \brief SpiClockMode_t clockMode - modo de reloj (elegido por el usuario). Debe ser elegido entre: \n
    SPI_CLOCK_MODE0 \n
    SPI_CLOCK_MODE1 \n
    SPI_CLOCK_MODE2 \n
    SPI_CLOCK_MODE3 \n */
  SpiClockMode_t clockMode;
  /** \brief SpiDataOrder_t dataOrder - orden de datos (elegido por el usuario). Debe ser elegido entre: \n
    SPI_DATA_MSB_FIRST \n
    SPI_DATA_LSB_FIRST \n */
  SpiDataOrder_t dataOrder;
  /** \brief SpiBaudRate_t  baudRate - velocidad (elegido por el usuario). Debe ser elegido entre: \n
    para atmega:         \n
    SPI_CLOCK_RATE_62  \n
    SPI_CLOCK_RATE_125 \n
    SPI_CLOCK_RATE_250 \n
    SPI_CLOCK_RATE_500 \n
    para xmega (mcu clock)  		          \n
    SPI_CLOCK_RATE_125   (4 MHz)              \n
    SPI_CLOCK_RATE_250   (4, 8 MHz)           \n
    SPI_CLOCK_RATE_500   (4, 8, 16 MHz)       \n
    SPI_CLOCK_RATE_750   (12 MHz )            \n
    SPI_CLOCK_RATE_1000  (4, 8, 16, 32 MHz)   \n
    SPI_CLOCK_RATE_1500  (12 MHz)             \n
    SPI_CLOCK_RATE_2000  (4, 8, 16, 32 MHz)   \n
    SPI_CLOCK_RATE_3000  (12 MHz)             \n
    SPI_CLOCK_RATE_4000  (8, 16, 32 MHz)      \n
    SPI_CLOCK_RATE_6000  (12 MHz)             \n
    SPI_CLOCK_RATE_8000  (16, 32 MHz)         \n
    SPI_CLOCK_RATE_16000 (32 MHz) */
  
  SpiBaudRate_t  baudRate;
  /** \brief HAL_UsartDescriptor_t spiDescriptor - descriptor Usart - contiene variables para el módulo HAL USART.
  */
  HAL_UsartDescriptor_t spiDescriptor;
  union
  {
    /** \brief dirección de la función a la que se debe notificar cuando una transmisión SPI termina. \n
               Cuando el método de funcionamiento es síncrono debe ser NULL. \n */
    void (* callback)(void);
    /** \brief dirección de la función a la que se debe notificar cuando se recibe un byte. \n
               El modo esclavo sólo utiliza el método asíncrono. \n */
    void (* slave_callback)(uint16_t);
  };
} HAL_SpiDescriptor_t;


/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Abre la interfaz SPI.
\param[in]	descriptor	puntero al descriptor SPI.
\retval	-1	no hay recursos libres.
\retval	0	El canal SPI está preparado.
******************************************************************************/
int HAL_OpenSpi(HAL_SpiDescriptor_t *descriptor);

/**************************************************************************//**
\brief Cierra el canal USART y los pins.
\param[in]	descriptor	puntero al descriptor SPI.
\retval	0	éxito
\retval -1	el canal no estaba abierto.
******************************************************************************/
int HAL_CloseSpi(HAL_SpiDescriptor_t *descriptor);

/**************************************************************************//**
\brief Escribe length bytes al spi. \n
 La función callback se utilizará para notificar cuando finalice la transmisión
 (solo para maestro).
\param[in]	descriptor	puntero al descriptor spi.
\param[in]	buffer		puntero al búffer de datos de la aplicación.
\param[in]	length		número de bytes a transferir.
\retval		-1			el módulo SPI no estaba abierto, hay datos sin asignar, el puntero a los datos o la longitud es cero.
\retval		0			éxito o un número.
\retval		num_bytes	número de bytes escritos si se utiliza el modo síncrono (callback es NULL), sólo para maestro.

******************************************************************************/
int HAL_WriteSpi(HAL_SpiDescriptor_t *descriptor, uint8_t *buffer, uint16_t length);

/**************************************************************************//**
\brief Para maestro: escribe un número de bytes al spi. \n
  La función callback se utilizará para notificar cuando la actividad ha terminado. \n
  Los datos leídos se almacenan en el  búffer. \n
  Para esclavo: lee un número de bytes del búffer interno del spi y los escribe \n
  en el búffer de la aplicación.
\param[in]		descriptor	puntero a la estructura HAL_SpiDescriptor_t.
\param[in, out]	buffer		puntero al búffer de datos de la aplicación.
\param[in]		length		número de bytes a transferir.
\retval			-1			el módulo spi no estaba abierto, hay datos sin asignar, el puntero a los datos o la longitud es NULL.
\retval			0			éxito para el maestro
\retval			num_bytes	número de bytes escritos si se utiliza el modo síncrono (callback es NULL) para
   el maestro \n
   o el número de datos leídos del búffer interno al de aplicación para el esclavo.
******************************************************************************/
int HAL_ReadSpi(HAL_SpiDescriptor_t *descriptor, uint8_t *buffer, uint16_t length);

/**************************************************************************//**
\brief Deshabilita el canal USART.
\param[in]	tty	canal spi.
******************************************************************************/
void halClearUsartSpi(SpiChannel_t tty);

/**************************************************************************//**
\brief Escribe length bytes en el spi.
\param[in]	tty		canal spi.
\param[in]	buffer	puntero al buffer con los datos a escribir.
\param[in]	length	cantidad de datos a escribir.
\return		número de bytes escritos.
******************************************************************************/
uint16_t halSyncUsartSpiWriteData(SpiChannel_t tty, uint8_t *buffer, uint16_t length);


/**************************************************************************//**
\brief Escribe y lee length bytes en y del spi.
\param[in]		tty		canal spi
\param[in, out]	buffer	puntero al buffer de datos
\param[in]		length	cantidad de datos a transferir
\return			número de bytes escritos y leídos
******************************************************************************/
uint16_t halSyncUsartSpiReadData(SpiChannel_t tty, uint8_t *buffer, uint16_t length);

/**************************************************************************//**
\brief Configura el USART para trabajar en modo SPI.
\param[in]	descriptor	puntero al descriptor spi.
******************************************************************************/
void halSetUsartSpiConfig(HAL_SpiDescriptor_t *descriptor);

/**************************************************************************//**
\brief Configura el USART para trabajar en modo SPI esclavo.
\param[in]	descriptor	puntero al descriptor spi.
\todo	no implementado
******************************************************************************/
void halSetSlaveSpiConfig(HAL_SpiDescriptor_t *descriptor);

/******************************************************************************
                   Inline static functions section
******************************************************************************/
/**************************************************************************//**
\brief Activa la interrupción de registro de datos vacío (data register empty).
\param[in]	tty	canal spi.
******************************************************************************/

INLINE void halEnableUsartSpiDremInterrupt(SpiChannel_t tty)
{
  UCSRnB(tty) |= (1 << UDRIE0);
}

/**************************************************************************//**
\brief Desactiva la interrupción de registro de datos vacío (data register empty).
\param[in]	tty	canal spi.
******************************************************************************/
INLINE void halDisableUsartSpiDremInterrupt(SpiChannel_t tty)
{
  UCSRnB(tty) &= ~(1 << UDRIE0);
}

/**************************************************************************//**
\brief Activa la interrupción de transmisión completada.
\param[in]	tty	canal spi.
******************************************************************************/
INLINE void halEnableUsartSpiTxcInterrupt(SpiChannel_t tty)
{
  UCSRnB(tty) |=  (1 << TXCIE0);
}

/**************************************************************************//**
\brief Desactiva la interrupción de transmisión completada.
\param[in]	tty	canal spi.
******************************************************************************/
INLINE void halDisableUsartSpiTxcInterrupt(SpiChannel_t tty)
{
  UCSRnB(tty) &=  ~(1 << TXCIE0);
}

/**************************************************************************//**
\brief Activa la interrupción de recepción completada.
\param[in]	tty	canal spi.
******************************************************************************/
INLINE void halEnableUsartSpiRxcInterrupt(SpiChannel_t tty)
{
  UCSRnB(tty) |= (1 << RXCIE0);
}

/**************************************************************************//**
\brief Desactiva la interrupción de recepción completada.
\param[in]	tty	canal spi.
******************************************************************************/
INLINE void halDisableUsartSpiRxcInterrupt(SpiChannel_t tty)
{
  UCSRnB(tty) &= ~(1 << RXCIE0);
}

#endif /* _SPI_H */

