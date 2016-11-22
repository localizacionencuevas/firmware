/**************************************************************************//**
  \file  spi.c

  \brief Implementación de USART SPI
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/
/******************************************************************************
                   Includes section
******************************************************************************/
#include "spi.h"

/******************************************************************************
                   Define(s) section
******************************************************************************/
/** \brief funcionamiento similar a SPI DORD */
#define UDORD0                  2
/** \brief funcionamiento similar a SPI CPHA */
#define UCPHA0                  1
/** \brief funcionamiento similar a SPI CPOL */
#define UCPOL0                  0
/** \brief número de modos de reloj distintos */
#define SPI_CLOCK_MODE_AMOUNT   4
/** \brief número de formas distintas de ordenar los datos */
#define SPI_DATA_ORDER_AMOUNT   2

/******************************************************************************
                   External global variables section
******************************************************************************/
/** \brief puntero a descriptor USART */
extern HAL_UsartDescriptor_t *halPointDescrip[NUM_USART_CHANNELS];

/******************************************************************************
                   Static functions prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Comprueba si una interfaz está ya cerrada.
\param[in]	pointer puntero a algún descriptor.
\retval		true	si la interfaz está cerrada.
\retval		false	si la interfaz no está cerrada.
******************************************************************************/
static bool isClosedPd(void *pointer);

/**************************************************************************//**
\brief Comprueba si una interfaz está ya abierta.
\param[in]	pointer	puntero a algún descriptor.
\retval		true	si la interfaz está abierta.
\retval		false	si la interfaz está cerrada.
******************************************************************************/
static bool isOpenedPd(void *pointer);

/**************************************************************************//**
\brief Comprueba si un descriptor SPI es correcto.
\param[in]	descriptor	puntero al descriptor SPI.
\param[in]	predicate	función de comprobación.
\retval		-1			la interfaz no está abierta.
\retval		otro		índice de interfaz - la interfaz está abierta.   
******************************************************************************/
static int halCheckUsartDescriptor(HAL_SpiDescriptor_t *descriptor, bool(* predicate)(void *));

/**************************************************************************//*
\brief Configura la USART en modo SPI.
\param[in]	descriptor	puntero al descriptor SPI.
\retval		-1			no hay recursos libres.
\retval		0			el canal SPI está preparado.
******************************************************************************/
//static int halOpenUsartSpi(HAL_SpiDescriptor_t *descriptor);

/**************************************************************************//*
\brief Limpia el canal USART y los pins.
\param[in]	descriptor	puntero al descriptor SPI.
\retval		0			éxito.
\retval		-1			el canal no estaba abierto.
******************************************************************************/
//static int halCloseUsartSpi(HAL_SpiDescriptor_t *descriptor);

/**************************************************************************//**
\brief Rellena la estructura de servicio para la transacción del bus.
\param[in]		descriptor	puntero al descriptor SPI.
\param[in, out]	buffer		puntero al buffer de datos.
\param[in]		length		longitud del buffer de datos.
\param[in]		transac		tipo de transacción.
\retval			-1			la interfaz está ocupada.
\retval			0			éxito.
******************************************************************************/
static int halFillServiceInfo(HAL_SpiDescriptor_t *descriptor, uint8_t *buffer, uint16_t length, uint8_t transac);

/**************************************************************************//*
\brief Escribe una cantidad de bytes en la USART. \n
 La función Callback function se utilizará para notificar cuando se termine 
 la transmisión.
\param[in]	descriptor	puntero al descritor SPI.
\param[in]	buffer		puntero al buffer de datos.
\param[in]	length		número de bytes a transmitir.
\retval		-1			el módulo SPI no estaba abierto, hay datos sin inicializar, el puntero del buffer es 0 o la longitud es 0.
			0			si éxito.
			otro		número de bytes escritos si se utiliza un método síncrono (la función callback es NULL).
******************************************************************************/
//static int halWriteUsartSpi(HAL_SpiDescriptor_t *descriptor, uint8_t *buffer, uint16_t length);

/**************************************************************************//*
\brief Lee un número de bytes de la usart.\n
 La función callback se utiliza para notificar cuando la actividad ha terminado.\n
 Los datos leído se almacenan en el búffer.
\param[in]	descriptor	puntero a la estructura HAL_SpiDescriptor_t
\param[in]	buffer		puntero al búffer de datos de la aplicación
\param[in]	length		número de bytes a transferir
\retval		-1			el módulo SPI no estaba abierto, hay datos sin asignar, el puntero a los datos o la longitud es cero.
\retval		0			éxito o un número.
\retval		otro		número de bytes escritos si se utiliza el modo síncrono (callback es NULL).
******************************************************************************/
//static int halReadUsartSpi(HAL_SpiDescriptor_t *descriptor, uint8_t *buffer, uint16_t length);

/******************************************************************************
                   Implementations section
******************************************************************************/

void halSetUsartSpiConfig(HAL_SpiDescriptor_t *descriptor)
{
  uint8_t clockMode[SPI_CLOCK_MODE_AMOUNT] = {((0 << UCPOL0) | (0 << UCPHA0)),
                                              ((0 << UCPOL0) | (1 << UCPHA0)),
                                              ((1 << UCPOL0) | (0 << UCPHA0)),
                                              ((1 << UCPOL0) | (1 << UCPHA0))};
  uint8_t dataOrder[SPI_DATA_ORDER_AMOUNT] = {(0 << UDORD0),
                                              (1 << UDORD0)};

  // setting of the spi gpio direct
  if (SPI_CHANNEL_0 == descriptor->tty)
    HAL_GPIO_USART0_EXTCLK_out();

  UBRRn(descriptor->tty) = 0;
  // Set MSPI mode
  UCSRnC(descriptor->tty) = (1 << UMSEL01) | (1 << UMSEL00);
  // Set clock mode and data order
  UCSRnC(descriptor->tty) |= (dataOrder[descriptor->dataOrder] | clockMode[descriptor->clockMode]);
  // Enable receiver and transmitter
  UCSRnB(descriptor->tty) = (1 << RXEN0) | (1 << TXEN0);
  // Set baud rate
  UBRRn(descriptor->tty) = descriptor->baudRate;
}

void halClearUsartSpi(SpiChannel_t tty)
{
  if (SPI_CHANNEL_0 == tty)
    HAL_GPIO_USART0_EXTCLK_in();

  UCSRnB(tty) = 0x00; // disable
}



uint16_t halSyncUsartSpiWriteData(SpiChannel_t tty, uint8_t *buffer, uint16_t length)
{
  uint16_t i;
  uint8_t temp;

  for (i = 0; i < length; i++)
  {
    // Wait for empty transmit buffer
    while (!(UCSRnA(tty) & (1 << UDRE0)));
    // Send data
    UDRn(tty) = *(buffer + i);
    // Wait for data to be received
    while (!(UCSRnA(tty) & (1 << RXC0)));
    // receives data to clear received usart buffer
    temp = UDRn(tty);
    (void)temp;
  }
  return i;
}


uint16_t halSyncUsartSpiReadData(SpiChannel_t tty, uint8_t *buffer, uint16_t length)
{
  uint16_t i;
  uint8_t debug;
  for (i = 0; i < length; i++)
  {
    // Wait for empty transmit buffer
    while (!(UCSRnA(tty) & (1 << UDRE0)));
    // Send data
    UDRn(tty) = *(buffer + i);
    // Wait for data to be received
    while (!(UCSRnA(tty) & (1 << RXC0)));
    // Receive data
    debug = UDRn(tty);
    *(buffer + i) = debug;//UDRn(tty);
  }
  return i;
}


static bool isClosedPd(void *pointer)
{
  return pointer ? false : true;
}


static bool isOpenedPd(void *pointer)
{
  return pointer ? true : false;
}

static int halCheckUsartDescriptor(HAL_SpiDescriptor_t *descriptor, bool(* predicate)(void *))
{
  int i;

  if (NULL == descriptor)
    return -1;

  if (false == halIsUsartChannelCorrect(descriptor->tty))
    return -1;

  i = HAL_GET_INDEX_BY_CHANNEL(descriptor->tty);
  if (false == predicate((void *)halPointDescrip[i]))
    return -1;

  return i;
}


// static int halOpenUsartSpi(HAL_SpiDescriptor_t *descriptor)
// {
//   int i; // Descriptor index
// 
//   i = halCheckUsartDescriptor(descriptor, isClosedPd);
//   if (-1 == i)
//     return -1;
// 
//   if (NULL != descriptor->callback)
//   {
//     descriptor->spiDescriptor.txCallback = descriptor->callback;
//     descriptor->spiDescriptor.tty = descriptor->tty;
//   }
// 
//   halPointDescrip[i] = &descriptor->spiDescriptor;
//   halSetUsartSpiConfig(descriptor);
//   return 0;
// }


//done: sustituir esta función por halOpenUsartSpi
int HAL_OpenSpi(HAL_SpiDescriptor_t *descriptor)
{
    //return halOpenUsartSpi(descriptor);
    int i; // Descriptor index

    i = halCheckUsartDescriptor(descriptor, isClosedPd);
    if (-1 == i)
    return -1;

    if (NULL != descriptor->callback)
    {
      descriptor->spiDescriptor.txCallback = descriptor->callback;
      descriptor->spiDescriptor.tty = descriptor->tty;
    }

    halPointDescrip[i] = &descriptor->spiDescriptor;
    halSetUsartSpiConfig(descriptor);
    return 0;
}


// static int halCloseUsartSpi(HAL_SpiDescriptor_t *descriptor)
// {
//   int i;
// 
//   i = halCheckUsartDescriptor(descriptor, isOpenedPd);
//   if (-1 == i)
//     return -1;
// 
//   halPointDescrip[i] = NULL;
//   halClearUsartSpi(descriptor->tty);
//   return 0;
// }


//done: sustituir esta función por halCloseUsartSpi
int HAL_CloseSpi(HAL_SpiDescriptor_t *descriptor)
{
    //return halCloseUsartSpi(descriptor);
    int i;

    i = halCheckUsartDescriptor(descriptor, isOpenedPd);
    if (-1 == i)
    return -1;

    halPointDescrip[i] = NULL;
    halClearUsartSpi(descriptor->tty);
    return 0;
}


static int halFillServiceInfo(HAL_SpiDescriptor_t *descriptor, uint8_t *buffer, uint16_t length, uint8_t transac)
{
  HalUsartService_t *halBufferControl;

  halBufferControl = &descriptor->spiDescriptor.service;
  if (halBufferControl->txPointOfWrite != halBufferControl->txPointOfRead)
    return -1; // there is unsent data

  descriptor->spiDescriptor.txBuffer = buffer;
  descriptor->spiDescriptor.txBufferLength = 0;
  halBufferControl->txPointOfWrite = length;
  halBufferControl->txPointOfRead = 0;
  descriptor->spiDescriptor.rxBuffer = buffer;
  descriptor->spiDescriptor.flowControl = transac;
  return 0;
}


// static int halWriteUsartSpi(HAL_SpiDescriptor_t *descriptor, uint8_t *buffer, uint16_t length)
// {
//   int i;
// 
//   if (!buffer || !length)
//     return -1;
// 
//   i = halCheckUsartDescriptor(descriptor, isOpenedPd);
//   if (-1 == i)
//     return -1;
// 
//   if (&descriptor->spiDescriptor != halPointDescrip[i])
//     return -1; // incorrect descriptor
// 
//   if (NULL != descriptor->callback)
//   {
//     if (-1 == halFillServiceInfo(descriptor, buffer, length, USART_SPI_WRITE_MODE))
//       return -1;
// 
//     halEnableUsartSpiRxcInterrupt(descriptor->tty);
//     halEnableUsartSpiDremInterrupt(descriptor->tty);
//     return 0;
//   }
//   else
//   {
//     return halSyncUsartSpiWriteData(descriptor->tty, buffer, length);
//   }
// }


//done: sustituir esta función por halWriteUsartSpi
int HAL_WriteSpi(HAL_SpiDescriptor_t *descriptor, uint8_t *buffer, uint16_t length)
{
    //return halWriteUsartSpi(descriptor, buffer, length);
    int i;

    if (!buffer || !length)
    return -1;

    i = halCheckUsartDescriptor(descriptor, isOpenedPd);
    if (-1 == i)
    return -1;

    if (&descriptor->spiDescriptor != halPointDescrip[i])
    return -1; // incorrect descriptor

    if (NULL != descriptor->callback)
    {
      if (-1 == halFillServiceInfo(descriptor, buffer, length, USART_SPI_WRITE_MODE))
      return -1;

      halEnableUsartSpiRxcInterrupt(descriptor->tty);
      halEnableUsartSpiDremInterrupt(descriptor->tty);
      return 0;
    }
    else
    {
      return halSyncUsartSpiWriteData(descriptor->tty, buffer, length);
    }
}


// static int halReadUsartSpi(HAL_SpiDescriptor_t *descriptor, uint8_t *buffer, uint16_t length)
// {
//   HAL_UsartDescriptor_t *spiDescrip;
//   int i;
// 
//   if (!buffer || !length)
//     return -1;
// 
//   i = halCheckUsartDescriptor(descriptor, isOpenedPd);
//   if (-1 == i)
//     return -1;
// 
//   spiDescrip = &descriptor->spiDescriptor;
//   if (spiDescrip != halPointDescrip[i])
//     return -1; // incorrect descriptor
// 
//   if (NULL != descriptor->callback)
//   {
//     if (-1 == halFillServiceInfo(descriptor, buffer, length, USART_SPI_READ_MODE))
//       return -1;
// 
//     halEnableUsartSpiRxcInterrupt(descriptor->tty);
//     halEnableUsartSpiDremInterrupt(descriptor->tty);
//     return 0;
//   }
//   else
//   {
//     return halSyncUsartSpiReadData(descriptor->tty, buffer, length);
//   }
// }


//done: sustituir esta función halReadUsartSpi
int HAL_ReadSpi(HAL_SpiDescriptor_t *descriptor, uint8_t *buffer, uint16_t length)
{
    //return halReadUsartSpi(descriptor, buffer, length);
    HAL_UsartDescriptor_t *spiDescrip;
    int i;

    if (!buffer || !length)
    return -1;

    i = halCheckUsartDescriptor(descriptor, isOpenedPd);
    if (-1 == i)
    return -1;

    spiDescrip = &descriptor->spiDescriptor;
    if (spiDescrip != halPointDescrip[i])
    return -1; // incorrect descriptor

    if (NULL != descriptor->callback)
    {
      if (-1 == halFillServiceInfo(descriptor, buffer, length, USART_SPI_READ_MODE))
      return -1;

      halEnableUsartSpiRxcInterrupt(descriptor->tty);
      halEnableUsartSpiDremInterrupt(descriptor->tty);
      return 0;
    }
    else
    {
      return halSyncUsartSpiReadData(descriptor->tty, buffer, length);
    }
}

// eof spi.c

