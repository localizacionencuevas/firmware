/**************************************************************************//**
\file  usart.c

\brief Implementación de usart. Modo asíncrono.
*******************************************************************************/
/******************************************************************************
                   Includes section
******************************************************************************/
#include "bcTypes.h"
#include "usart.h"
#include "spi.h"
#include <avr/io.h>
/******************************************************************************
                   Define(s) section
******************************************************************************/

/** \brief Canal usart.  */
#define HAL_USE_USART_CHANNEL_0 

/** \brief Devuelve una función controlador pasado un índice. */
#define HANDLERS_GET(A, I) memcpy_P(A, &halUsartHandlers[I], sizeof(HalUsartTask_t))
/** \brief Cantidad de bytes reservados en buffer de recepción. Algunos clientes 
  (Windows, por ejemplo), mandan más bytes después de CTS, así que hay que reservar 
  más espacio para ellos.*/
#define BUFFER_RESERV    1
/* \brief Timer
	\todo no usado, borrar */
//#define USART_HW_CONTROLLER_TIMER_PERIOD 10
#if defined(_USE_USART_ERROR_EVENT_)
  #define HAL_BM_FRAME_ERROR     (1 << 4)
  #define HAL_BM_DATA_OVERRUN    (1 << 3)
  #define HAL_BM_PARITY_ERROR    (1 << 2)
#endif

/******************************************************************************
                   Types definition section
******************************************************************************/
/**************************************************************************//**
  \brief Máscara de tareas HAL USART.
******************************************************************************/
typedef volatile uint8_t HalUsartTaskBitMask_t;

/**************************************************************************//**
  \brief Declaración de tareas HAL USART.
******************************************************************************/
typedef void (* HalUsartTask_t)(void);

/******************************************************************************
                   Global functions prototypes section
******************************************************************************/
/**************************************************************************//**
\brief  Controlador de tareas HAL USART. La acción exacta depende de la tarea USART interna.
******************************************************************************/
void halSigUsartHandler(void);

/**************************************************************************//**
  \brief Configura los parámetros del módulo USART.
  \param[in] usartmode	puntero a to HAL_UsartDescriptor_t.
******************************************************************************/
void halSetUsartConfig(HAL_UsartDescriptor_t *usartmode);

/**************************************************************************//**
\brief Publica una tarea específica.

\param[in]	taskId	identificador de la tarea.
******************************************************************************/
void halPostUsartTask(HalUsartTaskId_t taskId);


#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
/**************************************************************************//**
\brief Callback para controlar los pines. No usado.
******************************************************************************/
  void hwControlPinsPollCallback(void);
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

/******************************************************************************
                   Static function prototypes section
******************************************************************************/
#if defined(HAL_USE_USART_CHANNEL_0)
/**************************************************************************//**
\brief Wrapper para controlador de registro de datos vacío (data empty) del canal 0 usart.
******************************************************************************/
  static void halUsartTaskUsart0Dre(void);
/**************************************************************************//**
\brief Wrapper para controlador de envío de datos completado del canal 0 usart.
******************************************************************************/
  static void halUsartTaskUsart0Txc(void);
/**************************************************************************//**
\brief Wrapper para controlador de recepción de datos completado del canal 0 usart.
******************************************************************************/
  static void halUsartTaskUsart0Rxc(void);
  #if defined(_USE_USART_ERROR_EVENT_)
    static void halUsartTaskUsart0Err(void);
  #endif
#endif

#if defined(HAL_USE_USART_CHANNEL_1)
/**************************************************************************//**
\brief Wrapper para controlador de datos vacío (data empty) del canal 1 usart.
******************************************************************************/
  static void halUsartTaskUsart1Dre(void);
/**************************************************************************//**
\brief Wrapper para controlador envío de datos completado del canal 1 usart.
******************************************************************************/
  static void halUsartTaskUsart1Txc(void);
/**************************************************************************//**
\brief Wrapper para controlador recepción de datos completado del canal 1 usart.
******************************************************************************/
  static void halUsartTaskUsart1Rxc(void);
  #if defined(_USE_USART_ERROR_EVENT_)
    static void halUsartTaskUsart1Err(void);
  #endif
#endif

/**************************************************************************//**
\brief Controla los pines RTS y DTR, toma decisiones si la usart puede transmitir.

\param[in]  tty identificador de número de canal.
******************************************************************************/
static void halUsartHwController(UsartChannel_t tty);

/**************************************************************************//**
\brief Controlador de recepción completada.

\param[in]  tty identificador de número de canal.
******************************************************************************/
static void halSigUsartReceptionComplete(UsartChannel_t tty);

/**************************************************************************//**
\brief Pone la dirección del pin del reloj para el modo síncrono.

\param[in]  descriptor  puntero al descriptor del canal usart.
******************************************************************************/
static void halSetUsartClockPinDirection(HAL_UsartDescriptor_t *descriptor);

/******************************************************************************
                   Static variables section
******************************************************************************/
#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
  static HAL_AppTimer_t halUsartAppTimer;
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT
/** \brief Puntero a descriptor usart. */
HAL_UsartDescriptor_t *halPointDescrip[NUM_USART_CHANNELS] =
{
  #if defined(HAL_USE_USART_CHANNEL_0)
    NULL,
  #endif
  #if defined(HAL_USE_USART_CHANNEL_1)
    NULL
  #endif
};
/** \brief Máscara de tareas*/
static volatile HalUsartTaskBitMask_t halUsartTaskBitMask = 0; // HAL USART tasks' bit mask.
/** \brief Contiene punteros a funciones (controladores) para uso interno.*/
static const HalUsartTask_t PROGMEM_DECLARE(halUsartHandlers[HAL_USART_TASKS_NUMBER]) =
{
  #if defined(HAL_USE_USART_CHANNEL_0)
  /** \brief puntero a función para uso interno */
    halUsartTaskUsart0Dre,
  /** \brief puntero a función para uso interno */
    halUsartTaskUsart0Txc,
  /** \brief puntero a función para uso interno */
    halUsartTaskUsart0Rxc,
    #if defined(_USE_USART_ERROR_EVENT_)
      halUsartTaskUsart0Err,
    #endif
  #endif

  #if defined(HAL_USE_USART_CHANNEL_1)
    halUsartTaskUsart1Dre,
    halUsartTaskUsart1Txc,
    halUsartTaskUsart1Rxc,
    #if defined(_USE_USART_ERROR_EVENT_)
      halUsartTaskUsart1Err,
    #endif
  #endif
}; // List Of possible HAL USART tasks.

/******************************************************************************
  DTR service
******************************************************************************/
/** \brief DTR wake up habilitado. */
volatile bool halEnableDtrWakeUp = false;

/** \brief Función callback para DTR wake up. */
void (* dtrWakeUpCallback)(void) = NULL;

/******************************************************************************
                   Implementations section
******************************************************************************/
void halSigUsartHandler(void)
{
  HalUsartTask_t         handler;
  HalUsartTaskBitMask_t  mask = 1;
  HalUsartTaskId_t       index = 0;

  for ( ; index < HAL_USART_TASKS_NUMBER; index++, mask <<= 1)
  {
    if (halUsartTaskBitMask & mask)
    {
      ATOMIC_SECTION_ENTER
      halUsartTaskBitMask ^= mask;
      ATOMIC_SECTION_LEAVE
      HANDLERS_GET(&handler, index);
      handler();
    }
  }
}


void halPostUsartTask(HalUsartTaskId_t taskId)
{
  halUsartTaskBitMask |= (HalUsartTaskBitMask_t)1 << taskId;
}


void halUsartRxBufferFiller(UsartChannel_t tty, uint8_t data)
{
  uint16_t           old;
  uint8_t            i;
  HalUsartService_t *halUsartControl;

  i = HAL_GET_INDEX_BY_CHANNEL(tty);
  if (NULL == halPointDescrip[i])
  {// abnormal
    halDisableUsartRxcInterrupt(tty); // disable usart
    return;
  }

  if (halPointDescrip[i]->flowControl & USART_SPI_WRITE_MODE)
    return;

  if (halPointDescrip[i]->flowControl & USART_SPI_READ_MODE)
  { // For spi mode.
    *(uint8_t*)(halPointDescrip[i]->rxBuffer) = data;
    halPointDescrip[i]->rxBuffer++;
    return;
  } // For spi mode.

  halUsartControl = &halPointDescrip[i]->service;
  if (NULL != halPointDescrip[i]->rxBuffer)
  {
    old = halUsartControl->rxPointOfWrite;

    if (++halUsartControl->rxPointOfWrite == halPointDescrip[i]->rxBufferLength)
      halUsartControl->rxPointOfWrite = 0;

    if (halUsartControl->rxPointOfWrite == halUsartControl->rxPointOfRead)
    { // Buffer full.
      halUsartControl->rxPointOfWrite = old;
      return;
    } // Buffer full.

    halPointDescrip[i]->rxBuffer[old] = data;
    halUsartControl->rxBytesInBuffer++;

  }
}

#if defined(_USE_USART_ERROR_EVENT_)
void halUsartSaveErrorReason(UsartChannel_t tty, uint8_t status)
{
  HalUsartService_t *halUsartControl;
  uint8_t            i;

  i = HAL_GET_INDEX_BY_CHANNEL(tty);
  if (NULL == halPointDescrip[i])
  {// abnormal
    halDisableUsartRxcInterrupt(tty); // disable usart
    return;
  }

  halUsartControl = &halPointDescrip[i]->service;
  halUsartControl->errorReason = status;
}
#endif

int HAL_OpenUsart(HAL_UsartDescriptor_t *descriptor)
{
  uint8_t i; // Descriptor index

  if (NULL == descriptor)
    return -1;
  if (false == halIsUsartChannelCorrect(descriptor->tty))
    return -1;

#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
  if ((descriptor->flowControl & USART_FLOW_CONTROL_HARDWARE) &&
      (HW_CONTROL_PINS_PORT_ASSIGNMENT != descriptor->tty))
    return -1; // Hardware control cannot be used for this channel.
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

  i = HAL_GET_INDEX_BY_CHANNEL(descriptor->tty);
  if (NULL != halPointDescrip[i])
    return -1; // Channel is already opened.

  halPointDescrip[i] = descriptor;
#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
  if (HW_CONTROL_PINS_PORT_ASSIGNMENT == descriptor->tty)
  {
    if (descriptor->flowControl & USART_DTR_CONTROL)
      GPIO_USART_DTR_make_in();
    if (descriptor->flowControl & USART_FLOW_CONTROL_HARDWARE)
    {
      GPIO_USART_CTS_make_out();
      GPIO_USART_RTS_make_in();
      if (NULL == descriptor->rxBuffer)
        GPIO_USART_CTS_set(); // CTS_ON
      else
        GPIO_USART_CTS_clr(); // CTS_OFF
    }
  }
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

  if (USART_MODE_SYNC == descriptor->mode)
    halSetUsartClockPinDirection(descriptor);

  descriptor->service.txPointOfRead = 0;
  descriptor->service.txPointOfWrite = 0;
  if (NULL == descriptor->rxBuffer)
    descriptor->rxBufferLength = 0;
  if (NULL == descriptor->txBuffer)
    descriptor->txBufferLength = 0;
  descriptor->service.rxPointOfRead = 0;
  descriptor->service.rxPointOfWrite = 0;
  descriptor->service.usartShiftRegisterEmpty = 1;

  halSetUsartConfig(descriptor);

  return descriptor->tty;
}

int HAL_CloseUsart(HAL_UsartDescriptor_t *descriptor)
{
  uint8_t i;

  if (NULL == descriptor)
    return -1;
  if (false == halIsUsartChannelCorrect(descriptor->tty))
    return -1;
  i = HAL_GET_INDEX_BY_CHANNEL(descriptor->tty);
  if (NULL == halPointDescrip[i])
    return -1; // Channel is already closed.

  halCloseUsart(halPointDescrip[i]->tty);
#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
  if (halPointDescrip[i]->flowControl & USART_FLOW_CONTROL_HARDWARE)
    GPIO_USART_CTS_make_in();
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

  if (USART_MODE_SYNC == halPointDescrip[i]->mode)
  {
    halPointDescrip[i]->syncMode = USART_CLK_MODE_SLAVE;
    halSetUsartClockPinDirection(halPointDescrip[i]);
  }
  halPointDescrip[i] = NULL;

  return 0;
}


static void halUsartHwController(UsartChannel_t tty)
{
  uint8_t            i;
  HalUsartService_t *halUsartControl;

  i = HAL_GET_INDEX_BY_CHANNEL(tty);
  if (NULL == halPointDescrip[i])
    return; // Port closed.

  halUsartControl = &halPointDescrip[i]->service;
#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
  if (HW_CONTROL_PINS_PORT_ASSIGNMENT == tty)
  {
    uint8_t hw1 = 0;
    uint8_t hw2 = 0;

    if (halPointDescrip[i]->flowControl & USART_DTR_CONTROL)
      hw1 = GPIO_USART_DTR_read();

    if (halPointDescrip[i]->flowControl & USART_FLOW_CONTROL_HARDWARE)
      hw2 = GPIO_USART_RTS_read();

    if (hw1 || hw2)
    {
      halUsartAppTimer.interval = USART_HW_CONTROLLER_TIMER_PERIOD;
      halUsartAppTimer.mode = TIMER_ONE_SHOT_MODE;
      halUsartAppTimer.callback = hwControlPinsPollCallback;
      HAL_StartAppTimer(&halUsartAppTimer);
      return;
    }
  }
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

  uint16_t poW;
  uint16_t poR;

   ATOMIC_SECTION_ENTER
    poW = halUsartControl->txPointOfWrite;
    poR = halUsartControl->txPointOfRead;
   ATOMIC_SECTION_LEAVE

  if (poW != poR)
  {
    halSendUsartByte(tty, halPointDescrip[i]->txBuffer[poR++]);
    if (poR == halPointDescrip[i]->txBufferLength)
      poR = 0;
    halEnableUsartDremInterrupt(tty);

     ATOMIC_SECTION_ENTER
      halUsartControl->txPointOfRead = poR;
     ATOMIC_SECTION_LEAVE

  }
  else
  {
    // data register empty interrupt was disabled
    halEnableUsartTxcInterrupt(tty);// TX Complete interrupt enable
  }
}

int HAL_WriteUsart(HAL_UsartDescriptor_t *descriptor, uint8_t *buffer, uint16_t length)
{
  uint8_t            i;
  uint16_t           poW;
  uint16_t           poR;
  uint16_t           old;
  uint16_t           wasWrote = 0;
  bool               needStartTrmt = false;
  HalUsartService_t *halUsartControl;

  if (NULL == descriptor)
    return -1;
  if (false == halIsUsartChannelCorrect(descriptor->tty))
    return -1;
  if (!buffer || !length)
    return -1;
  i = HAL_GET_INDEX_BY_CHANNEL(descriptor->tty);
  if (descriptor != halPointDescrip[i])
    return -1; // Channel is not opened.

  halUsartControl = &descriptor->service;
  if (0 == descriptor->txBufferLength)
  { // Callback mode
    if (halUsartControl->txPointOfWrite != halUsartControl->txPointOfRead)
      return -1; // there is unsent data
    descriptor->txBuffer = buffer;
    halUsartControl->txPointOfWrite = length;
    halUsartControl->txPointOfRead = 0;
    needStartTrmt = true;
    wasWrote = length;
  } // Callback mode.
  else
  { // Polling mode.
     ATOMIC_SECTION_ENTER
      poW = halUsartControl->txPointOfWrite;
      poR = halUsartControl->txPointOfRead;
     ATOMIC_SECTION_LEAVE

    if (poW == poR)
      needStartTrmt = true; // Buffer empty.

    while (wasWrote < length)
    {
      old = poW;

      if (++poW == descriptor->txBufferLength)
        poW = 0;

      if (poW == poR)
      { // Buffer full.
        poW = old;
        break;
      } // Buffer full.

      descriptor->txBuffer[old] = buffer[wasWrote++];
    }

     ATOMIC_SECTION_ENTER
      halUsartControl->txPointOfWrite = poW;
     ATOMIC_SECTION_LEAVE
  } // Polling mode

  if (needStartTrmt)
  {
    halUsartControl->usartShiftRegisterEmpty = 0; // Buffer and shift register is full
    // Enable interrupt. Transaction will be launched in the callback.
    halEnableUsartDremInterrupt(descriptor->tty);
  }

  return wasWrote;
}

int HAL_ReadUsart(HAL_UsartDescriptor_t *descriptor, uint8_t *buffer, uint16_t length)
{
  uint8_t            i = 0;
  uint16_t           wasRead = 0;
  uint16_t           poW;
  uint16_t           poR;
  HalUsartService_t *halUsartControl;
#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
  uint16_t           number;
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

  if (NULL == descriptor)
    return -1;
  if (false == halIsUsartChannelCorrect(descriptor->tty))
    return -1;
  if (!buffer || !length)
    return -1;
  i = HAL_GET_INDEX_BY_CHANNEL(descriptor->tty);
  if (descriptor != halPointDescrip[i])
    return -1; // Channel is not opened.

  halUsartControl = &halPointDescrip[i]->service;
   ATOMIC_SECTION_ENTER
    poW = halUsartControl->rxPointOfWrite;
    poR = halUsartControl->rxPointOfRead;
   ATOMIC_SECTION_LEAVE

  while ((poR != poW) && (wasRead < length))
  {
    buffer[wasRead] = descriptor->rxBuffer[poR];
    if (++poR == descriptor->rxBufferLength)
      poR = 0;
    wasRead++;
  }

   ATOMIC_SECTION_ENTER
    halUsartControl->rxPointOfRead = poR;
    halUsartControl->rxBytesInBuffer -= wasRead;
#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
    number = halUsartControl->rxBytesInBuffer;
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT
   ATOMIC_SECTION_LEAVE

#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
  if ((HW_CONTROL_PINS_PORT_ASSIGNMENT == descriptor->tty) && (descriptor->flowControl & USART_FLOW_CONTROL_HARDWARE))
    if (number <= (descriptor->rxBufferLength >> BUFFER_RESERV))
      GPIO_USART_CTS_clr();
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

  return wasRead;
}

#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
int HAL_OnUsartCts(HAL_UsartDescriptor_t *descriptor)
{
  uint8_t i;

  if (NULL == descriptor)
    return -1;

  if (false == halIsUsartChannelCorrect(descriptor->tty))
    return -1;

  i = HAL_GET_INDEX_BY_CHANNEL(descriptor->tty);
  if (descriptor != halPointDescrip[i])
    return -1; // Channel is not opened.

  if (HW_CONTROL_PINS_PORT_ASSIGNMENT != descriptor->tty)
    return -1;

  GPIO_USART_CTS_set();// CTS_ON

  return 0;
}
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
int HAL_OffUsartCts(HAL_UsartDescriptor_t *descriptor)
{
  uint8_t i;

  if (NULL == descriptor)
    return -1;

  if (false == halIsUsartChannelCorrect(descriptor->tty))
    return -1;

  i = HAL_GET_INDEX_BY_CHANNEL(descriptor->tty);
  if (descriptor != halPointDescrip[i])
    return -1; // Channel is not opened.

  if (HW_CONTROL_PINS_PORT_ASSIGNMENT != descriptor->tty)
    return -1;

  GPIO_USART_CTS_clr(); // CTS_OFF

  return 0;
}
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
int HAL_ReadUsartRts(HAL_UsartDescriptor_t *descriptor)
{
  uint8_t i;

  if (NULL == descriptor)
    return -1;

  if (false == halIsUsartChannelCorrect(descriptor->tty))
    return -1;

  i = HAL_GET_INDEX_BY_CHANNEL(descriptor->tty);
  if (descriptor != halPointDescrip[i])
    return -1; // Channel is not opened.

  if (HW_CONTROL_PINS_PORT_ASSIGNMENT != descriptor->tty)
    return -1;

  return GPIO_USART_RTS_read();
}
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
int HAL_ReadUsartDtr(HAL_UsartDescriptor_t *descriptor)
{
  uint8_t i;

  if (NULL == descriptor)
    return -1;

  if (false == halIsUsartChannelCorrect(descriptor->tty))
    return -1;

  i = HAL_GET_INDEX_BY_CHANNEL(descriptor->tty);
  if (descriptor != halPointDescrip[i])
    return -1; // Channel is not opened.

  if (HW_CONTROL_PINS_PORT_ASSIGNMENT != descriptor->tty)
    return -1;

  return GPIO_USART_DTR_read();
}
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

// Interrupt handlers
#ifdef HW_CONTROL_PINS_PORT_ASSIGNMENT
void hwControlPinsPollCallback(void)
{
  halUsartHwController(HW_CONTROL_PINS_PORT_ASSIGNMENT);
}
#endif // HW_CONTROL_PINS_PORT_ASSIGNMENT

/**************************************************************************//**
\brief Controlador de envío completado.

\param[in]  tty identificador del canal usart.
******************************************************************************/
void halSigUsartTransmissionComplete(UsartChannel_t tty)
{
  uint8_t            i;
  HalUsartService_t *halUsartControl;
  uint16_t           poW;
  uint16_t           poR;

  i = HAL_GET_INDEX_BY_CHANNEL(tty);
  if (NULL == halPointDescrip[i])
  {
    //assert(false, USARTC_HALSIGUSARTTRANSMISSIONCOMPLETE_0);
    return; // Descriptor with "tty" channel is not found.
  }

  halUsartControl = &halPointDescrip[i]->service;

   ATOMIC_SECTION_ENTER
    poW = halUsartControl->txPointOfWrite;
    poR = halUsartControl->txPointOfRead;
   ATOMIC_SECTION_LEAVE

  if (poW == poR)
    halUsartControl->usartShiftRegisterEmpty = 1; // Buffer is empty, shift register is empty too.

  if (0 == halPointDescrip[i]->txBufferLength)
    halPointDescrip[i]->txBuffer = NULL; // nulling pointer for callback mode

  if (NULL != halPointDescrip[i]->txCallback)
    halPointDescrip[i]->txCallback();
}


static void halSigUsartReceptionComplete(UsartChannel_t tty)
{
  uint8_t            i;
  HalUsartService_t *halUsartControl;
  uint16_t           number;

  i = HAL_GET_INDEX_BY_CHANNEL(tty);
  if (NULL == halPointDescrip[i])
  {
    return; // Descriptor with "tty" channel is not found.
  }

  if (halPointDescrip[i]->flowControl & (USART_SPI_READ_MODE | USART_SPI_WRITE_MODE))
    return; // for spi mode

  halUsartControl = &halPointDescrip[i]->service;
   ATOMIC_SECTION_ENTER
    number = halUsartControl->rxBytesInBuffer;
   ATOMIC_SECTION_LEAVE

  if (number)
    if (NULL != halPointDescrip[i]->rxCallback)
      halPointDescrip[i]->rxCallback(number);
}

#if defined(_USE_USART_ERROR_EVENT_)
/**************************************************************************//**
\brief Controlador de error producido.

\param[in]  tty identificador de canal usart.
******************************************************************************/
static void halSigUsartErrorOccurred(UsartChannel_t tty)
{
  uint8_t            i;
  HalUsartService_t *halUsartControl;
  UsartErrorReason_t errReason = FRAME_ERROR;

  i = HAL_GET_INDEX_BY_CHANNEL(tty);
  if (NULL == halPointDescrip[i])
  {
    assert(false, USARTC_HALSIGUSARTERROROCCURED_0);
    return; // Descriptor with "tty" channel is not found.
  }

  halUsartControl = &halPointDescrip[i]->service;
  if (halUsartControl->errorReason & HAL_BM_FRAME_ERROR)
    errReason = FRAME_ERROR;
  else if (halUsartControl->errorReason & HAL_BM_DATA_OVERRUN)
    errReason = DATA_OVERRUN;
  else if (halUsartControl->errorReason & HAL_BM_PARITY_ERROR)
    errReason = PARITY_ERROR;
  else
  {
    assert(false, USARTC_HALUNKNOWNERRORREASON_0);
  }

  if (NULL != halPointDescrip[i]->errCallback)
    halPointDescrip[i]->errCallback(errReason);
}
#endif

/**************************************************************************//**
\brief Habilita DTR wake up.

\param[in]  callback  puntero a función callback.
******************************************************************************/
void HAL_EnableDtrWakeUp(void (* callback)(void))
{
  dtrWakeUpCallback = callback;
  halEnableDtrWakeUp = true;
}

/**************************************************************************//**
\brief Deshabilita DTR wake up.
******************************************************************************/
void HAL_DisableDtrWakeUp(void)
{
  halEnableDtrWakeUp = false;
}

int HAL_IsTxEmpty(HAL_UsartDescriptor_t *descriptor)
{
  uint8_t            i;
  HalUsartService_t *halUsartControl;
  uint16_t           poW;
  uint16_t           poR;

  if (NULL == descriptor)
     return -1;
  if (false == halIsUsartChannelCorrect(descriptor->tty))
     return -1;
  i = HAL_GET_INDEX_BY_CHANNEL(descriptor->tty);
  if (descriptor != halPointDescrip[i])
     return -1; // Channel is not opened.

  halUsartControl = &halPointDescrip[i]->service;
   ATOMIC_SECTION_ENTER
    poW = halUsartControl->txPointOfWrite;
    poR = halUsartControl->txPointOfRead;
   ATOMIC_SECTION_LEAVE
  if (poW == poR)
    return halUsartControl->usartShiftRegisterEmpty;
  return 0;
}

bool halIsUsartChannelCorrect(UsartChannel_t channel)
{
  switch (channel)
  {
#ifdef USART_CHANNEL_0
    case USART_CHANNEL_0:
#endif // USART_CHANNEL_0
#ifdef USART_CHANNEL_1
    case USART_CHANNEL_1:
#endif // USART_CHANNEL_0
#if defined(USART_CHANNEL_0) || defined(USART_CHANNEL_1)
      return true;
#endif
    default:
      return false;
  }
}


static void halSetUsartClockPinDirection(HAL_UsartDescriptor_t *descriptor)
{
  if (USART_CLK_MODE_MASTER == descriptor->syncMode)
  {
    switch (descriptor->tty)
    {
#ifdef USART_CHANNEL_0
      case USART_CHANNEL_0:
		HAL_GPIO_USART0_EXTCLK_out();
        break;
#endif // USART_CHANNEL_0
      default:
        break;
    }
  }
  else
  {
    switch (descriptor->tty)
    {
#ifdef USART_CHANNEL_0
      case USART_CHANNEL_0:
		  HAL_GPIO_USART0_EXTCLK_in();
		  HAL_GPIO_USART0_EXTCLK_pullup();

        break;
#endif // USART_CHANNEL_0
// #ifdef USART_CHANNEL_1
//       case USART_CHANNEL_1:
// 		  HAL_GPIO_USART1_EXTCLK_in();
// 		  HAL_GPIO_USART1_EXTCLK_pullup();
//         break;
// #endif // USART_CHANNEL_1
      default:
      break;
    }
  }
}

#if defined(HAL_USE_USART_CHANNEL_0)
static void halUsartTaskUsart0Dre(void)
{
  halUsartHwController(USART_CHANNEL_0);
}

static void halUsartTaskUsart0Txc(void)
{
  halSigUsartTransmissionComplete(USART_CHANNEL_0);
}

static void halUsartTaskUsart0Rxc(void)
{
  halSigUsartReceptionComplete(USART_CHANNEL_0);
}

#if defined(_USE_USART_ERROR_EVENT_)
static void halUsartTaskUsart0Err(void)
{
  halSigUsartErrorOccurred(USART_CHANNEL_0);
}
#endif // defined(_USE_USART_ERROR_EVENT_)
#endif // defined(HAL_USE_USART_CHANNEL_0)

#if defined(HAL_USE_USART_CHANNEL_1)
static void halUsartTaskUsart1Dre(void)
{
  halUsartHwController(USART_CHANNEL_1);
}

static void halUsartTaskUsart1Txc(void)
{
  halSigUsartTransmissionComplete(USART_CHANNEL_1);
}

static void halUsartTaskUsart1Rxc(void)
{
  halSigUsartReceptionComplete(USART_CHANNEL_1);
}

#if defined(_USE_USART_ERROR_EVENT_)
static void halUsartTaskUsart1Err(void)
{
  halSigUsartErrorOccurred(USART_CHANNEL_1);
}
#endif // defined(_USE_USART_ERROR_EVENT_)
#endif // defined(HAL_USE_USART_CHANNEL_1)
//#endif // defined(HAL_USE_USART)

//eof usart.c
