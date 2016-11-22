/**
\file ServiNet.c

\brief Implementación de una red de sensores.
*/
/** \mainpage Implementación de una red de sensores.
 *
 * \section comp_sec Compatibilidad
 *
 *	Compatible con el microcontrolador ATmega1281 y el transceptor AT86RF230
 *
 *

 */

/*- Includes ---------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "auxiliar.h"
#include "movingAverage.h"
#include "uart.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "nwkRoute.h"
#include "nwkSecurity.h"
#include "sysTimer.h"
#include "halUart.h"
#include "halBoard.h"
#include "halSleep.h"
#include "halLed.h"
#include "battery.h" //ZE
#include <avr/sleep.h>
#ifdef TEMP
#include "temperature.h"
#endif
#ifdef HUMIDICON
#include "humidicon.h"
#endif
#ifdef ACCELEROMETER
#if defined(ADXL345)
#include "ADXL345.h"
#elif defined(ADXL362)
#include "ADXL362.h"
#endif
#endif
#include "alarms.h"
#ifdef APP_ENABLE_OTA
#include "otaClient.h"
#endif

#include "relay.h"

/*
=================================================================================================
cada X segundos los end-devices envían un mensaje por broadcast.
si los routers no han enviado un mensaje de un end-device en Y segundos, envian un mensaje de broadcast, como los end-devices
cada Z segundos el coordinador manda la lectura de sus datos
=================================================================================================

=================================================================================================
trama aérea localizacion que envía el end-device por broadcast (END-DEVICE->router->coordinador)
control

El byte de control tiene esta forma:
AABBBBBB
A-> Alarma: 0 no se ha producido una alarma, 1 se ha producido una alarma
B-> Batería: 6 bits (desde 0V hasta 6.3V)
=================================================================================================

=================================================================================================
trama aérea localización que reenvía el router (end-device->ROUTER->coordinador->bbb)
dir_end dir_end rssi num_seq control
=================================================================================================

=================================================================================================
trama aérea localización que envía el router (ROUTER->router->coordinador->bbb)
vacío
=================================================================================================

=================================================================================================
trama aérea localización que reenvía el router (router->ROUTER->coordinador->bbb)
dir_rout dir_rout rssi num_seq
=================================================================================================

=================================================================================================
trama uart localización reenviada por router (end-device->router->COORDINADOR->bbb)
10 02 lng lng tipo_trama dir_rout dir_rout dir_end dir_end rssi num_seq control 10 03 CS
=================================================================================================

=================================================================================================
trama uart localizacion reenviada por router (router->router->COORDINADOR->bbb)
10 02 lng lng tipo_trama dir_rout1 dir_rout1 dir_rout2 dir_rout2 rssi num_seq 10 03 CS
=================================================================================================

=================================================================================================
trama uart sensores
10 02 lng lng tipo_trama tipo valor tipo valor ... 10 03 CS
=================================================================================================
*/

/*- Types ------------------------------------------------------------------*/
/// \brief estructura de mensaje de lecturas de sensores
typedef struct AppMessage_t
{
  /// \brief datos
  struct
  {
#ifdef TEMP
    /// \brief lectura de temperatura
    struct
    {
      /// \brief tipo de datos
      uint8_t   type;
      /// \brief dato
      int16_t   data;
    } temperature;
#endif
#ifdef HUMIDICON
    ///brief lectura de temperatura
    struct
    {
      /// \brief tipo de datos
      uint8_t   type;
      /// \brief dato
      int16_t   data;
    } temperature;
    ///brief lectura de humedad
    struct
    {
      /// \brief tipo de datos
      uint8_t   type;
      /// \brief dato
      int16_t   data;
    } humidicon;
#endif
  } data_field;
} AppMessage_t;

/** \brief estructura del nodo a localizar
    \details
     D7       |      D6      |      D5      |      D4      |      D3      |      D2      |      D1      |      D0
--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------
     R0       |      A0      |      B5      |      B4      |      B3      |      B2      |      B1      |      B0

- R0: Datos de paso a reposo 
- A0: Datos de la alarma
- B5-B0: Datos de la batería en dV
*/
typedef struct AppLocalizationMessage_t
{
  /// \brief control
  struct
  {
    /// \brief dato de la  batería
    uint8_t battery: 6;
    /// \brief dato de la alarma
    uint8_t alarm: 1;
    /// \brief dato de paso a reposo
    uint8_t powerDown: 1;
  } control;
} AppLocalizationMessage_t;

/// \brief estructura del router encargado de reenviar el mensaje
typedef struct AppRssiMessage_t
{
  /// \brief dirección
  uint16_t address;
  /// \brief rssi
  int8_t rssi;
  /// \brief nº de secuencia
  uint8_t seqNo;
  /** \brief byte de control
      \see AppLocalizationMessage_t
  */
  uint8_t control;
} AppRssiMessage_t;

/// \brief estructura del end-device que envía por primera vez
typedef struct AppVersionMessage_t
{
  /** \brief nº de versión
      \see VERSION
  */
  uint16_t version;
  //nuevo
  /// \brief control
  struct
  {
    /// \brief dato de la batería
    uint8_t battery: 6;

    /// \brief flag del acelerómetro (funciona/no funciona)
    uint8_t accelerometer: 1;

    /// \brief reservado
    uint8_t reserved: 1;
  } control;
  //nuevo
} AppVersionMessage_t;

/// \brief estados de la máquina de estados de la aplicación
typedef enum AppState_t
{
  APP_STATE_INITIAL,
  APP_STATE_SEND,
  APP_STATE_WAIT_CONF,
  APP_STATE_SENDING_DONE,
  APP_STATE_WAIT_SEND_TIMER,
  APP_STATE_SEND_POWER_DOWN, //mio
  APP_STATE_PREPARE_TO_SLEEP,
  APP_STATE_SLEEP,
  APP_STATE_WAKEUP,
  APP_STATE_WAITING_MESSAGE, //mio
  APP_STATE_WAIT_LOCALIZATION_CONF, //mio
  APP_STATE_SENDING_LOCALIZATION_DONE, //mio
  APP_STATE_INIT_SEND, //mio
  APP_STATE_INIT_WAIT_CONF, //mio
  APP_STATE_UPDATING,
  APP_STATE_PREPARE_TO_POWER_DOWN, //mio
  APP_STATE_POWER_DOWN, //mio
  APP_STATE_WAIT_POWER_DOWN, //mio
} AppState_t;

/// \brief tipos de datos
typedef enum dataField_t
{
  //DATAFIELD_TYPE_ALARM = 0x01,
  DATAFIELD_TYPE_TEMPERATURE = 0x02,
  DATAFIELD_TYPE_HUMIDICON = 0x03,
  /*
  DATAFIELD_TYPE_BATTERY = 0x03,
  DATAFIELD_TYPE_ACCELEROMETER_X = 0x04,
  DATAFIELD_TYPE_ACCELEROMETER_Y = 0x05,
  DATAFIELD_TYPE_ACCELEROMETER_Z = 0x06,
  DATAFIELD_TYPE_RSSI_ID = 0x09,
  DATAFIELD_TYPE_RSSI_DATA = 0x0A,
  DATAFIELD_TYPE_RSSI_SEQNO = 0x0B,
  DATAFIELD_TYPE_RSSI_LQI = 0x0C,*/
} dataField_t;

/*- Variables --------------------------------------------------------------*/
/// \brief timer para la primera lectura de la batería
static SYS_Timer_t appBatteryInitTimer;
/// \brief contador para el acelerometro
uint8_t countAcc = 0;
#ifdef DEBUG_WDR
/// \brief se ha producido un watchdog reset. Se envía un 1 en el dato de alarma
static bool isWatchDogReset = false;
#endif

/// \brief indica si se ha producido un reset inesperado (watchdog reset, brown-out reset, ...)
static bool isReset = false;

//inicialización y configuración
/// \brief flag que indica si el nodo se va a apagar
bool isPowerDown = false;
/// \brief contador para el número de tramas que lleva para apagarse
uint8_t powerDownCounter = 0;

/// \brief intervalo de tiempo entre envíos
uint32_t appNormalSendingInterval;
/// \brief estado de la máquina de estados de la aplicación
static AppState_t appState = APP_STATE_INITIAL;
/// \brief indica si ha sido inicializado
static bool initialized;
/// \brief mensaje de versión
static AppVersionMessage_t appVersionMsg;
/// \brief indica si los nodos que se conecten deben esperar una actualización OTA
static bool update;
/// \brief indica si el próximo dispositivo que se conecte debe esperar una actualización OTA
static bool nextUpdate;
/// \brief timer para controlar la actualización
static SYS_Timer_t appUpdatingTimer;

/** \brief  contador para controlar el envío de datos de los routers que no localizan */
static uint8_t countNoLocalizationRouter;
//medias móviles
/** \brief Contador de cilos del timer para la limpieza de la tabla de medias móviles. */
static uint8_t countMovingAverage;
/** \brief Tipo de medias móviles, 0-> sin medias móviles, 1 -> medias móviles de 3, 2 -> medias móviles de 4. */
static uint8_t typeMovingAverage;
/** \brief Timer periódico encargado de limpiar la tabla de medias móviles. */
static SYS_Timer_t movingAverageTimer;

#ifdef HUMIDICON
/// \brief timer para la lectura del sensor de humedad
static SYS_Timer_t appHumidiconTimer;
/// \brief Contador para alternar el envío de datos de los sensores o baliza de localización
static uint8_t humidiconCounter;
#endif
#ifdef ACCELEROMETER
/// \brief Indica si se ha inicializado el acelerómetro
static bool initAccelerometer;
#endif

// Relé
/// \brief Timer para el envío de la primera trama del relé
static SYS_Timer_t appRelayInitTimer;

//comandos y UART
/// \brief Indica si se ha mandado un comando y está esperando el ACK.
static bool blockCommand;
/// \brief Timer para esperar comandos en el ACK
static SYS_Timer_t appCommandTimer;
/// \brief Estado previo
AppState_t previousState;
///  \brief Destino del comando
uint16_t dest;
/// \brief Comando interno
commandInternalType_t command;

//alarma
/// \brief Contador de alarma
uint8_t numAlarm = 0;
/// \brief Indica si se ha producido una alarma
static bool inAlarm;

//datos, version y rssi
/// \brief mensaje con lecturas de sensores
static AppMessage_t appMsg;
/// \brief mensaje con los datos de la alarma y batería del localizador
static AppLocalizationMessage_t appMsgLocalization;
/// \brief timer para el envío de datos
static SYS_Timer_t appDataSendingTimer;
/// \brief estado de la red
static bool appNetworkStatus;
/// \brief Intervalo entre envío de datos utilizado
static uint32_t appSendingInterval;
/// \brief estructura NWK_DataReq_t para el envío de datos
static NWK_DataReq_t nwkDataReq;
/// \brief estructura NWK_DataReq_t para el envío de comandos
static NWK_DataReq_t nwkDataReqCommand;
/** \brief  Timer que apaga/enciende el localizador de red
    \see    appNetworkStatusTimerHandler
*/
static SYS_Timer_t appNetworkStatusTimer;
/// \brief Variable de control para el envío de tramas de routers cuando no ven ningún localizador
static bool appTimeout;
/// \brief Timer para controlar el envío de tramas de routers cuando no ven ningún localizador
static SYS_Timer_t appTimeoutTimer;
/// \brief Indica si debe enviar la trama de versión
static bool sendVersion;

//rssi
/// \brief Entre la recepción y el envío de una trama de localización se espera un tiempo aleatorio indicado en appRssiSendingTimer.
static SYS_Timer_t appRssiSendingTimer;
/// \brief Indica si se ha recibido una baliza de localización
static bool localization;

//localización
/// \brief Cola de mensajes de localización
static AppRssiMessage_t appRssiFifo[MAX_LENGTH_RSSI_FIFO];
/// \brief Índice de la cola de mensajes de localización.
static uint8_t indexRssi = 0;
/// \brief Dirección del coordinador local.
static uint16_t localCoordinatorAddress = 0;

//wdt
/// \brief nº de veces que ha saltado el watchdog.
static uint8_t wdtRetries = 0;
/// \brief nº de reintentos máximos del watchdog.
static uint8_t wdtMaxRetries;

/*- Prototypes -------------------------------------------------------------*/
/*medias móviles*/
/** \brief Función que limpia la estructura de medias móviles
    \param[in]  timer  puntero a Timer
*/
static void movingAverageTimerHandler(SYS_Timer_t *timer);

/*relé*/
/*************************************************************************//**
\brief      Fin del temporizador appRelayInitTimer. Pasa al estado APP_STATE_SEND.
\param[in]  timer puntero al temporizador
*****************************************************************************/
static void appRelayInitTimerHandler(SYS_Timer_t *timer);

/*commandos*/
/*************************************************************************//**
\brief      Confirmación del envío del comando. Borra el comando.
\param[in]  req puntero a la estructura del frame
*****************************************************************************/
static void appCommandConf(NWK_DataReq_t *req);

/*************************************************************************//**
\brief      Fin del temporizador appCommandTimer. Pasa al estado APP_STATE_SENDING_DONE.
\param[in]  timer puntero al temporizador
*****************************************************************************/
static void appCommandTimerHandler(SYS_Timer_t *timer);

/*************************************************************************//**
\brief      Fin del temporizador appUpdatingTimer. Pasa al estado APP_STATE_SEND.
\param[in]  timer puntero al temporizador
*****************************************************************************/
static void appUpdatingTimerHandler(SYS_Timer_t *timer);

/*datos*/
/*************************************************************************//**
\brief      Confirmación del envío de datos.
\param[in]  req puntero a la estructura del frame
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req);

/*************************************************************************//**
\brief      Confirmación del envío de trama de versión.
\param[in]  req puntero a la estructura del frame
*****************************************************************************/
static void appDataVersionConf(NWK_DataReq_t *req);

/*************************************************************************//**
\brief Confirmación del envío de datos (primer envío de un nodo con relé). 
\param[in]  req puntero a la estructura del frame
*****************************************************************************/
static void appDataRelayInitConf(NWK_DataReq_t *req);

/*************************************************************************//**
\brief Confirmación del envío de datos (primer envío de un end-device). Si
	   va a recibir una actualización, pone en marcha un temporizador para la
	   recepción de la actualización. Si no va a recibir la actualización
	   cambia la configuración de red para el envío normal.
\param[in]  req puntero a la estructura del frame
*****************************************************************************/
static void appDataInitConf(NWK_DataReq_t *req);

/*************************************************************************//**
\brief      Indicador del envío del comando. Ejecuta el comando.
\param[in]  ind puntero a la estructura del indicador
*****************************************************************************/
static bool appCommandInd(NWK_DataInd_t *ind);

/*************************************************************************//**
\brief      Indicador del envío de trama de lecturas de sensores.
\param[in]  ind puntero a la estructura del indicador
*****************************************************************************/
static bool appSensorsDataInd(NWK_DataInd_t *ind);

/*************************************************************************//**
\brief      Indicador del envío de trama de relé.
\param[in]  ind puntero a la estructura del indicador
*****************************************************************************/
static bool appDatatRelayInd(NWK_DataInd_t *ind);

/*************************************************************************//**
\brief      Indicador del envío de trama de versión.
\param[in]  ind puntero a la estructura del indicador
*****************************************************************************/
static bool appDataVersionInd(NWK_DataInd_t *ind);

/*************************************************************************//**
\brief Indicador del envío de datos. Manda en el ACK un código si el nodo
	   va a ser actualizado en el caso de que el emisor sea un end-device y
	   el receptor sea el coordinador especial.
\param[in]  ind puntero a la estructura del indicador
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind);

/*************************************************************************//**
\brief Envío de trama UART en el caso de coordinadores y aérea en el resto de casos.
\see appDataSend(void)
*****************************************************************************/
INLINE void appDataSendLocalization();

/*************************************************************************//**
\brief Envío de trama UART en el caso de coordinadores y aérea en el resto de casos.
*****************************************************************************/
static void appDataSend(void);

/*************************************************************************//**
\brief Envío de trama inicial de los end-devices.
*****************************************************************************/
INLINE void appDataInitSend(void);

/*************************************************************************//**
\brief Envío de trama de versión.
*****************************************************************************/
INLINE void appDataSendVersion(void);

/*************************************************************************//**
\brief      Fin del temporizador appBatteryInitTimer utilizado para la primera lectura de la batería.
\param[in]  timer puntero al temporizador
*****************************************************************************/
static void appBatteryInitTimerHandler(SYS_Timer_t *timer);

/*************************************************************************//**
\brief      Fin del temporizador appDataSendingTimer. Pasa al estado APP_STATE_SEND.
\param[in]  timer puntero al temporizador
*****************************************************************************/
static void appDataSendingTimerHandler(SYS_Timer_t *timer);

/*************************************************************************//**
\brief      Fin del temporizador appTimeoutTimer. Pasa al estado APP_STATE_WAIT_SEND_TIMER.
\param[in]  timer puntero al temporizador
*****************************************************************************/
static void appTimeoutTimerHandler(SYS_Timer_t *timer);

/*init*/
/*************************************************************************//**
\brief Función de inicialización específica para la aplicación de localización
*****************************************************************************/
INLINE void appInitLocalization(void);

/*************************************************************************//**
\brief Función de inicialización. Inicialización de timers y configuración de red.
*****************************************************************************/
static void appInit(void);

/*************************************************************************//**
\brief      Prepara una trama de localización (router-coordinador)
\param[in]	source        puntero a la trama
\param[in]	size          tamaño de la trama
\param[in]	sourceAddress dirección del nodo a localizar
\param[in]	sourceRssi    RSSI con el que llega la trama
\param[in]	sourceLqi     LQI con el que llega la trama
\param[in]	sourceSeqNo   número de secuencia
*****************************************************************************/
void appRssiBuildMessage(uint8_t *source, uint8_t size, uint16_t sourceAddress, int8_t sourceRssi, uint8_t sourceLqi, uint8_t sourceSeqNo);

/*network*/
/*************************************************************************//**
\brief      Fin del temporizador appNetworkStatusTimer. Apaga/enciende el indicador de red.
\param[in]  timer puntero al temporizador
*****************************************************************************/
static void appNetworkStatusTimerHandler(SYS_Timer_t *timer);

/*rssi*/
/*************************************************************************//**
\brief      Confirmación de la trama enviada en appRssiRouterSend.
\param[in]  req puntero al frame
*****************************************************************************/
static void appRssiConf(NWK_DataReq_t *req);

/*************************************************************************//**
\brief Envío de trama de localización.
\details Si el dispositivo es un router se encargará de enviar la trama de localización al recibir
		 un broadcast local, ya sea de un router o de un end-device.
*****************************************************************************/
static void appRssiRouterSend(void);

/*************************************************************************//**
\brief callback llamado al terminar appRssiSendingTimer.
\details Entre la recepción y el envío de una trama de localización se espera
       un tiempo aleatorio indicado en appRssiSendingTimer.
	     Entre dos envíos de tramas de localización también se espera un tiempo aleatorio.
\param[in]  timer puntero al temporizador
*****************************************************************************/
static void appRssiTimerHandler(SYS_Timer_t *timer);

/*UART send message*/
/*************************************************************************//**
\brief envía vía UART una trama de coordinador
\param[in]  data  puntero a la trama
\param[in]  size  tamaño de la trama
*****************************************************************************/
INLINE void appUartSendCoordinatorMsg(uint8_t *data, uint8_t size);

/*************************************************************************//**
\brief envía vía UART una trama de versión del coordinador
*****************************************************************************/
INLINE void appUartSendCoordinatorVersionMsg(void);

/*************************************************************************//**
\brief      envía vía UART una trama de versión
\param[in]  data    puntero a la trama aérea
\param[in]	size    tamaño de la trama aérea
\param[in]	srcAddr dirección de origen de la trama aérea
*****************************************************************************/
INLINE void appUartSendVersion(uint8_t *data, uint8_t size, uint16_t srcAddr);

/*************************************************************************//**
\brief timer para la lectura del sensor de humedad y temperatura.
\param[in]  timer puntero al temporizador
*****************************************************************************/
static void  appHumidiconTimerHandler(SYS_Timer_t *timer);

/*UART bytes received*/
/*************************************************************************//**
\brief      lee una serie de bytes de la UART.
\param[in]	bytes número de bytes a leer
*****************************************************************************/
void HAL_UartBytesReceived(uint16_t bytes);

/*************************************************************************//**
\brief Máquina de estados de la aplicación.
*****************************************************************************/
static void APP_TaskHandler(void);

/*config init*/
/// \brief inicialización: lee canal, dirección y panid de memoria, inicializa sensores y UART
INLINE void init(void);

/*main*/
/// \brief función principal del programa
int main(void);


/*- Implementations --------------------------------------------------------*/
/*medias móviles*/
/*************************************************************************//**
*****************************************************************************/
static void movingAverageTimerHandler(SYS_Timer_t *timer)
{
  countMovingAverage++;
  clearDataMovingAverage(countMovingAverage);
  (void)timer;
}


/*comandos*/
/*************************************************************************//**
*****************************************************************************/
void appCommandSearchAndSend(void)
{
  if(!commandIsEmpty() && !blockCommand)
  {
    int8_t index = commandGetFirstNonEndDeviceIndex();
    if (index != -1) //hemos encontrado un comando
    {
      //recogemos la dirección de destino y el comando en cuestión
      dest = commandGetAddressByIndex(index);
      command = commandGetCommandByIndex(index);
      commandSetState(index, 1);
      
      //enviamos el comando al destino
      nwkDataReqCommand.data = (uint8_t *)&command;
      nwkDataReqCommand.size = sizeof(command);
      nwkDataReqCommand.dstAddr = dest;
      HAL_LedOn(LED_DATA);
      
      //mandamos el comando
      NWK_DataReq(&nwkDataReqCommand);
      blockCommand = true;
    }
  }  
}

/*************************************************************************//**
*****************************************************************************/
static void appCommandConf(NWK_DataReq_t *req)
{
  HAL_LedOff(LED_DATA);

  if (NWK_SUCCESS_STATUS == req->status)
  {    
    if (!appNetworkStatus)
    {
      HAL_LedOn(LED_NETWORK);
      SYS_TimerStop(&appNetworkStatusTimer);
      appNetworkStatus = true;
    }
    
    //done: UART comando MESSAGE_TYPE_COMMAND_OK: destino dest y comando interno command.
    appUartSendNotification(translateCommandToExternal(command), dest, NOTIFICATION_CODE_OK);
  }
  else
  {
    if (appNetworkStatus)
    {
      HAL_LedOff(LED_NETWORK);
      SYS_TimerStart(&appNetworkStatusTimer);
      appNetworkStatus = false;
    }
    
    //done: UART comando MESSAGE_TYPE_COMMAND_NO_ACK: destino dest y comando interno command.
    appUartSendNotification(translateCommandToExternal(command), dest, NOTIFICATION_CODE_NO_ACK);
  }
  
 blockCommand = false;
 commandRemove(req->dstAddr);
 appCommandSearchAndSend();  
}

/*************************************************************************//**
*****************************************************************************/
void appCommandExecute(uint16_t command)
{
  switch (command)
  {
  case COMMAND_CONFIRM_ALARM:
  {
    inAlarm = false;
    SYS_TimerStop(&appCommandTimer);
    HAL_LedOff(LED_RED_KEYFOB);
    appState = APP_STATE_SENDING_DONE;
    appSendingInterval = appNormalSendingInterval;
  }
  break;
  case COMMAND_CONFIRM_ALARM_INTERNAL:
  {
    appState = previousState;
    //pasamos al ciclo donde el nodo duerme menos
    appSendingInterval = ALARM_SLEEPING;
  }
  break;
  case COMMAND_MESSAGE_INTERNAL:
  {
    appState = APP_STATE_WAITING_MESSAGE;
    SYS_TimerStart(&appCommandTimer);
  }
  break;

  case COMMAND_ENDDEVICE_UPDATE:
  {
    update = true;
  }
  break;
  case COMMAND_ENDDEVICE_NO_UPDATE:
  {
    update = false;
    nextUpdate = false;
  }
  break;

  case COMMAND_ENDDEVICE_ONE_UPDATE:
  {
    nextUpdate = true;
  }
  break;
  case COMMAND_ENDDEVICE_UPDATE_INTERNAL:
  {
    //hay que actualizar
    //no dejamos dormir
    appState = APP_STATE_UPDATING;
    initialized = true;
    SYS_TimerStart(&appUpdatingTimer);
    //al finalizar el temporizador pasamos al estado APP_STATE_SEND
  }
  break;
  //DONE: comando de versión
  case COMMAND_VERSION:
  {
    sendVersion = true;
  }
  break;
  case COMMAND_VERSION_INTERNAL:
  {
    //DONE: el próximo mensaje enviado irá directo al coordinador local y será el de envío de versión
    sendVersion = true;//ponemos la vble de control a true
  }
  break;

  //DONE: comando para controlar un relé
  case COMMAND_RELAY_ON_INTERNAL:
  {
    relayOn();
  }
  break;
  case COMMAND_RELAY_OFF_INTERNAL:
  {
    relayOff();
  }
  break;
  case COMMAND_MOVING_AVERAGE_OFF_INTERNAL:
  {
    typeMovingAverage = 0;
    //done: apagamos led
#ifdef DEBUG
    HAL_LedOff(LED_YELLOW);
#endif
  }
  break;
  case COMMAND_MOVING_AVERAGE_ON_3_INTERNAL:
  {
    typeMovingAverage = 1;
    //configuramos la cantidad de datos
    configMovingAverage(false);
    //done: encendemos led
#ifdef DEBUG
    HAL_LedOn(LED_YELLOW);
#endif
  }
  break;
  case COMMAND_MOVING_AVERAGE_ON_4_INTERNAL:
  {
    typeMovingAverage = 2;
    //configuramos la cantidad de datos
    configMovingAverage(true);
    //done: encendemos led
#ifdef DEBUG
    HAL_LedOn(LED_YELLOW);
#endif
  }
  break;
  }
}

/*************************************************************************//**
*****************************************************************************/
static void appCommandTimerHandler(SYS_Timer_t *timer)
{
  appState = APP_STATE_SENDING_DONE;
  (void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static void appRelayInitTimerHandler(SYS_Timer_t *timer)
{
  appState = APP_STATE_SEND;
  (void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static void appUpdatingTimerHandler(SYS_Timer_t *timer)
{
  if (appState != APP_STATE_PREPARE_TO_POWER_DOWN && appState != APP_STATE_WAIT_POWER_DOWN)
    appState = APP_STATE_SEND;
  (void)timer;
}


/*envío de datos*/
/*************************************************************************//**
*****************************************************************************/
static void appDataVersionConf(NWK_DataReq_t *req)
{
  if (NWK_SUCCESS_STATUS == req->status)
  {
    //watchdog reset para broadcast local y nodos que obtienen ACK
    wdt_reset();
    wdtRetries = 0;
    sendVersion = false;
    if (!appNetworkStatus)
    {
      HAL_LedOn(LED_NETWORK);
      SYS_TimerStop(&appNetworkStatusTimer);
      appNetworkStatus = true;
    }
  }
  else
  {
    if (appNetworkStatus)
    {
      HAL_LedOff(LED_NETWORK);
      SYS_TimerStart(&appNetworkStatusTimer);
      appNetworkStatus = false;
    }
  }
  if (appState != APP_STATE_PREPARE_TO_POWER_DOWN && appState != APP_STATE_WAIT_POWER_DOWN && appState != APP_STATE_WAITING_MESSAGE) //si estamos esperando un mensaje no permitimos cambiar el estado a APP_STATE_SENDING_DONE
    appState = APP_STATE_SENDING_DONE;
}

/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
  HAL_LedOff(LED_DATA);
  //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  if (appAddrType != ADDR_TYPE_SPECIAL_COORDINATOR && appAddrType != ADDR_TYPE_LOCAL_COORDINATOR && (NWK_NO_ACK_STATUS == req->status || NWK_NO_ROUTE_STATUS == req->status))
  {
    //watchdog reset para los nodos que no obtienen ACK
    wdt_reset();
    wdtRetries = 0;
  }

  if (NWK_SUCCESS_STATUS == req->status)
  {
    //watchdog reset para broadcast local y nodos que obtienen ACK
    wdt_reset();
    wdtRetries = 0;
    if (!appNetworkStatus)
    {
      HAL_LedOn(LED_NETWORK);
      SYS_TimerStop(&appNetworkStatusTimer);
      appNetworkStatus = true;
    }
    if(req->control != 0) //se ha recibido un comando en el código de control del ACK
    {
      appCommandExecute(req->control);
    }
    //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast

    if (appAddrType == ADDR_TYPE_ENDDEVICE) //(appAddr >= 0x8000)
    {
      if(!inAlarm)
        HAL_LedOff(LED_GREEN_KEYFOB);
    }
  }
  else
  {
    if (appNetworkStatus)
    {
      HAL_LedOff(LED_NETWORK);
      SYS_TimerStart(&appNetworkStatusTimer);
      appNetworkStatus = false;
    }
  }
  if (appState != APP_STATE_PREPARE_TO_POWER_DOWN && appState != APP_STATE_WAIT_POWER_DOWN && appState != APP_STATE_WAITING_MESSAGE) //si estamos esperando un mensaje no permitimos cambiar el estado a APP_STATE_SENDING_DONE
  {
    appState = APP_STATE_SENDING_DONE;  
  }
  
}

/*************************************************************************//**
*****************************************************************************/
static void appDataRelayInitConf(NWK_DataReq_t *req)
{
  HAL_LedOff(LED_DATA);
  //watchdog reset para los nodos que no obtienen ACK
  wdt_reset();
  wdtRetries = 0;
  if (NWK_SUCCESS_STATUS == req->status)
  {
    if (!appNetworkStatus)
    {
      HAL_LedOn(LED_NETWORK);
      SYS_TimerStop(&appNetworkStatusTimer);
      appNetworkStatus = true;
    }
    if(req->control != 0) //se ha recibido un comando en el código de control del ACK
    {
      appCommandExecute(req->control);
    }
    else
    {
      initialized = true;
      appState = APP_STATE_SEND;
    }
  }
  else
  {
    if (appNetworkStatus)
    {
      HAL_LedOff(LED_NETWORK);
      SYS_TimerStart(&appNetworkStatusTimer);
      appNetworkStatus = false;
    }
    initialized = false;
    SYS_TimerStart(&appRelayInitTimer);
    //appState = APP_STATE_SEND;
  }
}

/*************************************************************************//**
*****************************************************************************/
static void appDataInitConf(NWK_DataReq_t *req)
{
  HAL_LedOff(LED_DATA);

  //watchdog reset para los nodos que no obtienen ACK
  wdt_reset();
  wdtRetries = 0;

  if (NWK_SUCCESS_STATUS == req->status)
  {
    if (!appNetworkStatus)
    {
      HAL_LedOn(LED_NETWORK);
      SYS_TimerStop(&appNetworkStatusTimer);
      appNetworkStatus = true;
    }

    if(req->control != 0) //se ha recibido un comando en el código de control del ACK
    {
      appCommandExecute(req->control);
    }
    else
    {
      //DONE: no hace falta reconfigurar la red
      /*
      //reconfiguramos la red
      NWK_SetAddr(appAddr);
      NWK_SetPanId(appPanId);
      PHY_SetChannel(appChannel);
      */
      initialized = true;
      isReset = false;
      //cambiamos al estado normal
      if (appState != APP_STATE_WAIT_POWER_DOWN && appState != APP_STATE_PREPARE_TO_POWER_DOWN)
        appState = APP_STATE_SEND;
    }
  }
  else
  {
    if (appNetworkStatus)
    {
      HAL_LedOff(LED_NETWORK);
      SYS_TimerStart(&appNetworkStatusTimer);
      appNetworkStatus = false;
    }
//     //reconfiguramos la red
//         NWK_SetAddr(appAddr);
//         NWK_SetPanId(appPanId);
//         PHY_SetChannel(appChannel);
    initialized = true;

    //cambiamos al estado normal
    if (appState != APP_STATE_WAIT_POWER_DOWN && appState != APP_STATE_PREPARE_TO_POWER_DOWN)
      appState = APP_STATE_SEND;

#ifdef ADXL362
    if (/*NWK_NO_ACK_STATUS == req->status && */appAddrType == ADDR_TYPE_ENDDEVICE && initAccelerometer && appState != APP_STATE_PREPARE_TO_POWER_DOWN)
    {
      if(isReset) {
        appState = APP_STATE_SEND;
        isReset = false;
      }        
      else
        appState = APP_STATE_PREPARE_TO_POWER_DOWN;
    }
#endif
  }
}

/*************************************************************************//**
*****************************************************************************/
static bool appCommandInd(NWK_DataInd_t *ind)
{
#ifdef NWK_ENABLE_SECURITY
  if ((ind->options & NWK_OPT_ENABLE_SECURITY) == NWK_OPT_ENABLE_SECURITY)
#endif
    appCommandExecute((uint16_t)*ind->data);
  //coordAddr = ind->srcAddr;
  return true;
}

/*************************************************************************//**
*****************************************************************************/
static bool appSensorsDataInd(NWK_DataInd_t *ind) //recepción de trama
{
  if (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR || appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR) 
  {
    HAL_LedToggle(LED_DATA);
    if (getAddrType(ind->dstAddr) != ADDR_TYPE_GLOBAL_BROADCAST) //DONE
    {
      sendUARTMessage(ind->data, ind->size, ind->srcAddr, MESSAGE_TYPE_SENSORS);
    }

  }
  return true;
}

/*************************************************************************//**
*****************************************************************************/
static bool appDatatRelayInd(NWK_DataInd_t *ind) //recepción de trama
{
#ifdef NWK_ENABLE_SECURITY
  if ((ind->options & NWK_OPT_ENABLE_SECURITY) == NWK_OPT_ENABLE_SECURITY)
  {
#endif
    if (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR || appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR)//(appAddr == 0)
    {
      HAL_LedToggle(LED_DATA);
// direccion origen del broadcast (ind->data[1] << 8 | ind->data[0])
      if (getAddrType(ind->dstAddr) != ADDR_TYPE_GLOBAL_BROADCAST) //DONE
      {
        sendUARTMessage(ind->data, ind->size, ind->srcAddr, MESSAGE_TYPE_RELAY);
      }
    }
#ifdef NWK_ENABLE_SECURITY
  }
#endif
  return true;
}

/*************************************************************************//**
*****************************************************************************/
static bool appDataVersionInd(NWK_DataInd_t *ind)
{
  // comprobamos que está activada la seguridad
#ifdef NWK_ENABLE_SECURITY
  if((ind->options & NWK_OPT_ENABLE_SECURITY) == NWK_OPT_ENABLE_SECURITY)
  {
#endif
    //done: comprobar que el emisor es un end-device
    //si soy el nodo especial, si el destino es él y si el origen es un end-device envío en el ACK si se va a actualizar o no, en caso contrario envío 0
    if((appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR) && (ind->dstAddr == appAddr) && (getAddrType(ind->srcAddr) == ADDR_TYPE_ENDDEVICE))
    {
      if(update || nextUpdate)
      {
        nextUpdate = false;
        NWK_SetAckControl(COMMAND_ENDDEVICE_UPDATE_INTERNAL);
      }
      else
        NWK_SetAckControl(COMMAND_NO_COMMAND_INTERNAL);
    }
    appUartSendVersion(ind->data, ind->size, ind->srcAddr);
#ifdef NWK_ENABLE_SECURITY
  }
#endif
  return true;

}

/*************************************************************************//**
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind) //recepción de trama
{
  //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  if (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR || appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR)//(appAddr == 0)
  {
    HAL_LedToggle(LED_DATA);

    //buscamos si hay mensajes para este nodo
    //TO-DO: Comandos en el ACK
    uint16_t command = commandGet(ind->srcAddr);
    if(command != 0)
    {
      NWK_SetAckControl(COMMAND_MESSAGE_INTERNAL);
      if (command == COMMAND_CONFIRM_ALARM)
      {
        nwkDataReq.options = NWK_OPT_ACK_REQUEST /*| NWK_OPT_ENABLE_SECURITY*/;
        nwkDataReq.dstAddr = ind->srcAddr;
        nwkDataReq.dstEndpoint = APP_ENDPOINT;
        nwkDataReq.confirm = appCommandConf;
        HAL_LedOn(LED_DATA);
        NWK_DataReq(&nwkDataReq);
      }
    }
    else
      NWK_SetAckControl(COMMAND_NO_COMMAND_INTERNAL);

    // direccion origen del broadcast (ind->data[1] << 8 | ind->data[0])
    if (getAddrType(ind->dstAddr) != ADDR_TYPE_GLOBAL_BROADCAST && getAddrType(ind->srcAddr) != ADDR_TYPE_RELAY) //DONE
    {
      typeAddress_t typeAddressBrcst = getAddrType(ind->data[1] << 8 | ind->data[0]);
      uint8_t typeMsg = 0;
      if (typeAddressBrcst == ADDR_TYPE_GLOBAL_BROADCAST) //del coordinador
        typeMsg = MESSAGE_TYPE_SENSORS;
      //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
      else if (typeAddressBrcst == ADDR_TYPE_ROUTER)
        typeMsg = MESSAGE_TYPE_ROUTER_LOCALIZATION;
      else
        typeMsg = MESSAGE_TYPE_ENDDEVICE_LOCALIZATION;
      sendUARTMessage(ind->data, ind->size, ind->srcAddr, typeMsg);
    }
    return true;
  }
  else
    return true;
}

/*************************************************************************//**
*****************************************************************************/
INLINE void appDataSendLocalization(void)
{
  //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  if (appAddrType == ADDR_TYPE_ENDDEVICE)//(appAddr >= 0x8000)
  {
#ifdef ACCELEROMETER
    //DONE: sistema de alarma
    if (alarm)
    {
      appMsgLocalization.control.alarm = 1;
      alarm = 0;
      numAlarm = 1;
    }
    else if (appMsgLocalization.control.alarm && numAlarm < MAX_ALARMS)
    {
      appMsgLocalization.control.alarm = 1;
      numAlarm++;
    }
    else
    {
      appMsgLocalization.control.alarm = 0;
    }
#endif
/*
#ifdef DEBUG_WDR
    if (isWatchDogReset)
    {
      appMsgLocalization.control.alarm |= 2;
      isWatchDogReset = false;
    }
#endif
*/
    if(isPowerDown)
    {
      appMsgLocalization.control.powerDown = 1;
    }
    else
    {
      appMsgLocalization.control.powerDown = 0;
    }
#ifdef BATTERY
    //appMsgLocalization.control.battery = getBatteryValue();
    //done: se lee la batería actual y se compara con la anterior, si es distinta se calcula el offset del acelerómetro
    uint8_t battery = getBatteryValue();
    //cambio en la lectura de la batería, por lo que hay que recalcular el offset
#if defined(ACCELEROMETER) && defined(ADXL362)
    if (initAccelerometer && battery != appMsgLocalization.control.battery)
      ADXL362_setOffset8G(battery);
#endif
    appMsgLocalization.control.battery = battery;
#endif
  }
  if (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR || appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR) //(appAddr == 0) //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  {
#if TEMP
    appMsg.data_field.temperature.data = getTemperatureValue();
#endif
#if HUMIDICON
    appMsg.data_field.humidicon.data = getHumidityValue();
    appMsg.data_field.temperature.data = getTemperatureValue();
#endif
    //si el nodo que recibe la trama es coordinador lo mandamos por la UART
    //watchdog reset para el coordinador
    wdt_reset();
    wdtRetries = 0;
    //DONE: Cambiar por función inline con 2 parámetros
    appUartSendCoordinatorMsg((uint8_t *)&appMsg, sizeof(appMsg));
    SYS_TimerStart(&appDataSendingTimer);
#if HUMIDICON
    SYS_TimerStart(&appHumidiconTimer);
#endif
    appState = APP_STATE_WAIT_SEND_TIMER;
  }
  else if (appAddrType == ADDR_TYPE_RELAY)
  {
    nwkDataReq.dstEndpoint = APP_ENDPOINT;
    nwkDataReq.srcEndpoint = APP_ENDPOINT;
    nwkDataReq.dstAddr = localCoordinatorAddress; //enviamos el mensaje por broadcast local
    nwkDataReq.options = /*NWK_OPT_ENABLE_SECURITY | */ NWK_OPT_LINK_LOCAL; //NWK_OPT_LINK_LOCAL: broadcast local
    
    nwkDataReq.size = 0;

    nwkDataReq.confirm = appDataConf;
    HAL_LedOn(LED_DATA);
    NWK_DataReq(&nwkDataReq);
    appState = APP_STATE_WAIT_CONF;
  }
  else if (appAddrType == ADDR_TYPE_ROUTER)
  {
#if (HUMIDICON || TEMP)
    //DONE: si es router envía los datos del sensor de humedad y temperatura cada cierto tiempo (no siempre) respetando el protocolo normal.
    if (humidiconCounter == 0)
    {
#if TEMP
      appMsg.data_field.temperature.data = getTemperatureValue();
#endif
#if HUMIDICON
      appMsg.data_field.humidicon.data = getHumidityValue();
      appMsg.data_field.temperature.data = getTemperatureValue();
#endif
      nwkDataReq.dstEndpoint = APP_SENSORS_ENDPOINT;
      nwkDataReq.srcEndpoint = APP_SENSORS_ENDPOINT;
      nwkDataReq.dstAddr = localCoordinatorAddress; //enviamos el coordinador local
      nwkDataReq.options = /*NWK_OPT_ENABLE_SECURITY | */NWK_OPT_ACK_REQUEST; //NWK_OPT_LINK_LOCAL: broadcast local
      nwkDataReq.data = (uint8_t *)&appMsg;
      nwkDataReq.size = sizeof(appMsg);

      nwkDataReq.confirm = appDataConf;
      HAL_LedOn(LED_DATA);
      NWK_DataReq(&nwkDataReq);
      appState = APP_STATE_WAIT_CONF;
    }
    else
    {
#endif
      nwkDataReq.dstEndpoint = APP_ENDPOINT;
      nwkDataReq.srcEndpoint = APP_ENDPOINT;
      nwkDataReq.dstAddr = 0xFFFF; //enviamos el mensaje por broadcast local
      nwkDataReq.options = /*NWK_OPT_ENABLE_SECURITY | */NWK_OPT_LINK_LOCAL; //NWK_OPT_LINK_LOCAL: broadcast local
//       if (appAddrType == ADDR_TYPE_ENDDEVICE)
//       {
//         nwkDataReq.data = (uint8_t *)&appMsgLocalization;
//         nwkDataReq.size = sizeof(appMsgLocalization);
//       }
//       else if (appAddrType == ADDR_TYPE_ROUTER)
      nwkDataReq.size = 0;

      nwkDataReq.confirm = appDataConf;
      HAL_LedOn(LED_DATA);
      NWK_DataReq(&nwkDataReq);
      appState = APP_STATE_WAIT_CONF;
    }
#if (HUMIDICON || TEMP)
    humidiconCounter++;
    if (humidiconCounter >= APP_ROUTER_NUMBER_SEND)
    {
      humidiconCounter = 0;
    }
  }
  else
#endif
    if (appAddrType == ADDR_TYPE_NO_LOCALIZATION_ROUTER)
    {
      nwkDataReq.dstEndpoint = APP_ENDPOINT;
      nwkDataReq.srcEndpoint = APP_ENDPOINT;
      nwkDataReq.dstAddr = 0xFFFF; //enviamos el mensaje por broadcast local
      nwkDataReq.options = NWK_OPT_LINK_LOCAL; //NWK_OPT_LINK_LOCAL: broadcast local
      nwkDataReq.size = 0;
      nwkDataReq.confirm = appDataConf;
      HAL_LedOn(LED_DATA);
      NWK_DataReq(&nwkDataReq);
      appState = APP_STATE_WAIT_CONF;
      countNoLocalizationRouter++;
      if(countNoLocalizationRouter % NUMBER_SEND_VERSION_ROUTER_SPECIAL == 0)
        sendVersion = true;
    }
    else if(appAddrType == ADDR_TYPE_ENDDEVICE)
    {
      nwkDataReq.dstEndpoint = APP_ENDPOINT;
      nwkDataReq.srcEndpoint = APP_ENDPOINT;
      nwkDataReq.dstAddr = 0xFFFF; //enviamos el mensaje por broadcast local
      nwkDataReq.options = /*NWK_OPT_ENABLE_SECURITY | */NWK_OPT_LINK_LOCAL; //NWK_OPT_LINK_LOCAL: broadcast local
//      if (appAddrType == ADDR_TYPE_ENDDEVICE)
//      {
      nwkDataReq.data = (uint8_t *)&appMsgLocalization;
      nwkDataReq.size = sizeof(appMsgLocalization);
//      }
//       else if (appAddrType == ADDR_TYPE_ROUTER)
//         nwkDataReq.size = 0;
      nwkDataReq.confirm = appDataConf;
      HAL_LedOn(LED_DATA);
      NWK_DataReq(&nwkDataReq);
      appState = APP_STATE_WAIT_CONF;
      /*}
      if (appAddrType == ADDR_TYPE_ENDDEVICE)//(appAddr >= 0x8000) //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
      {*/
//       if(!inAlarm)
//         HAL_LedOn(LED_GREEN_KEYFOB);
    }
}

/*************************************************************************//**
*****************************************************************************/
INLINE void appDataInitSend(void)
{
  if (appAddrType == ADDR_TYPE_ENDDEVICE)
  {
    appVersionMsg.control.battery = getBatteryValue();
    appMsgLocalization.control.battery = appVersionMsg.control.battery;

#if defined(ACCELEROMETER) && defined(ADXL362)
    appVersionMsg.control.accelerometer = initAccelerometer; //1 OK, 0 algo falla
    if(initAccelerometer)
    {
      ADXL362_setOffset8G(appVersionMsg.control.battery);
      ADXL362_setConfigFreeFall();  
    }
#endif
#ifdef NWK_ENABLE_SECURITY
    nwkDataReq.options = NWK_OPT_ENABLE_SECURITY | NWK_OPT_ACK_REQUEST;
#else
    nwkDataReq.options = NWK_OPT_ACK_REQUEST;
#endif
    //DONE: cambiar el endpoint a APP_VERSION_ENDPOINT
    nwkDataReq.dstEndpoint = APP_VERSION_ENDPOINT; //APP_ENDPOINT;
    nwkDataReq.srcEndpoint = APP_VERSION_ENDPOINT; //APP_ENDPOINT;
    nwkDataReq.dstAddr = SPECIAL_COORDINATOR_ADDRESS; //enviamos el mensaje por broadcast local
    nwkDataReq.data = (uint8_t *)&appVersionMsg;
    nwkDataReq.size = sizeof(appVersionMsg);
    nwkDataReq.confirm = appDataInitConf;
    NWK_DataReq(&nwkDataReq);
    appState = APP_STATE_INIT_WAIT_CONF;
  }
  else if (appAddrType == ADDR_TYPE_RELAY)
  {
#ifdef NWK_ENABLE_SECURITY
    nwkDataReq.options = NWK_OPT_ENABLE_SECURITY | NWK_OPT_ACK_REQUEST;
#else
    nwkDataReq.options = NWK_OPT_ACK_REQUEST;
#endif
    nwkDataReq.dstEndpoint = APP_RELAY_ENDPOINT; 
    nwkDataReq.srcEndpoint = APP_RELAY_ENDPOINT; 
    nwkDataReq.dstAddr = localCoordinatorAddress;
    nwkDataReq.data = (uint8_t *)&appVersionMsg;
    nwkDataReq.size = 0;
    nwkDataReq.confirm = appDataRelayInitConf;
    NWK_DataReq(&nwkDataReq);
    appState = APP_STATE_INIT_WAIT_CONF;
  }
}

/*************************************************************************//**
*****************************************************************************/
INLINE void appDataSendVersion(void)
{
  if (appAddrType == ADDR_TYPE_ROUTER ||appAddrType == ADDR_TYPE_RELAY)
  {
#ifdef NWK_ENABLE_SECURITY
    nwkDataReq.options = NWK_OPT_ENABLE_SECURITY | NWK_OPT_ACK_REQUEST;
#else
    nwkDataReq.options = NWK_OPT_ACK_REQUEST;
#endif
    nwkDataReq.dstEndpoint = APP_VERSION_ENDPOINT;
    nwkDataReq.srcEndpoint = APP_VERSION_ENDPOINT;
    nwkDataReq.dstAddr = localCoordinatorAddress;
    nwkDataReq.data = (uint8_t *)&appVersionMsg;
    nwkDataReq.size = sizeof(appVersionMsg);
    nwkDataReq.confirm = appDataVersionConf;
    NWK_DataReq(&nwkDataReq);
    appState = APP_STATE_INIT_WAIT_CONF;
  }
  else if (appAddrType == ADDR_TYPE_NO_LOCALIZATION_ROUTER)
  {
#ifdef NWK_ENABLE_SECURITY
    nwkDataReq.options = NWK_OPT_ENABLE_SECURITY | NWK_OPT_LINK_LOCAL;
#else
    nwkDataReq.options = NWK_OPT_LINK_LOCAL;
#endif
    nwkDataReq.dstEndpoint = APP_VERSION_ENDPOINT;
    nwkDataReq.srcEndpoint = APP_VERSION_ENDPOINT;
    nwkDataReq.dstAddr = 0xFFFF;
    nwkDataReq.data = (uint8_t *)&appVersionMsg;
    nwkDataReq.size = sizeof(appVersionMsg);
    nwkDataReq.confirm = appDataVersionConf;
    NWK_DataReq(&nwkDataReq);
    appState = APP_STATE_INIT_WAIT_CONF;
    sendVersion = false;
  }
  else if (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR || appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR)
  {
    appUartSendCoordinatorVersionMsg();
    sendVersion = false;
    //si el nodo que recibe la trama es coordinador lo mandamos por la UART
    //watchdog reset para el coordinador
    wdt_reset();
    wdtRetries = 0;
    SYS_TimerStart(&appDataSendingTimer);
#if HUMIDICON
    SYS_TimerStart(&appHumidiconTimer);
#endif
    appState = APP_STATE_WAIT_SEND_TIMER;
  }
}

/*************************************************************************//**
*****************************************************************************/
static void appDataSend(void)
{
#ifdef LOCALIZATION
  //DONE: si es router y ha recibido el comando de versión hay que enviar los datos directamente al coordinador
  if (sendVersion && (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR ||
                      appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR ||
                      appAddrType == ADDR_TYPE_ROUTER || 
                      appAddrType == ADDR_TYPE_NO_LOCALIZATION_ROUTER || 
                      appAddrType == ADDR_TYPE_RELAY))
    appDataSendVersion();
  /*else if (sendVersion && appAddrType == ADDR_TYPE_NO_LOCALIZATION_ROUTER)
    appDataSendVersion();
    */
  else if(initialized)
    appDataSendLocalization();
  else
    appDataInitSend();
#else
  //TO-DO: función para el envío de datos sin localización
#endif
}

/*************************************************************************//**
*****************************************************************************/
#ifdef HUMIDICON
static void  appHumidiconTimerHandler(SYS_Timer_t *timer)
{
  humidity_TaskHandler();
}
#endif

/*************************************************************************//**
*****************************************************************************/
//done: timer para la primera lectura de la batería, comprobar que funciona
static void appBatteryInitTimerHandler(SYS_Timer_t *timer)
{
  battery_TaskHandler();
  _delay_ms(1); //con 400 us debería ser suficiente
  (void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static void appDataSendingTimerHandler(SYS_Timer_t *timer)
{
  //if (APP_STATE_WAIT_SEND_TIMER == appState)
  if (!appTimeout || APP_STATE_WAIT_SEND_TIMER == appState)
  {
    appState = APP_STATE_SEND;

    if(appAddrType == ADDR_TYPE_ENDDEVICE)
    {
      NWK_WakeupReq();
      HAL_LedOn(LED_NETWORK);
    }
  }
  else if (appState != APP_STATE_PREPARE_TO_POWER_DOWN && appState != APP_STATE_WAIT_POWER_DOWN)
  {
    SYS_TimerStart(&appDataSendingTimer);
    //DONE: reiniciar el timer de la lectura de humedad si hay sensor de humedad.
#ifdef HUMIDICON
    if (appAddrType != ADDR_TYPE_ENDDEVICE)
      SYS_TimerStart(&appHumidiconTimer);
#endif
  }
  (void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static void appTimeoutTimerHandler(SYS_Timer_t *timer)
{
  appTimeout = false;
  appState = APP_STATE_WAIT_SEND_TIMER;
  (void)timer;
}

/*inicializacion de la aplicacion*/
/*************************************************************************//**
*****************************************************************************/
INLINE void appInitLocalization(void)
{
  appTimeoutTimer.handler = appTimeoutTimerHandler;
  appTimeoutTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appTimeoutTimer.interval = (uint32_t)(APP_ENDDEVICE_SENDING_INTERVAL) * 2;

  appRssiSendingTimer.handler = appRssiTimerHandler;
  appRssiSendingTimer.mode = SYS_TIMER_INTERVAL_MODE;

#ifdef HUMIDICON
  appMsg.data_field.humidicon.type = DATAFIELD_TYPE_HUMIDICON;
  appMsg.data_field.humidicon.data = 0;
  appMsg.data_field.temperature.type = DATAFIELD_TYPE_TEMPERATURE;
  appMsg.data_field.temperature.data = 0;
#endif

#ifdef TEMP
  appMsg.data_field.temperature.type = DATAFIELD_TYPE_TEMPERATURE;
  appMsg.data_field.temperature.data = 0;
#endif

  appVersionMsg.control.battery = 0;
  appVersionMsg.control.accelerometer = 0;
  appVersionMsg.control.reserved = 0;

  if (appAddrType != ADDR_TYPE_LOCAL_COORDINATOR && appAddrType != ADDR_TYPE_SPECIAL_COORDINATOR)
  {
    appMsgLocalization.control.alarm = 0;
    appMsgLocalization.control.battery = 0;
  }
  else
  {
    if (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR || appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR)
    {
      blockCommand  = false;
      
      #ifdef NWK_ENABLE_SECURITY
      nwkDataReqCommand.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
      #else
      nwkDataReqCommand.options = NWK_OPT_ACK_REQUEST;
      #endif
      nwkDataReqCommand.dstEndpoint = APP_COMMAND_ENDPOINT;
      nwkDataReqCommand.srcEndpoint = APP_COMMAND_ENDPOINT;
      nwkDataReqCommand.confirm = appCommandConf;
    /*
    for (int i = 0; i<MAX_LENGTH_COMMAND_QUEUE; i++)
    {
#ifdef NWK_ENABLE_SECURITY
      nwkDataReqArray[i].options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
#else
      nwkDataReqArray[i].options = NWK_OPT_ACK_REQUEST;
#endif
      nwkDataReqArray[i].dstEndpoint = APP_COMMAND_ENDPOINT;
      nwkDataReqArray[i].srcEndpoint = APP_COMMAND_ENDPOINT;
      nwkDataReqArray[i].confirm = appDataConf;
    }
    */
    }    
  }
  if(appAddrType == ADDR_TYPE_ROUTER)
  {
    NWK_OpenEndpoint(APP_COMMAND_ENDPOINT, appCommandInd);
    for (uint8_t i = 0; i < MAX_LENGTH_RSSI_FIFO; i++)
    {
      appRssiFifo[i].address = 0;
      appRssiFifo[i].control = 0;
      appRssiFifo[i].rssi = 0;
      appRssiFifo[i].seqNo = 0;
    }
  }
  else if (appAddrType == ADDR_TYPE_NO_LOCALIZATION_ROUTER || appAddrType == ADDR_TYPE_RELAY)
  {
    NWK_OpenEndpoint(APP_COMMAND_ENDPOINT, appCommandInd);
  }
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
#ifdef LOCALIZATION
  appVersionMsg.version= VERSION;
  appInitLocalization();
#else
  //TO-DO: Inicialización de mensajes cuando no hay localización
#endif
 
  //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  if (appAddrType == ADDR_TYPE_ENDDEVICE)//(appAddr >= 0x8000)
  {
    countAcc = 0;
#ifdef ACCELEROMETER
    initAccelerometer = true;
#if defined(ADXL345)
    if(!ADXL345_init(appAddrType == ADDR_TYPE_ENDDEVICE))
      if(!ADXL345_init(appAddrType == ADDR_TYPE_ENDDEVICE))
#elif defined(ADXL362)
    if(!ADXL362_init(appAddrType == ADDR_TYPE_ENDDEVICE))
      if(!ADXL362_init(appAddrType == ADDR_TYPE_ENDDEVICE))
#endif
      {
        //no se ha podido iniciar el acelerómetro
        initAccelerometer = false;
      }
#endif
  }
#ifdef PHY_AT86RF212
  PHY_SetBand(APP_BAND);
  PHY_SetModulation(APP_MODULATION);
#endif
  PHY_SetRxState(true);

#ifdef NWK_ENABLE_SECURITY
  NWK_SetSecurityKey((uint8_t *)APP_SECURITY_KEY);
#endif

  NWK_OpenEndpoint(APP_SENSORS_ENDPOINT, appSensorsDataInd);
  NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);
  NWK_OpenEndpoint(APP_VERSION_ENDPOINT, appDataVersionInd);
  NWK_OpenEndpoint(APP_RELAY_ENDPOINT, appDatatRelayInd);

  appDataSendingTimer.interval = appSendingInterval; //APP_SENDING_INTERVAL;
  appDataSendingTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appDataSendingTimer.handler = appDataSendingTimerHandler;

  //TO-DO: Cambiar el sistema de comandos?
  //temporizador para la recepción de un comando desde el aviso en el ACK.
  //Este proceso funciona de la siguiente manera. El coordinador recibe un mensaje para un end device
  //y lo almacena en una estructura. Una vez recibido un mensaje por el coordinador, comprueba si hay
  //algún mensaje para ese end device. En caso afirmativo informa a través del ACK. El end device recibe
  //el ACK y comprueba si en el código de control hay un mensaje para él. En caso afirmativo no duerme y
  //activa este temporizador. Una vez recibido el mensaje apaga el temporizador. Si no se recibe al cabo
  //de un tiempo vuelve al estado normal para ahorrar energía.
  appCommandTimer.interval = 100;
  appCommandTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appCommandTimer.handler = appCommandTimerHandler;

  appRelayInitTimer.interval = 5000;
  appRelayInitTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appRelayInitTimer.handler = appRelayInitTimerHandler;

  appUpdatingTimer.interval = 3000;
  appUpdatingTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appUpdatingTimer.handler = appUpdatingTimerHandler;
#ifdef HUMIDICON
  //done: bug corregido
  //appHumidiconTimer.interval = APP_COORDINATOR_SENDING_INTERVAL - 100;
  appHumidiconTimer.interval = appNormalSendingInterval - 100;

  appHumidiconTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appHumidiconTimer.handler = appHumidiconTimerHandler;
#endif
  if (appAddrType != ADDR_TYPE_LOCAL_COORDINATOR && appAddrType != ADDR_TYPE_SPECIAL_COORDINATOR)//(appAddr != 0) //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  {
    appNetworkStatus = false;
    appNetworkStatusTimer.interval = 500;
    appNetworkStatusTimer.mode = SYS_TIMER_PERIODIC_MODE;
    appNetworkStatusTimer.handler = appNetworkStatusTimerHandler;
    SYS_TimerStart(&appNetworkStatusTimer);
  }
  else
    HAL_LedOn(LED_NETWORK);

#ifdef PHY_ENABLE_RANDOM_NUMBER_GENERATOR
  PHY_RandomReq();
#endif

  NWK_SetAddr(appAddr);
  NWK_SetPanId(appPanId);
  PHY_SetChannel(appChannel);
  if (appAddrType == ADDR_TYPE_ENDDEVICE)
  {
    //NWK_SetPanId(appPanId/*INIT_ENDDEVICE_PANID*/);
    //PHY_SetChannel(appChannel/*INIT_ENDDEVICE_CHANNEL*/);
    appState = APP_STATE_INIT_SEND;
  }
  else
  {
    appState = APP_STATE_SEND;
  }
  //watchdog timer
  //disable interrupts
  ATOMIC_SECTION_ENTER
  wdtRetries = 0;
  //reset watchdog
  wdt_reset();
  //set up WDT interrupt
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  //Start watchdog timer with 8s prescaller
  WDTCSR = (1<<WDIE) | (1<<WDP3) |(1<<WDP0);
  //Enable global interrupts
  ATOMIC_SECTION_LEAVE
}

/*red*/
/*************************************************************************//**
*****************************************************************************/
static void appNetworkStatusTimerHandler(SYS_Timer_t *timer)
{
  HAL_LedToggle(LED_NETWORK);
  (void)timer;
}

/*************************************************************************//**
*****************************************************************************/
void appRssiBuildMessage(uint8_t *source, uint8_t size, uint16_t sourceAddress, int8_t sourceRssi, uint8_t sourceLqi, uint8_t sourceSeqNo)
{
  //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  if (appAddrType == ADDR_TYPE_ROUTER && indexRssi < MAX_LENGTH_RSSI_FIFO &&
      (getAddrType(sourceAddress) == ADDR_TYPE_ENDDEVICE
       //  || getAddrType(sourceAddress) == ADDR_TYPE_ROUTER))
       //done: si sourceAddress es un router enviar rssi si es de la misma subred:
       || (getAddrType(sourceAddress) == ADDR_TYPE_ROUTER
       //si está habilitado (SEND_LOC_ONLY_SUBNET es 1) miramos si es de la misma subred. En caso contrario no se comprueba.
#if SEND_LOC_ONLY_SUBNET == 1       
        && ((sourceAddress&0xFF00) == (appAddr&0xFF00)) // si viene de un router y es de la misma subred
#endif
        )))
  {
    appRssiFifo[indexRssi].address = sourceAddress;
    if (getAddrType(sourceAddress) == ADDR_TYPE_ENDDEVICE)
    {
#ifdef BATTERY
      uint8_t pos = offsetof(struct AppLocalizationMessage_t, control);
      appRssiFifo[indexRssi].control = source[pos];
#endif
    }
    else /*if(getAddrType(sourceAddress) == ADDR_TYPE_ROUTER)*/
      appRssiFifo[indexRssi].control = 0;
    //inicio medias móviles
    //appRssiFifo[indexRssi].rssi = sourceRssi;
    if (typeMovingAverage == 0)
    {
      appRssiFifo[indexRssi].rssi = sourceRssi;
      //appRssiFifo[indexRssi].control = (appRssiFifo[indexRssi].control & 0x3F);
      appRssiFifo[indexRssi].control = (appRssiFifo[indexRssi].control & 0xFF);
    }
    else
    {
      //calculo de las medias móviles
      appRssiFifo[indexRssi].rssi = addDataMovingAverage(sourceAddress,sourceSeqNo,sourceRssi, countMovingAverage);
      appRssiFifo[indexRssi].control = (appRssiFifo[indexRssi].control & 0xFF);
      /* done: quitado el tipo de media utilizado
            if(typeMovingAverage == 1) //de 3, alarma a 1
              appRssiFifo[indexRssi].control = 0x40 | (appRssiFifo[indexRssi].control & 0x3F);
            else //de 4, alarma a 2
              appRssiFifo[indexRssi].control = 0x80 | (appRssiFifo[indexRssi].control & 0x3F);
      */
    }
    //fin medias móviles
    appRssiFifo[indexRssi].seqNo = sourceSeqNo;
    indexRssi++;
    localization = true;

    if (!SYS_TimerStarted(&appDataSendingTimer))
      SYS_TimerStart(&appDataSendingTimer);
    if (!SYS_TimerStarted(&appRssiSendingTimer))
    {
      if (appState != APP_STATE_WAIT_CONF)
        appState = APP_STATE_WAIT_SEND_TIMER;
      appRssiSendingTimer.interval = rand() >> 7; //4 ~2s, 5 ~1s, 6 ~0.5s, 7 ~0.25s,...
      SYS_TimerStart(&appRssiSendingTimer);
    }
  }
}

/*************************************************************************//**
*****************************************************************************/
static void appRssiConf(NWK_DataReq_t *req)
{
  uint8_t typeNode = getAddrType(appRssiFifo[0].address);
  HAL_LedOff(LED_DATA);
  //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  if ((appAddrType != ADDR_TYPE_LOCAL_COORDINATOR) && (appAddrType != ADDR_TYPE_SPECIAL_COORDINATOR) /*appAddr != 0*/  && (NWK_NO_ACK_STATUS == req->status || NWK_NO_ROUTE_STATUS == req->status))
  {
    //watchdog reset para los nodos que no obtienen ACK
    wdt_reset();
    wdtRetries = 0;
  }
  if (NWK_SUCCESS_STATUS == req->status)
  {
    //watchdog reset para broadcast local y nodos que obtienen ACK
    wdt_reset();
    wdtRetries = 0;
    if (!appNetworkStatus)
    {
      HAL_LedOn(LED_NETWORK);
      SYS_TimerStop(&appNetworkStatusTimer);
      appNetworkStatus = true;
    }
    for (uint8_t i = 0; i<indexRssi; i++)
    {
      if (i+1 < MAX_LENGTH_RSSI_FIFO)
        appRssiFifo[i] = appRssiFifo[i+1];
    }
    indexRssi--;
    if(indexRssi > 0)
    {
      //appState = APP_STATE_WAIT_SEND_TIMER;
      appRssiSendingTimer.interval = rand() >> 7; //4 ~2s, 5 ~1s, 6 ~0.5s, 7 ~0.25s,...
      SYS_TimerStart(&appRssiSendingTimer);
      localization = true;
    }
    else
    {
      localization = false;
    }
  }
  else
  {
    if (appNetworkStatus)
    {
      HAL_LedOff(LED_NETWORK);
      SYS_TimerStart(&appNetworkStatusTimer);
      appNetworkStatus = false;
    }
  }

  if (typeNode == ADDR_TYPE_ENDDEVICE)
  {
    //reiniciamos el timeout

    SYS_TimerStop(&appTimeoutTimer);
    appTimeout = true;
    //DONE: probar aquí a reiniciar el contador humidiconCounter
    humidiconCounter = 0;
    SYS_TimerStart(&appTimeoutTimer);
  }
  appState = APP_STATE_SENDING_DONE;
}

/*************************************************************************//**
*****************************************************************************/
void appRssiRouterSend(void)
{
  if (appAddrType == ADDR_TYPE_ROUTER)
  {
    nwkDataReq.dstAddr = localCoordinatorAddress; //0;
    nwkDataReq.options = /*NWK_OPT_ACK_REQUEST | *//*NWK_OPT_ENABLE_SECURITY*/ 0;
    nwkDataReq.dstEndpoint = APP_ENDPOINT;
    nwkDataReq.srcEndpoint = APP_ENDPOINT;
    nwkDataReq.data = (uint8_t *)&appRssiFifo[0];
    nwkDataReq.size = sizeof(appRssiFifo[0]);
    nwkDataReq.confirm = appRssiConf;

    HAL_LedOn(LED_DATA);
    NWK_DataReq(&nwkDataReq);
    if (getAddrType(appRssiFifo[0].address) == ADDR_TYPE_ENDDEVICE)
    {
      nwkDataReq.size = sizeof(appRssiFifo[0]);
      //js para esto comprobar si viene de un end-device
      //SYS_TimerStop(&appDataSendingTimer);
      //SYS_TimerStart(&appDataSendingTimer);
      //fin js
      //humidiconCounter = 0;
    }
    else
      nwkDataReq.size = sizeof(appRssiFifo[0]) - 1;
  }
  appState = APP_STATE_WAIT_LOCALIZATION_CONF;
}

/*************************************************************************//**
*****************************************************************************/
static void appRssiTimerHandler(SYS_Timer_t *timer)
{
  if (appState == APP_STATE_WAIT_CONF)
  {
    appRssiSendingTimer.interval = 10;
    SYS_TimerStart(&appRssiSendingTimer);
  }
  else
    appState = APP_STATE_SEND;
  (void)timer;
}

/*UART*/
/*************************************************************************//**
*****************************************************************************/
INLINE void appUartSendCoordinatorVersionMsg(void)
{
  uint8_t data[3] = {/*(appAddr & 0xFF), ((appAddr >> 8)  & 0xFF),    //address */
                     (VERSION & 0x00FF), ((VERSION >> 8)  & 0xFF),  //version
                     0x00                                           //control
                    }; 
  sendUARTMessage(data, sizeof(data), /*0xFFFF*/ appAddr, MESSAGE_TYPE_VERSION);
}

/*************************************************************************//**
*****************************************************************************/
INLINE void appUartSendCoordinatorMsg(uint8_t *data, uint8_t size)
{
  sendUARTMessage(data, size, appAddr /*0xFFFF //antes*/, MESSAGE_TYPE_SENSORS);
}

/*************************************************************************//**
*****************************************************************************/
INLINE void appUartSendVersion(uint8_t *data, uint8_t size, uint16_t srcAddr)
{
  sendUARTMessage(data, size, srcAddr, MESSAGE_TYPE_VERSION);
}

/*************************************************************************//**
*****************************************************************************/
void HAL_UartBytesReceived(uint16_t bytes)
{
  if (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR || appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR)
  {
    for (uint16_t i = 0; i < bytes; i++)
    {
      uint8_t byte = HAL_UartReadByte();
      uartStateMachine(byte);
    }
#ifdef DEBUG
    HAL_LedToggle(2);
#endif
  }
}

/*máquina de estados de la aplicación*/
/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void)
{
  switch (appState)
  {
  case APP_STATE_INITIAL:
  {
    appInit();
  }
  break;
  //envío de trama a red especial
  case APP_STATE_INIT_SEND:
  {
    isPowerDown = false;
    countAcc = 0; //pasamos el contador del acelerómetro a 0
    powerDownCounter = 0;  //ponemos el contador de envío de pasar a reposo a 0
    
    if (!SYS_TimerStarted(&appBatteryInitTimer))
    {
      appDataSend();
    }
  }
  break;
  case APP_STATE_SEND:
  {
    if(localization)
      appRssiRouterSend();
    else
      appState = APP_STATE_SENDING_LOCALIZATION_DONE;
  }
  break;
  case APP_STATE_SENDING_LOCALIZATION_DONE:
  {
    if(!appTimeout)
    {
      if (appAddrType == ADDR_TYPE_ENDDEVICE)
      {
#ifdef BATTERY
        battery_TaskHandler();
#endif
      }
      else if (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR || appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR)
      {
#ifdef TEMP
        temperature_TaskHandler();
#endif
      }
      appDataSend();
    }
  }
  break;

  case APP_STATE_SENDING_DONE:
  {
    if (appAddrType == ADDR_TYPE_ENDDEVICE)
    {
      if (isPowerDown)
      {
         appState = APP_STATE_PREPARE_TO_SLEEP;
      }
      else
#ifdef ADXL362 
      {      
        countAcc++;
        if (countAcc >= APP_ENDDEVICE_MAX_INACTIVITY)
        {
          countAcc = 0;
          
          if(ADXL362_sleep())
          {
            appState = APP_STATE_PREPARE_TO_SLEEP;
            isPowerDown = true;            
           /* if(!isPowerDown)
            {
              powerDownCounter = 0;  //ponemos el contador de envío de pasar a reposo a 0
              isPowerDown = true;
            }*/            
            //appState = APP_STATE_PREPARE_TO_POWER_DOWN;
          }          
          else
          {
            ADXL362_enableActivityInterrupt();
            
            appState = APP_STATE_PREPARE_TO_SLEEP;
          }
        }
        else
        {
#endif
          appState = APP_STATE_PREPARE_TO_SLEEP;
#ifdef ADXL362
        }
      }      
#endif
    }
    else
    {
      if (!SYS_TimerStarted(&appDataSendingTimer))
      {
        SYS_TimerStart(&appDataSendingTimer);
        appState = APP_STATE_WAIT_SEND_TIMER;
#ifdef HUMIDICON
        SYS_TimerStart(&appHumidiconTimer);
#endif
      }
    }
  }
  break;
  
  case APP_STATE_SEND_POWER_DOWN:
  {
    //enviamos que nos vamos a apagar
    
    appDataSend();
    powerDownCounter++;
    
    if(powerDownCounter >= NUMBER_POWER_DOWN)
    {
      
      //pasamos a dormir
      appState = APP_STATE_PREPARE_TO_POWER_DOWN;
    }      
  }
  break;

  case APP_STATE_PREPARE_TO_SLEEP:
  {
    if (!NWK_Busy())
    {
      NWK_SleepReq();

      appState = APP_STATE_SLEEP;
    }
  }
  break;

  case APP_STATE_SLEEP:
  {
#ifdef ACCELEROMETER
#if defined (ADXL345)
    if (!initAccelerometer || !ADXL345_inFallDetection())
#elif defined (ADXL362)
    if (!initAccelerometer || !ADXL362_inFallDetection())
#endif
    {
#endif
      despierta = false;
      HAL_LedOff(LED_NETWORK);
      HAL_LedOff(LED_DATA);
      HAL_Sleep(appSendingInterval);
      appState = APP_STATE_WAKEUP;
#ifdef ACCELEROMETER
    }
#endif
  }
  break;

  case APP_STATE_WAKEUP:
  {
#ifdef ACCELEROMETER
#if defined(ADXL345)
    //despertar si estoy en FreeFallDetection y dejar este estado hasta que se ponga a false FreeFallDetection o llegue a un estado de alarma
    if (!initAccelerometer || !ADXL345_inFallDetection() || alarm)
#elif defined(ADXL362)
    if (!initAccelerometer || !ADXL362_inFallDetection() || alarm)
#endif
    {
#endif
      NWK_WakeupReq();
      HAL_LedOn(LED_NETWORK);
      if (isPowerDown)
        appState = APP_STATE_SEND_POWER_DOWN;
      else
        appState = APP_STATE_SEND;
#ifdef ACCELEROMETER
    }
#if defined(ADXL345)
    else if (initAccelerometer && ADXL345_inFallDetection())
#elif defined(ADXL362)
    else if (initAccelerometer && ADXL362_inFallDetection())
#endif
    {
      SYS_TimerStart(&appDataSendingTimer);
      appState = APP_STATE_WAIT_SEND_TIMER;
    }
#endif
  }
  break;
  
  case APP_STATE_UPDATING:
  {
    if (blockOTAReceived)
    {
      SYS_TimerStop(&appUpdatingTimer);
      SYS_TimerStart(&appUpdatingTimer);
      blockOTAReceived = false;
    }
    if (SYS_TimerStarted(&appUpdatingTimer))
    {
      wdt_reset();
      wdtRetries = 0;
    }
  }
  break;
  
#if defined(ACCELEROMETER) && defined(ADXL362)
  //TO-DO: eliminar estado APP_WAIT_POWER_DOWN
  case APP_STATE_WAIT_POWER_DOWN:
  {
    if (ADXL362_sleep())
      appState = APP_STATE_PREPARE_TO_POWER_DOWN;
  }
  break;
  case APP_STATE_PREPARE_TO_POWER_DOWN:
  {
    if (!NWK_Busy())
    {
      NWK_SleepReq();
      wdt_disable();
      wdtRetries = 0;

      SYS_TimerStop(&appUpdatingTimer);
      SYS_TimerStop(&appCommandTimer);

      while (ASSR & ((1 << TCN2UB) | (1 << OCR2AUB) | (1 << OCR2BUB) | (1 << TCR2AUB) | (1 << TCR2BUB)));

      ADXL362_toSleep();
      EICRB &= ~(1<<ISC70);
      SMCR = (1 << SM1) | (1 << SE); // power-down
      asm("sleep");

      SMCR = 0;
      EICRB |= (1<<ISC70);

      //watchdog timer
      //disable interrupts
      ATOMIC_SECTION_ENTER
      //reset watchdog
      wdt_reset();
      //set up WDT interrupt
      WDTCSR |= (1<<WDCE)|(1<<WDE);
      //Start watchdog timer with 8s prescaller
      WDTCSR = (1<<WDIE)|(1<<WDP3)|(1<<WDP0);
      //Enable global interrupts
      ATOMIC_SECTION_LEAVE
      ADXL362_setActivityThreshold(13); 
      initialized = false;

      NWK_WakeupReq();
      appState = APP_STATE_INIT_SEND;
    }
  }
  break;
#endif
  default:
    break;
  }
}


/*inicialización*/
/*************************************************************************//**
*****************************************************************************/
INLINE void init(void)
{
#ifdef APP_ADDR
  appAddr = APP_ADDR;
#else
  appAddr = pgm_read_word_far(BOOT_ADDR_APP_ADDR);
#endif
#ifdef APP_PANID
  appPanId = APP_PANID;
#else
  appPanId = pgm_read_word_far(BOOT_ADDR_APP_PANID);
#endif
#ifdef APP_CHANNEL
  appChannel = APP_CHANNEL;
#else
  appChannel = (pgm_read_word_far(BOOT_ADDR_APP_CHANNEL) & 0xFF);
#endif
  appAddrType = getAddrType(appAddr);
  
   /*partes comunes*/
   HAL_BoardInit();
   HAL_LedInit();
   
  //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  if (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR || appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR)//(appAddr == 0)
  {
    appNormalSendingInterval = APP_COORDINATOR_SENDING_INTERVAL;
    wdtMaxRetries = WDT_MAX_RETRIES_COORDINATOR;
    initialized = true;
#ifdef HUMIDICON
    humidity_Init(true);
#endif
  }
  else if (appAddrType == ADDR_TYPE_NO_LOCALIZATION_ROUTER)
  {
    appNormalSendingInterval = APP_COORDINATOR_SENDING_INTERVAL * 5;
    wdtMaxRetries = (WDT_MAX_RETRIES_COORDINATOR - 1) * 5 + 1;
    initialized = true;
    sendVersion = true;
  }
  //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  else if (appAddrType == ADDR_TYPE_ROUTER) //(appAddr < 0x8000)
  {
#ifdef HUMIDICON
    humidity_Init(true);
#endif
    appNormalSendingInterval = APP_ROUTER_SENDING_INTERVAL;
    wdtMaxRetries = WDT_MAX_RETRIES_ROUTER;

    //la dirección del coordinador local es el byte más significativo de la dirección del router
    localCoordinatorAddress = appAddr >> 8;
    initialized = true;

    //configuración de medias móviles
    configMovingAverage(true); //por defecto ponemos las medias móviles
#ifdef DEBUG
    HAL_LedOn(LED_YELLOW);     //y encendemos el led de debug
#endif
    typeMovingAverage = 1;
    movingAverageTimer.interval = 10000;
    movingAverageTimer.mode = SYS_TIMER_PERIODIC_MODE;
    movingAverageTimer.handler = movingAverageTimerHandler;
    SYS_TimerStart(&movingAverageTimer);
  }
  else if (appAddrType == ADDR_TYPE_RELAY)
  {
    appNormalSendingInterval = APP_COORDINATOR_SENDING_INTERVAL;
    wdtMaxRetries = WDT_MAX_RETRIES_COORDINATOR;
    initialized = false;
    sendVersion = false;
    //la dirección del coordinador local es el byte más significativo de la dirección del router
    localCoordinatorAddress = appAddr >> 8;
    relayInit(true);
  }
  else if (appAddrType == ADDR_TYPE_ENDDEVICE) //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  {
    appNormalSendingInterval = APP_ENDDEVICE_SENDING_INTERVAL;
    wdtMaxRetries = WDT_MAX_RETRIES_ENDDEVICE;
    initialized = false;

    //HAL_LedOff(LED_YELLOW);
    //se ha producido un reset
    if((MCUSR & (1<<WDRF))  == (1<<WDRF)) //wdg reset
    {
      isReset = true;
#ifdef DEBUG_WDR
      isWatchDogReset = true;
#endif
    }
    else if((MCUSR & (1<<BORF))  == (1<<BORF)) //brown-out reset
    {
      isReset = true;      
      //HAL_LedOn(LED_YELLOW);
    }
    else
    {
      isReset = false;
    }
    
    MCUSR = 0;
  }
  
  appSendingInterval = appNormalSendingInterval;

  //utilizamos como semilla para los números aleatorios la dirección del nodo
  srand(appAddr);
  SYS_Init();

#ifdef BATTERY
  if (appAddrType == ADDR_TYPE_ENDDEVICE)
  {
    battery_Init();
    battery_TaskHandler();

    appBatteryInitTimer.interval = 50; //tras agotar este timer terminar de configurar el acelerómetro
    appBatteryInitTimer.mode = SYS_TIMER_INTERVAL_MODE;
    appBatteryInitTimer.handler = appBatteryInitTimerHandler;

    //iniciamos el temporizador para la primera lectura de la batería
    SYS_TimerStart(&appBatteryInitTimer);
  }
#endif

  //watchdog timer
  //disable interrupts
  ATOMIC_SECTION_ENTER
  wdtRetries = 0;
  //reset watchdog
  wdt_reset();
  //set up WDT interrupt
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  //Start watchdog timer with 8s prescaller
  WDTCSR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0);
  //Enable global interrupts
  ATOMIC_SECTION_LEAVE

  //quitado de aquí
  HAL_UartInit(38400);
#ifdef APP_ENABLE_OTA
  OTA_ClientInit();
#endif


  appState = APP_STATE_INITIAL;
  numAlarm = 0;
  indexRssi = 0;
}


/*************************************************************************//**
*****************************************************************************/
int main(void)
{
  //inicializar
  init();
  while (1)
  {
    SYS_TaskHandler();
    if (appAddrType == ADDR_TYPE_LOCAL_COORDINATOR || appAddrType == ADDR_TYPE_SPECIAL_COORDINATOR) //fixed
      HAL_UartTaskHandler(); //solo para coordinador
#ifdef APP_ENABLE_OTA
    OTA_ClientTaskHandler();
#endif
    APP_TaskHandler();
  }
}


/*interrupciones*/
/*************************************************************************//**
\brief Interrupción para reiniciar el watchdog
*****************************************************************************/
//watchdog timer
ISR(WDT_vect)
{
  wdt_disable();
  if (wdtRetries <= wdtMaxRetries)
  {
    //wdt
    //reset watchdog
    wdt_reset();
    //set up WDT interrupt
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    //Start watchdog timer with 8s prescaller
    WDTCSR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0);
    wdtRetries++;
  }
  else
  {
    //reiniciar el dispositivo
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDE);
    WDTCSR = (1 << WDE);
  }
}