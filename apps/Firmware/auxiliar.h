/**
 \file auxiliar.h

 \brief Tipos, variables y funciones auxiliares
 */

#ifndef _AUXILIAR_H_
#define _AUXILIAR_H_
#include <stdbool.h>
/*- Definitions ------------------------------------------------------------*/
/*CONFIGURACION DE LEDS*/
/** \brief Led indicador de red. */
#define LED_NETWORK        0
/** \brief Led indicador de datos. */
#define LED_DATA           1 //DS1
/** \brief Led amarillo. */
#define LED_YELLOW		   2   //DS2
/** \brief Led verde de los llaveros. */
#define LED_GREEN_KEYFOB   3
/** \brief Led rojo de los llaveros. */
#define LED_RED_KEYFOB     4

/** \brief Dirección del bootloader donde se encuentra el canal. */
#define BOOT_ADDR_APP_CHANNEL    (SYS_DEVICE_SIZE - 10)
/** \brief Dirección del bootloader donde se encuentra la dirección. */ 
#define BOOT_ADDR_APP_ADDR       (SYS_DEVICE_SIZE - 8) 
/** \brief Dirección del bootloader donde se encuentra el panid. */
#define BOOT_ADDR_APP_PANID      (SYS_DEVICE_SIZE - 6) 

/** \brief Dirección del coordinador especial. */
#define SPECIAL_COORDINATOR_ADDRESS 0x007F


/** \brief Tamaño del búffer para las tramas de los localizadores en los routers. */
#define MAX_LENGTH_RSSI_FIFO 50
/** \brief Tamaño del búffer para los comandos. */
#define MAX_LENGTH_COMMAND_QUEUE 10

/** \brief Número de tramas que va a enviar antes de apagarse */
#define NUMBER_POWER_DOWN 2

/** \brief Número de tramas vacías entre tramas de versión para routers especiales. */
#define NUMBER_SEND_VERSION_ROUTER_SPECIAL 5

/*VARIABLES*/
/** \brief Tipo de dirección del nodo. */
uint8_t volatile appAddrType;
/** \brief Dirección del nodo. */
uint16_t volatile appAddr;
/** \brief PanId del nodo. */
uint16_t volatile appPanId;
/** \brief Canal del nodo. */
uint8_t volatile appChannel;

/*TIPOS DE DIRECCIÓN*/
/**
\brief Tipos de dispositivo según su dirección.

\see getAddrType(uint16_t address)                               
*/
typedef enum
{
/// Reservado
  ADDR_TYPE_NO_TYPE = -1,
/// Coordinador local.
  ADDR_TYPE_LOCAL_COORDINATOR = 0,
/// Router, enruta y se encarga de tareas de localización.
  ADDR_TYPE_ROUTER = 1,
/// End-device.
  ADDR_TYPE_ENDDEVICE = 2,
/// Broadcast a nivel de red.
  ADDR_TYPE_GLOBAL_BROADCAST = 3,
/// Reservado a broadcast a nivel de subred. No implementado.
  ADDR_TYPE_SUBNET_BROADCAST = 4,
/// Servidor de OTA.
  ADDR_TYPE_OTASERVER = 5,
/// Coordinador especial (0x0080) encargado de responder a la primera trama de los end-devices.  
  ADDR_TYPE_SPECIAL_COORDINATOR = 6,
/// Router especial, solo se encargan de enrutar, no de localizar.
  ADDR_TYPE_NO_LOCALIZATION_ROUTER = 7,
/// Nodo con relé como actuador
  ADDR_TYPE_RELAY = 8  
} typeAddress_t;


/*TIPOS DE TRAMA*/
/**
\brief Tipos de trama.
*/
typedef enum
{
///Trama con lecturas de sensores (humedad y temperatura)
MESSAGE_TYPE_SENSORS = 0x0,
///Trama de localización (router)
MESSAGE_TYPE_ROUTER_LOCALIZATION = 0x1,
///Trama de localización (end-device)
MESSAGE_TYPE_ENDDEVICE_LOCALIZATION = 0x2, 
///Trama de versión
MESSAGE_TYPE_VERSION = 0x3, 
///Trama de relé (primera trama)
MESSAGE_TYPE_RELAY = 0x4,
///Trama comando
MESSAGE_TYPE_COMMAND = 0x5
} typeMessage_t;


/*FUNCIONES AUXILIARES*/
/**************************************************************************//**
\brief  Devuelve el tipo de dispositivo de la dirección pasada por parámetro.
\details
Direcciones    | Valor                            | Uso
-------------  | ---------------------------------|-------------
0x0000         | ADDR_TYPE_NO_TYPE                | Reservado (posible coordinador global)
0x0001-0x007E  | ADDR_TYPE_LOCAL_COORDINATOR      | Direcciones de los coordinadores locales
0x007F         | ADDR_TYPE_SPECIAL_COORDINATOR    | Dirección del coordinador especial
0x0080         | ADDR_TYPE_NO_TYPE                | Dirección del coordinador especial
0x0081-0x00FE  | ADDR_TYPE_NO_LOCALIZATION_ROUTER | Routers que funcionan como tales pero que no intervienen en las tareas de localización
0x00FF         | ADDR_TYPE_NO_TYPE                | Reservado
0xss00         | ADDR_TYPE_NO_TYPE                | Reservado, ss es la dirección de la subred ss cuyo coordinador local tiene la dirección 0x00ss. Intervienen en tareas de localización
0xss01-0xssEF  | ADDR_TYPE_ROUTER                 | Direcciones para los routers de la subred ss cuyo coordinador local es el 0x00ss
0xssF0-0xssFE  | ADDR_TYPE_RELAY                  | Nodos especiales para controlar relés
0xssFF         | ADDR_TYPE_SUBNET_BROADCAST       | Reservado a broadcast a nivel de subred ss
0x8000         | ADDR_TYPE_OTASERVER              | Servidor OTA
0x8001-0xEFFF  | ADDR_TYPE_ENDDEVICE              | Direcciones de los end-devices
0xF000-0xFFFE  | ADDR_TYPE_NO_TYPE                | Reservado
0xFFFF         | ADDR_TYPE_GLOBAL_BROADCAST       | Dirección broadcast

\param[in]	address	Dirección.

\return El tipo de dirección.

\see typeAddress_t
******************************************************************************/
static typeAddress_t getAddrType(uint16_t address)
{
  if (address == 0x0000)
    return ADDR_TYPE_NO_TYPE;
  if ((address > 0x0000) && (address < 0x007F))
    return ADDR_TYPE_LOCAL_COORDINATOR;
  else if (address == 0x007F)
    return ADDR_TYPE_SPECIAL_COORDINATOR; //DONE: Cambiar a un tipo especial
  else if (address == 0x0080)
    return ADDR_TYPE_NO_TYPE;
  else if ((address > 0x0080) && (address < 0x00FF))
    return ADDR_TYPE_NO_LOCALIZATION_ROUTER;
  else if (address == 0x00FF)
    return ADDR_TYPE_NO_TYPE;
  /*
  else if ((address > 0x0080) && (address < 0x0100))
    return ADDR_TYPE_NO_TYPE;
  */
  else if ((address >=0x0100) && (address < 0x8000))
  {
    if (((address & 0xFF) > 0x00) && ((address & 0xFF) < 0xF0))
      return ADDR_TYPE_ROUTER;
    else if ((address & 0xFF) == 0xFF)
      return ADDR_TYPE_SUBNET_BROADCAST;
    else
      return ADDR_TYPE_RELAY;
  }
  else if (address == 0x8000)
    return ADDR_TYPE_OTASERVER;
  else if ((address > 0x8000) && (address < 0xF000))
    return ADDR_TYPE_ENDDEVICE;
  else if (address == 0xF000)
    return ADDR_TYPE_NO_TYPE;
  else if ((address > 0xF000) && (address < 0xFFFF))
    return ADDR_TYPE_NO_TYPE;
  else //if (address == 0xFFFF)
    return ADDR_TYPE_GLOBAL_BROADCAST;
}
/*FIN FUNCIONES AUXILIARES*/

#endif // _AUXILIAR_H_