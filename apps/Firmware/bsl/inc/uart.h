/**************************************************************************//**
\file  uart.h

\brief Comunicaci�n UART en ambos sentidos: env�o de tramas y 
	   recepci�n de comandos
******************************************************************************/

#ifndef _UART_H_
#define _UART_H_

#include <stdio.h>
#include "auxiliar.h"
#include "nwk.h"
#include "halUart.h"
#include "halLed.h"
/** \brief tama�o del comando */
#define MAX_UART_BUFFER_SIZE 2
/*Estructuras y enumeraciones*/
/**\brief Estados de la m�quina de estados para la lectura de comandos.
 */
typedef enum AppUartState_t
{
  APP_UART_STATE_START_PRE,
  APP_UART_STATE_START_POST,
  APP_UART_STATE_READ_SIZE,
  APP_UART_STATE_READ_NUM_DESTINATION,
  APP_UART_STATE_READ_DESTINATION,
  APP_UART_STATE_READ_DATA,
  APP_UART_STATE_END_PRE,
  APP_UART_STATE_END_POST,
  APP_UART_STATE_READ_CHECKSUM,
} AppUartState_t;

//DONE: separar comandos internos de comandos UART
/** 
\brief Comandos a nivel de red.
\details
Comando                              | Valor | Uso
-------------------------------------|-------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
COMMAND_NO_COMMAND_INTERNAL          | 0x00  | No hay comando.
COMMAND_MESSAGE_INTERNAL             | 0x01  | Hay mensajes para este nodo. El nodo receptor el comando no duerme hasta que recibe el mensaje.
COMMAND_ERROR_NO_SPACE               | 0x02  | No hay espacio para almacenar el comando.
COMMAND_CONFIRM_ALARM_INTERNAL       | 0x03  | La alarma ha sido recibida por el coordinador y transmitida por la UART. Pasa al modo recepci�n de confirmaci�n del operario. El intervalo de env�o de datos pasa a ser ALARM_SLEEPING.
COMMAND_ENDDEVICE_UPDATE_INTERNAL    | 0x04  | Indica al end-device que va a recibir una actualizaci�n v�a OTA.
COMMAND_VERSION_INTERNAL             | 0x05  | Solicita al nodo destinatario que env�e el n�mero de versi�n de su firmware (Solo para routers).
COMMAND_RELAY_OFF_INTERNAL           | 0x06  | Indica al nodo destinatario que apague el rel�.
COMMAND_RELAY_ON_INTERNAL            | 0x07  | Indica al nodo destinatario que encienda el rel�.
COMMAND_MOVING_AVERAGE_OFF_INTERNAL  | 0x08  | Indica al router destino que debe mandar los datos del rssi en crudo.
COMMAND_MOVING_AVERAGE_ON_3_INTERNAL | 0x09  | Indica al router destino que debe mandar los datos del rssi usando medias m�viles con 3 datos (el �ltimo y los 2 anteriores).
COMMAND_MOVING_AVERAGE_ON_3_INTERNAL | 0x0A  | Indica al router destino que debe mandar los datos del rssi usando medias m�viles con 4 datos (el �ltimo y los 3 anteriores).
*/
typedef enum commandInternalType_t
{  
  COMMAND_NO_COMMAND_INTERNAL = 0x00,
  COMMAND_MESSAGE_INTERNAL = 0x01,
  COMMAND_ERROR_NO_SPACE = 0x02,
  COMMAND_CONFIRM_ALARM_INTERNAL = 0x03,
  COMMAND_ENDDEVICE_UPDATE_INTERNAL = 0x04,
  COMMAND_VERSION_INTERNAL = 0x05,
  COMMAND_RELAY_OFF_INTERNAL = 0x06,
  COMMAND_RELAY_ON_INTERNAL = 0x07,
  COMMAND_MOVING_AVERAGE_OFF_INTERNAL = 0x08,
  COMMAND_MOVING_AVERAGE_ON_3_INTERNAL = 0x09,
  COMMAND_MOVING_AVERAGE_ON_4_INTERNAL = 0x0A,
} commandInternalType_t;

/**
\brief Comandos a nivel de UART.
\details
Comando                      | Valor  | Uso
-----------------------------|--------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------
COMMAND_CONFIRM_ALARM        | 0x0101 | Indica que la alarma ha sido confirmada por un operador. El nodo puede pasar al modo normal (intervalo de env�o de datos de APP_SENDING_INTERVAL) y apagar el led.
COMMAND_ENDDEVICE_NO_UPDATE  | 0x0400 | Indica que los end-devices que se conecten al coordinador NO van a recibir una actualizaci�n v�a OTA y, por tanto, deben conectarse a la red habitual.
COMMAND_ENDDEVICE_UPDATE     | 0x0401 | Indica que los end-devices que se conecten al coordinador van a recibir una actualizaci�n v�a OTA.
COMMAND_ENDDEVICE_ONE_UPDATE | 0x0402 | Indica que el siguiente end-devices que se conecte al coordinador va a recibir una actualizaci�n v�a OTA.
COMMAND_VERSION              | 0x0500 | Solicita al nodo destinatario que env�e el n�mero de versi�n de su firmware (routers y coordinadores).
COMMAND_RELAY_OFF            | 0x0600 | Indica que se debe apagar el rel�.
COMMAND_RELAY_ON             | 0x0601 | Indica que se debe encender el rel�.
COMMAND_MOVING_AVERAGE_OFF   | 0x0700 | Indica al router destino que debe mandar los datos del rssi en crudo.
COMMAND_MOVING_AVERAGE_ON_3  | 0x0703 | Indica al router destino que debe mandar los datos del rssi usando medias m�viles con 3 datos (el �ltimo y los 2 anteriores).
COMMAND_MOVING_AVERAGE_ON_4  | 0x0704 | Indica al router destino que debe mandar los datos del rssi usando medias m�viles con 4 datos (el �ltimo y los 3 anteriores).
*/
typedef enum commandExternalType_t
{
  COMMAND_CONFIRM_ALARM = 0x0101,
  COMMAND_ENDDEVICE_NO_UPDATE = 0x0400, //1002070001800000041003B1
  COMMAND_ENDDEVICE_UPDATE = 0x0401, //1002070001800001041003B2
  COMMAND_ENDDEVICE_ONE_UPDATE = 0x0402, //1002070001800002041003B3
  COMMAND_VERSION = 0x0500, //100207000101000005100333
  COMMAND_RELAY_OFF = 0x0600,
  COMMAND_RELAY_ON = 0x0601,
  COMMAND_MOVING_AVERAGE_OFF = 0x0700,
  COMMAND_MOVING_AVERAGE_ON_3 = 0x0703,
  COMMAND_MOVING_AVERAGE_ON_4 = 0x0704
} commandExternalType_t;


/**
\brief C�dicos de notificaci�n para los comandos
\details
Comando                                 | Valor  | Uso
----------------------------------------|--------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------
NOTIFICATION_CODE_OK                    |  0x00  | Indica que el comando ha llegado a su destino.
NOTIFICATION_CODE_NO_ACK                |  0x01  | Indica que no se ha recibido el ACK de la direcci�n destino del comando.
NOTIFICATION_CODE_NO_SPACE              |  0x02  | Indica que no hay espacio suficiente en la cola de mensajes.
NOTIFICATION_CODE_INVALID_ADDRESS       |  0x03  | Indica que la direcci�n destino no es v�lida. Nota: no se puede enviar un mensaje a un END-DEVICE en un firmware con localizacion, no hay ning�n mecanismo implementado para garantizar que el END-DEVICE recibe el mensaje.
*/
typedef enum notificationCode_t
{
  NOTIFICATION_CODE_OK     = 0x00,
  NOTIFICATION_CODE_NO_ACK = 0x01,  
  NOTIFICATION_CODE_NO_SPACE = 0x02,
  NOTIFICATION_CODE_INVALID_ADDRESS = 0x03,
//  NOTIFICATION_CODE_MAX_LENGTH_EXCEEDED = 0x04
} notificationCode_t;

/**\brief Estructura de datos para la gesti�n de comandos.
   \see commandInternalType_t
*/
typedef struct command_t
{
  /** \brief estado. A 1 cuando se almacena, a 0 cuando se borra. */
  uint8_t    state;
  /** \brief direcci�n de destino de comando. */
  uint16_t   dstAddr;
  /** \brief comando.  */
  commandInternalType_t	 command;
} command_t;

/*VARIABLES*/
//variables de red
/** \brief	Cola de comandos a nivel de red. */
NWK_DataReq_t nwkDataReqArray[MAX_LENGTH_COMMAND_QUEUE];

/*PROTOTIPOS*/
/**************************************************************************//**
\brief		Env�o de datos por la UART.
\param[in]	data		  Puntero a trama.
\param[in]	size		  Tama�o de la trama.
\param[in]	routerAddress Direcci�n del router que reenv�a la trama. Si no lo hubiera se manda 0xFFFF
\param[in]	type		  Tipo de mensaje.
******************************************************************************/
void sendUARTMessage(uint8_t *data, uint8_t size, uint16_t routerAddress, typeMessage_t type);

/**************************************************************************//**
\brief		Saca el primer comando que hay para el nodo con direcci�n pasada por par�metro.
\see		commandInternalType_t  
\param[in]	destinationAddress Direcci�n de destino.
\retval		0			si no hay comando para esa direcci�n de destino.
\retval		otro_num	el propio comando para esa direcci�n.
******************************************************************************/
commandInternalType_t commandGet(uint16_t destinationAddress);

/**************************************************************************//**
\brief		Almacena en la cola el comando y la direccion pasado por par�metro.
\see		commandInternalType_t  
\param[in]	destinationAddress	direcci�n de destino.
\param[in]	command				comando.
\retval		false	no se ha podido almacenar el comando.
\retval		true	en otro caso.
******************************************************************************/
bool commandPut(commandInternalType_t command, uint16_t destinationAddress);

/**************************************************************************//**
\brief		Elimina el primer comando con la direcci�n destino pasada por par�metro.
\see		commandInternalType_t  
\param[in]	destinationAddress	direcci�n de destino.
******************************************************************************/
void commandRemove(uint16_t destinationAddress);

/**************************************************************************//**
\brief		Devuelve si la cola de comandos est� vac�a o no.
\see		  commandInternalType_t
\retval		true	si la cola est� vac�a.
\retval		false	en otro caso.
******************************************************************************/
bool commandIsEmpty(void);

/**************************************************************************//**
\brief		Devuelve el �ndice del primer comando cuyo destino es un router o un nodo con rel�.
\see		  commandInternalType_t
\retval		-1 si no hay ning�n comando cuyo destino sea un router o un nodo con rel�.
\retval		otro_num  que es el �ndice.
******************************************************************************/
int8_t commandGetFirstNonEndDeviceIndex(void);

/**************************************************************************//**
\brief		  Devuelve la direcci�n de destino del comando que hay en la posici�n de la cola pasado por par�metro.
\see		    commandInternalType_t
\param[in]  index el �ndice de la cola.
\retval		  0xFFFF si el �ndice pasado por par�metro no es v�lido.
\retval		  otro_num  que es la direcci�n.
******************************************************************************/
uint16_t commandGetAddressByIndex(int8_t index);

/**************************************************************************//**
\brief		  Devuelve el comando interno que hay en la posici�n de la cola pasado por par�metro.
\see		    commandInternalType_t
\param[in]  index el �ndice de la cola.
\retval		  0xFF si el �ndice pasado por par�metro no es v�lido.
\retval		  otro_num  que es la direcci�n.
******************************************************************************/
commandInternalType_t commandGetCommandByIndex(int8_t index);

/**************************************************************************//**
\brief		  Cambia el valor del estado que hay en la posici�n de la cola pasado por par�metro.
\see		    commandInternalType_t
\param[in]  index el �ndice de la cola.
\param[in]  state el nuevo valor del estado.
\retval		  true  si el valor ha sido cambiado.
\retval		  false en caso contrario.
******************************************************************************/
bool commandSetState(int8_t index, uint8_t state);

/**************************************************************************//**
\brief	M�quina de estados para la recepci�n de comandos por la UART.
\see		commandExternalType_t  
\param[in]	byte	byte recibido por la UART.
******************************************************************************/
void uartStateMachine(uint8_t byte);

/**************************************************************************//**
\brief		Funci�n que ejecuta un comando. A implementar en ServiNet.c.
\param[in]	command	comando a ejecutar.
******************************************************************************/
void appCommandExecute(uint16_t command);

/*****************************************************************************//**
\brief      Busca en la lista de comandos el primero cuyo destino sea un router o un nodo con rel� y lo env�a. A implementar en ServiNet.c.
*********************************************************************************/
void appCommandSearchAndSend(void);

/**************************************************************************//**
\brief Funci�n que traduce un comando externo a uno interno.

\param[in]
	command comando externo
\return  el comando interno traducido
******************************************************************************/
commandInternalType_t translateCommandToInternal(commandExternalType_t command);

/**************************************************************************//**
\brief Funci�n que traduce un comando interno a uno externo.

\param[in]
	command comando interno
\return  el comando externo traducido
******************************************************************************/
commandExternalType_t translateCommandToExternal(commandInternalType_t command);

/**************************************************************************//**
\brief Funci�n que env�a una notificaci�n UART de un comando.

\param[in] command  comando interno
\param[in] address  direcci�n de destino del comando
\param[in] notiticationCode indica el c�digo de notificaci�n.
\see		notificationCode_t
******************************************************************************/
void appUartSendNotification(commandExternalType_t command, uint16_t address, notificationCode_t notiticationCode);

#endif //_UART_H_