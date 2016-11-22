/**************************************************************************//**
\file  uart.c

\brief Comunicación UART en ambos sentidos: envío de tramas y 
	   recepción de comandos
******************************************************************************/
#include "uart.h"

/*VARIABLES*/

//variables de comandos
/** \brief	Parte alta del comando. */
uint8_t commandH;

/** \brief	Cola de comandos a nivel de aplicación. */
command_t commandQueue[MAX_LENGTH_COMMAND_QUEUE];
/** \brief	Índice de commandQueue. */
uint8_t commandQueueIndex;

//variables control uart
/** \brief	Estado actual de máquina de estados de recepción de comandos vía UART. */
AppUartState_t uartState; // = APP_UART_STATE_START_PRE;
/** \brief	Número de destinos del comando. */
uint8_t uartNumDestination;
/** \brief	Donde se almacenan los destinos del comando. */
uint16_t uartDestinations[MAX_LENGTH_COMMAND_QUEUE];
/** \brief	Búffer para la recepción de comandos. */
uint8_t uartBuffer[MAX_UART_BUFFER_SIZE];
/** \brief  Índice dentro del búffer de recepción de comandos*/
uint8_t indexUartBuffer;
/** \brief	Longitud de la trama recibida por UART. */
uint16_t uartSize;
/* \todo	Borrar, no usado. */
//uint8_t uartPtr;
/** \brief	Suma de los bytes recibidos por UART. */
uint16_t uartCsum;
/** \brief	Checksum recibido. */
uint8_t uartCsumByte;
/** \brief	Indica la posición para la recepción de destinos de un comando. */
uint8_t uartPosition;
/** \brief	Indica si el byte anterior recibido es el byte de escape. */
bool uartEscapePrevious;


void appUartSendNotification(commandExternalType_t command, uint16_t address, notificationCode_t notificationCode)
{

  uint8_t data[3] = {(command & 0x00FF), ((command >> 8)  & 0xFF),   //comando
                     notificationCode                               //código de notificación
  };
  sendUARTMessage(data, sizeof(data), address, MESSAGE_TYPE_COMMAND);
}

//Vemos si la trama es de un router o de un end-device. Si es de un end-device no se manda por la UART.
//Hay que mandar también la dirección de origen de la trama ind->srcAddr para construir la trama que se mandará por la UART
//Con la dirección que va en el payload se elige le tipo de trama que se enviará.
//       10 02 ll ll xx aa aa cc cc rs sn co 10 03 cs -> de un end-device
//       10 02 ll ll xx ay ay ax ax rs sn 10 03 cs    -> de un router
// 			co -> byte de control
// 			rs -> rssi
// 			sn -> número de secuencia
// 			aa -> dirección del ancla
// 			cc -> dirección del casco
// 			xx -> tipo de trama
// 			t# -> tipo de sensor
// 			v# -> valor del sensor
// 			cs -> checksum
void sendUARTMessage(uint8_t *data, uint8_t size, uint16_t routerAddress, typeMessage_t type)
{
  uint8_t checksum = 0;
  //uint8_t type = 0;
  //uint8_t typeBroadcastAddress = getAddrType(broadcastAddress);
  uint16_t tam = size;

  //inicio de trama
  HAL_UartWriteByte(0x10);
  HAL_UartWriteByte(0x02);

  //enviamos el tamaño
  //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  //if (typeBroadcastAddress == ADDR_TYPE_GLOBAL_BROADCAST) //del coordinador
  /*if (type == MESSAGE_TYPE_SENSORS)
    tam += 3; //2 de longitud + 1 del tipo
  else if (type ==MESSAGE_TYPE_ROUTER_LOCALIZATION || type == MESSAGE_TYPE_ENDDEVICE_LOCALIZATION || type == MESSAGE_TYPE_VERSION)*/
  
  tam += 5; //2 de la longitud + 1 del tipo + 2 de la dirección del router
  if ((tam & 0xFF) == 0x10)
  {
    HAL_UartWriteByte(0x10);
    checksum += 0x10;
  }
  HAL_UartWriteByte(tam & 0xFF);
  checksum += tam & 0xFF;
  if ((tam >> 8) == 0x10)
  {
    HAL_UartWriteByte(0x10);
    checksum += 0x10;
  }
  HAL_UartWriteByte(tam >> 8);
  checksum += tam >> 8;

  //enviamos el tipo
  /*if (typeBroadcastAddress == ADDR_TYPE_GLOBAL_BROADCAST) //del coordinador
    type = MESSAGE_TYPE_COORDINATOR;
  //DONE: función auxiliar para saber si una dirección es coordinador, router, end-device o broadcast
  else if (typeBroadcastAddress == ADDR_TYPE_ROUTER)
    type = MESSAGE_TYPE_ROUTER_LOCALIZATION;
  else
    type = MESSAGE_TYPE_ENDDEVICE_LOCALIZATION;*/
  
  if ((type) == 0x10)
  {
    HAL_UartWriteByte(0x10);
    checksum += 0x10;
  }
  HAL_UartWriteByte(type);
  checksum += type;

  //enviamos la direccion del router si la hay
  if (getAddrType(routerAddress) != ADDR_TYPE_GLOBAL_BROADCAST)
  {
    if ((routerAddress & 0xFF) == 0x10)
    {
      HAL_UartWriteByte(0x10);
      checksum += 0x10;
    }
    HAL_UartWriteByte(routerAddress & 0xFF);
    checksum += routerAddress & 0xFF;
    if ((routerAddress >> 8) == 0x10)
    {
      HAL_UartWriteByte(0x10);
      checksum += 0x10;
    }
    HAL_UartWriteByte(routerAddress >> 8);
    checksum += routerAddress >> 8;
  }

  //enviamos los datos de la trama
  for (uint8_t i = 0; i < size; i++)
  {
    if (data[i] == 0x10)
    {
      HAL_UartWriteByte(0x10);
      checksum += 0x10;
    }
    HAL_UartWriteByte(data[i]);
    checksum += data[i];
  }

  //enviamos fin de trama
  HAL_UartWriteByte(0x10);
  HAL_UartWriteByte(0x03);

  //enviamos de checksum
  checksum += 0x10 + 0x02 + 0x10 + 0x03;
  HAL_UartWriteByte(checksum);
}


commandInternalType_t commandGet(uint16_t destinationAddress)
{
  for (uint8_t i = 0; i < commandQueueIndex; i++)
  {
    if (commandQueue[i].dstAddr == destinationAddress)
    {
      uint16_t command = commandQueue[i].command;
//marcamos como enviado
      commandQueue[i].state = 1;
      return command;
    }
  }
  return 0;
}

int8_t commandGetFirstNonEndDeviceIndex(void)
{
  for (int8_t i = 0; i < commandQueueIndex; i++)
  {
    typeAddress_t typeDstAddr = getAddrType(commandQueue[i].dstAddr);
    if((typeDstAddr == ADDR_TYPE_ROUTER || typeDstAddr == ADDR_TYPE_RELAY) /* && (commandQueue[i].state == 1)*/)
    {
      return i;
    }
  }
  return -1;
}


uint16_t commandGetAddressByIndex(int8_t index)
{
  if(index < commandQueueIndex)
  {
    return commandQueue[index].dstAddr;
  }
  return 0xFFFF;
}


commandInternalType_t commandGetCommandByIndex(int8_t index)
{
  if(index < commandQueueIndex)
  {
    return commandQueue[index].command;
  }
  return 0xFF;
}


bool commandSetState(int8_t index, uint8_t state)
{
  if (index < commandQueueIndex)
  {
    commandQueue[index].state = state;
    return true;
  }
  return false;
}


bool commandPut(commandInternalType_t command, uint16_t destinationAddress)
{
  if (commandQueueIndex == MAX_LENGTH_COMMAND_QUEUE)
    return false;
  commandQueue[commandQueueIndex].command = command;
  commandQueue[commandQueueIndex].dstAddr = destinationAddress;
  commandQueueIndex++;
  return true;
}


void commandRemove(uint16_t destinationAddress)
{
  for (uint8_t i = 0; i<commandQueueIndex; i++)
  {
    if ((commandQueue[i].dstAddr == destinationAddress) && (commandQueue[i].state == 1))
    {
//quitamos de la lista este comando
      for (uint8_t j = i; j<commandQueueIndex-1; j++)
      {
        if (j+1 < MAX_LENGTH_COMMAND_QUEUE)
          commandQueue[j] = commandQueue[j+1];
      }
      commandQueue[commandQueueIndex-1].state = 0;
      commandQueue[commandQueueIndex-1].command = 0;
      commandQueue[commandQueueIndex-1].dstAddr = 0;
      commandQueueIndex--;
      return;
    }
  }
}

bool commandIsEmpty(void)
{
  return (commandQueueIndex == 0);
}

void uartStateMachine(uint8_t byte)
{
  switch (uartState)
  {
  case APP_UART_STATE_START_PRE:
  {
    uartSize = 0;
    //uartPtr = 0;
    uartCsum = 0;
    uartCsumByte = 0;
    uartCsum += byte;
    if (0x10 == byte)
      uartState = APP_UART_STATE_START_POST;
  }
  break;

  case APP_UART_STATE_START_POST:
  {
    uartCsum += byte;
    if (0x02 == byte)
      uartState = APP_UART_STATE_READ_SIZE;
    else
      uartState = APP_UART_STATE_START_PRE;
    uartPosition = 0;
    uartEscapePrevious = false;
  }
  break;

  case APP_UART_STATE_READ_SIZE: //2bytes
  {
    if(uartEscapePrevious == false && byte == 0x10)
    {
      uartEscapePrevious = true;
    }
    else
    {
      /**/
      if (uartEscapePrevious == true && byte != 0x10)
      {
        uartState = APP_UART_STATE_START_PRE;
      }        
      /**/
      else
      {
        uartEscapePrevious = false;
        if (uartPosition == 0)
        {
          uartSize = byte;
          uartPosition++;
        }
        else
        {
          uartSize |= byte << 8;
          if (uartSize < 7)
            uartState = APP_UART_STATE_START_PRE;
          uartSize -= 2; //2 es el tamaño de la longitud en la trama
          uartState = APP_UART_STATE_READ_NUM_DESTINATION;
          uartPosition = 0;
        }
      }      
    }
    uartCsum += byte;
  }
  break;

  case APP_UART_STATE_READ_NUM_DESTINATION: //1bytes
  {
    if(uartEscapePrevious == false && byte == 0x10)
      uartEscapePrevious = true;
    else
    {
      /**/
      if (uartEscapePrevious == true && byte != 0x10)
      {
        uartState = APP_UART_STATE_START_PRE;
      }        
      /**/
      else
      {
        uartEscapePrevious = false;

        uartNumDestination = byte;
        uartPosition++;
        uartState = APP_UART_STATE_READ_DESTINATION;
        uartPosition = 0;
      }
    }
    uartCsum += byte;
  }
  break;

  case APP_UART_STATE_READ_DESTINATION:
  {
    if(uartEscapePrevious == false && byte == 0x10)
      uartEscapePrevious = true;
    else
    {
      /**/
      if (uartEscapePrevious == true && byte != 0x10)
      {
        uartState = APP_UART_STATE_START_PRE;
      }        
      /**/
      else
      {
        uartEscapePrevious = false;
        uint8_t position = uartPosition>>1;
        if (position< MAX_LENGTH_COMMAND_QUEUE)
        {
          if ((uartPosition&0x01) == 0)
          {
            uartDestinations[position] = byte;
            uartPosition++;
          }
          else
          {
            uartDestinations[position] |= byte<<8;
            uartPosition++;
            if ((uartNumDestination<<1)  == uartPosition)
            {
              uartState = APP_UART_STATE_READ_DATA;
              //appUartPosition = 0;
              indexUartBuffer = 0;
            }
          }
        }
        else
        {
          ;//enviar datos
        }    
      }      
    }
    uartCsum += byte;
  }
  break;

  case APP_UART_STATE_READ_DATA:
  {
    if(uartEscapePrevious == false && byte == 0x10)
      uartEscapePrevious = true;
    else
    {
      /**/
      if (uartEscapePrevious == true && byte != 0x10)
      {
        uartState = APP_UART_STATE_START_PRE;
      }
      else
      {       
        /**/
        uartEscapePrevious = false;
        if(indexUartBuffer < MAX_UART_BUFFER_SIZE)
        { 
          uartBuffer[indexUartBuffer] = byte;
          indexUartBuffer++;  
        }
                
        uartPosition++;
        if (indexUartBuffer >= MAX_UART_BUFFER_SIZE)
          uartState = APP_UART_STATE_END_PRE;
      }        
    }
    uartCsum += byte;
  }
  break;

  case APP_UART_STATE_END_PRE:
  {
    uartCsum += byte;
    if (0x10 == byte)
      uartState = APP_UART_STATE_END_POST;
    else
      //¿escribimos un error en el puerto serie?
      uartState = APP_UART_STATE_START_PRE;
  }
  break;

  case APP_UART_STATE_END_POST:
  {
    uartCsum += byte;
    if (0x03 == byte)
      uartState = APP_UART_STATE_READ_CHECKSUM;
    else
    {
      //¿escribimos un error en el puerto serie?
      uartState = APP_UART_STATE_START_PRE;
    }
  }
  break;

  case APP_UART_STATE_READ_CHECKSUM:
  {
    if (byte == (uartCsum & 0xFF))  //envio mensaje
    {
      uint16_t command = ((uartBuffer[1]<<8) | (uartBuffer[0]));
	    commandH =  (uartBuffer[1]);
      //DONE: sistema de mensajes para controlar el relé
      /*if (command == COMMAND_VERSION || command == COMMAND_RELAY_ON || command == COMMAND_RELAY_OFF || 
	      command == COMMAND_ENDDEVICE_NO_UPDATE || command == COMMAND_ENDDEVICE_UPDATE || 
		  command == COMMAND_ENDDEVICE_ONE_UPDATE || command == COMMAND_CONFIRM_ALARM)
      {*/
        //appMsg.data_field.alarm.data = 1;
        for (uint8_t i=0; i<uartNumDestination; i++)
        {
          if (uartDestinations[i] == appAddr) //es el propio coordinador
          {
            appCommandExecute(command);
            appUartSendNotification(command, appAddr, NOTIFICATION_CODE_OK);
          }
          else if (getAddrType(uartDestinations[i]) != ADDR_TYPE_ENDDEVICE  && (((uartDestinations[i] & 0xFF00) >> 8) == appAddr || getAddrType(uartDestinations[i]) == ADDR_TYPE_NO_LOCALIZATION_ROUTER))//es un router o un nodo especial, por lo que le podemos mandar directamente el mensaje si pertenece a la misma subred que el coordinador
          {            
            uint8_t newCommand = 0;
            switch(command)
            {
              case COMMAND_MOVING_AVERAGE_OFF:
              {
                newCommand = COMMAND_MOVING_AVERAGE_OFF_INTERNAL;
              }
              break;
              case COMMAND_MOVING_AVERAGE_ON_3:
              {
                newCommand = COMMAND_MOVING_AVERAGE_ON_3_INTERNAL;
              }
              break;
              case COMMAND_MOVING_AVERAGE_ON_4:
              {
                newCommand = COMMAND_MOVING_AVERAGE_ON_4_INTERNAL;
              }
              break;
              case COMMAND_RELAY_ON:
              {
                newCommand = COMMAND_RELAY_ON_INTERNAL;
              }
              break;
              case COMMAND_RELAY_OFF:
              {
                newCommand = COMMAND_RELAY_OFF_INTERNAL;
              }
              default:
              {
                newCommand = commandH;
              }
              break;
            }
            /*
			      nwkDataReqArray[i].data = (uint8_t *)&newCommand;
			      nwkDataReqArray[i].size = sizeof(newCommand);
            nwkDataReqArray[i].dstAddr = uartDestinations[i];
            HAL_LedOn(LED_DATA);
            NWK_DataReq(&nwkDataReqArray[i]);
            */
            if(!commandPut(newCommand, uartDestinations[i]))
            {
              //appUartSendNotification(COMMAND_NO_SPACE); //escribimos un error en el puerto serie
              appUartSendNotification(command, uartDestinations[i], NOTIFICATION_CODE_NO_SPACE);
            }
            else
            {
              appCommandSearchAndSend();
            }          
          }

          else if (getAddrType(uartDestinations[i]) == ADDR_TYPE_ENDDEVICE) //es un end_device, hay que guardar el mensaje en la estructura y enviarlo cuando sea conveniente
          {
#if LOCALIZATION
            //no se puede enviar un mensaje a un END-DEVICE bajo localizacion, no hay ningún mecanismo implementado para garantizar que el END-DEVICE recibe el mensaje
            //enviar error
            appUartSendNotification(command, uartDestinations[i], NOTIFICATION_CODE_INVALID_ADDRESS);
#else
            if(!commandPut(command, uartDestinations[i]))
              //appUartSendNotification(COMMAND_NO_SPACE); //escribimos un error en el puerto serie
              appUartSendNotification(command, uartDestinations[i], NOTIFICATION_CODE_NO_SPACE);
#endif
          }
        }
      }
      uartState = APP_UART_STATE_START_PRE;
    /*}


    else
    {
      uartState = APP_UART_STATE_START_PRE;
      //¿escribimos un error en el puerto serie?
    }*/
  }
  break;
  }
}


commandInternalType_t translateCommandToInternal(commandExternalType_t command)
{
  //commandInternalType_t newCommand = 0;
  switch(command)
  {
    case COMMAND_MOVING_AVERAGE_OFF:
    {
      return COMMAND_MOVING_AVERAGE_OFF_INTERNAL;
    }
    break;
    case COMMAND_MOVING_AVERAGE_ON_3:
    {
      return COMMAND_MOVING_AVERAGE_ON_3_INTERNAL;
    }
    break;
    case COMMAND_MOVING_AVERAGE_ON_4:
    {
      return COMMAND_MOVING_AVERAGE_ON_4_INTERNAL;
    }
    break;
    case COMMAND_RELAY_ON:
    {
      return COMMAND_RELAY_ON_INTERNAL;
    }
    break;
    case COMMAND_RELAY_OFF:
    {
      return COMMAND_RELAY_OFF_INTERNAL;
    }
    default:
    {
      return 0;
    }
    break;
  }
}


commandExternalType_t translateCommandToExternal(commandInternalType_t command)
{
  switch(command)
  {
    case COMMAND_VERSION_INTERNAL:
    {
      return COMMAND_VERSION;
    }break;
    
    case COMMAND_RELAY_OFF_INTERNAL:
    {
      return COMMAND_RELAY_OFF;
    }break;
    
    case COMMAND_RELAY_ON_INTERNAL:
    {
      return COMMAND_RELAY_ON;
    }break;
    
    case COMMAND_MOVING_AVERAGE_OFF_INTERNAL:
    {
      return COMMAND_MOVING_AVERAGE_OFF;
    }break;
    
    case COMMAND_MOVING_AVERAGE_ON_3_INTERNAL:
    {
      return COMMAND_MOVING_AVERAGE_ON_3;
    }break;
    
    case COMMAND_MOVING_AVERAGE_ON_4_INTERNAL:
    {
      return COMMAND_MOVING_AVERAGE_ON_4;
    }break;
    default:
    {
      return 0;
    }
  }
}