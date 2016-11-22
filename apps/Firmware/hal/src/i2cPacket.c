/**************************************************************************//**
\file  i2cPacket.c

\brief Implementación de la interfaz i2c.
******************************************************************************/

/******************************************************************************
                   Includes section
******************************************************************************/

#include "i2cPacket.h"

/******************************************************************************
                   Define(s) section
******************************************************************************/
/* states of the i2c transaction */
/** \brief Cerrar i2c. */
#define I2C_CLOSE                    0
/** \brief i2c idle. */
#define I2C_IDLE                     1
/** \brief Escribir dirección interna para operación de escritura. */
#define I2C_WRITE_IADDR_WRITE_DATA   2
/** \brief Escribir dirección interna para operación de lectura. */
#define I2C_WRITE_IADDR_READ_DATA    3
/** \brief Escribir datos. */
#define I2C_WRITE_DATA               4
/** \brief Leer datos. */
#define I2C_READ_DATA                5
/** \brief Transacción finalizada con éxito. */
#define I2C_TRANSAC_SUCCESS          6
/** \brief Fallo en la transacción. */
#define I2C_TRANSAC_FAIL             7

/******************************************************************************
                   Global variables section
******************************************************************************/
/** \brief descriptor i2c. */
static HAL_I2cDescriptor_t *halI2cDesc;

/******************************************************************************
                   Implementations section
******************************************************************************/
void halI2cBusReset(void)
{
  halI2cDesc->service.state = I2C_TRANSAC_FAIL;
  halResetI2c();
  halSig2WireSerialHandler();
}

int HAL_OpenI2cPacket(HAL_I2cDescriptor_t *descriptor)
{
  if (NULL == descriptor)
    return -1;
  if (halI2cDesc)
    return -1;
  halI2cDesc = descriptor;
  halInitI2c(halI2cDesc->clockRate);
  halI2cDesc->service.state = I2C_IDLE;
  halI2cDesc->service.index = 0;
  return 0;
}

int HAL_CloseI2cPacket(HAL_I2cDescriptor_t *descriptor)
{
  if (!descriptor)
    return -1;
  if (descriptor != halI2cDesc)
    return -1;
  halI2cDesc = NULL;
  if (I2C_CLOSE != descriptor->service.state)
  {
    descriptor->service.state = I2C_CLOSE;
    return 0;
  }
  return -1;
}

int HAL_WriteI2cPacket(HAL_I2cDescriptor_t *descriptor)
{
  HalI2cPacketService_t *service;
  if (!descriptor)
    return -1;
  if (halI2cDesc != descriptor)
    return -1;
  if (!halI2cDesc->data && !halI2cDesc->length)
    return -1;
  service = &halI2cDesc->service;
  if (I2C_IDLE != service->state)
    return -1;
  service->index = halI2cDesc->lengthAddr;
  if (HAL_NO_INTERNAL_ADDRESS == halI2cDesc->lengthAddr)
    service->state = I2C_WRITE_DATA;
  else
    service->state = I2C_WRITE_IADDR_WRITE_DATA;
  halSendStartI2c();
  return 0;
}

int HAL_ReadI2cPacket(HAL_I2cDescriptor_t *descriptor)
{
  HalI2cPacketService_t *service;
  if (!descriptor)
    return -1;
  if (halI2cDesc != descriptor)
    return -1;
  if (!halI2cDesc->data && !halI2cDesc->length)
    return -1;
  service = &halI2cDesc->service;
  if (I2C_IDLE != service->state)
    return -1;
  service->index = halI2cDesc->lengthAddr;
  if (HAL_NO_INTERNAL_ADDRESS == halI2cDesc->lengthAddr)
    service->state = I2C_READ_DATA;
  else
    service->state = I2C_WRITE_IADDR_READ_DATA;
  halSendStartI2c();
  return 0;
}


void halSendStartDoneI2c(void)
{
  HalI2cPacketService_t *service = &halI2cDesc->service;
  if ((I2C_WRITE_IADDR_WRITE_DATA == service->state) ||
      (I2C_WRITE_IADDR_READ_DATA == service->state) ||
      (I2C_WRITE_DATA == service->state))
  {
    halWriteI2c(((halI2cDesc->id << 1) + 0));
  }
  else if (I2C_READ_DATA == service->state)
  {
    halWriteI2c(((halI2cDesc->id << 1) + 1));
  }
  else
  { // abnormal
    halI2cBusReset();
  }
}

/***************************************************************************//**
\brief Envía datos al bus i2c. Si es el último byte manda la condición 
de parada.
******************************************************************************/
void halWriteData(void)
{
  HalI2cPacketService_t *service = &halI2cDesc->service;

  if (service->index < halI2cDesc->length)
  {
    halWriteI2c(halI2cDesc->data[service->index++]);
  }
  else
  {
    service->state = I2C_TRANSAC_SUCCESS;
    halSendStopI2c();
    halSig2WireSerialHandler();
  }
}

/**************************************************************************//**
\brief Envía la dirección interna del dispositivo al bus i2c. Cambia el estado 
del i2c si la dirección se ha enviado.
******************************************************************************/
void halWriteInternalAddress(void)
{
  uint8_t data;
  HalI2cPacketService_t *service = &halI2cDesc->service;
  data = (uint8_t)(halI2cDesc->internalAddr >> --service->index * 8);
  halWriteI2c(data);
  if (0 == service->index)
  {
    if (I2C_WRITE_IADDR_WRITE_DATA == service->state)
      service->state = I2C_WRITE_DATA;
    else
      service->state = I2C_READ_DATA;
  }
}

void halWriteDoneI2c(void)
{
  HalI2cPacketService_t *service = &halI2cDesc->service;
  if (I2C_WRITE_DATA == service->state)
  {
    halWriteData();
  }
  else if ((I2C_WRITE_IADDR_WRITE_DATA == service->state) || (I2C_WRITE_IADDR_READ_DATA == service->state))
  {
    halWriteInternalAddress();
  }
  else if (I2C_READ_DATA == service->state)
  {
    halSendStartI2c();
  }
  else
  { // abnormal
    halI2cBusReset();
  }
}


void halMasterReadWriteAddressAckI2c(void)
{
  HalI2cPacketService_t *service = &halI2cDesc->service;
  if (I2C_READ_DATA == service->state)
  {
    if (1 == halI2cDesc->length)
      halReadI2c(false); // send nack
    else
      halReadI2c(true);  // send ack
  }
  else
  { // abnormal
    halI2cBusReset();
  }
}


void halReadDoneI2c(uint8_t data)
{
  HalI2cPacketService_t *service = &halI2cDesc->service;
  if (I2C_READ_DATA == service->state)
  {
    halI2cDesc->data[service->index++] = data;
    if (service->index < (halI2cDesc->length - 1))
      halReadI2c(true);  // send ACK
    else
      halReadI2c(false); // send NACK
  }
  else
  { // abnormal
    halI2cBusReset();
  }
}


void halReadLastByteDoneI2c(uint8_t data)
{
  HalI2cPacketService_t *service = &halI2cDesc->service;

  if (I2C_READ_DATA == service->state)
  {
    halI2cDesc->data[service->index++] = data;
    service->state = I2C_TRANSAC_SUCCESS;
    halSendStopI2c();    
    halSig2WireSerialHandler();
  }
  else
  { // abnormal
    halI2cBusReset();
  }
}

void halSig2WireSerialHandler(void)
{
  HalI2cPacketService_t *service = &halI2cDesc->service;
  if (halI2cDesc->f)
  {
    if (I2C_TRANSAC_SUCCESS == service->state)
    {
      halWaitEndOfStopStation();     
	  service->state = I2C_IDLE;
	  halI2cDesc->f(1); 
    }
	else
		halI2cDesc->f(0);        
  }
  else
  {
    service->state = I2C_IDLE;
  }
}



/*halTwi.c*/
/******************************************************************************
                   Implementations section
******************************************************************************/
void halInitI2c(I2cClockRate_t rate)
{
  TWCR = 0x00;
  TWSR = HAL_I2C_PRESCALER; // prescaler
  // Set bit rate
  TWBR = rate;
}

/**************************************************************************//**
\brief Interrupción.
******************************************************************************/
ISR(TWI_vect)
{
  switch (TWSR & 0xF8)
  {
    case TWS_START:
    case TWS_RSTART:
      halSendStartDoneI2c();
      break;

    case TWS_MT_SLA_ACK:
    case TWS_MT_DATA_ACK:
      halWriteDoneI2c();
      break;

    case TWS_BUSERROR:
    case TWS_MT_SLA_NACK:
    case TWS_MT_DATA_NACK:
    case TWS_MR_SLA_NACK:
      halI2cBusReset();
      break;

    case TWS_MR_SLA_ACK:
      halMasterReadWriteAddressAckI2c();
      break;

    case TWS_MR_DATA_ACK:
      halReadDoneI2c(halReadByteI2c());
      break;

    case TWS_MR_DATA_NACK:
      halReadLastByteDoneI2c(halReadByteI2c());
      break;

    default:
      break;
  }
}

