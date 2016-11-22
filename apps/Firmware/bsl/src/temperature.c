/**************************************************************************//**
\file  temperature.c

\brief Driver del sensor de temperatura LM73.
******************************************************************************/

#include "temperature.h"

/**/
// device address on i2c bus
#define LM73_DEVICE_ADDRESS  (0x92 >> 1)

// device registers internal address
#define LM73_DATA_REGISTER 0x00
#define LM73_CONFIGURATION_REGISTER 0x01
#define LM73_UPPER_LIMIT_REGISTER 0x02
#define LM73_LOWER_LIMIT_REGISTER 0x03
#define LM73_CONTROL_STATUS_REGISTER 0x04
#define LM73_IDENTIFICATION_REGISTER 0x07

/******************************************************************************
                   Types section
******************************************************************************/
// states
typedef enum
{
  IDLE,      // idle
  DATA       // performs request
} Lm73States_t;

typedef struct
{
  int16_t lm73Data;           // Contains the result of sampling
  bool lm73Result;            // Result of operation. true - there are no errors, false - in other case.
  bool(* lm73Callback)(void); // callback
} Lm73Control_t;

/******************************************************************************
                          Prototypes section
******************************************************************************/
static void lm73I2cPacketReadDone(bool result);

/******************************************************************************
                   Global variables section
******************************************************************************/
static Lm73States_t lm73State = IDLE; // Monitors current state
static Lm73Control_t lm73Control;
static HAL_I2cDescriptor_t desc =
{
  .tty = TWI_CHANNEL_0,
  .clockRate = I2C_CLOCK_RATE_62,
  .f = lm73I2cPacketReadDone,
  .id = LM73_DEVICE_ADDRESS,
  .data = (uint8_t*)(&lm73Control.lm73Data),
  .length = 2,
  .lengthAddr = HAL_NO_INTERNAL_ADDRESS
};

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Abre el componente.
\return
  BC_SUCCESS - El componente está listo para ser usado. \n
  BC_FAIL - En cualquier otro caso.
******************************************************************************/
result_t openLm73(void)
{
  if (IDLE == lm73State)
    return BC_SUCCESS;
  return BC_FAIL;
}

/**************************************************************************//**
\brief  Cierra el componente.
\return
  BC_SUCCESS - El componente se ha cerrado. \n
  BC_FAIL - En cualquier otro caso.
******************************************************************************/
result_t closeLm73(void)
{
  if (IDLE == lm73State)
    return BC_SUCCESS;
  return BC_FAIL;
}

/**************************************************************************//**
\brief Función callback que se llama cuando un paquete se lee. Realiza la conversión del dato.

\param[in]
  result - el resultado de la operación
******************************************************************************/
static void lm73I2cPacketReadDone(bool result)
{
  int16_t i;

  lm73Control.lm73Result = result; // stores the result of operation
  if (false == lm73Control.lm73Result)
  { // there were some errors on the i2c interface
    lm73Control.lm73Data = 0;
    //bspPostTask(BSP_TEMPERATURE);
    bspLm73Handler();
    return;
  }

  if (DATA == lm73State)
  {
    i = (uint8_t)lm73Control.lm73Data;
    i <<= 8;
    i |= ((lm73Control.lm73Data >> 8) & 0x00FF);
    lm73Control.lm73Data = (i >> 7);    
  }
  bspLm73Handler();
}

/**************************************************************************//**
\brief  Lee el dato del sensor LM73.

\param[in]
    f - función de callback.

\return
  BC_FAIL - la anterior petición no se ha completado, 
			la dirección de callback es 0, la interfaz i2c está ocupada, 
			hay un error en la interfaz i2c. \n
  BC_SUCCESS - En otro caso.
******************************************************************************/
result_t readLm73Data(bool (*f)(void))
{
  if (IDLE != lm73State)
    return BC_FAIL;

  if (!f)
    return BC_FAIL;

  if (-1 == HAL_OpenI2cPacket(&desc))
    return BC_FAIL;
  lm73State = DATA;
  lm73Control.lm73Callback = f;

  if (-1 == HAL_ReadI2cPacket(&desc))
  {
    lm73State = IDLE;
    HAL_CloseI2cPacket(&desc);
    return BC_FAIL;
  }

  return BC_SUCCESS;
}

/**************************************************************************//**
\brief Cierra la comunicación I2C y cierra el driver.
******************************************************************************/
void bspLm73Handler(void)
{
  HAL_CloseI2cPacket(&desc); // free
  lm73State = IDLE;
  lm73Control.lm73Callback();
}

/******************************************************************************
\brief Devuelve la última lectura de la temperatura.
\return
	El valor de la temperatura
*******************************************************************************/
int16_t getTemperatureValue(void){
  return lm73Control.lm73Data;
}
/**/
 
/***************************************************************************//**
\brief Abre el sensor de temperatura.
*******************************************************************************/
result_t BSP_OpenTemperatureSensor(void)
{
  result_t result;
  if (BC_SUCCESS == (result = openLm73()))
  {
//#if BSP_MNZB_EVB_SUPPORT == 1
    //bspOnPeriphery(1);
//#endif /* BSP_MNZB_EVB_SUPPORT */
  }
  return result;
}

/***************************************************************************//**
\brief Cierra el sensor de temperatura.
*******************************************************************************/
result_t BSP_CloseTemperatureSensor(void)
{
  result_t result;
  if (BC_SUCCESS == (result = closeLm73()))
  {
//#if BSP_MNZB_EVB_SUPPORT == 1
   // bspOffPeriphery(1);
//#endif /* BSP_MNZB_EVB_SUPPORT */
  }
  return result;
}

/**************************************************************************//**
\brief Encapsula la apertura, lectura y cierre del driver
******************************************************************************/
void temperature_TaskHandler(void){
  if(BC_SUCCESS == openLm73())
    readLm73Data(closeLm73);
}
