/**
 \file humidicon.c

 \brief Driver del sensor de humedad.
 */


#include "humidicon.h"
#include "sysTimer.h"

/**/
// device address on i2c bus
/** \brief Dirección i2c del dispositivo. */
#define HUMIDICON_DEVICE_ADDRESS  0x27

// device registers internal address
// #define HUMIDICON_DATA_REGISTER 0x00
// #define HUMIDICON_CONFIGURATION_REGISTER 0x01
// #define HUMIDICON_UPPER_LIMIT_REGISTER 0x02
// #define HUMIDICON_LOWER_LIMIT_REGISTER 0x03
// #define HUMIDICON_CONTROL_STATUS_REGISTER 0x04
// #define HUMIDICON_IDENTIFICATION_REGISTER 0x07


/******************************************************************************
                   Types section
******************************************************************************/
// states
/** \brief Estados posibles. */
typedef enum
{
  /** \brief Idle */
  HUM_IDLE,
  /** \brief Leyendo datos */
  HUM_DATA
} humidiconStates_t;

/** \brief Estructura de control de la comunicación con el sensor. Contiene variables internas y un puntero a función. */
typedef struct
{
  /** \brief Dato de humedad, interno. */
  int16_t humidiconHumData;
  /** \brief Dato de temperatura, interno. */
  int16_t humidiconTempData;
  /** \brief Resultado de la operación, interno: true - si no hay errores, false - en otro caso. */
  bool humidiconResult;
  /** \brief Puntero a función callback. */
  bool(* humidiconCallback)(void); // callback
} humidiconControl_t;

/** \brief Posibles modos de funcionamiento. */
typedef enum
{
  /** \brief Lee solo la humedad. */
  HUMIDITY,
  /** \brief Lee humedad y temperatura. */
  TEMPERATURE,  
} TypeRead;

/******************************************************************************
                          Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief  lee y convierte datos

\param[in]
  result - el resultado de la operación
******************************************************************************/
static void humidiconI2cPacketReadDone(bool result);

/******************************************************************************
                   Global variables section
******************************************************************************/
/** \brief temporizador. */
static SYS_Timer_t  timer;
/** \brief estado actual del driver */
static humidiconStates_t humidiconState = HUM_IDLE; // Monitors current state
/** \brief estructura de control */
static humidiconControl_t humidiconControl;
/** \brief tipo de lectura. */
static TypeRead typeRead;

/** \brief Dato leído del sensor. */
static uint32_t humidiconData;
/** \brief Descriptor I2C para mandar leer. */
static HAL_I2cDescriptor_t desc =
{
  .tty = TWI_CHANNEL_0,
  .clockRate = I2C_CLOCK_RATE_100,
  .f = humidiconI2cPacketReadDone,
  .id = HUMIDICON_DEVICE_ADDRESS,
  .data = (uint8_t*)(&humidiconData),
  .length = 2,
  .lengthAddr = HAL_NO_INTERNAL_ADDRESS
};

/** \brief Descriptor I2C para leer el dato. */
static HAL_I2cDescriptor_t descWakeUp =
{
  .tty = TWI_CHANNEL_0,
  .clockRate = I2C_CLOCK_RATE_100,
  .f = callback,
  .id = HUMIDICON_DEVICE_ADDRESS,
  .length = 1,
  .lengthAddr = HAL_NO_INTERNAL_ADDRESS
};

/******************************************************************************
                   Implementations section
******************************************************************************/
result_t openHumidicon(void)
{
  if (HUM_IDLE == humidiconState)
    return BC_SUCCESS;
  return BC_FAIL;
}


result_t closeHumidicon(void)
{
  if (HUM_IDLE == humidiconState)
    return BC_SUCCESS;
  return BC_FAIL;
}


static void humidiconI2cPacketReadDone(bool result)
{
  uint32_t i;
  humidiconControl.humidiconResult = result; // stores the result of operation
  if (false == humidiconControl.humidiconResult)
  {
    // there were some errors on the i2c interface
    humidiconControl.humidiconHumData = 0x8000;
    humidiconControl.humidiconTempData = 0x8000;
    //bspPostTask(BSP_TEMPERATURE);
    bspHumidiconHandler();
    return;
  }
  if (HUM_DATA == humidiconState)
  {
     humidiconControl.humidiconHumData= humidiconData & 0xFFFF;
    
     i = (uint8_t)humidiconControl.humidiconHumData;
     i <<= 8;
     i |= ((humidiconControl.humidiconHumData >> 8) & 0x00FF);
     if ((i & 0xC000) == 0){
      i = i*1000/16382;
      humidiconControl.humidiconHumData = i;
      //tempdata
      if (typeRead == TEMPERATURE) {
        int32_t j;
        j = (humidiconData >>18);
        humidiconControl.humidiconTempData = j;
        j = (uint8_t)humidiconControl.humidiconTempData;
        j <<= 8;
        j |= ((humidiconControl.humidiconTempData >> 8) & 0x00FC);
        humidiconControl.humidiconTempData =  j*1650/16382 - 400;        
      }
      else{
        humidiconControl.humidiconTempData = 0x8000;
      }
    }      
    else{
      humidiconControl.humidiconHumData = 0x8000;
      humidiconControl.humidiconTempData = 0x8000;
    }      
  }
  bspHumidiconHandler();
}

/**************************************************************************//**
\brief Callback del temporizador para leer datos del sensor.

\param[in]
    timer - temporizador
******************************************************************************/
void readHandler(SYS_Timer_t *timer){
  readHumidiconData(closeHumidicon);
}


void callback(bool result)
{
  HAL_CloseI2cPacket(&descWakeUp);
  SYS_TimerStart(&timer);
}

/**************************************************************************//**
\brief Abre la comunicación i2c con el sensor.
******************************************************************************/
void prepareHumidiconData()
{
  HAL_OpenI2cPacket(&descWakeUp);
  if (-1 == HAL_WriteI2cPacket(&descWakeUp))
    HAL_CloseI2cPacket(&descWakeUp);
}

result_t readHumidiconData(bool (*f)(void))
{
  if (HUM_IDLE != humidiconState)
    return BC_FAIL;
  if (!f)
    return BC_FAIL;
  if (-1 == HAL_OpenI2cPacket(&desc))
    return BC_FAIL;
  humidiconState = HUM_DATA;
  humidiconControl.humidiconCallback = f;
  if (-1 == HAL_ReadI2cPacket(&desc))
  {
    humidiconState = HUM_IDLE;
    HAL_CloseI2cPacket(&desc);
    return BC_FAIL;
  }
  return BC_SUCCESS;
}


void bspHumidiconHandler(void)
{
  HAL_CloseI2cPacket(&desc); // free
  humidiconState = HUM_IDLE;
  closeHumidicon();
}

int16_t getHumidityValue(void)
{
  return humidiconControl.humidiconHumData;
}


int16_t getTemperatureValue(void)
{
  return humidiconControl.humidiconTempData;
}


void humidity_Init (bool withTemperature){
  humidiconControl.humidiconHumData = 0x8000;
  humidiconControl.humidiconTempData = 0x8000;
  typeRead = withTemperature;
  humidity_TaskHandler();
}


void humidity_TaskHandler(void)
{
  if (typeRead == TEMPERATURE){
    desc.length = 4;
  }
  timer.interval = 37;
  timer.mode = SYS_TIMER_INTERVAL_MODE;
  timer.handler = readHandler;   
  if(BC_SUCCESS == openHumidicon())
    prepareHumidiconData();  
}
