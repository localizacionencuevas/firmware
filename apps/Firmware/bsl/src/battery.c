/**************************************************************************//**
\file  battery.c

\brief Driver del sensor de batería.
******************************************************************************/
/******************************************************************************
                   Includes section
******************************************************************************/
#include "battery.h"

/******************************************************************************
                   Define(s) section
******************************************************************************/


/******************************************************************************
                   Types section
******************************************************************************/
/**************************************************************************//**
\brief Estados posibles de la lectura de la batería.
******************************************************************************/
typedef enum
{
  IDLE,      // idle
  BUSY,      // opened and ready to be used
  DATA       // performs request
} batteryStates_t;

/******************************************************************************
                   Global variables section
******************************************************************************/
// Monitors current state
/** \brief Estado actual de la lectura de la batería. */
static batteryStates_t batteryState = IDLE;
/** \brief Estructura para la lectura de la batería. */
//static BatteryControl_t batteryControl;
/** \brief Descriptor ADC para realizar la lectura. */
static HAL_AdcDescriptor_t adcDescriptor;
/** \brief Valor de la batería. */
static int16_t batValue;
/** \brief Valor leído del ADC. */
uint8_t batteryData;

/******************************************************************************
                   Implementations section
******************************************************************************/
void battery_Init(void)
{
  DDRE  |= (1<<DDE3);
  PORTE |= (1<<PE3);
  batValue = 0;
  adcDescriptor.tty = ADC_0;
  adcDescriptor.resolution = RESOLUTION_8_BIT;
  adcDescriptor.sampleRate = ADC_4800SPS;
  adcDescriptor.voltageReference = INTERNAL_1d1V;
  adcDescriptor.bufferPointer = &(batteryData);
  adcDescriptor.selectionsAmount = 1;
  adcDescriptor.callback = closeBattery;
}


result_t openBattery(void)
{
  if (IDLE == batteryState)
  {
    batteryData=0;
    batteryState = BUSY;
    HAL_OpenAdc(&adcDescriptor);
    PORTE  &= ~(1<<PE3);
    return BC_SUCCESS;
  }
  return BC_FAIL;
}


void closeBattery(void)
{
  if (batteryState == DATA)
	batValue = (batteryData * 1100ul * 4ul) / 25500ul; //255ul
  HAL_CloseAdc(&adcDescriptor);
  PORTE |= (1<<PE3); 
  batteryState = IDLE;
}


result_t readBatteryData(void)
{
  if (BUSY != batteryState)
    return BC_FAIL;
  batteryState = DATA;
  HAL_ReadAdc(&adcDescriptor, HAL_ADC_CHANNEL0);
  return BC_SUCCESS;
}


int16_t getBatteryValue(void)
{
  return batValue;
}


void battery_TaskHandler(void)
{  
  if(openBattery())
    readBatteryData();  
}