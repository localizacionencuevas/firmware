/**************************************************************************//**
\file  adc.h

\brief Implementación de la interfaz ADC.
******************************************************************************/
/******************************************************************************
                   Includes section
******************************************************************************/
#include "bcTypes.h"
#include "sysTypes.h"
#include <util/delay.h>

/******************************************************************************
                   Defines section
******************************************************************************/
#ifndef _ADC_H
#define _ADC_H
/**\brief canal del ADC. */
#define ADC_0  0x78


/******************************************************************************
                   Types section
******************************************************************************/
//typedef bool result_t;
/** \brief Canal del ADC.*/
typedef uint8_t AdcHwChannel_t;
/**************************************************************************//**
\brief Servicio ADC - contiene variables para el módulo ADC. 
*******************************************************************************/
typedef struct
{
/**
\brief contador para la conversión de datos
*/ 
  volatile uint16_t halAdcCurCount;
} HalAdcService_t;

/**************************************************************************//**
\brief Frecuencia de muestreo del ADC
******************************************************************************/
typedef enum
{
  ADC_77KSPS,
  ADC_39KSPS,
  ADC_19200SPS,
  ADC_9600SPS,
  ADC_4800SPS
} HAL_AdcSampleRate_t;

/**************************************************************************//**
\brief Resolución del ADC
******************************************************************************/
typedef enum
{
  RESOLUTION_8_BIT,
  RESOLUTION_10_BIT
} HAL_AdcResolution_t;

/**************************************************************************//**
\brief Referencia de voltaje del ADC.
******************************************************************************/
typedef enum
{
  /** \brief AREF, Vref interno apagado. \n
  Si la ganancia seleccionada es de 10x o 200x, solo se debe utilizar la referencia interna de 2.56V. \n
  Para conversión diferencail, no se puede utilizar la referencia interna de 1.1V.
  */
  AREF = (0 << 6),
  /** \brief AVCC con un condensador externo en el pin AREF */
  AVCC = (1 << 6),
  /** \brief Referencia interna de 1.1V con un condensador externo en el pin AREF */
  INTERNAL_1d1V = (2 << 6),
  /** \brief Referencia interna de 2.56V con un condensador externo en el pin AREF */
  INTERNAL_2d56V = (3 << 6)
} HAL_AdcVoltageReference_t;

/**************************************************************************//**
\brief Parámetros de configuración del ADC.
******************************************************************************/
typedef struct
{
  /** \brief Servicio ADC - contiene variables para el módulo ADC. */
  HalAdcService_t service;
  /** \brief tty - ADC_HW_CHANNEL_n a utilizar. El rango de "n" depende de la plataforma. */
  AdcHwChannel_t tty;
  /** \brief Resolución de la conversión: \n
    RESOLUTION_8_BIT  \n
    RESOLUTION_10_BIT \n
    */
  HAL_AdcResolution_t resolution;
  /** \brief Frecuencia de muestreo. */
  HAL_AdcSampleRate_t sampleRate;
  /** \brief Voltaje de referencia del ADC. Puede ser: \n
      AREF \n
      AVCC \n
      INTERNAL_1d1V \n
      INTERNAL_2d56V \n
    */
  HAL_AdcVoltageReference_t voltageReference;
  /** \brief Puntero del búffer de datos. */
  volatile void *bufferPointer;
  /** \brief Número de muestras. */
  volatile uint16_t selectionsAmount;
  /** \brief Puntero a la función callback que se va a llamar después de todas las conversiones. */
  void (* callback)(void);
} HAL_AdcDescriptor_t;



/**************************************************************************//**
\brief Número de canal.
\arg Si se utiliza una ganancia de 10x, se espera una resolución de 8 bits. \n
\arg Si se utiliza una ganancia de 200x, se espera una resolución de 7 bit. \n
\arg Si el usuario desea ver rápidamente la polaridad del resultado,
es suficiente con leer el bit más significativo. Si el bit es uno, 
el resultado es negativo, y si es cero, el resultado es positivo. 
\arg Para llegar a la precisión dada, no se debe utilizar las ganancias de 10x o 200x 
para voltajes inferiores a 2.7V.
******************************************************************************/
typedef enum
{
  /** \brief Canal ADC0 */
  HAL_ADC_CHANNEL0 = 0,
  /** \brief Canal ADC1 */
  HAL_ADC_CHANNEL1 = 1,
  /** \brief Canal ADC2 */
  HAL_ADC_CHANNEL2 = 2,
  /** \brief Canal ADC3 */
  HAL_ADC_CHANNEL3 = 3,
  /** \brief ADC0 - ADC0 con ganancia 10x */
  HAL_ADC_DIFF_CHANNEL0 = 8,
  /** \brief ADC1 - ADC0 con ganancia 10x */
  HAL_ADC_DIFF_CHANNEL1 = 9,
  /** \brief ADC0 - ADC0 con ganancia 200x */
  HAL_ADC_DIFF_CHANNEL2 = 10,
  /** \brief ADC1 - ADC0 con ganancia 200x */
  HAL_ADC_DIFF_CHANNEL3 = 11,
  /** \brief ADC2 - ADC2 con ganancia 10x */
  HAL_ADC_DIFF_CHANNEL4 = 12,
  /** \brief ADC3 - ADC2 con ganancia 10x */
  HAL_ADC_DIFF_CHANNEL5 = 13,
  /** \brief ADC2 - ADC2 con ganancia 200x */
  HAL_ADC_DIFF_CHANNEL6 = 14,
  /** \brief ADC3 - ADC2 con ganancia 200x */
  HAL_ADC_DIFF_CHANNEL7 = 15
} HAL_AdcChannelNumber_t;


/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Configura los parametros del ADC.
\deprecated No usada
******************************************************************************/
void halOpenAdc(void);

/**************************************************************************//**
\brief Comienza la conversión del canal ADC pasado por parámetro.
\param[in] channel	Número de canal.
******************************************************************************/
void halStartAdc(HAL_AdcChannelNumber_t channel);

/**************************************************************************//**
\brief Abre el ADC para hacer medidas en un canal ADC.

La función debe ser llamada antes de HAL_ReadAdc() para comenzar a trabajar 
con el ADC.


\param[in] descriptor	Puntero a una instancia de HAL_AdcDescriptor_t

\retval -1	Parámetro no soportado o el ADC está ocupado
\retval  0	Éxito
******************************************************************************/
int HAL_OpenAdc(HAL_AdcDescriptor_t *descriptor);

/**************************************************************************//**
\brief Comienza el ADC con los parámetros definidos en HAL_OpenAdc()

\param[in] descriptor	Puntero a una instancia de HAL_AdcDescriptor_t
\param[in] channel		Canal del ADC.

\retval -1	Ha sido imposible abrir el ADC (número de canal no soportado)
\retval  0	Éxito
******************************************************************************/
int HAL_ReadAdc(HAL_AdcDescriptor_t *descriptor, HAL_AdcChannelNumber_t channel);

/**************************************************************************//**
\brief Cierra el ADC.
\deprecated No usada
******************************************************************************/
void halCloseAdc(void);

/**************************************************************************//**
\brief Cierra el ADC - debe ser llamdao después de terminar de trabajar con el ADC.

\param[in] descriptor	Puntero a una instancia de HAL_AdcDescriptor_t

\retval -1	El modulo no estaba abierto
\retval  0	Éxito
******************************************************************************/
int HAL_CloseAdc(HAL_AdcDescriptor_t *descriptor);

/**************************************************************************//**
\brief Controlador de interrupciones.
******************************************************************************/
void halSigAdcHandler(void);

#endif /* _ADC_H */
// eof adc.h
