/**
 \file humidicon.h

 \brief Header del driver del sensor de humedad.
 */


#ifndef HUMIDICON_H_
#define HUMIDICON_H_

//#include <util/delay.h>
#include "bcTypes.h"
#include "i2cPacket.h"


/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Cierra el i2c y pone en marcha un temporizador para leer los datos.
\param[in]  result  resultado de la comunicaci�n i2c de escritura
******************************************************************************/
void callback(bool result);

/**************************************************************************//**
\brief  Abre el sensor.

\retval BC_SUCCESS  el sensor est� listo para ser usado.
\retval BC_FAIL     en otro caso.
******************************************************************************/
result_t openHumidicon(void);

/**************************************************************************//**
\brief Cierra el sensor.

\retval BC_FAIL     hay una operaci�n en marcha.
\retval BC_SUCCESS  en otro caso.
******************************************************************************/
result_t closeHumidicon(void);


/**************************************************************************//**
\brief Lee datos del sensor.
\param[in]  f puntero a la funci�n de callback.
\retval BC_SUCCESS  �xito.
\retval BC_FAIL     en otro caso.
******************************************************************************/
result_t readHumidiconData(bool (*f)(void));

/**************************************************************************//**
\brief  Devuelve el �ltimo valor le�do de humedad.

\return La humedad relativa en tanto por mil.
******************************************************************************/
int16_t getHumidityValue(void);

/**************************************************************************//**
\brief  Devuelve el �ltimo valor le�do de temperatura.

\return La temperatura medida en d�cimas de grado cent�grado.
******************************************************************************/
int16_t getTemperatureValue(void);

/**************************************************************************//**
\brief Cierra las comunicaciones i2c.
******************************************************************************/
void bspHumidiconHandler(void);

/**************************************************************************//**
\brief Inicializa el sensor de humedad y temperatura.

\param[in]  withTemperature si se desea medir tambi�n la temperatura.
******************************************************************************/
void humidity_Init(bool withTemperature);

/**************************************************************************//**
\brief Manda leer al sensor de humedad y temperatura.
******************************************************************************/
void humidity_TaskHandler(void);

#endif /* HUMIDICON_H_ */