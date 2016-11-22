/**************************************************************************//**
\file  battery.h

\brief Driver del sensor de batería.
******************************************************************************/

#ifndef _BATTERY_H
#define _BATTERY_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include "adc.h"

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief  Inicializa la batería.
******************************************************************************/
void battery_Init(void);

/**************************************************************************//**
\brief Abre el componente.

\retval	BC_SUCCESS	El componente está listo para ser usado.
\retval	BC_FAIL		En cualquier otro caso.
******************************************************************************/
result_t openBattery(void);

/**************************************************************************//**
\brief  Cierra el componente y escala el valor de la lectura de la batería.
******************************************************************************/
void closeBattery(void);

/**************************************************************************//**
\brief  Comienza la petición al ADC del canal de la batería.

\retval	BC_FAIL		El componente de la batería no está abierto.
\retval	BC_SUCCESS	En otro caso.
******************************************************************************/
result_t readBatteryData(void);

/**************************************************************************//**
\brief Devuelve el valor de la lectura de la batería.

\return  El valor de la batería.
******************************************************************************/
int16_t getBatteryValue(void);

/**************************************************************************//**
\brief  Encapsula en la misma llamada la apertura y la lectura de la batería.
******************************************************************************/
void battery_TaskHandler(void);

#endif /* _BATTERY_H */
// eof battery.h