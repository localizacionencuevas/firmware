/**************************************************************************//**
\file  battery.h

\brief Driver del sensor de bater�a.
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
\brief  Inicializa la bater�a.
******************************************************************************/
void battery_Init(void);

/**************************************************************************//**
\brief Abre el componente.

\retval	BC_SUCCESS	El componente est� listo para ser usado.
\retval	BC_FAIL		En cualquier otro caso.
******************************************************************************/
result_t openBattery(void);

/**************************************************************************//**
\brief  Cierra el componente y escala el valor de la lectura de la bater�a.
******************************************************************************/
void closeBattery(void);

/**************************************************************************//**
\brief  Comienza la petici�n al ADC del canal de la bater�a.

\retval	BC_FAIL		El componente de la bater�a no est� abierto.
\retval	BC_SUCCESS	En otro caso.
******************************************************************************/
result_t readBatteryData(void);

/**************************************************************************//**
\brief Devuelve el valor de la lectura de la bater�a.

\return  El valor de la bater�a.
******************************************************************************/
int16_t getBatteryValue(void);

/**************************************************************************//**
\brief  Encapsula en la misma llamada la apertura y la lectura de la bater�a.
******************************************************************************/
void battery_TaskHandler(void);

#endif /* _BATTERY_H */
// eof battery.h