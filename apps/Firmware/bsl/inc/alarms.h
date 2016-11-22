/**************************************************************************//**
\file  alarms.h

\brief Variables y tipos de alarma.
******************************************************************************/

#ifndef _ALARMS_H
#define _ALARMS_H
#include <stdint.h>
#include <stdbool.h>

/** \brief Tipos posibles de alarma */
typedef enum Alarm_t{
	/** \brief No hay alarma. */
	NO_ALARM,
	/** \brief Alarma normal (al pulsar bot�n, por ejemplo). */
	ALARM_BUTTON,
	/** \brief Alarma al producirse una ca�da. */
	ALARM_FALL, 
	/** \brief Alarma de ca�da desde una gran altura. */
	ALARM_CONTINUOUS_FREEFALL,
	/** \brief Alarma de ca�da y posterior estado de inconsciencia o imposibilidad de movimiento. */
	ALARM_FALL_MOTIONLESS,
	//ALARM_EARTHQUAKE //alarma de actividad s�smica, no v�lido para END_DEVICES
} Alarm_t;

/** \brief Indica si se ha producido una alarma */
bool volatile alarm;
/** \brief Indica si hay que despertar al nodo */
bool volatile despierta;
/** \brief  Tipo de alarma actual, inicialmente NO_ALARM.
*/
Alarm_t volatile typeAlarm;

#endif //_ALARMS_H