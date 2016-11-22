/**************************************************************************//**
\file  temperature.h

\brief Driver del sensor de temperatura LM73.
******************************************************************************/

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_
#include "bcTypes.h"
#include "i2cPacket.h"

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Abre el componente.
\return
  BC_SUCCESS - El componente está listo para ser usado. \n
  BC_FAIL - En cualquier otro caso.
******************************************************************************/
result_t openLm73(void);

/**************************************************************************//**
\brief  Cierra el componente.
\return
  BC_SUCCESS - El componente se ha cerrado. \n
  BC_FAIL - En cualquier otro caso.
******************************************************************************/
result_t closeLm73(void);

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
result_t readLm73Data(bool (*f)(void));

/******************************************************************************
\brief Devuelve la última lectura de la temperatura.
\return
	El valor de la temperatura
*******************************************************************************/
int16_t getTemperatureValue(void);

/**************************************************************************//**
\brief Cierra la comunicación I2C y cierra el driver.
******************************************************************************/
void bspLm73Handler(void);

/**/
/***************************************************************************//**
\brief Abre el sensor de temperatura.
\deprecated Sustituida por openLm73.
\see openLm73(void)
\return
  BC_FAIL - El sensor ya estaba abierto. \n
  BC_SUCCESS - En cualquier otro caso.
*******************************************************************************/
result_t BSP_OpenTemperatureSensor(void);

/***************************************************************************//**
\brief Cierra el sensor de temperatura.
\deprecated Sustituida por closeLm73.
\see closeLm73(void)
\return
  BC_FAIL - si ha ocurrido un error hardware o 
			si hay una petición de lectura incompleta. \n
  BC_SUCCESS - en cualquier otro caso.
*******************************************************************************/
result_t BSP_CloseTemperatureSensor(void);

/***************************************************************************//**
\brief Lee el dato del sensor LM73
\deprecated Sustituida por readLm73Data
\see readLm73Data(bool (*f)(void))
*******************************************************************************/
result_t BSP_ReadTemperatureData(void (*f)(void));//(bool result, int16_t data));

/**************************************************************************//**
\brief Encapsula la apertura, lectura y cierre del driver
******************************************************************************/
void temperature_TaskHandler(void);


#endif /* TEMPERATURE_H_ */