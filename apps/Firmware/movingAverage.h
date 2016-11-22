/**
 \file movingAverage.h
 
 \brief Implementaci�n de medias m�viles para el c�lculo de rssi.
 */

#ifndef _MOVING_AVERAGE_H_
#define _MOVING_AVERAGE_H_

#include <stdint.h>
#include <stdbool.h>
#include <sysTypes.h>

/** \brief Tama�o de la ventana para calcular los datos de la media m�vil. */
#define  WINDOWS_SIZE 4

/** \brief N�mero m�ximo de nodos distintos que se pueden almacenar en la tabla para el c�lculo de medias m�viles. */
#define MOVING_AVERAGE_SIZE_TABLE 50

/** \brief Datos que se almacenan en el campo de datos de la cada entrada de la tabla. */
typedef struct AddressData_t
{
  /** \brief N�mero de secuencia. */
  uint8_t nSeq;
  /** \brief Valor en crudo del rssi. */
  int8_t rssi;
} AddressData_t;

/** \brief Datos que se almacenan en cada fila de la tabla */
typedef struct MovingAverageData_t
{
  /** \brief Direcci�n. */
  uint16_t address;
  /** \brief Tiempo. Indica en que ciclo se ha escrito la �ltima entrada de la tabla con la direcci�n ateriormente indicada. */
  uint8_t time;
  /** \brief �ndice del array datos, indica en qu� posici�n debe escribir la pr�xima entrada de datos. */
  uint8_t index;
  /** \brief Contiene los �ltimos index valores recibidos con la direcci�n indicada. */
  AddressData_t datos[WINDOWS_SIZE];
} MovingAverageData_t;

/**  \brief Tabla donde se almacenan los datos para el c�lculo de las medias m�viles. */
MovingAverageData_t table [MOVING_AVERAGE_SIZE_TABLE];
/** \brief �ndice de la tabla. */
uint8_t indexMovingAverage;
/** \brief Indica el modo de funcionamiento, si es true se realizar�n los c�lculos con 4 muestras y si es false con 3 */
bool movingAverageWithFour;

// /*
// \brief Calcula la media m�vil de los rssi de la direcci�n pasada.
// \param[in]  address direcci�n de la que se quiere sacar la media m�vil.
// \return     Media m�vil.
// \deprecated No usado.
// \todo       Borrar funci�n.
// */
// uint8_t calcMovingAverage(uint16_t address);

/** 
\brief A�ade una entrada a la tabla para el c�lculo de medias m�viles
\param[in]  address direcci�n del nodo.
\param[in]  nSeq    n�mero de secuencia de la trama.
\param[in]  rssi    rssi de la trama.
\param[in]  time    iteraci�n en el que ha llegado la trama.
\return     La media m�vil.
*/
int8_t addDataMovingAverage(uint16_t address, uint8_t nSeq, int8_t rssi, uint8_t time);

/** 
\brief Elimina las entradas de la tabla antiguas
\param[in]  time  iteraci�n actual.
*/
void clearDataMovingAverage(uint8_t time);

/** 
\brief Configura c�mo se realizan las medias m�viles
\param[in]  withFour  A true si se utilizan 4 medidas (la actual y 3 anteriores) o 3 medidas (la actual y 2 anteriores).
*/
void configMovingAverage(bool withFour);

/** \brief      elimina un elemento de la tabla
    \param[in]  index �ndice del elemento a eliminar
*/
void removeDataMovingAverage(uint8_t index);

#endif // _MEDIA_MOVILES_H_
