/**
 \file movingAverage.h
 
 \brief Implementación de medias móviles para el cálculo de rssi.
 */

#ifndef _MOVING_AVERAGE_H_
#define _MOVING_AVERAGE_H_

#include <stdint.h>
#include <stdbool.h>
#include <sysTypes.h>

/** \brief Tamaño de la ventana para calcular los datos de la media móvil. */
#define  WINDOWS_SIZE 4

/** \brief Número máximo de nodos distintos que se pueden almacenar en la tabla para el cálculo de medias móviles. */
#define MOVING_AVERAGE_SIZE_TABLE 50

/** \brief Datos que se almacenan en el campo de datos de la cada entrada de la tabla. */
typedef struct AddressData_t
{
  /** \brief Número de secuencia. */
  uint8_t nSeq;
  /** \brief Valor en crudo del rssi. */
  int8_t rssi;
} AddressData_t;

/** \brief Datos que se almacenan en cada fila de la tabla */
typedef struct MovingAverageData_t
{
  /** \brief Dirección. */
  uint16_t address;
  /** \brief Tiempo. Indica en que ciclo se ha escrito la última entrada de la tabla con la dirección ateriormente indicada. */
  uint8_t time;
  /** \brief Índice del array datos, indica en qué posición debe escribir la próxima entrada de datos. */
  uint8_t index;
  /** \brief Contiene los últimos index valores recibidos con la dirección indicada. */
  AddressData_t datos[WINDOWS_SIZE];
} MovingAverageData_t;

/**  \brief Tabla donde se almacenan los datos para el cálculo de las medias móviles. */
MovingAverageData_t table [MOVING_AVERAGE_SIZE_TABLE];
/** \brief Índice de la tabla. */
uint8_t indexMovingAverage;
/** \brief Indica el modo de funcionamiento, si es true se realizarán los cálculos con 4 muestras y si es false con 3 */
bool movingAverageWithFour;

// /*
// \brief Calcula la media móvil de los rssi de la dirección pasada.
// \param[in]  address dirección de la que se quiere sacar la media móvil.
// \return     Media móvil.
// \deprecated No usado.
// \todo       Borrar función.
// */
// uint8_t calcMovingAverage(uint16_t address);

/** 
\brief Añade una entrada a la tabla para el cálculo de medias móviles
\param[in]  address dirección del nodo.
\param[in]  nSeq    número de secuencia de la trama.
\param[in]  rssi    rssi de la trama.
\param[in]  time    iteración en el que ha llegado la trama.
\return     La media móvil.
*/
int8_t addDataMovingAverage(uint16_t address, uint8_t nSeq, int8_t rssi, uint8_t time);

/** 
\brief Elimina las entradas de la tabla antiguas
\param[in]  time  iteración actual.
*/
void clearDataMovingAverage(uint8_t time);

/** 
\brief Configura cómo se realizan las medias móviles
\param[in]  withFour  A true si se utilizan 4 medidas (la actual y 3 anteriores) o 3 medidas (la actual y 2 anteriores).
*/
void configMovingAverage(bool withFour);

/** \brief      elimina un elemento de la tabla
    \param[in]  index índice del elemento a eliminar
*/
void removeDataMovingAverage(uint8_t index);

#endif // _MEDIA_MOVILES_H_
