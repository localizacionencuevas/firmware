/**************************************************************************//**
\file  ADXL345.c

\brief This module provides access to ADXL345 accelerator.

\author
   Rubén Barrilero - UCLM

\internal
  History:
    09/03/11 R. Barrilero - Created
*****************************************************************************/
/******************************************************************************
                   Includes section
******************************************************************************/
#include "ADXL345.h"

#ifdef ADXL345
/******************************************************************************
                   Types section
******************************************************************************/

/******************************************************************************
                   Global variables section
******************************************************************************/
HAL_SpiDescriptor_t spiDesc;

uint8_t u8InterruptSourceState;
static Data_Accelerometer_s dataAccelerometer;
uint8_t stateFreeFall;

uint16_t InitialStatus[3];
uint16_t Acceleration[3]; // Acceleration for X-, Y-, Z- axis
uint16_t DeltaAcceleration[3]; // Acceleration[] - Initial_Status[]
uint16_t DeltaVectorSum; // Vector sum of the DeltaAcceleration[]
bool inFallDetection;	// true -> en proceso de detección de caída
// 		   impedimos que el dispositivo se duerma
// false-> no estamos en proceso de detección de caída
//		   dejamos que el dispositivo duerma

bool longInactivity;   //true -> está en un largo periodo de inactividad
//temporizadores
static SYS_Timer_t freeFallTimer; //300ms
static SYS_Timer_t strikeTimer; //200ms
static SYS_Timer_t motionlessTimer; //3500ms
static SYS_Timer_t longMotionlessTimer; //10000ms
static SYS_Timer_t resetTimer; //100ms
//static SYS_Timer_t freeFallBetweenTimer; //100ms

static SYS_Timer_t quakeTimer; //1000s

static bool func;

/******************************************************************************
                   Prototypes section
******************************************************************************/

/******************************************************************************
\brief lee un byte de la dirección pasada por parámetro

\param[in] address - dirección a leer

\return el byte que se quería leer
******************************************************************************/
uint8_t ADXL345_oneByteRead(uint8_t address);

/******************************************************************************
\brief escribe un byte en la dirección pasada por parametro

\param[in] address - dirección en la que se quiere escribir \n
		   data - byte que se quiere escribir 
******************************************************************************/
void ADXL345_oneByteWrite(uint8_t address, uint8_t data);

/******************************************************************************
\brief lee varios bytes de un conjunto de direcciones consecutivas.

\param[in] startAddress - dirección de comienzo \n
		   size - número de bytes que se van a leer
\param[out] buffer - puntero al buffer donde se  van a escribir los datos
******************************************************************************/
void ADXL345_multiByteRead(uint8_t startAddress, uint8_t* buffer, uint8_t size);

/******************************************************************************
\brief escribe varios bytes a un conjunto de direcciones consecutivas.

\param[in] startAddress - dirección de comienzo \n
		   size - número de bytes que se van a escribir \n
		   buffer - puntero al buffer con los datos que se quieren escribir
******************************************************************************/
void ADXL345_multiByteWrite(uint8_t startAddress, uint8_t* buffer, uint8_t size);

/******************************************************************************
\brief realiza la división entre dos números.

\param[in] value - dividendo \n
		   lsb - divisor
\return resultado redondeado
******************************************************************************/
uint8_t ADXL345_divide (uint32_t lsb, uint32_t value);

/******************************************************************************
                   Implementations section
******************************************************************************/
/******************************************************************************
\brief Devuelve el ID del acelerómetro.

\return El ID del acelerómetro.
******************************************************************************/
uint8_t ADXL345_getDevId(void)
{
  //valor fijo de 0xE5.
  return ADXL345_oneByteRead((uint8_t)ADXL345_DEVID_REG);
}

/******************************************************************************
\brief Devuelve el umbral del tap. Factor de escala = 62.5mg/LSB
\return El valor del umbral del tap en LSB.
******************************************************************************/
uint8_t ADXL345_getTapThreshold(void)
{
  //Factor de escala = 62,5mg/LSB => 0xFF = 16g.
  return ADXL345_oneByteRead(ADXL345_THRESH_TAP_REG);
}

/******************************************************************************
\brief Asigna un valor al umbral del tap. Factor de escala = 62.5mg/LSB
\param[in] threshold - valor del umbral en LSB.
******************************************************************************/
void ADXL345_setTapThreshold(uint8_t threshold)
{
  //Factor de escala = 62,5mg/LSB => 0xFF = 16g.
  ADXL345_oneByteWrite(ADXL345_THRESH_TAP_REG, threshold);
}

/******************************************************************************
\brief Devuelve la duración mínima del tap. Factor de escala 625 us/LSB.
\return Duración del tap en LSB.
******************************************************************************/
uint8_t ADXL345_getTapDuration(void)
{
  //Máximo tiempo de evento para ser considerado "Tap".
  //Factor de escala: 625 useg/LSB.
  //Valor 0 => detección de golpeo (doble) desactivada.
  return ADXL345_oneByteRead(ADXL345_DUR_REG);//*625;
}

/******************************************************************************
\brief Asigna un valor a la duración mínima del tap. Factor de escala 625us/LSB.

\param[in] duration - Duración del tap en LSB.
******************************************************************************/
void ADXL345_setTapDuration(uint32_t duration)
{
  //Máximo tiempo de evento para ser considerado "Tap".
  //Factor de escala: 625 useg/LSB.
  //Valor 0 => detección de golpeo (simple/doble) desactivada.
  ADXL345_oneByteWrite(ADXL345_DUR_REG, duration/*ADXL345_divide((uint32_t)625,duration)*/);
}

/******************************************************************************
\brief

\param[in] 
******************************************************************************/
uint32_t ADXL345_getTapLatency(void)
{
  //Tiempo de latencia desde el primero golpeo hasta comienzo de ventana de detección del segundo.
  //Factor de escala: 125 useg/LSB.
  //Útil para discriminar oscilaciones debidas al primer golpeo.
  //Valor 0 => Golpeo doble desactivado.
  return ADXL345_oneByteRead(ADXL345_LATENT_REG)*125;
}

void ADXL345_setTapLatency(uint32_t latency_us)
{
  //Tiempo de latencia desde el primero golpeo hasta comienzo de ventana de detección del segundo.
  //Factor de escala: 125 useg/LSB.
  //Útil para discriminar oscilaciones debidas al primer golpeo.
  //Valor 0 => Golpeo doble desactivado.
  ADXL345_oneByteWrite(ADXL345_LATENT_REG, ADXL345_divide((uint32_t)1250, latency_us));
}


uint32_t ADXL345_getWindowTime(void)
{
  //Timepo de ventana para la detección del segundo golpeo.
  //Factor de escala: 125 useg/LSB.
  //Valor 0 => Golpeo doble desactivado.
  return ADXL345_oneByteRead(ADXL345_WINDOW_REG)*125;
}

void ADXL345_setWindowTime(uint32_t window_us)
{
  //Timepo de ventana para la detección del segundo golpeo.
  //Factor de escala: 125 useg/LSB.
  //Valor 0 => Golpeo doble desactivado.
  ADXL345_oneByteWrite(ADXL345_WINDOW_REG, ADXL345_divide((uint32_t)1250, window_us));
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*		  0				 0				0			   0		   Suppress        TAP_X		   TAP_Y		 TAP_Z		*/
/****************************************************************************************************************************/

uint8_t ADXL345_getTapAxisControl(void)
{
  //Suppress = 1 => Deshabilita la detección de golpeo doble
  //TAP_ = 1 => hace participar al eje de la detección de golpeo
  return ADXL345_oneByteRead(ADXL345_TAP_AXES_REG);
}

void ADXL345_setTapAxisControl(uint8_t settings)
{
  //Suppress = 1 => Deshabilita la detección de golpeo doble
  //TAP_ = 1 => hace participar al eje de la detección de golpeo
  ADXL345_oneByteWrite(ADXL345_TAP_AXES_REG, settings);
}

uint8_t ADXL345_getOffset(uint8_t axis)
{
  //Offset añadido en cada medida del eje "axis". Utilizado para definir la cantidad de señal que consideramos "ruido" permanente. Depende de la fijación de la superficie.
  //Factor de escala = 15,6mg/LSB => 0x7F = 2g => 0x80 = -2g.
  uint8_t address = 0;

  if (axis == ADXL345_X)
  {
    address = ADXL345_OFSX_REG;
  }
  else if (axis == ADXL345_Y)
  {
    address = ADXL345_OFSY_REG;
  }
  else if (axis == ADXL345_Z)
  {
    address = ADXL345_OFSZ_REG;
  }

  return ADXL345_oneByteRead(address);
}

void ADXL345_setOffset(uint8_t axis, uint8_t offset)
{
  //Offset añadido en cada medida del eje "axis". Utilizado para definir la cantidad de señal que consideramos "ruido" permanente. Depende de la fijación de la superficie.
  //Factor de escala = 15,6mg/LSB => 0x7F = 2g => 0x80 = -2g.
  uint8_t address = 0;

  if (axis == ADXL345_X)
  {
    address = ADXL345_OFSX_REG;
  }
  else if (axis == ADXL345_Y)
  {
    address = ADXL345_OFSY_REG;
  }
  else if (axis == ADXL345_Z)
  {
    address = ADXL345_OFSZ_REG;
  }

  ADXL345_oneByteWrite(address, offset);
}

uint8_t ADXL345_getActivityThreshold(void)
{
  //Umbral para detectar actividad
  //Factor de escala = 62,5mg/LSB => 0xFF = 16g.
  return ADXL345_oneByteRead(ADXL345_THRESH_ACT_REG);
}

void ADXL345_setActivityThreshold(uint8_t threshold)
{
  //Umbral para detectar actividad
  //Factor de escala = 62,5mg/LSB => 0xFF = 16g.
  ADXL345_oneByteWrite(ADXL345_THRESH_ACT_REG, threshold);
}

uint8_t ADXL345_getInactivityThreshold(void)
{
  //Umbral para detectar inactividad
  //Puede utilizarse para recurrir a modos Sleep
  //Factor de escala = 62,5mg/LSB => 0xFF = 16g.
  return ADXL345_oneByteRead(ADXL345_THRESH_INACT_REG);
}

void ADXL345_setInactivityThreshold(uint8_t threshold)
{
  //Umbral para detectar inactividad
  //Puede utilizarse para recurrir a modos Sleep
  //Factor de escala = 62,5mg/LSB => 0xFF = 16g.
  return ADXL345_oneByteWrite(ADXL345_THRESH_INACT_REG, threshold);
}

uint8_t ADXL345_getTimeInactivity(void)
{
  //Tiempo que la aceleración debe ser menor que el umbral de inactividad para que se declare
  //Factor de escala = 1 sec/LSB => 0xFF = 255 sec
  return ADXL345_oneByteRead(ADXL345_TIME_INACT_REG);
}

void ADXL345_setTimeInactivity(uint8_t timeInactivity)
{
  //Tiempo que la aceleración debe ser menor que el umbral de inactividad para que se declare
  //Factor de escala = 1 sec/LSB => 0xFF = 255 sec
  ADXL345_oneByteWrite(ADXL345_TIME_INACT_REG, timeInactivity);
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*	  ACT ac/dc        ACT_X		  ACT_Y			 ACT_Z		 INACT ac/dc     INACT_X		 INACT_Y		 INACT_Z	*/
/****************************************************************************************************************************/

uint8_t ADXL345_getActivityInactivityControl(void)
{
  //ac/dc = 1 (ac-coupled) => Compara con el umbral la diferencia del valor actual y el valor que se medió cuando se entró por última vez en act/inact
  //ac/dc = 0 (dc-coupled) => Compara la lectura con el valor fijado en THRESH_ACT.
  //ACT_ = 1 => El eje participa en detección de actividad. Si un eje supera THRESH_ACT, se considera actividad. Todos a 0 => Detecc de act off.
  //INACT_ = 1 => El eje participa en detección de inactividad. Si todos por debajo de THRESH_INACT el tiempo TIME_INACT_REG, se considera inactividad. Todos a 0 => Detecc de inact off.
  return ADXL345_oneByteRead(ADXL345_ACT_INACT_CTL_REG);
}

void ADXL345_setActivityInactivityControl(uint8_t settings)
{
  //ac/dc = 1 (ac-coupled) => Compara con el umbral la diferencia del valor actual y el valor que se medió cuando se entró por última vez en act/inact
  //ac/dc = 0 (dc-coupled) => Compara la lectura con el valor fijado en THRESH_ACT.
  //ACT_ = 1 => El eje participa en detección de actividad. Si un eje supera THRESH_ACT, se considera actividad. Todos a 0 => Detecc de act off.
  //INACT_ = 1 => El eje participa en detección de inactividad. Si todos por debajo de THRESH_INACT el tiempo TIME_INACT_REG, se considera inactividad. Todos a 0 => Detecc de inact off.
  ADXL345_oneByteWrite(ADXL345_ACT_INACT_CTL_REG, settings);
}

uint8_t ADXL345_getFreefallThreshold(void)
{
  //Umbral para detectar caída libre
  //Factor de escala = 62,5mg/LSB => 0xFF = 16g.
  //Recomendados: 0x05 - 0x09
  return ADXL345_oneByteRead(ADXL345_THRESH_FF_REG);
}

void ADXL345_setFreefallThreshold(uint8_t threshold)
{
  //Umbral para detectar caída libre
  //Factor de escala = 62,5mg/LSB => 0xFF = 16g.
  //Recomendados: 0x05 - 0x09
  ADXL345_oneByteWrite(ADXL345_THRESH_FF_REG, threshold);
}

/*uint16_t ADXL345_getFreefallTime(void)
{	//Tiempo que la aceleración debe ser menor que el umbral de caída libre para que se declare
	//Factor de escala = 5 ms/LSB => 0xFF = 1275 ms
	//Recomendados: 0x14 - 0x46
    return oneByteRead(ADXL345_TIME_FF_REG)*5;
}*/

uint16_t ADXL345_getFreefallTime(void)
{
  //Tiempo que la aceleración debe ser menor que el umbral de caída libre para que se declare
  //Factor de escala = 5 ms/LSB => 0xFF = 1275 ms
  //Recomendados: 0x14 - 0x46
  return ADXL345_oneByteRead(ADXL345_TIME_FF_REG);
}

/*void ADXL345_setFreefallTime(uint16_t freefallTime_ms)
{	//Tiempo que la aceleración debe ser menor que el umbral de caída libre para que se declare
	//Factor de escala = 5 ms/LSB => 0xFF = 1275 ms
	//Recomendados: 0x14 - 0x46
    uint16_t freefallTime = freefallTime_ms / 5;

    oneByteWrite(ADXL345_TIME_FF_REG, freefallTime);
}*/

void ADXL345_setFreefallTime(uint16_t freefallTime_ms)
{
  //Tiempo que la aceleración debe ser menor que el umbral de caída libre para que se declare
  //Factor de escala = 5 ms/LSB => 0xFF = 1275 ms
  //Recomendados: 0x14 - 0x46
  ADXL345_oneByteWrite(ADXL345_TIME_FF_REG, freefallTime_ms);
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*		   0			 0				 0			LOW_POWER								Rate							*/
/****************************************************************************************************************************/

/************************	Rate	********************************/
/*	Output Data Rate (Hz) | bandwidth (Hz) | Rate Code | Idd (uA) |
					3200			  1600		  1111		 140
					1600			   800		  1110		  90
					 800			   400		  1101		 140
					 400			   200		  1100		 140
					 200			   100		  1011		 140
					 100			    50		  1010		 140 (default)
							Continue (See datasheet pag. 14)	   */
/*******************************************************************/

void ADXL345_setPowerMode(bool LowPowerMode)
{
  //No modifica el data rate, solo si está activo el modo bajo consumo.

  //Obtenemos el valor de registro para no pisar con ceros el output data rate.
  uint8_t registerContents = ADXL345_oneByteRead(ADXL345_BW_RATE_REG);

  registerContents = (uint8_t)(LowPowerMode << 4) | registerContents;

  ADXL345_oneByteWrite(ADXL345_BW_RATE_REG, registerContents);
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*		   0			 0				Link		AUTO_SLEEP		measure			Sleep				Wake_up				*/
/****************************************************************************************************************************/

/******************* Wake_up *******************/
/*   "frecuencia de muestreo en sleep mode"    */
/* Rate Code     |         frequency (Hz)      |
          00							8
          01							4
          10							2
          11							1
************************************************/

uint8_t ADXL345_getPowerControl(void)
{
  //Link: Enlaza la detección de actividad e inactividad cuando ambas están activas, solo se dará una interrupción de actividad si sucede a una inactividad.
  //AUTO_SLEEP: Conmuta entre el modo sleep y normal cuando las funciones actividad/inactividad están activas y dichos eventos son detectados, si Link = 0 => autosleep-mode = desactivado.
  //measure: 0 = Standby ; 1 = Modo de medida. A la salida del reset este valor está a 0 con mínimo consumo.
  //Sleep: 0 = normal; 1 = sleep_mode => desactiva DATA_READY, cancela envío de datos a FIFO y fija sampling rate a wakeup rate. En modo sleep solo corre la función actividad. Para salir 1º sleep = 0, 2º measure = 0, 3º measure = 1
  //wake_up: frecuencia de lectura en sleep_mode
  return ADXL345_oneByteRead(ADXL345_POWER_CTL_REG);
}

void ADXL345_setPowerControl(uint8_t settings)
{
  //Link: Enlaza la detección de actividad e inactividad cuando ambas están activas, solo se dará una interrupción de actividad si sucede a una inactividad.
  //AUTO_SLEEP: Conmuta entre el modo sleep y normal cuando las funciones actividad/inactividad están activas y dichos eventos son detectados, si Link = 0 => autosleep-mode = desactivado.
  //measure: 0 = Standby ; 1 = Modo de medida. A la salida del reset este valor está a 0 con mínimo consumo.
  //Sleep: 0 = normal; 1 = sleep_mode => desactiva DATA_READY, cancela envío de datos a FIFO y fija sampling rate a wakeup rate. En modo sleep solo corre la función actividad. Para salir 1º sleep = 0, 2º measure = 0, 3º measure = 1
  //wake_up: frecuencia de lectura en sleep_mode
  ADXL345_oneByteWrite(ADXL345_POWER_CTL_REG, settings);
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*	  DATA_READY     SINGLE_TAP		DOUBLE_TAP		Activity	  Inactivity	  FREE_FALL      Watermark 	 	Overrun		*/
/****************************************************************************************************************************/

uint8_t ADXL345_getInterruptEnableControl(void)
{
  //Activa las fuentes de interrupción con 1
  return ADXL345_oneByteRead(ADXL345_INT_ENABLE_REG);
}

void ADXL345_setInterruptEnableControl(uint8_t settings)
{
  //Activa las fuentes de interrupción con 1
  ADXL345_oneByteWrite(ADXL345_INT_ENABLE_REG, settings);
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*	  DATA_READY     SINGLE_TAP		DOUBLE_TAP		Activity	  Inactivity	  FREE_FALL      Watermark 	 	Overrun		*/
/****************************************************************************************************************************/
uint8_t ADXL345_getInterruptMappingControl(void)
{
  //genera la interrupción en el pin INT1 (bit a cero) o al pin INT2 (bit a 1). Aquellas que comparten se combinan con OR.
  return ADXL345_oneByteRead(ADXL345_INT_MAP_REG);
}

void ADXL345_setInterruptMappingControl(uint8_t settings)
{
  //genera la interrupción en el pin INT1 (bit a cero) o al pin INT2 (bit a 1). Aquellas que comparten se combinan con OR.
  ADXL345_oneByteWrite(ADXL345_INT_MAP_REG, settings);
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*	  DATA_READY     SINGLE_TAP		DOUBLE_TAP		Activity	  Inactivity	  FREE_FALL      Watermark 	 	Overrun		*/
/****************************************************************************************************************************/
uint8_t ADXL345_getInterruptSource(void)
{
  //Devuelve a 1 la fuente que generó una interrupción.
  //DATA_READY, Watermark y Overrun indican el evento independientemente de que en el registro INT_ENABLE estén fijados a 1 o 0. El estado a 1 de estas interrupciones se elimina leyendo DATAX, DATAY y DATAZ.
  //El resto de bits se limpian cuando se realiza la lectura del registro INT_SOURCE.
  return ADXL345_oneByteRead(ADXL345_INT_SOURCE_REG);
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*		   0           ACT_X		   ACT_Y		  ACT_Z			 Asleep			TAP_X          TAP_Y		  TAP_Z		*/
/****************************************************************************************************************************/

uint8_t ADXL345_getTapSource(void)
{
  //Devuelve el primer eje involucrado (1) y no involucrados(0) en un evento de golpeo (tap) o actividad.
  //En sucesivos datos no se borra, se sobreescribe. Debe ser leído antes de eliminar la interrupción.
  //Si se deshabilita la participación en detección de alguno de los ejes se límpia su bit correspondiente.
  return ADXL345_oneByteRead(ADXL345_ACT_TAP_STATUS_REG);
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*	  SELF_TEST			     SPI			    INT_INVERT		   0		      FULL_RES		     Justify				 Range	 			*/
/****************************************************************************************************************************/
uint8_t ADXL345_getDataFormatControl(void)
{
  //SELF_TEST: Aplica una fuerza sobre el sensor para autotesteo, causando una variación en el dato de salida.
  //SPI: 1 => SPI 3 wire mode / 0 => SPI 4 wire mode.
  //INT_INVERT: 0 => Activo en alta / 1 => Activo en baja.
  //FULL_RES: 0 => 10 bits de resolución para el dato de salida para todos los rangos de "Range" / 1 => Aumenta los bits conforme a lo dispuesto en los bits "Range" (4g->11bits, 8->12, 16->13), pero conservando el factor de escala de 4 mg/LSB.
  //Justify: 1 => MSB mode (justificado a la izquierda) / 0 => justificación a la derecha con extensión de signo (LSB).
  //Range: (ver datasheet. Pag. 27. tabla 21.
  return ADXL345_oneByteRead(ADXL345_DATA_FORMAT_REG);
}

void ADXL345_setDataFormatControl(uint8_t settings)
{
  //SELF_TEST: Aplica una fuerza sobre el sensor para autotesteo, causando una variación en el dato de salida.
  //SPI: 1 => SPI 3 wire mode / 0 => SPI 4 wire mode.
  //INT_INVERT: 0 => Activo en alta / 1 => Activo en baja.
  //FULL_RES: 0 => 10 bits de resolución para el dato de salida para todos los rangos de "Range" / 1 => Aumenta los bits conforme a lo dispuesto en los bits "Range" (4g->11bits, 8->12, 16->13), pero conservando el factor de escala de 4 mg/LSB.
  //Justify: 1 => MSB mode (justificado a la izquierda) / 0 => justificación a la derecha con extensión de signo (LSB).
  //Range: (ver datasheet. Pag. 27. tabla 21.)
  ADXL345_oneByteWrite(ADXL345_DATA_FORMAT_REG, settings);
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*		   0			 0				 0			LOW_POWER								Rate							*/
/****************************************************************************************************************************/

/************************	Rate	********************************/
/*	Output Data Rate (Hz) | bandwidth (Hz) | Rate Code | Idd (uA) |
					3200			  1600		  1111		 140
					1600			   800		  1110		  90
					 800			   400		  1101		 140
					 400			   200		  1100		 140
					 200			   100		  1011		 140
					 100			    50		  1010		 140 (default)
					 50					25		  1001		  90
					 25				  12.5		  1000        60
					 12.5			  6.25		  0111		  50
					 6.25			  3.13		  0110		  45
					 3.13			  1.56		  0101		  40
					 1.56			  0.78	      0100		  34
					 0.78			  0.39		  0011		  23
					 0.39			  0.20		  0010		  23
					 0.20			  0.10		  0001		  23
					 0.10			  0.05		  0000		  23
							Continue (See datasheet pag. 14)	   */
/*******************************************************************/
void ADXL345_setDataRate(uint8_t rate)
{
  //Get the current register contents, so we don't clobber the power bit.
  uint8_t registerContents = ADXL345_oneByteRead(ADXL345_BW_RATE_REG);

  registerContents &= 0x10;
  registerContents |= rate;

  ADXL345_oneByteWrite(ADXL345_BW_RATE_REG, registerContents);
}

void ADXL345_getOutput(uint16_t* readBuffer)
{
  //Devuelve los datos para cada uno de los ejes. (X0, X1, Y0, Y1, Z0, Z1)
  //En complemento a 2 con DATAx0 como el byte menos significativo y DATAx1 como el más significativo.
  static uint8_t buffer[6];

  buffer[0] = ADXL345_oneByteRead(ADXL345_DATAX0_REG);
  buffer[1] = ADXL345_oneByteRead(ADXL345_DATAX1_REG);
  buffer[2] = ADXL345_oneByteRead(ADXL345_DATAY0_REG);
  buffer[3] = ADXL345_oneByteRead(ADXL345_DATAY1_REG);
  buffer[4] = ADXL345_oneByteRead(ADXL345_DATAZ0_REG);
  buffer[5] = ADXL345_oneByteRead(ADXL345_DATAZ1_REG);

  //multiByteRead(ADXL345_DATAX0_REG, buffer, 6);

  *readBuffer++ = ((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0];//buffer[0];
  *readBuffer++ = ((uint16_t)buffer[3] << 8) | (uint16_t)buffer[2];//buffer[2];
  *readBuffer =   ((uint16_t)buffer[5] << 8) | (uint16_t)buffer[4];//buffer[4];
}

void ADXL345_ReadAxis(void)
{
  //Devuelve los datos para cada uno de los ejes. (X0, X1, Y0, Y1, Z0, Z1)
  //En complemento a 2 con DATAx0 como el byte menos significativo y DATAx1 como el más significativo.
  static uint8_t buffer[6];

  buffer[0] = ADXL345_oneByteRead(ADXL345_DATAX0_REG);
  buffer[1] = ADXL345_oneByteRead(ADXL345_DATAX1_REG);
  buffer[2] = ADXL345_oneByteRead(ADXL345_DATAY0_REG);
  buffer[3] = ADXL345_oneByteRead(ADXL345_DATAY1_REG);
  buffer[4] = ADXL345_oneByteRead(ADXL345_DATAZ0_REG);
  buffer[5] = ADXL345_oneByteRead(ADXL345_DATAZ1_REG);

  //multiByteRead(ADXL345_DATAX0_REG, buffer, 6);

  dataAccelerometer.x = ((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0];//buffer[0];
  dataAccelerometer.y = ((uint16_t)buffer[3] << 8) | (uint16_t)buffer[2];//buffer[2];
  dataAccelerometer.z = ((uint16_t)buffer[5] << 8) | (uint16_t)buffer[4];//buffer[4];
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*			  FIFO_MODE				  Trigger		     ---------------------Samples----------------------------	 		*/
/****************************************************************************************************************************/
uint8_t ADXL345_getFifoControl(void)
{
  //FIFO_MODE: 00 => Bypass /
  //01 => FIFO (recoge hasta 32 datos y para hasta que deja de estar lleno) /
  //10 => Stream (se llena 32 y se sobreescribe el más viejo) /
  //11 => TRIGGER. Deja de leer cuando llega al trigger, se espera a que ocurra el evento y vuelve a guardar (recoge hasta 32 datos y para hasta que deja de estar lleno).
  //Trigger: 1 => interrupción en pin INT2 / 0 => interrupción en pin INT1
  //Samples: Depende de FIFO_MODE: Para FIFO y Stream indica el número de muestras necesarias para que se de un evento watermark. Para trigger, cuantas muestras guarda antes de que se produzca el evento.
  return ADXL345_oneByteRead(ADXL345_FIFO_CTL);
}

void ADXL345_setFifoControl(uint8_t settings)
{
  //FIFO_MODE: 00 => Bypass /
  //01 => FIFO (recoge hasta 32 datos y para hasta que deja de estar lleno) /
  //10 => Stream (se llena 32 y se sobreescribe el más viejo) /
  //11 => TRIGGER. Deja de leer cuando llega al trigger, se espera a que ocurra el evento y vuelve a guardar (recoge hasta 32 datos y para hasta que deja de estar lleno).
  //Trigger: 1 => interrupción en pin INT2 / 0 => interrupción en pin INT1
  //Samples: Depende de FIFO_MODE: Para FIFO y Stream indica el número de muestras necesarias para que se de un evento watermark. Para trigger, cuantas muestras guarda antes de que se produzca el evento.
  ADXL345_oneByteWrite(ADXL345_FIFO_STATUS, settings);
}

/****************************************************************************************************************************/
/* |------D7------|------D6------|------D5------|------D4------|------D3------|------D2------|------D1------|------D0------|*/
/*	 FIFO_TRIGGER		  0		  -------------------------------- Entries ----------------------------------------------	*/
/****************************************************************************************************************************/
uint8_t ADXL345_getFifoStatus(void)
{
  //FIFO_TRIGGER: Indica si ocurrión un evento trigger (1) o no (0).
  //Entries: Número de muestras en FIFO.
  return ADXL345_oneByteRead(ADXL345_FIFO_STATUS);
}

uint8_t ADXL345_divide (uint32_t lsb, uint32_t value)
{
  uint8_t rg_value;

  if (value > (uint32_t)(255*lsb))
  {
    /* Maximum value allowed */
    rg_value = 0xFF;
  }
  else
  {
    rg_value = (uint8_t)(value / lsb);

    if ((value % lsb) > (lsb/2))
    {
      rg_value+=1;
    }
  }
  return rg_value;
}

uint8_t ADXL345_oneByteRead(uint8_t address)
{
  uint8_t tx = (ADXL345_SPI_READ | (address & 0x3F));
  uint8_t rx = 0x00;

  HAL_GPIO_ADXL345_CS_clr();

  //Send address to read from.
  HAL_WriteSpi(&spiDesc, (uint8_t *)&tx, 1);
  //Read back contents of address.
  HAL_ReadSpi(&spiDesc, (uint8_t *)&rx, 1);

  HAL_GPIO_ADXL345_CS_set();

  return rx;
}

void ADXL345_oneByteWrite(uint8_t address, uint8_t data)
{
  uint8_t tx = (ADXL345_SPI_WRITE | (address & 0x3F));

  HAL_GPIO_ADXL345_CS_clr();

  //Send address to write to.
  HAL_WriteSpi(&spiDesc, (uint8_t *)&tx, 1);
  //Send data to be written.
  HAL_WriteSpi(&spiDesc, (uint8_t *)&data, 1);

  HAL_GPIO_ADXL345_CS_set();
}

void ADXL345_multiByteRead(uint8_t startAddress, uint8_t* buffer, uint8_t size)
{
  uint8_t tx = (ADXL345_SPI_READ | ADXL345_MULTI_BYTE | (startAddress & 0x3F));

  HAL_GPIO_ADXL345_CS_clr();

  //Send address to start reading from.
  HAL_WriteSpi(&spiDesc, (uint8_t *)&tx, 1);
  //Read back multiple bytes of address.
  HAL_ReadSpi(&spiDesc, (uint8_t *)&buffer, size);

  HAL_GPIO_ADXL345_CS_set();
}

void ADXL345_multiByteWrite(uint8_t startAddress, uint8_t* buffer, uint8_t size)
{
  uint8_t tx = (ADXL345_SPI_WRITE | ADXL345_MULTI_BYTE | (startAddress & 0x3F));

  HAL_GPIO_ADXL345_CS_clr();

  //Send address to start writing to.
  HAL_WriteSpi(&spiDesc, (uint8_t *)&tx, 1);
  //Send multiple bytes to be written.
  HAL_WriteSpi(&spiDesc, (uint8_t *)&buffer, size);

  HAL_GPIO_ADXL345_CS_set();
}
// eof ADXL345.c

int16_t ADXL345_getXValue(void)
{
  return dataAccelerometer.x;
}

int16_t ADXL345_getYValue(void)
{
  return dataAccelerometer.y;
}

int16_t ADXL345_getZValue(void)
{
  return dataAccelerometer.z;
}

int16_t ADXL345_getValue(uint8_t pos)
{
  if (pos == 0)
  {
    return dataAccelerometer.x;
  }
  else if (pos == 1)
  {
    return dataAccelerometer.y;
  }
  else
  {
    return dataAccelerometer.z;
  }
}

bool ADXL345_inFallDetection(void)
{
  return inFallDetection;
}

//#if APP_ENDDEVICE

static void ADXL345_freeFallTimerHandler(SYS_Timer_t *timer)
{
  SYS_TimerStart(&resetTimer);
  (void)timer;
}

static void ADXL345_strikeTimerHandler(SYS_Timer_t *timer)
{
  SYS_TimerStart(&resetTimer);
  (void)timer;
}

static void ADXL345_longMotionlessTimerHandler(SYS_Timer_t *timer)
{
  (void)timer;
}

static void ADXL345_motionlessTimerHandler(SYS_Timer_t *timer)
{
  ADXL345_setConfigFreeFall();
  (void)timer;
}

static void ADXL345_resetTimerHandler(SYS_Timer_t *timer)
{
  ADXL345_setConfigFreeFall();
  (void)timer;
}

/*static void freeFallBetweenTimerHandler(SYS_Timer_t *timer)
{
  (void)timer;
}*/

static void ADXL345_quakeTimerHandler(SYS_Timer_t *timer)
{
  //activamos la interrupción
  ADXL345_setInterruptEnableControl((uint8_t)(ADXL345_ACTIVITY));
  (void)timer;
}

bool ADXL345_init(bool funcionamiento)
{

  func = funcionamiento;
  if (funcionamiento) // si es 1-> enddevice
  {
    //temporizadores
    freeFallTimer.interval = 430; //0.5*9.8*0.430^2 = 0.90m
    freeFallTimer.mode = SYS_TIMER_INTERVAL_MODE;
    freeFallTimer.handler = ADXL345_freeFallTimerHandler;
    strikeTimer.interval = 100;
    strikeTimer.mode = SYS_TIMER_INTERVAL_MODE;
    strikeTimer.handler = ADXL345_strikeTimerHandler;
    motionlessTimer.interval = 3500;
    motionlessTimer.mode = SYS_TIMER_INTERVAL_MODE;
    motionlessTimer.handler = ADXL345_motionlessTimerHandler;
    longMotionlessTimer.interval = 10000;
    longMotionlessTimer.mode = SYS_TIMER_INTERVAL_MODE;
    longMotionlessTimer.handler = ADXL345_longMotionlessTimerHandler;
    resetTimer.interval = 50;//freeFallTimer.interval-strikeTimer.interval+10; //100;
    resetTimer.mode = SYS_TIMER_INTERVAL_MODE;
    resetTimer.handler = ADXL345_resetTimerHandler;
    /*
      freeFallBetweenTimer.interval = 100;
      freeFallBetweenTimer.mode = SYS_TIMER_INTERVAL_MODE;
      freeFallBetweenTimer.handler = freeFallBetweenTimerHandler;
    */
    //configuracion SPI
    spiDesc.tty       = SPI_CHANNEL_0;
    spiDesc.baudRate  = SPI_CLOCK_RATE_2000;
    spiDesc.clockMode = SPI_CLOCK_MODE3;
    spiDesc.dataOrder = SPI_DATA_MSB_FIRST;
    spiDesc.callback  = NULL;

    if (-1 != HAL_OpenSpi(&spiDesc)) 
    {
      HAL_GPIO_ADXL345_CS_out();
      HAL_GPIO_ADXL345_CS_pullup();
      if (ADXL345_getDevId() == ADXL345_DEVICE_ID)
      {
        ADXL345_setDataFormatControl(0x23);//4-wire / INT low active / low res (10bit)  / right-justified mode with sign extension / 16g //(0x27); //4-wire / INT low active / low res (10bit)  / MSB first / 16g
        /* Set Tap Configuration */
        ADXL345_setTapThreshold(80);
        //Next three registers can be written in multibyte form
        ADXL345_setTapDuration(ADXL345_divide((uint32_t)625,8000)/*8000*/);
        ADXL345_setTapLatency(100000);
        ADXL345_setWindowTime(300000);
        ADXL345_setTapAxisControl(0x0F); //Three axis, double tap detection.

        /* Set PowerMode, DataRate and Interrupts */
        u8InterruptSourceState = 0x00;
        ADXL345_setDataRate(0x1A); //100 Hz
        ADXL345_getInterruptSource(); //Clean all possible active interruptions
        ADXL345_setInterruptMappingControl(0xFF);
        ADXL345_setInterruptEnableControl(0x00);

        HAL_GPIO_IRQ_7_in();
        HAL_GPIO_IRQ_7_pullup();
        EIMSK |= (1 << INT7);
        ADXL345_setPowerControl(0x08); //measure mode //8Hz
        /*freefall configuration*/
        ADXL345_setConfigFreeFall();
        return true;
      }
    }

    return false;
  }
  else	//para detección de movimiento
  {
    quakeTimer.interval = 2000;
    quakeTimer.mode = SYS_TIMER_INTERVAL_MODE;
    quakeTimer.handler = ADXL345_quakeTimerHandler;

    spiDesc.tty       = SPI_CHANNEL_0;
    spiDesc.baudRate  = SPI_CLOCK_RATE_2000;
    spiDesc.clockMode = SPI_CLOCK_MODE3;
    spiDesc.dataOrder = SPI_DATA_MSB_FIRST;
    spiDesc.callback  = NULL;

    if (-1 != HAL_OpenSpi(&spiDesc))
    {
      HAL_GPIO_ADXL345_CS_out();
      HAL_GPIO_ADXL345_CS_pullup();
      if (ADXL345_getDevId() == ADXL345_DEVICE_ID)
      {
        ADXL345_setDataFormatControl(0x27); //4-wire / INT low active / low res (10bit)  / MSB first / 16g

        /* Set PowerMode, DataRate and Interrupts */
        ADXL345_setDataRate(0x1A); //100 Hz
        ADXL345_getInterruptSource(); //Clean all possible active interruptions
        ADXL345_setInterruptMappingControl(0xFF);
        ADXL345_setInterruptEnableControl(0x00);

        HAL_GPIO_IRQ_7_in();
        HAL_GPIO_IRQ_7_pullup();

        EIMSK |= (1 << INT7);

        ADXL345_setPowerControl(0x08); //measure mode //8Hz
        ADXL345_setActivityInactivityControl(0xFF); //ac-coupled for activity, ac-coupled for inactivity
        ADXL345_setActivityThreshold(0x02);
        ADXL345_setInterruptEnableControl((uint8_t)(ADXL345_ACTIVITY));
        return true;
      }
    }
    return false;
  }
}

void ADXL345_setConfigFreeFall(void) //freefall init
{
  //estamos fuera de la detección de caída
  inFallDetection = false;
  //inicializamos
  stateFreeFall = STATE_INITIAL;
  //THRESH_FF a 0.75g
  ADXL345_setFreefallThreshold(FREE_FALL_THRESHOLD);
  //TIME_FF a 30ms
  ADXL345_setFreefallTime(FREE_FALL_TIME);
  InitialStatus[0]=0x1000; // X axis=0g, unsigned short int, 13 bit resolution, 0x1000 = 4096 = 0g, +/-0xFF = +/-256 = +/-1g
  InitialStatus[1]=0x0F00; // Y axis=-1g
  InitialStatus[2]=0x1000; // Z axis=0g
  ADXL345_setActivityInactivityControl(0x7F); // dc-coupled for activity, ac-coupled for inactivity
  ADXL345_setTimeInactivity(LONG_INACTIVITY_TIME);
  ADXL345_setInactivityThreshold(LONG_INACTIVITY_THRESHOLD);
  //ADXL345_setTimeInactivity(STABLE_TIME);
  ADXL345_setActivityThreshold(STRIKE_THRESHOLD);
  //TO-DO: Cambiar según la aplicación
  //ADXL345_setInterruptEnableControl((uint8_t)(ADXL345_DOUBLETAP|ADXL345_FREEFALL|ADXL345_INACTIVITY)); //es la que estaba puesta
  ADXL345_setInterruptEnableControl((uint8_t)(ADXL345_FREEFALL|ADXL345_DOUBLETAP|ADXL345_FREEFALL|ADXL345_ACTIVITY|ADXL345_INACTIVITY));
  //ADXL345_setInterruptEnableControl((uint8_t)(ADXL345_DOUBLETAP|ADXL345_FREEFALL|ADXL345_ACTIVITY|ADXL345_INACTIVITY));
}




void ADXL345_sleep(void)
{
  //al detectar actividad el dispositivo se despertará
  ADXL345_setActivityInactivityControl(0xFF);
  ADXL345_setActivityThreshold(WAKE_UP_THRESHOLD);
  //ADXL345_setInterruptEnableControl((uint8_t)(ADXL345_DOUBLETAP|ADXL345_ACTIVITY|ADXL345_FREEFALL));
  ADXL345_setInterruptEnableControl((uint8_t)(ADXL345_FREEFALL));
}

void ADXL345_close(void)
{

  HAL_CloseSpi(&spiDesc);
}

bool ADXL345_inLongInactivity(void)
{
  return longInactivity;
}

//#if APP_ENDDEVICE
ISR(INT7_vect)
{
  EIMSK &= ~(1<<INT7);
  if (func)
  {
    //freefall detection application note AN-1023

    uint8_t source = ADXL345_getInterruptSource(); //falta para doble tap,...

    if (((source & ADXL345_DOUBLETAP) >> ADXL345_DOUBLETAP_DESP) && stateFreeFall == STATE_INITIAL)
    {
      typeAlarm= ALARM_BUTTON;
      despierta = true;
      alarm = true;
    }
    else if ((source & ADXL345_ACTIVITY) >> ADXL345_ACTIVITY_DESP) // Activity interrupt asserted
    {
      if (stateFreeFall == STATE_INITIAL) //hay que despertar si el dispositivo estaba durmiendo
      {
        if(!inFallDetection)
        {
          longInactivity = false;
          despierta = true;
        }
      }
      if (stateFreeFall == STATE_FREE_FALL)  // Waiting for strike, and now strike is detected
      {
        SYS_TimerStop(&freeFallTimer);
        if (SYS_TimerStarted(&strikeTimer))
        {
          SYS_TimerStop(&strikeTimer);

          stateFreeFall = STATE_STRIKE;
          ADXL345_setActivityThreshold(STABLE_THRESHOLD);
          ADXL345_setInactivityThreshold(NOMOVEMENT_THRESHOLD);

          SYS_TimerStart(&motionlessTimer);
          ADXL345_setActivityInactivityControl(0xFF); //ac-coupled for activity, ac-coupled for inactivity
          ADXL345_setInterruptEnableControl((uint8_t)(/*ADXL345_DOUBLETAP|*/ADXL345_FREEFALL|ADXL345_INACTIVITY));
          //despierta = true;
          //alarm = true;
          //typeAlarm = 0x61;
        }
      }
      else if(stateFreeFall == STATE_MOTIONLESS) // Waiting for long time motionless, but a movement is detected
      {
        //restart
        SYS_TimerStop(&longMotionlessTimer);
        ADXL345_setConfigFreeFall();
      }
    }
    else if ((source & ADXL345_INACTIVITY)>>ADXL345_INACTIVITY_DESP) // Inactivity interrupt asserted
    {
      if (stateFreeFall == STATE_INITIAL) //lo mandamos a dormir profundamente
      {
        if( !inFallDetection)
        {
          longInactivity = true;
        }
      }
      else if(stateFreeFall == STATE_STRIKE)
      {
        // Waiting for stable, and now stable is detected
        SYS_TimerStop(&motionlessTimer);
        ADXL345_getOutput(Acceleration);
        DeltaVectorSum = 0;
        for(uint8_t i=0; i<3; i++)
        {
          if(Acceleration[i]<0x1000)
          {
            Acceleration[i]=Acceleration[i]+0x1000;
          }
          else //if(Acceleration[i]>= 0x1000)
          {
            Acceleration[i]=Acceleration[i]-0x1000;
          }
          if(Acceleration[i]>InitialStatus[i])
          {
            DeltaAcceleration[i]=Acceleration[i]-InitialStatus[i];
          }
          else
          {
            DeltaAcceleration[i]=InitialStatus[i]-Acceleration[i];
          }
          DeltaVectorSum=DeltaVectorSum+DeltaAcceleration[i]*DeltaAcceleration[i];
        }
        if (DeltaVectorSum>DELTA_VECTOR_SUM_THRESHOLD)// The stable status is different from the initial status
        {
          //valid fall detection
          stateFreeFall = STATE_MOTIONLESS;
          ADXL345_setActivityThreshold(STABLE_THRESHOLD);

          //caida normal

          typeAlarm = ALARM_FALL;
          despierta = true;
          alarm = true;

          ADXL345_setTimeInactivity(NOMOVEMENT_TIME);
          SYS_TimerStart(&longMotionlessTimer);
          ADXL345_setActivityInactivityControl(0xFF); //ac-coupled for activity, ac-coupled for inactivity
          ADXL345_setInterruptEnableControl((uint8_t)(/*ADXL345_DOUBLETAP|*/ADXL345_FREEFALL|ADXL345_INACTIVITY|ADXL345_ACTIVITY));
        }
        else
        {
          ADXL345_setActivityThreshold(STRIKE_THRESHOLD);
          ADXL345_setConfigFreeFall();
        }
      }
      else if (stateFreeFall == STATE_MOTIONLESS)
      {
        // Wait for long time motionless, and now it is detected
        if (!SYS_TimerStarted(&longMotionlessTimer))
        {


          typeAlarm = ALARM_FALL_MOTIONLESS;
          despierta = true;
          alarm = true;

          //reiniciamos
          ADXL345_setConfigFreeFall();
        }
      }
    }
    else if ((source & ADXL345_FREEFALL)>>ADXL345_FREEFALL_DESP) // Free fall interrupt asserted
    {
      if (stateFreeFall == STATE_INITIAL) //free fall
      {

        despierta = true; //despertamos el micro para controlar los tiempos
        longInactivity = false;
        inFallDetection = true;
        stateFreeFall = STATE_FREE_FALL;
        ADXL345_setActivityThreshold(STRIKE_THRESHOLD);
        ADXL345_setInactivityThreshold(NOMOVEMENT_THRESHOLD);
        ADXL345_setTimeInactivity(STABLE_TIME);
        SYS_TimerStart(&freeFallTimer); //300ms
        SYS_TimerStart(&strikeTimer); //200ms

        //ADXL345_setInterruptEnableControl((uint8_t)(ADXL345_DOUBLETAP|ADXL345_FREEFALL|ADXL345_ACTIVITY));
        ADXL345_setInterruptEnableControl((uint8_t)(ADXL345_FREEFALL|ADXL345_ACTIVITY));

        //SYS_TimerStart(&freeFallBetweenTimer);

      }
      else if (stateFreeFall == STATE_FREE_FALL) //continuous free fall detected
      {
        if (!SYS_TimerStarted(&freeFallTimer) /*&& SYS_TimerStarted(&freeFallBetweenTimer)*/)
        {

          //SYS_TimerStop(&freeFallBetweenTimer);

          //hay que mirar que la lectura anterior se hizo hace menos de 100ms
          SYS_TimerStop(&resetTimer);
          SYS_TimerStop(&strikeTimer);
          SYS_TimerStart(&resetTimer);
          stateFreeFall = STATE_CONTINUOUS_FREE_FALL;
          //ADXL345_setActivityThreshold(STRIKE_THRESHOLD);
          //ADXL345_setInactivityThreshold(NOMOVEMENT_THRESHOLD);
          // si el contador no sigue funcionando es que se ha detectado una caída desde una altura considerable

          despierta = true;
          alarm = true;
          typeAlarm = ALARM_CONTINUOUS_FREEFALL;

        }
        else
        {
          /*
              if (SYS_TimerStarted(&freeFallBetweenTimer))
              {
                SYS_TimerStop(&freeFallBetweenTimer);
                SYS_TimerStart(&freeFallBetweenTimer);
              }
          */
          
          SYS_TimerStop(&resetTimer);
          SYS_TimerStop(&strikeTimer);
          SYS_TimerStart(&strikeTimer);
        }
      }
      else if (stateFreeFall == STATE_CONTINUOUS_FREE_FALL) //continuous free fall
      {
        SYS_TimerStop(&resetTimer);  // si seguimos detectando caída libre
        SYS_TimerStart(&resetTimer); // reiniciamos el timer del reset
      }
    }
  }
  /*}

  #else
  ISR(INT7_vect)
  { */
//   else
//   {
//     EIMSK &= ~(1<<INT7);
//     uint8_t source = ADXL345_getInterruptSource(); //falta para doble tap,...
// 
//     if ((source & ADXL345_ACTIVITY) >> ADXL345_ACTIVITY_DESP)
//     {
//       typeAlarm= ALARM_EARTHQUAKE;
//       despierta = true;
//       alarm = true;
//       //desactivamos la interrupción
//       ADXL345_setInterruptEnableControl((uint8_t)(0/*ADXL345_ACTIVITY*/));
//       //inicializamos un contador para nueva alarma en 1 segundo
//       SYS_TimerStart(&quakeTimer);
//     }
//   }
  EIMSK |= (1 << INT7);
}

//#endif
#endif