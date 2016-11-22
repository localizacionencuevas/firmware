/**************************************************************************//**
\file  ADXL362.h

\brief Declaraciones de la interfaz para ADXL362.

*******************************************************************************/

#ifndef _ADXL362_H
#define _ADXL362_H

/******************************************************************************
                   Includes section
******************************************************************************/

#include "alarms.h"
#include "bcTypes.h"
#include <halGpio.h>
#include "spi.h"
#include <sysTimer.h>
#include <halSleep.h>
#include "config.h"

/* \brief Tiempo de inactividad para dormir. No usado*/
//#define TIME_INACTIVITY 10000//150000 //150000 ms = 2 minutos y 30 segundos

/* \brief Suma de la diferencia en los tres ejes entre posición inicial y final. No usado. */
//#define DELTA_VECTOR_SUM_THRESHOLD 0x7D70 //1g=0xFF, 0x7D70=0.7g^2

/** \brief Pendiente de la recta para la corrección del offset en el eje x. */
#define ADXL362_OFFSET_8G_X_SLOPE 2
/** \brief Corte con el eje y de la recta para la correción del offset en el eje x. */
#define ADXL362_OFFSET_8G_X_INTERCEPT -10

/** \brief Pendiente de la recta para la corrección del offset en el eje y. */
#define ADXL362_OFFSET_8G_Y_SLOPE 4
/** \brief Corte con el eje y de la recta para la correción del offset en el eje y. */
#define ADXL362_OFFSET_8G_Y_INTERCEPT -56

/** \brief Pendiente de la recta para la corrección del offset en el eje z. */
#define ADXL362_OFFSET_8G_Z_SLOPE 4
/** \brief Corte con el eje y de la recta para la correción del offset en el eje z. */
#define ADXL362_OFFSET_8G_Z_INTERCEPT -36

/** \brief código de escritura. */
#define ADXL362_SPI_WRITE 0x0A
/** \brief código de lectura. */
#define ADXL362_SPI_READ 0x0B
/** \brief código de acceso a la FIFO. */
#define ADXL362_SPI_FIFO 0x0D
/** \brief Contiene el Analog Devices device ID, 0xAD. */
#define ADXL362_DEVID_REG 0x00
/** \brief Contiene el Analog Devices MEMS device ID, 0x1D. */
#define ADXL362_DEVID_MST_REG 0x01
/** \brief Contiene el device ID, 0xF2 (362 octal). */
#define ADXL362_PARTID_REG 0x02
/** \brief Contiene el product revision ID. */
#define ADXL362_REVID_REG 0x03
/** \brief Contiene los 8 bits más significativos de la aceleración en el eje X. */
#define ADXL362_XDATA_REG 0x08
/** \brief Contiene los 8 bits más significativos de la aceleración en el eje Y. */
#define ADXL362_YDATA_REG 0x09
/** \brief Contiene los 8 bits más significativos de la aceleración en el eje Z. */
#define ADXL362_ZDATA_REG 0x0A

/** \brief Posición del flag de interrupción por actividad en el registro ADXL362_STATUS_REG 
    \see ADXL362_STATUS_REG
*/
#define ADXL362_ACT_INT   0x10

/** \brief Posición del flag de interrupción por inactividad en el registro ADXL362_STATUS_REG 
    \see ADXL362_STATUS_REG
*/
#define ADXL362_INACT_INT 0x20

/** \brief Registro de estado
    \details
     D7       |      D6      |      D5      |      D4      |      D3      |      D2      |      D1      |      D0
--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------
ERR_USER_REGS	|     AWAKE	   |    INACT     |     ACT      | FIFO_OVERRUN |FIFO_WATERMARK|  FIFO_READY  |   DATA_READY

- ERR_USER_REGS: SEU Error Detect. 1 indicates one of two conditions: either an SEU event, such as an alpha particle of a
power glitch, has disturbed a user register setting or the ADXL362 is not configured. This bit is high upon
both startup and soft reset, and resets as soon as any register write commands are performed.
- AWAKE: Indicates whether the accelerometer is in an active (AWAKE = 1) or inactive (AWAKE = 0) state, based on the activity
and inactivity functionality. To enable autosleep, activity and inactivity detection must be in linked mode or loop
mode (LINK/LOOP bits in the ACT_INACT_CTL register); otherwise, this bit defaults to 1 and should be ignored.
- INACT: Inactivity. 1 indicates that the inactivity detection function has detected an inactivity or a free fall condition.
- ACT: Activity. 1 indicates that the activity detection function has detected an overthreshold condition.
- FIFO_OVERRUN: FIFO Overrun. 1 indicates that the FIFO has overrun or overflowed, such that new data replaces unread data.
See the Using FIFO Interrupts section for details (datasheet p. 40).
- FIFO_WATERMARK: FIFO Watermark. 1 indicates that the FIFO contains at least the desired number of samples, as set in the
- FIFO_SAMPLES register. See the Using FIFO Interrupts section for details (datasheet p.40).
- FIFO_READY: FIFO Ready. 1 indicates that there is at least one sample available in the FIFO output buffer.
See the Using FIFO Interrupts section for details (datasheet p. 40).
- DATA_READY: Data Ready. 1 indicates that a new valid sample is available to be read. This bit clears when a FIFO read is
performed. See the Data Ready Interrupt section for more details (datasheet p. 40).
*/
#define ADXL362_STATUS_REG 0x0B

/** \brief Contiene los 8 bits menos significativos del número de entradas de la FIFO. */
#define ADXL362_FIFO_ENTRIES_L_REG 0x0C

/** \brief Contiene los 2 bits más significativos del número de entradas de la FIFO. El resto de bits no se deben tener en cuenta. */
#define ADXL362_FIFO_ENTRIES_H_REG 0x0D

/** \brief Contiene los 8 bits menos significativos de la aceleración en el eje X. */
#define ADXL362_XDATA_L_REG 0x0E

/** \brief Contiene los 4 bits más significativos de la aceleración en el eje X. El resto de bits contienen el valor del MSB. */
#define ADXL362_XDATA_H_REG 0x0F

/** \brief Contiene los 8 bits menos significativos de la aceleración en el eje Y. */
#define ADXL362_YDATA_L_REG 0x10

/** \brief Contiene los 4 bits más significativos de la aceleración en el eje Y. El resto de bits contienen el valor del MSB. */
#define ADXL362_YDATA_H_REG 0x11

/** \brief Contiene los 8 bits menos significativos de la aceleración en el eje X. */
#define ADXL362_ZDATA_L_REG 0x12

/** \brief Contiene los 4 bits más significativos de la aceleración en el eje Z. El resto de bits contienen el valor del MSB. */
#define ADXL362_ZDATA_H_REG 0x13

/** \brief Contiene los 8 bits menos significativos de la temperatura. */
#define ADXL362_TEMP_L_REG 0x14

/** \brief Contiene los 4 bits más significativos de la temperatura en el eje Z. El resto de bits contienen el valor del MSB. */
#define ADXL362_TEMP_H_REG 0x15

/** \brief Escribiendo 0x52 en este registro se reinicia el acelerómetro. */
#define ADXL362_SOFT_RESET_REG 0x1F

/** \brief Contiene los 8 bits menos significativos del umbral de actividad. */
#define ADXL362_THRESH_ACT_L_REG 0x20

/** \brief Contiene los 3 bits más significativos del umbral de actividad. */
#define ADXL362_THRESH_ACT_H_REG 0x21

/** \brief Tiempo mínimo de actividad en nº de muestras. */
#define ADXL362_TIME_ACT_REG 0x22

/** \brief Contiene los 8 bits menos significativos del umbral de inactividad. */
#define ADXL362_THRESH_INACT_L_REG 0x23

/** \brief Contiene los 3 bits más significativos del umbral de inactividad. */
#define ADXL362_THRESH_INACT_H_REG 0x24

/** \brief Contiene los 8 LSB del tiempo mínimo de inactividad en nº de muestras. */
#define ADXL362_TIME_INACT_L_REG 0x25
/** \brief Contiene los 8 MSB del tiempo mínimo de inactividad en nº de muestras. */
#define ADXL362_TIME_INACT_H_REG 0x26

/** \brief Registro de control de actividad/inactividad

\details
     D7       |      D6      |      D5      |      D4      |      D3      |      D2      |      D1      |      D0
--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------
     0        |      0       |  LINK/LOOP   |   LINK/LOOP  |  INACT_REF   |   INACT_EN   |   ACT_REF    |   ACT_EN

- LINK/LOOP:
  - X0	Default Mode.
Activity and inactivity detection are both enabled, and their interrupts (if mapped) must be acknowledged
by the host processor by reading the STATUS register. Autosleep is disabled in this mode. Use this mode
for free fall detection applications.
  - 01	Linked Mode.
Activity and inactivity detection are linked sequentially such that only one is enabled at a time.
Their interrupts (if mapped) must be acknowledged by the host processor by reading the STATUS register.
  - 11	Loop Mode.
Activity and inactivity detection are linked sequentially such that only one is enabled at a time,
and their interrupts are internally acknowledged (do not need to be serviced by the host processor).
To use either linked or looped mode, both ACT_EN (Bit 0) and INACT_EN (Bit 2) must be set to 1;
otherwise, the default mode is used. For additional information, refer to the Linking Activity
and Inactivity Detection section  (datasheet p. 16).
- INACT_REF: Referenced/Absolute Inactivity Select.
  - 1 = inactivity detection function operates in referenced mode.
  - 0 = inactivity detection function operates in absolute mode.
- INACT_EN: Inactivity Enable.
  - 1 = enables the inactivity (underthreshold) functionality.
- ACT_REF: Referenced/Absolute Activity Select.
  - 1 = activity detection function operates in referenced mode.
  - 0 = activity detection function operates in absolute mode.
- ACT_EN: Activity Enable.
  - 1 = enables the activity (overthreshold) functionality.
*/
#define ADXL362_ACT_INACT_CTL_REG 0x27

/** \brief Registro de control de la FIFO
    \details

     D7       |      D6      |      D5      |      D4      |      D3      |      D2      |      D1      |      D0
--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------
      0       |       0      |      0       |       0      |      AH      |   FIFO_TEMP  |  FIFO_MODE   |   FIFO_MODE
      
- AH: Above Half. This bit is the MSB of the FIFO_SAMPLES register, allowing FIFO samples a range of 0 to 511.
- FIFO_TEMP: Store Temperature Data to FIFO.
  - 1 = temperature data is stored in the FIFO together with x-, y-, and z-axis acceleration data.
- FIFO_MODE: Enable FIFO and Mode Selection.
  - 00 FIFO is disabled.
  - 01 Oldest saved mode.
  - 10 Stream mode.
  - 11 Triggered mode.
*/
#define ADXL362_FIFO_CONTROL_REG 0x28

/** \brief    Registro de muestras de la FIFO. Especifica el número de muestras a guardar en la FIFO. D3 es el MSB.
    \details
    
     D7       |      D6      |      D5      |      D4      |      D3      |      D2      |      D1      |      D0
--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------
     X        |       X      |      X       |       X      |      AH      |       X      |      X       |       X       

- AH: Above Half. The following bit map is duplicated from the FIFO Control Register section to indicate the AH bit.
*/
#define ADXL362_FIFO_SAMPLES_REG 0x29

/** \brief Registro de configuración de interrupciones para INT1
    \details
    
     D7       |      D6      |      D5      |      D4      |      D3      |      D2      |      D1      |      D0
--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------
   INT_LOW    |     AWAKE    |    INACT     |     ACT      | FIFO_OVERRUN |FIFO_WATERMARK|  FIFO_READY  | DATA_READY   
     
- INT_LOW: 1 = INT1 pin is active low.
- AWAKE: 1 = maps the awake status to INT1 pin.
- INACT: 1 = maps the inactivity status to INT1 pin.
- ACT: 1 = maps the activity status to INT1 pin.
- FIFO_OVERRUN: 1 = maps the FIFO overrun status to INT1 pin.
- FIFO_WATERMARK: 1 = maps the FIFO watermark status to INT1 pin.
- FIFO_READY: 1 = maps the FIFO ready status to INT1 pin.
- DATA_READY: 1 = maps the data ready status to INT1 pin. 

*/
#define ADXL362_INTMAP1_REG 0x2A

/** \brief Registro de configuración de interrupciones para INT2
    \details
    
     D7       |      D6      |      D5      |      D4      |      D3      |      D2      |      D1      |      D0
--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------
   INT_LOW    |     AWAKE    |    INACT     |     ACT      | FIFO_OVERRUN |FIFO_WATERMARK|  FIFO_READY  | DATA_READY   
     
- INT_LOW: 1 = INT2 pin is active low.
- AWAKE: 1 = maps the awake status to INT2 pin.
- INACT: 1 = maps the inactivity status to INT2 pin.
- ACT: 1 = maps the activity status to INT2 pin.
- FIFO_OVERRUN: 1 = maps the FIFO overrun status to INT2 pin.
- FIFO_WATERMARK: 1 = maps the FIFO watermark status to INT2 pin.
- FIFO_READY: 1 = maps the FIFO ready status to INT2 pin.
- DATA_READY: 1 = maps the data ready status to INT2 pin. 

*/
#define ADXL362_INTMAP2_REG 0x2B

/** \brief Registro de configuración de filtro de control
    \details
    
     D7       |      D6      |      D5      |      D4      |      D3      |      D2      |      D1      |      D0
--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------
    RANGE	    |     RANGE    |      0       |    HALF_BW   |  EXT_SAMPLE  |     ODR      |     ODR      |     ODR      

- RANGE: Measurement Range Selection.
  - 00	±2 g (reset default)
  - 01	±4 g
  - 1X	±8 g  
- HALF_BW: Halved Bandwidth. Additional information is provided in the Antialiasing section (datasheet p.14).
  - 1 = the bandwidth of the antialiasing filters is set to 1/4 the output data rate (ODR) for more conservative filtering.
  - 0 = the bandwidth of the filters is set to 1/2 the ODR for a wider bandwidth.  
- EXT_SAMPLE: External Sampling Trigger.
  - 1 = the INT2 pin is used for external conversion timing control. Refer to the Using Synchronized Data Sampling section for more information (datasheet p. 40).
- ODR: Output Data Rate. Selects ODR and configures internal filters to a bandwidth of 1/2 or 1/4 the selected ODR, depending on the HALF_BW bit setting.
  - 000	=	12.5 Hz
  - 001	=	25 Hz
  - 010	=	50 Hz
  - 011	=	100 Hz (reset default)
  - 100	=	200 Hz
  - 101-111 =	400 Hz
  
*/
#define ADXL362_FILTER_CTL_REG 0x2C


/** \brief Registro de configuración de modos de consumo
    \details
    
     D7       |      D6      |      D5      |      D4      |      D3      |      D2      |      D1      |      D0
--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------
      0       |   EXT_CLK    |   LOW_NOISE  |  LOW_NOISE   |    WAKEUP    |  AUTOSLEEP   |    MEASURE   |   MEASURE    

- EXT_CLK: External Clock. See the Using an External Clock section for additional details.
  - 1 = the accelerometer runs off the external clock provided on the INT1 pin.
- LOW_NOISE: Selects Power vs. Noise Tradeoff:
  - 00	Normal operation (reset default).
  - 01	Low noise mode.
  - 10	Ultralow noise mode.
  - 11	Reserved.
- WAKEUP: Wake-Up Mode. See the Operating Modes section for a detailed description of wake-up mode.
  - 1 = the part operates in wake-up mode.
- AUTOSLEEP: Autosleep. Activity and inactivity detection must be in linked mode or loop mode
   (LINK/LOOP bits in ACT_INACT_CTL register) to enable autosleep; otherwise, the bit is ignored.
   See the Motion Detection section for details (datasheet p. 15).
  - 1 = autosleep is enabled, and the device enters wake-up mode automatically upon detection of inactivity.
- MEASURE: Selects Measurement Mode or Standby.
  - 00	Standby.
  - 01	Reserved.
  - 10	Measurement mode.
  - 11	Reserved.
*/
#define ADXL362_POWER_CTL_REG 0x2D

/** \brief Registro de configuración de self test
    \details
    
     D7       |      D6      |      D5      |      D4      |      D3      |      D2      |      D1      |      D0
--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------
     0        |      0       |      0       |      0       |      0       |      0       |      0       |      ST       
- ST: Self Test, 1 = a self test force is applied to the x-, y-, and z-axes.
*/
#define ADXL362_SELF_TEST_REG 0x2E

//Frecuencias del ODR
/** \brief Frecuencia del ODR de 400 Hz */
#define ADXL362_400HZ       0x05

/** \brief Frecuencia del ODR de 200 Hz */
#define ADXL362_200HZ       0x04

/** \brief Frecuencia del ODR de 100 Hz */
#define ADXL362_100HZ       0x03

/** \brief Frecuencia del ODR de 50 Hz */
#define ADXL362_50HZ        0x02

/** \brief Frecuencia del ODR de 25 Hz */
#define ADXL362_25HZ        0x01

/** \brief Frecuencia del ODR de 12.5 Hz */
#define ADXL362_12HZ5       0x00

//rangos de medida posibles
/** \brief  Rango de medida de 2 g
    \see    ADXL362_FILTER_CTL_REG */
#define ADXL362_RANGE_2G 0x0

/** \brief  Rango de medida de 4 g
    \see    ADXL362_FILTER_CTL_REG */
#define ADXL362_RANGE_4G 0x40

/** \brief  Rango de medida de 8 g
    \see    ADXL362_FILTER_CTL_REG */
#define ADXL362_RANGE_8G 0x80

//bits del registro ADXL362_STATUS_REG (origen de interrupción)
/** \brief posición 7 del registro ADXL362_STATUS_REG 
    \see ADXL362_STATUS_REG */
#define ADXL362_ERR_USER_REGS 0x80

/** \brief posición 6 del registro ADXL362_STATUS_REG
    \see ADXL362_STATUS_REG */
#define ADXL362_AWAKE 0x40

/** \brief posición 5 del registro ADXL362_STATUS_REG
    \see ADXL362_STATUS_REG */
#define ADXL362_INACT 0x20

/** \brief posición 4 del registro ADXL362_STATUS_REG
    \see ADXL362_STATUS_REG */
#define ADXL362_ACT 0x10

/** \brief posición 3 del registro ADXL362_STATUS_REG
    \see ADXL362_STATUS_REG */
#define ADXL362_FIFO_OVERRUN 0x08

/** \brief posición 2 del registro ADXL362_STATUS_REG
    \see ADXL362_STATUS_REG */
#define ADXL362_FIFO_WATERMARK 0x04

/** \brief posición 1 del registro ADXL362_STATUS_REG
    \see ADXL362_STATUS_REG */
#define ADXL362_FIFO_READY 0x02

/** \brief posición 0 del registro ADXL362_STATUS_REG 
    \see ADXL362_STATUS_REG */
#define ADXL362_DATA_READY 0x01


/** \brief código de reset */
#define ADXL362_RESET 0x52

/** \brief ID device */
#define ADXL362_DEVICE_ID	0xAD
/** \brief ID MEMS */
#define ADXL362_DEVICE_ID_MEMS 0x1D
/** \brief Part ID */
#define ADXL362_PART_ID 0xF2

/** \brief ADXL362 CS */
HAL_GPIO_PIN(ADXL362_CS, F, 3);

/*

typedef enum
{
  ADXL362_NORMAL,
  ADXL362_SLEEP
}  typeADXL362State_t2;
*/

/**
\brief Estados posibles de la máquina de estados del acelerómetro
*/
typedef enum
{
  /// Estado inicial, no se ha detectado no actividad ni caída libre
  ADXL362_STATE_INITIAL,
  /// Se ha detectado caída libre
  ADXL362_STATE_FREE_FALL,
  /// Se ha detectado caída libre continua
  ADXL362_STATE_CONTINUOUS_FREE_FALL,
  /// Se ha detectado un golpe tras la caída libre
  ADXL362_STATE_STRIKE,
  /// Se ha detectado inmovilidad tras la caída libre
  ADXL362_STATE_MOTIONLESS,
  /// Se ha detectado no actividad y hay que ir a dormir, no usado
  ADXL362_STATE_NO_ACTIVITY,
  /// Estado dormir, no usado
  ADXL362_STATE_SLEEP
} typeADXL362State_t;


/** \brief  Lectura de varios registros del acelerómetro.
    \param[in]  startAddress  dirección del primer registro.
    \param[in]  buffer        búffer donde alamacenar los datos leídos.
    \param[in]  size          número de registros a leer.
*/
void ADXL362_multiByteRead(uint8_t startAddress, uint8_t* buffer, uint8_t size);

/** \brief  Lectura de aceleración en el eje x.
    \return La aceleración en el eje x.
*/
int16_t ADXL362_getDataXMultiRead(void);

/** \brief  Lectura de aceleración en el eje y.
    \return La aceleración en el eje y.
*/
int16_t ADXL362_getDataYMultiRead(void);

/** \brief  Lectura de aceleración en el eje z.
    \return La aceleración en el eje z.
*/
int16_t ADXL362_getDataZMultiRead(void);


/** \brief  Calcula el offset para 8g con el valor de la batería pasado por parámetro.
    \param[in]  battery valor de la batería en dV.    
*/
void ADXL362_setOffset8G(uint8_t battery);

/** \brief  Lee size bytes de la cola FIFO.
    \param[in]  buffer  búffer donde se almacenan los datos leídos.
    \param[in]  size    nº de bytes a leer en el búffer.
    \todo       Función no implementada.
*/
void ADXL362_readFIFO (uint16_t* buffer, uint16_t size);

/** \brief  Lee un byte de la dirección pasada por parámetro.
    \param[in]  address Dirección de la que se va leer un byte.
    \return byte leído. */
uint8_t ADXL362_oneByteRead(uint8_t address);

/**  \brief Escribe un byte en la dirección pasada por parámetro.
     \param[in] address dirección en la que se quiere escribir un byte.
     \param[in] data    dato a escribir.*/
void ADXL362_oneByteWrite(uint8_t address, uint8_t data);

/** \brief Lee varios bytes desde la dirección pasada por parámetro.
    \param[in]  startAddress  dirección de inicio.
    \param[in]  buffer        puntero al búffer destino de los datos.
    \param[in]  size          cantidad de bytes a leer.
*/
//void ADXL362_multiByteRead(uint8_t startAddress, uint8_t* buffer, uint8_t size);

/** \brief Escribe varios bytes desde la dirección pasada por parámetro.
    \param[in]  startAddress  dirección de inicio.
    \param[in]  buffer        puntero al búffer con los datos a escribir.
    \param[in]  size          cantidad de bytes a escribir.
*/
void ADXL362_multiByteWrite(uint8_t startAddress, uint8_t* buffer, uint8_t size);

/** \brief  Devuelve el Analog Devices device ID.
    \return El Analog Devices device ID (0xAD).
*/
uint8_t ADXL362_getDevId(void);

/** \brief  Devuelve el Analog Devices MEMS device ID.
    \return El Analog Devices MEMS device ID (0x1D).
*/
uint8_t ADXL362_getDevIdMems(void);

/** \brief  Devuelve el device ID.
    \return El device ID (0xF2, 362 octal).
*/
uint8_t ADXL362_getPartId(void);

/** \brief  Devuelve el ID de revisión del producto.
    \return Devuelve el ID de revisión del producto.
*/
uint8_t ADXL362_getRevId(void);

/** \brief  Devuelve los 8 bits más significativos de la aceleración en el eje X.
    \return Los 8 bits más significativos de la aceleración en el eje X.
*/
uint8_t ADXL362_getXDataH(void);

/** \brief  Devuelve los 8 bits más significativos de la aceleración en el eje Y.
    \return Los 8 bits más significativos de la aceleración en el eje Y.
*/
uint8_t ADXL362_getYDataH(void);

/** \brief  Devuelve los 8 bits más significativos de la aceleración en el eje Z.
    \return Los 8 bits más significativos de la aceleración en el eje Z.
*/
uint8_t ADXL362_getZDataH(void);

/** \brief  Devuelve el contenido del registro ADXL362_STATUS_REG.
    \see ADXL362_STATUS_REG
    \return El contenido del registro ADXL362_STATUS_REG.

*/
uint8_t ADXL362_getStatus(void);

/** \brief  Devuelve el número de entradas en la FIFO.
    \return El número de entradas en la FIFO (entero sin signo de 10 bits).

*/
uint16_t ADXL362_getFifoEntries(void);

/** \brief  Devuelve la aceleración en el eje X.
    \return La aceleración en el eje X (entero con signo de 12 bits).
*/
int16_t ADXL362_getDataX(void);

/** \brief  Devuelve la aceleración en el eje Y.
    \return La aceleración en el eje Y (entero con signo de 12 bits).
*/
int16_t ADXL362_getDataY(void);

/** \brief  Devuelve la aceleración en el eje Z.
    \return La aceleración en el eje Z (entero con signo de 12 bits).
*/
int16_t ADXL362_getDataZ(void);

/** \brief  Devuelve la temperatura.
    \return El valor de la temperatura (entero con signo de 12 bits).
*/
uint16_t ADXL362_getTemp(void);

/** \brief Reinicia el sensor. */
void ADXL362_reset(void);

/** \brief  Devuelve el umbral de actividad.
    \return El umbral de actividad en mg (entero de 11 bits).
*/
uint16_t ADXL362_getActivityThreshold(void);

/** \brief      Asigna un umbral de actividad.
    \param[in]  activityThreshold El umbral de actividad que se quiere asignar en mg (entero de 11 bits).
*/
void ADXL362_setActivityThreshold(uint16_t activityThreshold);

/** \brief  Devuelve el tiempo de actividad mínimo de actividad.
    \return El tiempo mínimo de actividad en número de muestras. 
*/
uint8_t ADXL362_getTimeActivity(void);

/** \brief      Asigna el tiempo mínimo de actividad.
    \param[in]  timeActivity El tiempo mínimo de actividad en número de muestras. 
*/
void ADXL362_setTimeActivity(uint8_t timeActivity);

/** \brief  Devuelve el umbral de inactividad.
    \return El umbral de inactividad en mg (entero de 11 bits).
*/
uint16_t ADXL362_getInactivityThreshold(void);

/** \brief      Asigna un umbral de inactividad.
    \param[in]  inactivityThreshold El umbral de inactividad que se quiere asignar en mg (entero de 11 bits).
*/
void ADXL362_setInactivityThreshold(uint16_t inactivityThreshold);

/** \brief  Devuelve el tiempo mínimo de inactividad.
    \return El tiempo mínimo de inactividad en número de muestras. 
*/
uint16_t ADXL362_getTimeInactivity(void);

/** \brief      Asigna el tiempo mínimo para la inactividad.
    \param[in] timeInactivity  El tiempo mínimo de inactividad en número de muestras. 
*/
void ADXL362_setTimeInactivity(uint16_t timeInactivity);

/** \brief  Devuelve el contenido del registro  ADXL362_ACT_INACT_CTL_REG.
    \see ADXL362_ACT_INACT_CTL_REG
    \return El contenido del registro  ADXL362_ACT_INACT_CTL_REG.
    
*/
uint8_t ADXL362_getActivityInactivityControl(void);

/** \brief      Configura el registro ADXL362_ACT_INACT_CTL_REG.
    \param[in]  settings  configuración del registro ADXL362_ACT_INACT_CTL_REG.
    \see ADXL362_ACT_INACT_CTL_REG
*/
void ADXL362_setActivityInactivityControl(uint8_t settings);

/** \brief Devuelve el contenido del registro ADXL362_FIFO_CONTROL_REG.
    \see ADXL362_FIFO_CONTROL_REG    
    \return el contenido del registro ADXL362_FIFO_CONTROL_REG.
*/     
uint8_t ADXL362_getFifoControl(void);

/** \brief      Configura el registro ADXL362_FIFO_CONTROL_REG.
    \param[in]  fifoControl  configuración del registro ADXL362_FIFO_CONTROL_REG.
    \see ADXL362_FIFO_CONTROL_REG
*/
void ADXL362_setFifoControl(uint8_t fifoControl);

/** \brief  Devuelve el contenido del registro ADXL362_FIFO_SAMPLES_REG.
    \see    ADXL362_FIFO_SAMPLES_REG
    \return El contenido del registro ADXL362_FIFO_SAMPLES_REG.
*/
uint8_t ADXL362_getFifoSamples(void);

/** \brief      Configura el registro ADXL362_FIFO_SAMPLES_REG.
    \param[in]  fifoSamples configuración del registro ADXL362_FIFO_SAMPLES_REG.
    \see        ADXL362_FIFO_SAMPLES_REG
*/
void ADXL362_setFifoSamples(uint8_t fifoSamples);

/** \brief  Devuelve el contenido del registro ADXL362_INTMAP1_REG.
    \see    ADXL362_INTMAP1_REG
    \return El contenido del registro ADXL362_INTMAP1_REG.
*/
uint8_t ADXL362_getIntMap1(void);

/** \brief      Configura el registro ADXL362_INTMAP1_REG.
    \see        ADXL362_INTMAP1_REG
    \param[in]  settings  configuración del registro ADXL362_INTMAP1_REG.
*/
void ADXL362_setIntMap1(uint8_t settings);

/** \brief  Devuelve el contenido del registro ADXL362_INTMAP2_REG.
    \see    ADXL362_INTMAP2_REG
    \return El contenido del registro ADXL362_INTMAP2_REG.
*/
uint8_t ADXL362_getIntMap2(void);

/** \brief      Configura el registro ADXL362_INTMAP2_REG.
    \see        ADXL362_INTMAP2_REG
    \param[in]  settings  configuración del registro ADXL362_INTMAP2_REG.
*/
void ADXL362_setIntMap2(uint8_t settings);

/** \brief  Devuelve el contenido del registro ADXL362_FILTER_CTL_REG.
    \see    ADXL362_FILTER_CTL_REG
    \return El contenido del registro ADXL362_FILTER_CTL_REG.
*/
uint8_t ADXL362_getFilterControl(void);

/** \brief      Configura el registro ADXL362_FILTER_CTL_REG.
    \see        ADXL362_FILTER_CTL_REG
    \param[in]  settings  configuración del registro ADXL362_FILTER_CTL_REG.
*/
void ADXL362_setFilterControl(uint8_t settings);

/** \brief  Devuelve el contenido del registro ADXL362_POWER_CTL_REG.
    \see    ADXL362_POWER_CTL_REG
    \return El contenido del registro ADXL362_POWER_CTL_REG.
*/
uint8_t ADXL362_getPowerControl(void);

/** \brief      Configura el registro ADXL362_POWER_CTL_REG.
    \see        ADXL362_POWER_CTL_REG
    \param[in]  settings  configuración del registro ADXL362_POWER_CTL_REG.
*/
void ADXL362_setPowerControl(uint8_t settings);

/** \brief  Devuelve el contenido del registro ADXL362_SELF_TEST_REG.
    \see    ADXL362_SELF_TEST_REG
    \return El contenido del registro ADXL362_SELF_TEST_REG.
*/
uint8_t ADXL362_getSelfTest(void);

/** \brief      Configura el registro ADXL362_SELF_TEST_REG.
    \see        ADXL362_SELF_TEST_REG
    \param[in]  settings  configuración del registro ADXL362_SELF_TEST_REG.
*/
void ADXL362_setSelfTest(uint8_t settings);

/** \brief Devuelve un booleano que indica si está en una posible caída o no.
     \retval true si está detectando una caída.
     \retval false si no lo está. */
bool ADXL362_inFallDetection(void);

/** \brief Inicializa y configura el sensor.
    \param[in] funcionamiento  true si es un end-device, false si no lo es.
    \retval true  si el sensor ha sido configurado. 
    \retval false si el sensor no ha sido configurado. */
bool ADXL362_init(bool funcionamiento);

/** \brief  Devuelve si puede ir a dormir o no.
    \retval true  si puede ir a dormir.
    \retval false si no puede ir a dormir (ha detectado movimiento o caída libre).
*/
bool ADXL362_sleep(void);

/** \brief Prepara el acelerómetro para ir a dormir. */
void ADXL362_toSleep(void);

/** \brief Habilita la interrupción por actividad si es posible. */
void ADXL362_enableActivityInterrupt(void);

/** \brief Confiugura el acelerómetro para la detección de caídas. */
void ADXL362_setConfigFreeFall(void);

#endif /* _ADXL362_SENSOR_H */
// eof ADXL362.h
