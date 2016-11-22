/**************************************************************************//**
\file  ADXL345.h

\brief Declarations of ADXL345 interface.

\author
   Rubén Barrilero - UCLM

\internal
  History:
    09/03/11 R. Barrilero - Created
*******************************************************************************/

#ifndef _ADXL345_H
#define _ADXL345_H

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

/******************************************************************************
                   Types section
******************************************************************************/
// \cond
typedef void (*BSP_AccelerometerEventFunc_t)(uint8_t);
// \endcond

/******************************************************************************
                   Define(s) section
******************************************************************************/
#define ADXL345_DEVID_REG          0x00
#define ADXL345_THRESH_TAP_REG     0x1D
#define ADXL345_OFSX_REG           0x1E
#define ADXL345_OFSY_REG           0x1F
#define ADXL345_OFSZ_REG           0x20
#define ADXL345_DUR_REG            0x21
#define ADXL345_LATENT_REG         0x22
#define ADXL345_WINDOW_REG         0x23
#define ADXL345_THRESH_ACT_REG     0x24
#define ADXL345_THRESH_INACT_REG   0x25
#define ADXL345_TIME_INACT_REG     0x26
#define ADXL345_ACT_INACT_CTL_REG  0x27
#define ADXL345_THRESH_FF_REG      0x28
#define ADXL345_TIME_FF_REG        0x29
#define ADXL345_TAP_AXES_REG       0x2A
#define ADXL345_ACT_TAP_STATUS_REG 0x2B
#define ADXL345_BW_RATE_REG        0x2C
#define ADXL345_POWER_CTL_REG      0x2D
#define ADXL345_INT_ENABLE_REG     0x2E
#define ADXL345_INT_MAP_REG        0x2F
#define ADXL345_INT_SOURCE_REG     0x30
#define ADXL345_DATA_FORMAT_REG    0x31
#define ADXL345_DATAX0_REG         0x32
#define ADXL345_DATAX1_REG         0x33
#define ADXL345_DATAY0_REG         0x34
#define ADXL345_DATAY1_REG         0x35
#define ADXL345_DATAZ0_REG         0x36
#define ADXL345_DATAZ1_REG         0x37
#define ADXL345_FIFO_CTL           0x38
#define ADXL345_FIFO_STATUS        0x39
 
//Data rate codes.
#define ADXL345_3200HZ      0x0F
#define ADXL345_1600HZ      0x0E
#define ADXL345_800HZ       0x0D
#define ADXL345_400HZ       0x0C
#define ADXL345_200HZ       0x0B
#define ADXL345_100HZ       0x0A
#define ADXL345_50HZ        0x09
#define ADXL345_25HZ        0x08
#define ADXL345_12HZ5       0x07
#define ADXL345_6HZ25       0x06

#define ADXL345_SPI_READ    0x80
#define ADXL345_SPI_WRITE   0x00
#define ADXL345_MULTI_BYTE  0x40

#define ADXL345_DEVICE_ID	0xE5

//IRQ sources
#define ADXL345_OVERRUN              0x01
#define ADXL345_WATERMARK            0x02
#define ADXL345_FREEFALL             0x04
#define ADXL345_INACTIVITY           0x08
#define ADXL345_ACTIVITY             0x10
#define ADXL345_DOUBLETAP            0x20
#define ADXL345_SINGLETAP            0x40
#define ADXL345_DATAREADY            0x80

#define ADXL345_OVERRUN_DESP         0x00
#define ADXL345_WATERMARK_DESP       0x01
#define ADXL345_FREEFALL_DESP        0x02
#define ADXL345_INACTIVITY_DESP      0x03
#define ADXL345_ACTIVITY_DESP        0x04
#define ADXL345_DOUBLETAP_DESP       0x05
#define ADXL345_SINGLETAP_DESP       0x06
#define ADXL345_DATAREADY_DESP       0x07


//configuracion para deteccion de caida libre
#define STRIKE_THRESHOLD 0x1C//0x20 //62.5mg/LSB, 0x20=2g
#define STRIKE_WINDOW 0x0A //20ms/LSB, 0x0A=10=200ms
#define STABLE_THRESHOLD 0x08 //62.5mg/LSB, 0x10=0.5g
#define STABLE_TIME 0x02 //1s/LSB, 0x02=2s
#define STABLE_WINDOW 0xAF //20ms/LSB, 0xAF=175=3.5s
#define NOMOVEMENT_THRESHOLD 0x04//0x03 //62.5mg/LSB, 0x03=0.1875g
#define NOMOVEMENT_TIME 0x0A //1s/LSB, 0x0A=10s
#define FREE_FALL_THRESHOLD 0x09//0x0C //62.5mg/LSB, 0x0C=0.75g
#define FREE_FALL_TIME 0x07//0x10//0x06 //5ms/LSB, 0x06=30ms
#define FREE_FALL_OVERTIME 0x0F //20ms/LSB, 0x0F=15=300ms
#define FREE_FALL_INTERVAL 0x05 //20ms/LSB, 0x05=100ms
#define DELTA_VECTOR_SUM_THRESHOLD 0x7D70 //1g=0xFF, 0x7D70=0.7g^2
#define LONG_INACTIVITY_TIME 0xF0 //1s/LSB, 0xF0= 240s = 4 min.
#define LONG_INACTIVITY_THRESHOLD 0x02 //62.5mg/LSB, 0x02=0.125g
#define WAKE_UP_THRESHOLD 0x02 //62.5mg/LSB, 0x02=0.125g

HAL_GPIO_PIN(ADXL345_CS, F, 3);

typedef enum
{
	ADXL345_X,
	ADXL345_Y,
	ADXL345_Z
}Axys_e;

typedef enum{
	STATE_INITIAL,
	STATE_FREE_FALL,
	STATE_STRIKE,
	STATE_MOTIONLESS,
	STATE_CONTINUOUS_FREE_FALL,
}StateFreeFall_e;

typedef struct  
{
	int16_t x;
	int16_t y;
	int16_t z;
}Data_Accelerometer_s;



/******************************************************************************
                   Prototypes section
******************************************************************************/

void ADXL345_ReadAxis (void);
bool ADXL345_init(bool funcionamiento);
void ADXL345_setConfigFreeFall(void);
bool ADXL345_inFallDetection(void);
bool ADXL345_inLongInactivity(void);
void ADXL345_sleep(void);
int16_t ADXL345_getXValue(void);
int16_t ADXL345_getYValue(void);
int16_t ADXL345_getZValue(void);


#endif /* _ADXL345_SENSOR_H */
// eof ADXL345.h
