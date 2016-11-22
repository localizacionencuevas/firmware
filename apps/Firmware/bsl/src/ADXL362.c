/**************************************************************************//**
\file  ADXL362.c

\brief Este módulo da acceso al acelerómetro ADXL362.
*****************************************************************************/
/******************************************************************************
                   Includes section
******************************************************************************/
#include "ADXL362.h"
#include "util/delay.h"
#include "halLed.h"
//#include "halGpio.h"

#ifdef ADXL362

//HAL_GPIO_PIN(IRQ_0, D, 0);

/** \brief rectas del offset dependiendo del voltaje para los 3 ejes. */
typedef struct offsetLines_t
{
  /** \brief recta para el eje x del acelerómetro. */
  struct
  {
    /** \brief intersección con el eje y. */
    int16_t yIntercept;
    /** \brief pendiente de la recta. */
    int16_t slope;
  } xAxis;
  /** \brief recta para el eje y del acelerómetro. */
  struct
  {
    /** \brief intersección con el eje y. */
    int16_t yIntercept;
    /** \brief pendiente de la recta. */
    int16_t slope;
  } yAxis;
  /** \brief recta para el eje z del acelerómetro. */
  struct
  {
    /** \brief intersección con el eje y. */
    int16_t yIntercept;
    /** \brief pendiente de la recta. */
    int16_t slope;
  } zAxis;
}  offsetLines_t;

// valor leído de la batería y utilizado para corregir las lecturas del voltaje
//uint8_t xBattery;

// estructura offset en los 3 ejes producido por la alimentación
/** \brief estructura para almacenar el offset en los 3 ejes*/
typedef struct offsetAxis_t
{
  /** \brief eje x*/
  int16_t x;
  /** \brief eje y*/
  int16_t y;
  /** \brief eje z*/
  int16_t z;
} offsetAxis_t;

/** \brief variable para almacenar las rectas de corrección para 8g. */
offsetLines_t offsetLines8G;
/** \brief offest en los 3 ejes producido por la alimentación a 8g. */
offsetAxis_t offset8G;

/** \brief descriptor SPI necesario para la comunicación con el acelerómetro. */
HAL_SpiDescriptor_t spiDesc;

/** \brief Indica si está detectando una caída libre. */
bool inFallDetection;

/** \brief  Indica si el acelerómetro ha interrumpido al mcu. */
bool accelerometerInt = false;

/** \brief  Estado del acelerómetro.
    \see    typeADXL362State_t */
typeADXL362State_t state;

/** \brief Timer para detección de caída libre */
static SYS_Timer_t freeFallTimer; 
/** \brief Timer para detección de impacto */
static SYS_Timer_t strikeTimer;
/** \brief Timer para detección de inmovilidad */
static SYS_Timer_t motionlessTimer;
/** \brief Timer para detección de inmovilidad de larga duración (posible pérdida de conocimiento) */
static SYS_Timer_t longMotionlessTimer;
/** \brief Timer para reinicio */ 
static SYS_Timer_t resetTimer;
/** \brief Timer para la lectura de aceleración al detectar una caída */
static SYS_Timer_t readFallTimer; 

/** \brief  Aceleración inicial de los ejex x, y, z */
int16_t InitialStatus[3];
/** \brief  Aceleración de los ejex x, y, z */
int16_t Acceleration[3]; 
/** \brief Calcula la diferencia  entre el estado inicial y la aceleración tras un golpe: Acceleration[] - Initial_Status[] */
int16_t DeltaAcceleration[3];
/** \brief Vector suma de DeltaAcceleration */
uint32_t DeltaVectorSum;

/*- Prototypes -------------------------------------------------------------*/
/** 
    \brief fin del temporizador readFallTimer. Lee la aceleración pasados 500ms desde el impacto contra el suelo. De esta forma se evita la influencia del filtro
    \param[in]  timer puntero al timer
*/
static void ADXL362_readFallTimerHandler(SYS_Timer_t *timer);

/** \brief      fin del temporizador freeFallTimer (detección de caída libre).
    \param[in]  timer puntero al timer
*/
static void ADXL362_freeFallTimerHandler(SYS_Timer_t *timer);

/** \brief      fin del temporizador strikeTimer para la detección del golpe contra el suelo
    \param[in]  timer puntero al timer
*/
static void ADXL362_strikeTimerHandler(SYS_Timer_t *timer);

/** \brief      fin del temporizador longMotionlessTimer para la detección de inactividad después de una caída (pérdida de conocimiento). No usado.
    \param[in]  timer puntero al timer
*/
static void ADXL362_longMotionlessTimerHandler(SYS_Timer_t *timer);

/** \brief      fin del temporizador motionlessTimer para la detección de inactividad después de una caída (sin pérdidad de conocimiento)
    \param[in]  timer puntero al itmer
*/
static void ADXL362_motionlessTimerHandler(SYS_Timer_t *timer);

/** \brief      fin del temporizador resetTimer para configurar el acelerómetro para la detección de caídas
    \param[in]  timer puntero al timer
*/
static void ADXL362_resetTimerHandler(SYS_Timer_t *timer);

/*- Implementations --------------------------------------------------------*/
void ADXL362_setOffset8G(uint8_t battery)
{
  offset8G.x = offsetLines8G.xAxis.yIntercept + offsetLines8G.xAxis.slope * battery;
  offset8G.y = offsetLines8G.yAxis.yIntercept + offsetLines8G.yAxis.slope * battery;
  offset8G.z = offsetLines8G.zAxis.yIntercept + offsetLines8G.zAxis.slope * battery;
}

uint8_t ADXL362_oneByteRead(uint8_t address)
{
  uint8_t rx = 0x00;
  uint8_t tx = (ADXL362_SPI_READ);  
  HAL_GPIO_ADXL362_CS_clr();
  int8_t escritura = HAL_WriteSpi(&spiDesc, (uint8_t *)&tx, 1);
  if (escritura == -1)
    return 0;
  //Send address to read from.
  escritura = HAL_WriteSpi(&spiDesc, (uint8_t *)&address, 1);
  if (escritura == -1)
    return 0;
  //Read back contents of address.
  HAL_ReadSpi(&spiDesc, (uint8_t *)&rx, 1);
  HAL_GPIO_ADXL362_CS_set();
  return rx;
}

void ADXL362_oneByteWrite(uint8_t address, uint8_t data)
{
  uint8_t tx = (ADXL362_SPI_WRITE);
  HAL_GPIO_ADXL362_CS_clr();
  HAL_WriteSpi(&spiDesc, (uint8_t *)&tx, 1);
  //Send address to write to.
  HAL_WriteSpi(&spiDesc, (uint8_t *)&address, 1);
  //Send data to be written.
  HAL_WriteSpi(&spiDesc, (uint8_t *)&data, 1);
  HAL_GPIO_ADXL362_CS_set();
}

void ADXL362_multiByteRead(uint8_t startAddress, uint8_t* buffer, uint8_t size)
{
  uint8_t tx = (ADXL362_SPI_READ);
  HAL_GPIO_ADXL362_CS_clr();
  HAL_WriteSpi(&spiDesc, (uint8_t *)&tx, 1);
  //Send address to start reading from.
  HAL_WriteSpi(&spiDesc, (uint8_t *)&startAddress, 1);
  //Read back multiple bytes of address.
  HAL_ReadSpi(&spiDesc, /*(uint8_t *)*/ buffer, size);
  HAL_GPIO_ADXL362_CS_set();
}

uint8_t ADXL362_getDevId(void)
{
  return ADXL362_oneByteRead(ADXL362_DEVID_REG);
}

uint8_t ADXL362_getDevIdMems(void)
{
  return ADXL362_oneByteRead(ADXL362_DEVID_MST_REG);
}

uint8_t ADXL362_getPartId(void)
{
  return ADXL362_oneByteRead(ADXL362_PARTID_REG);
}

uint8_t ADXL362_getRevId(void)
{
  return ADXL362_oneByteRead(ADXL362_REVID_REG);
}

uint8_t ADXL362_getXDataH(void)
{
  return ADXL362_oneByteRead(ADXL362_XDATA_REG);
}

uint8_t ADXL362_getYDataH(void)
{
  return ADXL362_oneByteRead(ADXL362_YDATA_REG);
}

uint8_t ADXL362_getZDataH(void)
{
  return ADXL362_oneByteRead(ADXL362_ZDATA_REG);
}

uint8_t ADXL362_getStatus(void)
{
  return ADXL362_oneByteRead(ADXL362_STATUS_REG);
}

uint16_t ADXL362_getFifoEntries(void)
{
  uint16_t numberEntries;
  numberEntries = (ADXL362_oneByteRead(ADXL362_FIFO_ENTRIES_H_REG) << 8) | (ADXL362_oneByteRead(ADXL362_FIFO_ENTRIES_L_REG));
  return numberEntries;
}

int16_t ADXL362_getDataXMultiRead(void)
{
  uint8_t buffer[2];
  ADXL362_multiByteRead(ADXL362_XDATA_L_REG, buffer, 2);
  return ((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0];

}

int16_t ADXL362_getDataYMultiRead(void)
{
  uint8_t buffer[2];
  ADXL362_multiByteRead(ADXL362_YDATA_L_REG, buffer, 2);
  return ((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0];
}

int16_t ADXL362_getDataZMultiRead(void)
{
  uint8_t buffer[2];
  ADXL362_multiByteRead(ADXL362_ZDATA_L_REG, buffer, 2);
  return ((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0];
}

int16_t ADXL362_getDataX(void)
{
  uint16_t dataX;
  dataX = (ADXL362_oneByteRead(ADXL362_XDATA_H_REG) << 8) | (ADXL362_oneByteRead(ADXL362_XDATA_L_REG));
  return dataX;
}

int16_t ADXL362_getDataY(void)
{
  uint16_t dataY;
  dataY = (ADXL362_oneByteRead(ADXL362_YDATA_H_REG) << 8) | (ADXL362_oneByteRead(ADXL362_YDATA_L_REG));
  return dataY;
}

int16_t ADXL362_getDataZ(void)
{
  uint16_t dataZ;
  dataZ = (ADXL362_oneByteRead(ADXL362_ZDATA_H_REG) << 8) | (ADXL362_oneByteRead(ADXL362_ZDATA_L_REG));
  return dataZ;
}

uint16_t ADXL362_getTemp(void)
{
  uint16_t dataTemp;
  dataTemp = (ADXL362_oneByteRead(ADXL362_TEMP_H_REG) << 8) | (ADXL362_oneByteRead(ADXL362_TEMP_L_REG));
  return dataTemp;
}

void ADXL362_reset(void)
{
  ADXL362_oneByteWrite(ADXL362_SOFT_RESET_REG, ADXL362_RESET);
}

/* 11 bits */
uint16_t ADXL362_getActivityThreshold(void)
{
  uint16_t activityThreshold;
  activityThreshold = (ADXL362_oneByteRead(ADXL362_THRESH_ACT_H_REG) << 8) | (ADXL362_oneByteRead(ADXL362_THRESH_ACT_L_REG));
  return activityThreshold;
}

void ADXL362_setActivityThreshold(uint16_t activityThreshold)
{
  uint8_t byte = 0xFF & activityThreshold;
  ADXL362_oneByteWrite(ADXL362_THRESH_ACT_L_REG, byte);

  byte = activityThreshold >> 8;
  ADXL362_oneByteWrite(ADXL362_THRESH_ACT_H_REG, byte);
}

uint8_t ADXL362_getTimeActivity(void)
{

  return ADXL362_oneByteRead(ADXL362_TIME_ACT_REG);
}

void ADXL362_setTimeActivity(uint8_t timeActivity)
{
  return ADXL362_oneByteWrite(ADXL362_TIME_ACT_REG, timeActivity);
}

/**/
/* 11 bits */
uint16_t ADXL362_getInactivityThreshold(void)
{
  uint16_t inactivityThreshold;
  inactivityThreshold = (ADXL362_oneByteRead(ADXL362_THRESH_INACT_H_REG) << 8) | (ADXL362_oneByteRead(ADXL362_THRESH_INACT_L_REG));
  return inactivityThreshold;
}

void ADXL362_setInactivityThreshold(uint16_t inactivityThreshold)
{
  uint8_t byte = 0xFF & inactivityThreshold;
  ADXL362_oneByteWrite(ADXL362_THRESH_INACT_L_REG, byte);

  byte = inactivityThreshold >> 8;
  ADXL362_oneByteWrite(ADXL362_THRESH_INACT_H_REG, byte);
}

uint16_t ADXL362_getTimeInactivity(void)
{
  uint16_t inactivityTime;
  inactivityTime = (ADXL362_oneByteRead(ADXL362_TIME_INACT_H_REG) << 8) | (ADXL362_oneByteRead(ADXL362_TIME_INACT_L_REG));
  return inactivityTime;
}

void ADXL362_setTimeInactivity(uint16_t timeInactivity)
{
  uint8_t byte = (timeInactivity >> 8);
  ADXL362_oneByteWrite(ADXL362_TIME_INACT_H_REG, byte);

  byte = timeInactivity & 0xFF;
  ADXL362_oneByteWrite(ADXL362_TIME_INACT_L_REG, byte);
}

uint8_t ADXL362_getActivityInactivityControl(void)
{
  return ADXL362_oneByteRead(ADXL362_ACT_INACT_CTL_REG);
}

void ADXL362_setActivityInactivityControl(uint8_t settings)
{
  ADXL362_oneByteWrite(ADXL362_ACT_INACT_CTL_REG, settings);
}

uint8_t ADXL362_getFifoControl(void)
{
  return ADXL362_oneByteRead(ADXL362_FIFO_CONTROL_REG);
}

void ADXL362_setFifoControl(uint8_t fifoControl)
{
  ADXL362_oneByteWrite(ADXL362_FIFO_CONTROL_REG, fifoControl);
}

uint8_t ADXL362_getFifoSamples(void)
{
  return ADXL362_oneByteRead(ADXL362_FIFO_SAMPLES_REG);
}

void ADXL362_setFifoSamples(uint8_t fifoSamples)
{
  ADXL362_oneByteWrite(ADXL362_FIFO_SAMPLES_REG, fifoSamples);
}

uint8_t ADXL362_getIntMap1(void)
{
  return ADXL362_oneByteRead(ADXL362_INTMAP1_REG);
}

void ADXL362_setIntMap1(uint8_t settings)
{
  ADXL362_oneByteWrite(ADXL362_INTMAP1_REG, settings);
}

uint8_t ADXL362_getIntMap2(void)
{
  return ADXL362_oneByteRead(ADXL362_INTMAP2_REG);
}

void ADXL362_setIntMap2(uint8_t settings)
{
  ADXL362_oneByteWrite(ADXL362_INTMAP2_REG, settings);
}

uint8_t ADXL362_getFilterControl(void)
{
  return ADXL362_oneByteRead(ADXL362_FILTER_CTL_REG);
}

void ADXL362_setFilterControl(uint8_t settings)
{
  ADXL362_oneByteWrite(ADXL362_FILTER_CTL_REG, settings);
}

uint8_t ADXL362_getPowerControl(void)
{
  return ADXL362_oneByteRead(ADXL362_POWER_CTL_REG);
}

void ADXL362_setPowerControl(uint8_t settings)
{
  ADXL362_oneByteWrite(ADXL362_POWER_CTL_REG, settings);
}

uint8_t ADXL362_getSelfTest(void)
{
  return ADXL362_oneByteRead(ADXL362_SELF_TEST_REG);
}

void ADXL362_setSelfTest(uint8_t settings)
{
  ADXL362_oneByteWrite(ADXL362_SELF_TEST_REG, settings);
}

bool ADXL362_inFallDetection(void)
{
  return inFallDetection;
}

void ADXL362_setConfigFreeFall(void)
{
  EIMSK &= ~(1<<INT7);
  inFallDetection = false;
  state = ADXL362_STATE_INITIAL;
  //sleep = false;
  //wakeup = false;
  //ver configuración de caída libre en la pág. 37 del datasheet
  //configuramos los registros del acelerómetro
  if(accelerometerInt)
    ADXL362_setActivityInactivityControl(0x06); //default mode, absolute inactivity mode, absolute activity mode, and Inactivity enable, activity disable
  else
    ADXL362_setActivityInactivityControl(0x07); //default mode, absolute inactivity mode, referenced activity mode, and Inactivity enable, activity enable
  ADXL362_setFifoControl(0x0);
  
  ADXL362_setFilterControl(0x83); //8 g range, Conservative anti-aliasing, and 100 Hz ODR  /**//*0x93/*0x53/**//* 0x13/**/

  //ADXL362_setPowerControl(0x0A); //Wakeup mode, and Measurement mode
  ADXL362_setPowerControl(0x02); //Measurement mode
  
  //umbrales de actividad/inactividad
  ADXL362_setActivityThreshold(13); 

  ADXL362_setTimeActivity(2/*5*/); //2 samples 
  
  //ADXL362_setInactivityThreshold(75); //300 mg
  //ADXL362_setInactivityThreshold(100); //400 mg
  ADXL362_setInactivityThreshold(112); //448 mg
  ADXL362_setTimeInactivity(17/*20*/); //35//35ms
  _delay_ms(50);
  ADXL362_getStatus();

  //INT_MAPs, todo a int2
  ADXL362_getIntMap1();
  ADXL362_setIntMap1(0x00);
  ADXL362_getIntMap2();
  ADXL362_setIntMap2(0xB0);  //0xB0 para actividad/inactividad
  ADXL362_getStatus();

  //interrupciones
  HAL_GPIO_IRQ_7_in();
  EIMSK |= (1 << INT7);
}

static void ADXL362_readFallTimerHandler(SYS_Timer_t *timer)
{
      //done: obtenemos los datos de aceleracion y comparamos con los datos normales
      Acceleration[0] = ADXL362_getDataXMultiRead() - offset8G.x;
      Acceleration[1] = ADXL362_getDataYMultiRead() - offset8G.y;
      Acceleration[2] = ADXL362_getDataZMultiRead() - offset8G.z;

      DeltaVectorSum = 0;
      for(uint8_t i = 0; i < 3; i++)
      {
        if(Acceleration[i] > InitialStatus[i])
        {
          DeltaAcceleration[i] = Acceleration[i] - InitialStatus[i];
        }
        else
        {
          DeltaAcceleration[i] = InitialStatus[i] - Acceleration[i];
        }
        DeltaVectorSum = DeltaVectorSum + DeltaAcceleration[i] * DeltaAcceleration[i];
      }
      //done: DELTA_VECTOR_SUM_THRESHOLD = 0x77A1, a 8g 0x77A1 = (700mg/4)**2, 0x3D09 = (500/4)**2
      if (
          //prueba caída de espaldas o bajar el umbral? a 0x6726 = (650mg/4)**2, de momento Acceleration[1] < 10
          Acceleration[1] < 10 /*¿10LSB?*/ ||       
          DeltaVectorSum > /*DELTA_VECTOR_SUM_THRESHOLD*/ /*0x77A1*/ 0x6726 /*0x3D09*//*5625*/)// The stable status is different from the initial status
      {
        //valid fall detection
        state = ADXL362_STATE_MOTIONLESS;
        //todo:configuracion del umbral de actividad
        //ADXL345_setActivityThreshold(STABLE_THRESHOLD);
        //caida normal
        
        typeAlarm = ALARM_FALL;
        despierta = true;
        alarm = true;
#ifdef DEBUG
        HAL_LedOn(2);
#endif
        //tiempo de inactividad
        //ADXL345_setTimeInactivity(NOMOVEMENT_TIME);
        //ADXL362_setTimeInactivity(1000); //10 s = 1000 LSB a 100 Hz

        //SYS_TimerStart(&longMotionlessTimer);
        //modo de funcionamiento de referencia
        //ADXL345_setActivityInactivityControl(0xFF); //ac-coupled for activity, ac-coupled for inactivity
        //ADXL362_setActivityInactivityControl(0x0F); //inactividad modo de referencia, actividad en modo referencia
        ADXL362_setConfigFreeFall();
      }
      else
      {
        ADXL362_setConfigFreeFall();
      }
}

static void ADXL362_freeFallTimerHandler(SYS_Timer_t *timer)
{
  SYS_TimerStart(&resetTimer);
  (void)timer;
}

static void ADXL362_strikeTimerHandler(SYS_Timer_t *timer)
{
  SYS_TimerStart(&resetTimer);
  (void)timer;
}

static void ADXL362_longMotionlessTimerHandler(SYS_Timer_t *timer)
{
  (void)timer;
}

static void ADXL362_motionlessTimerHandler(SYS_Timer_t *timer)
{
  ADXL362_setConfigFreeFall();
  (void)timer;
}

static void ADXL362_resetTimerHandler(SYS_Timer_t *timer)
{
  ADXL362_setConfigFreeFall();
  (void)timer;
}

void ADXL362_toSleep(void)
{
  ADXL362_setActivityInactivityControl(0x07);
  ADXL362_setActivityThreshold(ACCEL_WAKE_UP_TRESHOLD); 
}

void ADXL362_enableActivityInterrupt(void)
{
  if(state == ADXL362_STATE_INITIAL)
  {
    accelerometerInt = false;
    ATOMIC_SECTION_ENTER
    ADXL362_setActivityInactivityControl(0x07); //default mode, absolute inactivity mode, absolute activity mode, and Inactivity enable, activity enable
    ATOMIC_SECTION_LEAVE
  }
  else
    accelerometerInt = true;
}

//done:cambiar
bool ADXL362_sleep(void)
{
  return !accelerometerInt;
}


bool ADXL362_init(bool funcionamiento)
{

  //func = funcionamiento;
  if (funcionamiento) // si es 1-> enddevice
  {   
    //delante y antena hacia arriba
    InitialStatus[0] = 0;
    InitialStatus[1] = 200;
    InitialStatus[2] = 150;

    freeFallTimer.interval = 270; //430; //200; //430; //0.5*9.8*0.430^2 = 0.90m
    freeFallTimer.mode = SYS_TIMER_INTERVAL_MODE;
    freeFallTimer.handler = ADXL362_freeFallTimerHandler;
    strikeTimer.interval = 200;//100;
    strikeTimer.mode = SYS_TIMER_INTERVAL_MODE;
    strikeTimer.handler = ADXL362_strikeTimerHandler;
    motionlessTimer.interval = 3500;
    motionlessTimer.mode = SYS_TIMER_INTERVAL_MODE;
    motionlessTimer.handler = ADXL362_motionlessTimerHandler;
    longMotionlessTimer.interval = 10000;
    longMotionlessTimer.mode = SYS_TIMER_INTERVAL_MODE;
    longMotionlessTimer.handler = ADXL362_longMotionlessTimerHandler;
    resetTimer.interval = 3000;//300;//freeFallTimer.interval-strikeTimer.interval+10; //100;
    resetTimer.mode = SYS_TIMER_INTERVAL_MODE;
    resetTimer.handler = ADXL362_resetTimerHandler;

    readFallTimer.interval = 500;
    readFallTimer.mode = SYS_TIMER_INTERVAL_MODE;
    readFallTimer.handler = ADXL362_readFallTimerHandler;
    
    //offest inicial
    offset8G.x = 0;
    offset8G.y = 0;
    offset8G.z = 0;

    //rectas de offset
    //DONE: rellenar con valores
    offsetLines8G.xAxis.slope = ADXL362_OFFSET_8G_X_SLOPE;
    offsetLines8G.xAxis.yIntercept = ADXL362_OFFSET_8G_X_INTERCEPT;
    offsetLines8G.yAxis.slope = ADXL362_OFFSET_8G_Y_SLOPE;
    offsetLines8G.yAxis.yIntercept = ADXL362_OFFSET_8G_Y_INTERCEPT;
    offsetLines8G.zAxis.slope = ADXL362_OFFSET_8G_Z_SLOPE;
    offsetLines8G.zAxis.yIntercept = ADXL362_OFFSET_8G_Z_INTERCEPT;
    
    accelerometerInt = false;

    //configuración SPI
    spiDesc.tty       = SPI_CHANNEL_0;
    spiDesc.baudRate  = SPI_CLOCK_RATE_1000;
    spiDesc.clockMode = SPI_CLOCK_MODE0; //SPI_CLOCK_MODE3;
    spiDesc.dataOrder = SPI_DATA_MSB_FIRST;
    spiDesc.callback  = NULL;
    if (-1 != HAL_OpenSpi(&spiDesc))
    {
      uint8_t lectura;

      //configuración de CS
      HAL_GPIO_ADXL362_CS_out();
      HAL_GPIO_ADXL362_CS_pullup();
      //ponemos CS a 1
      HAL_GPIO_ADXL362_CS_set();

      //reset
      ADXL362_reset();
      _delay_ms(1);

      lectura = ADXL362_getDevId();
      //es lo esperado
      if (lectura == ADXL362_DEVICE_ID)
      {
        //ADXL362_setConfigFreeFall();
        return true;
      }
      else
      {
        ADXL362_reset();
        _delay_ms(1);
      }
    }
  }
  return false;
}
/** \brief Atiende a las distintas interrupciones generadas por el acelerómetro. */
ISR(INT7_vect)
{
  uint8_t status = ADXL362_getStatus();
  switch (state)
  {
  case ADXL362_STATE_INITIAL:
  {
    if ((status & ADXL362_INACT_INT) == ADXL362_INACT_INT)
    {
      //no pasamos a dormir
      //caída libre
      state = ADXL362_STATE_FREE_FALL;

      despierta = true; //despertamos el micro para controlar los tiempos
      //longInactivity = false;
      inFallDetection = true;
      //activar umbral de actividad
      //timers de freeFall y de strike
      //porrazo con el suelo, no hace falta quitar offset
      ADXL362_setActivityThreshold(437); //a 8g 1LSB = 4mg, 437*4 = 1748mg
      //ADXL362_setActivityThreshold(300); //a 8g 1LSB = 4mg, 300*4 = 1200mg
      ADXL362_setTimeActivity(1); //3//1
      ADXL362_setActivityInactivityControl(0x07); //default mode, absolute inactivity mode, referenced activity mode, and Inactivity enable, activity enable

      SYS_TimerStart(&freeFallTimer); //430ms
      SYS_TimerStart(&strikeTimer);   //200ms
      accelerometerInt = true;
    }
    else if ((status & ADXL362_ACT_INT) == ADXL362_ACT_INT)
    {
      //se ha detectado actividad, luego no pasamos a dormir
      accelerometerInt = true;
      //done: deshabilitamos la interrupción por actividad
      ADXL362_setActivityInactivityControl(0x06); //default mode, absolute inactivity mode, absolute activity mode, and Inactivity enable, activity disable
    }
  }
  break;

  case ADXL362_STATE_FREE_FALL:
  {
    if ((status & ADXL362_INACT_INT) == ADXL362_INACT_INT)
    {
      if (!SYS_TimerStarted(&freeFallTimer))
      {
        //hay que mirar que la lectura anterior se hizo hace menos de 100ms
        SYS_TimerStop(&strikeTimer);
        SYS_TimerStop(&resetTimer);
        SYS_TimerStart(&resetTimer);
        state = ADXL362_STATE_CONTINUOUS_FREE_FALL;
#ifdef DEBUG
        HAL_LedOn(2);
#endif
        // si el contador no sigue funcionando es que se ha detectado una caída desde una altura considerable
        despierta = true;
        alarm = true;
        typeAlarm = ALARM_CONTINUOUS_FREEFALL;
      }
      else
      {
        SYS_TimerStop(&resetTimer);
        SYS_TimerStop(&strikeTimer);
        SYS_TimerStart(&strikeTimer);
      }
    }

    else if ((status & ADXL362_ACT_INT) == ADXL362_ACT_INT)
    {
      SYS_TimerStop(&freeFallTimer);
      if (SYS_TimerStarted(&strikeTimer))
      {
        SYS_TimerStop(&strikeTimer);
        state = ADXL362_STATE_STRIKE;
        //HAL_LedOn(2); //todo: quitar
        ADXL362_setActivityThreshold(125); //500mg = 125 LSB a 8g
        ADXL362_setInactivityThreshold(63); //250mg = 63 LSB a 8g
        
        SYS_TimerStart(&motionlessTimer);

        //modo de referencia para ambos
        //ADXL345_setActivityInactivityControl(0xFF); //ac-coupled for activity, ac-coupled for inactivity //modo de referencia

        //interrupciones por inactividad
        //ADXL345_setInterruptEnableControl((uint8_t)(/*ADXL345_DOUBLETAP|*/ADXL345_FREEFALL|ADXL345_INACTIVITY));
        ADXL362_setActivityInactivityControl(0x0E); //inactividad modo de referencia, actividad desactivado
      }
    }
  }
  break;

  case ADXL362_STATE_CONTINUOUS_FREE_FALL:
  {
    if ((status & ADXL362_INACT_INT) == ADXL362_INACT_INT)
    {
      SYS_TimerStop(&resetTimer);  // si seguimos detectando caída libre
      SYS_TimerStart(&resetTimer); // reiniciamos el timer del reset
    }
  }
  break;

  case ADXL362_STATE_STRIKE:
  {
    if ((status & ADXL362_INACT_INT) == ADXL362_INACT_INT)
    {
      // Waiting for stable, and now stable is detected
      SYS_TimerStop(&motionlessTimer);
      //todo timer 500 ms para comprobar orientación
      SYS_TimerStart(&readFallTimer);
    }
  }
  break;
  
  /**/
  case ADXL362_STATE_MOTIONLESS:
  {
    if ((status & ADXL362_INACT_INT) == ADXL362_INACT_INT)
    {
      // Wait for long time motionless, and now it is detected
      if (!SYS_TimerStarted(&longMotionlessTimer))
      {
        typeAlarm = ALARM_FALL_MOTIONLESS;
        despierta = true;
        alarm = true;
        //reiniciamos el acelerómetro
        ADXL362_setConfigFreeFall();
      }
    }
    else if ((status & ADXL362_ACT_INT) == ADXL362_ACT_INT)//restart
    {
      SYS_TimerStop(&longMotionlessTimer);
      ADXL362_setConfigFreeFall();
    }
  }
  break;
  
  default:
    break;
  }
}

#endif