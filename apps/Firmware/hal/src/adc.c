/**************************************************************************//**
\file  adc.c

\brief Implementación de la interfaz ADC.
******************************************************************************/
/******************************************************************************
*   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
*   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
******************************************************************************/

/******************************************************************************
Includes section
******************************************************************************/
#include "adc.h"


/******************************************************************************
Defines section
******************************************************************************/
/** \brief máscara para todos los canales */
#define ALL_CHANNEL_MASK          0x1F
/** \brief máscara para el canal 1 */
#define CHANNEL_MASK_1            0x01
/** \brief máscara para el canal 2 */
#define CHANNEL_MASK_2            0x03
/** \brief máscara para el canal 3 */
#define CHANNEL_MASK_3            0x04
/** \brief máscara para el canal 4 */
#define CHANNEL_MASK_4            0x0C
/** \brief delay para estabilizar */
#define DELAY_FOR_STABILIZE       125

/******************************************************************************
					Variables section
******************************************************************************/
/** \brief Descriptor del ADC */
HAL_AdcDescriptor_t *adcDesc = NULL;
/** \brief Divisores */
PROGMEM_DECLARE(const uint8_t halAdcDivider[5]) =  {3, 4, 5, 6, 7};
/******************************************************************************
                   Implementations section
******************************************************************************/

void halStartAdc(HAL_AdcChannelNumber_t channel)
{
  adcDesc->service.halAdcCurCount = 0;
  /* disable digital buffers */
  if (HAL_ADC_CHANNEL3 >= channel)
  {
    DIDR0 = (1 << channel);
  }
  else
  {
    if ((HAL_ADC_DIFF_CHANNEL0 == channel) || (HAL_ADC_DIFF_CHANNEL2 == channel))
      DIDR0 = CHANNEL_MASK_1;
    else if ((HAL_ADC_DIFF_CHANNEL1 == channel) || (HAL_ADC_DIFF_CHANNEL3 == channel))
      DIDR0 = CHANNEL_MASK_2;
    else if ((HAL_ADC_DIFF_CHANNEL4 == channel) || (HAL_ADC_DIFF_CHANNEL6 == channel))
      DIDR0 = CHANNEL_MASK_3;
    else if ((HAL_ADC_DIFF_CHANNEL5 == channel) || (HAL_ADC_DIFF_CHANNEL7 == channel))
      DIDR0 = CHANNEL_MASK_4;
  }
  uint8_t tmp = ADMUX & ALL_CHANNEL_MASK;
  /* clear previous channel number */
  ADMUX &= ~ALL_CHANNEL_MASK;
  /* set current channel number */
  ADMUX |= channel;
  /* if new differential channel is settled then must make 125 us delay for gain stabilize. */
  if ((tmp != channel) && (HAL_ADC_CHANNEL3 < channel))
    _delay_us(DELAY_FOR_STABILIZE);
  if (adcDesc->selectionsAmount > 1)
    ADCSRA |= ((1 << ADIE)  | (1 << ADATE) | (1 << ADSC));  // Starts running mode
  else
    ADCSRA |= ((1 << ADIE) | (1 << ADSC)); // Starts one conversion
}

int HAL_OpenAdc(HAL_AdcDescriptor_t *descriptor)
{
  if (adcDesc)
    return -1;
  adcDesc = descriptor;
  if ((NULL == adcDesc->bufferPointer) || (0 == adcDesc->selectionsAmount))
    return -1;
  if (adcDesc->resolution > RESOLUTION_10_BIT)
    return -1;
  /* unsupported voltage reference */
  if (adcDesc->voltageReference & 0x3F)
    return -1;
  /* adc speed must be only 9600 or 4800 SPS for 10 bit resolution */
  if ((RESOLUTION_10_BIT == adcDesc->resolution) && (adcDesc->sampleRate < ADC_9600SPS))
    return -1;
  /*mio c&p de halOpenAdc*/
  //halOpenAdc();
  /* sets voltage reference */
  ADMUX = adcDesc->voltageReference;
  /* Enable left adjust result */
  if (RESOLUTION_8_BIT == adcDesc->resolution)
    ADMUX |= (1 << ADLAR);
  uint8_t tmp;
  memcpy_P(&tmp, &(halAdcDivider[adcDesc->sampleRate]), 1);
  ADCSRA = tmp | (1 << ADEN);
  /*fin_mio*/
  return 0;
}

int HAL_ReadAdc(HAL_AdcDescriptor_t *descriptor, HAL_AdcChannelNumber_t channel)
{
  if (adcDesc != descriptor)
    return -1;
  if (((channel > HAL_ADC_CHANNEL3) && (channel < HAL_ADC_DIFF_CHANNEL0)) || (channel > HAL_ADC_DIFF_CHANNEL7))
    return -1;
  halStartAdc(channel);
  return 0;
}

int HAL_CloseAdc(HAL_AdcDescriptor_t *descriptor)
{
  if (adcDesc != descriptor)
    return -1;
  adcDesc = NULL;
  /*mio c&p de halOpenAdc*/
  //halCloseAdc();
  ADMUX  = 0;
  ADCSRA = 0;
  // Digital input enable
  DIDR0 = 0;
  /*fin_mio*/
  return 0;
}

void halSigAdcHandler(void)
{
  if (adcDesc->callback)
    adcDesc->callback();
}

/**************************************************************************//**
\brief interrupción por conversión del ADC terminada
******************************************************************************/
ISR(ADC_vect)
{
  // Read ADC conversion result
  if (RESOLUTION_8_BIT == adcDesc->resolution)
    ((uint8_t *)adcDesc->bufferPointer)[adcDesc->service.halAdcCurCount++] = ADCH;
  else
    ((uint16_t *)adcDesc->bufferPointer)[adcDesc->service.halAdcCurCount++] = ADC;
  if (adcDesc->service.halAdcCurCount == adcDesc->selectionsAmount)
  {
    // Disable ADC Interrupt
    ADCSRA &= ~(1 << ADIE);
    halSigAdcHandler();
  }
}

// eof adc.c
