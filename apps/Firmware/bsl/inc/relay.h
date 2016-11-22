/**
 \file relay.h

 \brief Header del driver para el control del rel�.
 */


#ifndef RELAY_H_
#define RELAY_H_

#include <halGpio.h>
#include <stdbool.h>

/** \brief Pin conectado al rel�. */
HAL_GPIO_PIN(RELAY, B, 5);

/** \brief Funci�n inline para encender el rel�. */
INLINE void relayOn(void)
{
  HAL_GPIO_RELAY_set();
}

/** \brief Funci�n inline para apagar el rel�. */
INLINE void relayOff(void)
{
  HAL_GPIO_RELAY_clr();
}

/** \brief      Funci�n inline para inicializar el rel�.
    \param[in]  isRelayOn  booleano que indica si el rel� tiene que inicializarse encendido o apagado.
*/
INLINE void relayInit(bool isRelayOn)
{
  HAL_GPIO_RELAY_out();
  if (isRelayOn)
    relayOn();
  else
    relayOff();  
}

#endif