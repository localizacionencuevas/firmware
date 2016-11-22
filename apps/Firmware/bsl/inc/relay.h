/**
 \file relay.h

 \brief Header del driver para el control del relé.
 */


#ifndef RELAY_H_
#define RELAY_H_

#include <halGpio.h>
#include <stdbool.h>

/** \brief Pin conectado al relé. */
HAL_GPIO_PIN(RELAY, B, 5);

/** \brief Función inline para encender el relé. */
INLINE void relayOn(void)
{
  HAL_GPIO_RELAY_set();
}

/** \brief Función inline para apagar el relé. */
INLINE void relayOff(void)
{
  HAL_GPIO_RELAY_clr();
}

/** \brief      Función inline para inicializar el relé.
    \param[in]  isRelayOn  booleano que indica si el relé tiene que inicializarse encendido o apagado.
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