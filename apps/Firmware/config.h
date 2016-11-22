/**
 \file config.h
 
 \brief Archivo de configuraci�n
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

#ifndef DOXYGEN
/*- Definitions ------------------------------------------------------------*/

/* Direcci�n del nodo. Si no est� definido aqu� toma los datos del bootloader*/
//#define APP_ADDR                0x1005 //si no est� definido aqu� toma los datos del bootloader
/* Panid del nodo. Si no est� definido aqu� toma los datos del bootloader*/
//#define APP_PANID               0x534E //si no est� definido aqu� toma los datos del bootloader
/* Canal del nodo. Si no est� definido aqu� toma los datos del bootloader*/
//#define APP_CHANNEL             0x0F   //si no est� definido aqu� toma los datos del bootloader 

/* end-point de la aplicaci�n. */
#define APP_ENDPOINT          1
/* end-point de las tramas de actualizaciones OTA. */
#define APP_OTA_ENDPOINT      2
/* end-point de las tramas de comandos. */
#define APP_COMMAND_ENDPOINT	3
/* end-point de las tramas de lectura de sensores. */
#define APP_SENSORS_ENDPOINT	4
/* end-point de las tramas de versi�n. */
#define APP_VERSION_ENDPOINT	5
/* end-point de la primera trama de los nodos con rel�. */
#define APP_RELAY_ENDPOINT    6

/* Clave de seguridad. Debe ser un string de 16 caracteres. */
#define APP_SECURITY_KEY        "3u2VCfeTForPCZlk"

/* N�mero de canal de la UART. */
#define HAL_UART_CHANNEL        1
/* Tama�o del b�ffer RX de la UART. */
#define HAL_UART_RX_FIFO_SIZE   1
/* Tama�o del b�ffer TX de la UART. */
#define HAL_UART_TX_FIFO_SIZE   100

/* Activa/desactiva el env�o de tramas de localizaci�n de otras subredes: 1 solo para la misma subred, 0 env�a tambi�n de otras subredes*/
#define SEND_LOC_ONLY_SUBNET 1

/* OTA habilitada. */
#define APP_ENABLE_OTA

//#define PHY_ENABLE_RANDOM_NUMBER_GENERATOR

/* N�mero de b�ffers NWK. */
#define NWK_BUFFERS_AMOUNT                  10
/* Tama�o de la tabla de duplicados rechazados. */
#define NWK_DUPLICATE_REJECTION_TABLE_SIZE  50
/* Permanencia de los duplicados rechazados en la tabla.*/
#define NWK_DUPLICATE_REJECTION_TTL         2000 // ms
/* Tama�o de la tabla de enrutado.*/
#define NWK_ROUTE_TABLE_SIZE                100
/* Puntuaci�n por defecto de una ruta. */
#define NWK_ROUTE_DEFAULT_SCORE             3
/* Tiempo de espera de un NWK ACK. */
#define NWK_ACK_WAIT_TIME                   1000 // ms
/* N�mero de grupos de direcciones (multicast). */
#define NWK_GROUPS_AMOUNT                   3
/* Tama�o para el algoritmo de route discovery (debe estar definido NWK_ENABLE_ROUTE_DISCOVERY). */
#define NWK_ROUTE_DISCOVERY_TABLE_SIZE      5
/* Tiempo para el algoritmo de route discovery (debe estar definido NWK_ENABLE_ROUTE_DISCOVERY). */
#define NWK_ROUTE_DISCOVERY_TIMEOUT         1000 // ms

/* Habilita el enrutado. */
#define NWK_ENABLE_ROUTING

/* Modo de seguridad. 1 por SW, 0 por HW. */
#define SYS_SECURITY_MODE                   1

/* Habilita la seguridad */
#define NWK_ENABLE_SECURITY

//#define NWK_ENABLE_ROUTE_DISCOVERY

/* Tiempo entre env�o de tramas de coordinadores. */
#define APP_COORDINATOR_SENDING_INTERVAL 60000 //ms
/* Tiempo entre env�o de tramas de lectura de sensores de routers. */
#define APP_ROUTER_SENSOR_INTERVAL 1800000 //ms
/* N�mero de tramas de localizaci�n entre env�o de tramas de lectura de sensores de routers. */
#define APP_ROUTER_NUMBER_BEACONS 9 //tramas de localizaci�n entre env�o de tramas de lectura de sensores
/* Tiempo entre env�o de tramas de end-devices. */
#define APP_ENDDEVICE_SENDING_INTERVAL 2000 //ms
/* Tiempo de inactividad para que los end-devices se duerman. */
#define APP_ENDDEVICE_TIME_INACTIVITY 180000 //ms
/* Tiempo entre tramas de routers, interno. */
#define APP_ROUTER_SENDING_INTERVAL ((APP_ROUTER_SENSOR_INTERVAL) / (APP_ROUTER_NUMBER_BEACONS+1))

/* N�mero de tramas m�ximo sin detectar actividad para dormir. */
#define APP_ENDDEVICE_MAX_INACTIVITY (APP_ENDDEVICE_TIME_INACTIVITY/APP_ENDDEVICE_SENDING_INTERVAL)
/* Contador para WDT para coordinadores. */
#define WDT_MAX_RETRIES_COORDINATOR  ((APP_COORDINATOR_SENDING_INTERVAL / 8000) + 1)//((int) ((APP_COORDINATOR_SENDING_INTERVAL + 1000) / 8000))
/* Contador para WDT para routers. */
#define WDT_MAX_RETRIES_ROUTER  ((APP_ROUTER_SENDING_INTERVAL / 8000) + 1) //((int) ((APP_ROUTER_SENDING_INTERVAL + 1000) / 8000))
/* Contador para WDT para end-devices. */
#define WDT_MAX_RETRIES_ENDDEVICE  ((APP_ENDDEVICE_SENDING_INTERVAL / 8000) + 1) //((int) ((APP_ENDDEVICE_SENDING_INTERVAL + 1000) / 8000))

//end device
/* N�mero de alarmas hasta pasar a modo normal. */
#define MAX_ALARMS 10
/* Los end-devices tienen acel�rometro. */
#define ACCELEROMETER 1

/* El aceler�metro utilizado es el ADXL345. */
//#define ADXL345 1
/* El aceler�metro utilizado es el ADXL362. */
#define ADXL362 1
/* Los end-devices tienen lectura de bater�a acel�rometro. */
#define BATTERY 1

/* Intervalo entre env�o de alarmas mientras no recibe ACK. No usado. */
#define ALARM_SENDING_INTERVAL 2000
/* Intervalo entre env�o de alarmas cuando recibe ACK pero no confirmaci�n de alarma. No usado. */
#define ALARM_SLEEPING		   2000
/* Umbral en LSB para despertar el dispositivo. 1 LSB = 4 mg. */
#define ACCEL_WAKE_UP_TRESHOLD 300  //300*4 = 1200 mg

//router

//sensores de routers y coordinadores locales
//#define TEMP 1
/* Los routers y coordinadores tienen sensor de temperatura y humedad */
#define HUMIDICON 1

//todos
/* La localizaci�n est� habilitada */
#define LOCALIZATION 1

/*  Versi�n del firmware, por ejemplo: para la versi�n 0 del mes 3 del a�o 2014 se utilizar�:     
        #define VERSION 14030     
*/
#define VERSION 16020 //Versi�n 0 del 02/2016

#ifdef HUMIDICON
/* N�mero de tramas enviadas por un router en APP_ROUTER_SENSOR_INTERVAL ms*/
#define APP_ROUTER_NUMBER_SEND (APP_ROUTER_NUMBER_BEACONS + 1)
#endif

/* Indica si est� activado el debug para el watchdog reset. Si est� activado env�a en el campo alarma un 1 indicando que se ha producido un watchdog reset. */
// #define DEBUG_WDR 
/* Indica si est� activado el debug (LEDS) para detecci�n de ca�das en los end-devices y medias m�viles en los routers.*/
// #define DEBUG




#else //definiciones para la documentaci�n, no tocar.

/** \brief Direcci�n del nodo. Si no est� definido aqu� toma los datos del bootloader*/
#define APP_ADDR
/** \brief Panid del nodo. Si no est� definido aqu� toma los datos del bootloader*/
#define APP_PANID
/** \brief Canal del nodo. Si no est� definido aqu� toma los datos del bootloader*/
#define APP_CHANNEL

/** \brief end-point de la aplicaci�n. */
#define APP_ENDPOINT
/** \brief end-point de las tramas de actualizaciones OTA. */
#define APP_OTA_ENDPOINT
/** \brief end-point de las tramas de comandos. */
#define APP_COMMAND_ENDPOINT
/** \brief end-point de las tramas de lectura de sensores. */
#define APP_SENSORS_ENDPOINT
/** \brief end-point de las tramas de versi�n. */
#define APP_VERSION_ENDPOINT
/** \brief end-point de la primera trama de los nodos con rel�. */
#define APP_RELAY_ENDPOINT

/** \brief Clave de seguridad. Debe ser un string de 16 caracteres. */
#define APP_SECURITY_KEY

/** \brief N�mero de canal de la UART. */
#define HAL_UART_CHANNEL
/** \brief Tama�o del b�ffer RX de la UART. */
#define HAL_UART_RX_FIFO_SIZE
/** \brief Tama�o del b�ffer TX de la UART. */
#define HAL_UART_TX_FIFO_SIZE

/** \brief Activa/desactiva el env�o de tramas de localizaci�n de otras subredes: 1 solo para la misma subred, 0 env�a tambi�n de otras subredes*/
#define SEND_LOC_ONLY_SUBNET 0

/** \brief OTA habilitada. */
#define APP_ENABLE_OTA

//#define PHY_ENABLE_RANDOM_NUMBER_GENERATOR

/** \brief N�mero de b�ffers NWK. */
#define NWK_BUFFERS_AMOUNT
/** \brief Tama�o de la tabla de duplicados rechazados. */
#define NWK_DUPLICATE_REJECTION_TABLE_SIZE
/** \brief Permanencia de los duplicados rechazados en la tabla.*/
#define NWK_DUPLICATE_REJECTION_TTL
/** \brief Tama�o de la tabla de enrutado.*/
#define NWK_ROUTE_TABLE_SIZE
/** \brief Puntuaci�n por defecto de una ruta. */
#define NWK_ROUTE_DEFAULT_SCORE
/** \brief Tiempo de espera de un NWK ACK. */
#define NWK_ACK_WAIT_TIME
/** \brief N�mero de grupos de direcciones (multicast). */
#define NWK_GROUPS_AMOUNT
/** \brief Tama�o para el algoritmo de route discovery (debe estar definido NWK_ENABLE_ROUTE_DISCOVERY). */
#define NWK_ROUTE_DISCOVERY_TABLE_SIZE
/** \brief Tiempo para el algoritmo de route discovery (debe estar definido NWK_ENABLE_ROUTE_DISCOVERY). */
#define NWK_ROUTE_DISCOVERY_TIMEOUT

/** \brief Habilita el enrutado. */
#define NWK_ENABLE_ROUTING

/** \brief Modo de seguridad. 1 por SW, 0 por HW. */
#define SYS_SECURITY_MODE

/** \brief Habilita la seguridad */
#define NWK_ENABLE_SECURITY


/** \brief Tiempo entre env�o de tramas de coordinadores. */
#define APP_COORDINATOR_SENDING_INTERVAL
/** \brief Tiempo entre env�o de tramas de lectura de sensores de routers. */
#define APP_ROUTER_SENSOR_INTERVAL
/** \brief N�mero de tramas de localizaci�n entre env�o de tramas de lectura de sensores de routers. */
#define APP_ROUTER_NUMBER_BEACONS
/** \brief Tiempo entre env�o de tramas de end-devices. */
#define APP_ENDDEVICE_SENDING_INTERVAL
/** \brief Tiempo de inactividad para que los end-devices se duerman. */
#define APP_ENDDEVICE_TIME_INACTIVITY
/** \brief Tiempo entre tramas de routers, interno. */
#define APP_ROUTER_SENDING_INTERVAL

/** \brief N�mero de tramas m�ximo sin detectar actividad para dormir. */
#define APP_ENDDEVICE_MAX_INACTIVITY
/** \brief Contador para WDT para coordinadores. */
#define WDT_MAX_RETRIES_COORDINATOR
/** \brief Contador para WDT para routers. */
#define WDT_MAX_RETRIES_ROUTER
/** \brief Contador para WDT para end-devices. */
#define WDT_MAX_RETRIES_ENDDEVICE

//end device
/** \brief N�mero de alarmas hasta pasar a modo normal. */
#define MAX_ALARMS
/** \brief Los end-devices tienen acel�rometro. */
#define ACCELEROMETER

/** \brief El aceler�metro utilizado es el ADXL345. */
#define ADXL345
/** \brief El aceler�metro utilizado es el ADXL362. */
#define ADXL362
/** \brief Los end-devices tienen lectura de bater�a acel�rometro. */
#define BATTERY

/** \brief Intervalo entre env�o de alarmas mientras no recibe ACK. No usado. */
#define ALARM_SENDING_INTERVAL
/** \brief Intervalo entre env�o de alarmas cuando recibe ACK pero no confirmaci�n de alarma. No usado. */
#define ALARM_SLEEPING
/** \brief Umbral en LSB para despertar el dispositivo. 1 LSB = 4 mg. */
#define ACCEL_WAKE_UP_TRESHOLD 

//router

//sensores de routers y coordinadores locales
/** \brief Los routers y coordinadores tienen sensor de temperatura y humedad */
#define HUMIDICON

//todos
/** \brief La localizaci�n est� habilitada */
#define LOCALIZATION

/** \brief Versi�n del firmware, por ejemplo: para la versi�n 0 del mes 3 del a�o 2016 se utilizar�:
     \verbatim
        #define VERSION 16030
     \endverbatim     
 */
#define VERSION


/** \brief N�mero de tramas enviadas por un router en APP_ROUTER_SENSOR_INTERVAL ms*/
#define APP_ROUTER_NUMBER_SEND


/** \brief Indica si est� activado el debug para el watchdog reset. Si est� activado env�a en el campo alarma un 1 indicando que se ha producido un watchdog reset. */
#define DEBUG_WDR 
/** \brief Indica si est� activado el debug (LEDS) para detecci�n de ca�das en los end-devices y medias m�viles en los routers.*/
#define DEBUG
#endif

#endif // _CONFIG_H_
