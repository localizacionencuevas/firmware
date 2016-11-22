/**************************************************************************//**
\file  bcTypes.h

\brief Definiciones auxiliares
******************************************************************************/
#include <stddef.h>
#include <stdbool.h>

/** \brief Los datos se almacenan en la memoria del programa*/
#define PROGMEM_DECLARE(x) x __attribute__((__progmem__))
/** \brief booleano */
typedef bool result_t;
/** \brief éxito */
#define BC_SUCCESS false
/** \brief fracaso */
#define BC_FAIL    true

