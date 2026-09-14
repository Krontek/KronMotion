/*===========================================================================
 * kron_pi.c  --  the global process image
 *
 * The one definition of the shared memory every layer talks through, and of
 * the HAL driver pointer the fieldbus registers itself into.  Declared in
 * kron_pi.h; kept here so a build without a generated plc.c still links.
 *===========================================================================*/

#include <stddef.h>

#include "kron_pi.h"

KRON_PROCESS_IMAGE  Kron_PI;
KRON_HAL_Driver    *Kron_HAL = NULL;
