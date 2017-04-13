/*
 * dwm1000_platform.h - platform specific includes
 */

#ifndef _DWM1000_PLATFORM_H_
#define _DWM1000_PLATFORM_H_

#define DWM1000_DEBUG 1

// uncomment appropriate platform
#define PLATFORM_FIREFLY
// #define PLATFORM_K22

#if   defined(PLATFORM_FIREFLY)
#include <nrk.h>
#include <stdio.h>
#elif defined(PLATFORM_K22)
#include "fsl_debug_console.h"
#endif

#if   defined(PLATFORM_FIREFLY)
#define DEBUG_PRINTF(fmt, ...) if (DWM1000_DEBUG) printf(fmt, __VA_ARGS__);
#define DEBUG_PRINT(fmt) if (DWM1000_DEBUG) printf(fmt);
#elif defined(PLATFORM_K22)
#define DEBUG_PRINTF(fmt, ...) if (DWM1000_DEBUG) PRINTF(fmt, __VA_ARGS__);
#define DEBUG_PRINT(fmt) if (DWM1000_DEBUG) PRINTF(fmt);
#endif


#endif /* _DWM1000_PLATFORM_H_ */
