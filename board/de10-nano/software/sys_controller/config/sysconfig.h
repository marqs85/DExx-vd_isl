#ifndef SYSCONFIG_H_
#define SYSCONFIG_H_

#include "system.h"

#define DExx_FW
#define INC_ADV7513
#define VIP

#if ALT_VIP_CL_DIL_0_SPAN == 256
#define VIP_DIL_B
#elif ALT_VIP_CL_DIL_0_SPAN == 128
#define VIP_DIL_A
#else
#error VIP Deinterlacer II: unexpected run-time control address span
#endif

#ifndef DEBUG
#define OS_PRINTF(...)
#define ErrorF(...)
#define printf(...)
#else
#include <stdio.h>
//#include "utils.h"
#define OS_PRINTF printf
#define ErrorF printf
// use reduced printf
//#define printf alt_printf
//#define printf dd_printf
#endif

#define MAINLOOP_INTERVAL_US   10000

#endif /* SYSCONFIG_H_ */
