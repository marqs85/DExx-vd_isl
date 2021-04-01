#ifndef SYSCONFIG_H_
#define SYSCONFIG_H_

#include "system.h"

#define DExx_FW
#define INC_ADV7513
#define VIP

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

#define WAITLOOP_SLEEP_US   10000

#endif /* SYSCONFIG_H_ */
