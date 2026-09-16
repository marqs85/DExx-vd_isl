//
// Copyright (C) 2019-2026  Markus Hiienkari <mhiienka@niksula.hut.fi>
//
// This file is part of Open Source Scan Converter project.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "alt_clock_manager.h"
#include "alt_cache.h"
#include "alt_watchdog.h"
#include "alt_interrupt.h"
#include "sys/alt_sys_init.h"

extern int main (int, char **, char **);

void _mainCRTStartup(void)
{
    // Clear BSS
    __asm__(
        "ldr     r0, =__bss_start__;"
        "ldr     r1, =_bss_end__;"
        "mov     r2, #0x00000000;"
        "clbss_l:cmp     r0, r1;"
        "itt lo;"
        "strlo   r2, [r0];"
        "addlo   r0, r0, #4;"
        "blo     clbss_l"
    );

    // Disable Watchdog
    alt_wdog_stop(ALT_WDOG0);

    alt_int_global_init();

    // Enable L1 & L2 Cache Systems
    alt_cache_system_enable();

    alt_sys_init();
    main(0, NULL, NULL);
}