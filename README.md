DExx-vd_isl
==============

DExx-vd_isl is advanced video digitizer add-on module for a number of Terasic FPGA development boards. It has been initially used as a development board for OSSC Pro scan converter but over time has been tailored towards a standalone product. Please check the [wikipage](https://junkerhq.net/xrgb/index.php?title=DExx-vd_isl) for more information on usage and latest features.

[PCB](pcb/) and firmware of the project are open-source except for certain Intel VIP modules which are planned to be replaced by custom implementations over time. For most parts RTL and SW are derived from OSSC Pro [project](https://github.com/marqs85/ossc_pro/) with only small amount of adaptation code for the supported development boards.

Supported Terasic boards and board specific notes
----------------------------------------------------
* [DE10-Nano](board/de10-nano)
* [C5G](board/c5g)
* [DE2-115](board/de2-115)

Requirements for building and debugging firmware
---------------------------------------------------
* Hardware
  * DExx-vd_isl module and compatible development board

* Software
  * [Altera Quartus Prime + device support files for target FPGA family](http://dl.altera.com/?edition=lite) (v 21.1 or higher - free Lite Edition suffices)
  * Make


Building RTL (bitstream)
--------------------------
1. Load the project (board/\<board\>/\<boardname\>.qpf) in Quartus
2. Generate QSYS output files (only needed before first compilation or when QSYS structure has been modified)
    * Open Platform Designer (Tools -> Platform Designer)
    * Load platform configuration (sys.qsys)
    * Generate output (Generate -> Generate HDL, Generate)
    * Close Platform Designer
3. Build software image if not yet done so that it becomes part of the bitstream (see next section)
4. Generate the FPGA bitstream (Processing -> Start Compilation)
5. Ensure that there are no major timing violations by looking into Timing Analyzer report

NOTE: If the software image (software/sys_controller/mem_init/sys_onchip_memory2_0.hex) was not up to date at the time of compilation, bitstream can be quickly rebuilt with updated hex by running "Processing->Update Memory Initialization File" and "Processing->Start->Start Assembler" in Quartus.

NOTE2: Without Intel VIP license it's only possible to generate a time limited bitstream which needs connection to USB Blaster during use. VIP modules can be excluded by commenting out the "VIP" define from both top-level RTL and sysconfig.h in which case scaler mode is disabled.


Building software image
--------------------------
1. Generate software BSP (first-time only)
~~~~
cd board/<board>/software/sys_controller_bsp
nios2-bsp-generate-files --settings=settings.bsp --bsp-dir=.
~~~~
2. Enter software root directory:
~~~~
cd board/<board>/software/sys_controller
~~~~
3. Build SW for target configuration:
~~~~
make [OPTIONS] [TARGET]
~~~~
OPTIONS may include following definitions:
* APP_CFLAGS_DEFINED_SYMBOLS="-DDEBUG" (debug message are printed to stdout connected to JTAG UART, shown via nios2-terminal)

TARGET is typically one of the following:
* all (Default target. Compiles an ELF file)
* mem_init_generate (Generates a memory initialization file required for bitstream)
* clean (cleans ELF and intermediate files. Should be invoked every time OPTIONS are changed between compilations)

4. Optionally test updated SW by directly downloading memory image to block RAM via JTAG (with DE10-Nano TCK needs to be reduced, see board specific notes)
~~~~
nios2-download sys_controller.elf --go
~~~~


Bitstream deployment
----------------------
Depending on target platform, bitstream can be either stored into serial flash chip or SD card where it is automatically loaded every time FPGA is subsequently powered on.

To program flash on compatible boards, FPGA configuration file must be first converted into JTAG indirect Configuration file (.jic). Open conversion tool ("File->Convert Programming Files") in Quartus, click "Open Conversion Setup Data", select "ossc.cof" and press Generate. Then open Programmer, add generated file (output_files/<filename>.jic) and press Start after which flash is programmed. Installed/updated firmware is activated after power-cycling the board.

See board specific notes for building a SD card image containing the bitstream.


Debugging
------------
1. Rebuild the software in debug mode:
~~~~
make clean && make APP_CFLAGS_DEBUG_LEVEL="-DDEBUG" mem_init_generate
~~~~
2. Download memory image via JTAG and open terminal for UART
~~~~
nios2-download sys_controller.elf --go && nios2-terminal
~~~~
Remember to close nios2-terminal after debug session, otherwise any JTAG transactions will hang/fail.
