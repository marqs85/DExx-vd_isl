DExx-vd_isl on DE10-Nano
===========================

Hardware notes
-----------------
* DE10-Nano has no series resistors between GPIO and FPGA (make sure JP1 is open)
* Uses GPIO0 block and nearby Arduino header
  * TODO: test feasibility of separate fw built to use GPIO1 (useful for people using a Mister RAM expansion on GPIO0)
* Uses Nios2 soft-CPU to be in line with other board adaptations
  * TODO: evaluate if hard ARM core could be utilized with minimal configuration/code changes
* DE10-Nano user LEDs are too tightly packed and all same color so they are not much of use
* SPDIF input of ADV7513 is not connected (board can be modified to support SPDIF, see below)


SPDIF mod
------------
In order to enable SPDIF support, a small modification needs to be applied on DE10-Nano.

Connect one end of a ~10cm kynar wire to pin 3 of ADV7513 as shown below.
![DE10-Nano top SPDIF mod](http://www.infocult.com/m/ossc_pro/img/de10-nano_spdif_top.jpg)

On bottom side of the PCB, connect other end of the wire to Arduino IO4/D4 pin.
![DE10-Nano bottom SPDIF](http://www.infocult.com/m/ossc_pro/img/de10-nano_spdif_bottom.jpg)


Building SD card image
-------------------------
See [README-bootloader.txt](README-bootloader.txt)


Debugging notes
------------------
Nios2 JTAG doesn't work with the default TCK frequency of the on-board USB-Blaster. Before SW download, open "Quartus Programmer -> Hardward Setup" and set the frequency to 6.5MHz.
