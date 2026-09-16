# DExx-vd_isl on DE10-Nano

## Hardware notes
* DE10-Nano has no series resistors between GPIO and FPGA (make sure JP1 is open)
* Uses GPIO0 block by default and Arduino header (D[7:4]) closest to FPGA
  * GPIO1 can be used by loading alternative FPGA bitstream (see below - useful for people using a Mister RAM expansion on GPIO0)
* Uses HPS ARM core
* DE10-Nano user LEDs are too tightly packed and all same color so they are not much of use
* SPDIF input of ADV7513 is not connected (board can be modified to support SPDIF, see below)
* Due to booting from HPS, EPCQ flash is not accessible. Settings are stored on SD card


## DE10-Nano HW modifications (optional)

### SPDIF mod
In order to enable SPDIF support, a small modification needs to be applied on DE10-Nano.

Connect one end of a ~10cm kynar wire to pin 3 of ADV7513 as shown below.
![DE10-Nano top SPDIF mod](http://www.infocult.com/m/ossc_pro/img/de10-nano_spdif_top.jpg)

On bottom side of the PCB, connect other end of the wire to Arduino IO4/D4 pin.
![DE10-Nano bottom SPDIF](http://www.infocult.com/m/ossc_pro/img/de10-nano_spdif_bottom.jpg)

### HPS GPIO mod
HPS_LTC_GPIO is not connected to DE10-Nano LTC connector (despite User Manual diagram claiming so) due to missing R105.
The connection can be made by shorting pads of R105 footprint underside the board.
![DE10-Nano HPS GPIO mod](http://www.infocult.com/m/ossc_pro/img/de10-nano_hps_gpio_bot.jpg)


## Building SD card image
See [README-bootloader.txt](README-bootloader.txt)


## Firmware installation and upgrade
On initial installation unzip sdcard_cv.img.zip and write it into SD card in RAW mode.
Upgrade can be done by just updating the firmware files (DE10-Nano-vd_isl.bin, DE10-Nano-vd_isl_gpio0.rbf, DE10-Nano-vd_isl_gpio1.rbf and u-boot.scr) on the FAT partition generated on SD card.


## Selecting between GPIO0 and GPIO1 block
By default the board boots with bitstream that assumes DExx-vd_isl connected to GPIO0. If you want to use GPIO1 instead and completed the optional HPS GPIO mod, you can connect a jumper on LTC header as shown below to make bootloader use alternative fw instead. Be careful not to place the jumper on the wrong end on the header which will damage the board.
![DE10-Nano HPS GPIO jumper](http://www.infocult.com/m/ossc_pro/img/de10-nano_hps_gpio_top.jpg)
If you did not make the mod but still want to use GPIO1, then you can replace DE10-Nano-vd_isl_gpio0.rbf on the SD card with DE10-Nano-vd_isl_gpio1.rbf to force loading GPIO1 version by default.


## Debugging notes
To upload SW on the ARM core, easiest way is to connect the board to network via ethernet and controlling it via UART.
Once the board is powered on, press any key on the serial console before it boots to get access to U-Boot prompt.

1. Configure IP address of the board and host for U-Boot, and save settings to SD:
~~~~
setenv ipaddr 1.2.3.4
setenv ipaddr_host 1.2.3.5
saveenv
~~~~

2. Setup TFTP to host

3. Compile the application with debug output by adding -DDEBUG on APP_CFLAGS_USER_FLAGS.

4. Share bitstream and application binary files (DE10-Nano-vd_isl.bin, DE10-Nano-vd_isl_gpio0.rbf and DE10-Nano-vd_isl_gpio1.rbf) on TFTP

5. Reset board via HPS Warm reset button to load updated FW via network