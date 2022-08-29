DExx-vd_isl on DE2-115
===========================

Hardware notes
-----------------
* DE2-115 has series resistors between GPIO and FPGA (close solder jumper JP1)
* 4-pin audio cable is connected to 14-pin IO connector (EX_IO[3:0])
* The firmware utilizes IR receiver and character display of DE2-115, thus these parts can be omitted from DExx-vd_isl
* Video output is through DE2-115 VGA and custom HDMI HSMC expansion card
* Scaler mode performance is lower compared to other boards due to SDRAM and Cyclone IV limitations
