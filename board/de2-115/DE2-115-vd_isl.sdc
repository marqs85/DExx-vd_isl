#**************************************************************
# Create Clock
#**************************************************************

create_clock -period 20 -name clk50 [get_ports CLOCK_50]
create_clock -period 20 -name clk50_2 [get_ports CLOCK2_50]
create_clock -period 20 -name clk50_3 [get_ports CLOCK3_50]

create_clock -period 108MHz -name pclk_isl [get_ports GPIO[2]]
create_clock -period 185MHz -name pclk_si [get_ports GPIO[0]]

#**************************************************************
# Create Generated Clock
#**************************************************************
#derive_pll_clocks
create_generated_clock -source {pll_sys|altpll_component|auto_generated|pll1|inclk[0]} -divide_by 176 -multiply_by 95 -duty_cycle 50.00 -name clk27 {pll_sys|altpll_component|auto_generated|pll1|clk[0]}
create_generated_clock -source {pll_sys|altpll_component|auto_generated|pll1|inclk[0]} -divide_by 32 -multiply_by 95 -duty_cycle 50.00 -name clk_vip {pll_sys|altpll_component|auto_generated|pll1|clk[1]}
create_generated_clock -source {pll_sys|altpll_component|auto_generated|pll1|inclk[0]} -divide_by 44 -multiply_by 95 -duty_cycle 50.00 -name clk100 {pll_sys|altpll_component|auto_generated|pll1|clk[2]}

create_generated_clock -name sd_clk -divide_by 2 -source {pll_sys|altpll_component|auto_generated|pll1|clk[2]} [get_pins sys:sys_inst|sdc_controller_top:sdc_controller_0|sdc_controller:sdc0|sd_clock_divider:clock_divider0|SD_CLK_O|q]

create_generated_clock -name dram_clk -master_clock clk_vip -multiply_by 1 -source {pll_sys|altpll_component|auto_generated|pll1|clk[1]} [get_ports DRAM_CLK]

create_generated_clock -name pclk_si_out_hsmc -master_clock pclk_si -source [get_ports GPIO[0]] -multiply_by 1 [get_ports HDMI_TX_HSMC_CLK]
create_generated_clock -name pclk_si_out_vga -master_clock pclk_si -source [get_ports GPIO[0]] -multiply_by 1 [get_ports VGA_CLK]
create_generated_clock -name sd_clk_out -master_clock sd_clk -source [get_pins sys:sys_inst|sdc_controller_top:sdc_controller_0|sdc_controller:sdc0|sd_clock_divider:clock_divider0|SD_CLK_O|q] -multiply_by 1 [get_ports {SD_CLK}]

# specify div2 capture and output clocks
#set pclk_capture_div_pin [get_pins pclk_capture_div2|q]
#create_generated_clock -name pclk_isl_div2 -master_clock pclk_isl -source [get_ports GPIO[2]] -divide_by 2 $pclk_capture_div_pin
#set pclk_si_div_pin [get_pins pclk_out_div2|q]
#create_generated_clock -name pclk_si_div2 -master_clock pclk_si -source [get_ports GPIO[0]] -divide_by 2 $pclk_si_div_pin

#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************
derive_clock_uncertainty



#**************************************************************
# Set IO Delay
#**************************************************************

# SD card (IO constraints from Kingston specsheet, slightly reduced t_ODLY)
set_input_delay -clock sd_clk_out -min 2.5 [get_ports {SD_CMD SD_DAT[*]}] -add_delay
set_input_delay -clock sd_clk_out -max 12 [get_ports {SD_CMD SD_DAT[*]}] -add_delay
set_output_delay -clock sd_clk_out -min -2 [get_ports {SD_CMD SD_DAT[*]}] -add_delay
set_output_delay -clock sd_clk_out -max 6 [get_ports {SD_CMD SD_DAT[*]}] -add_delay

# ISL51002
set ISL_dmin [expr 3.4-0.5*(1000/165)]
set ISL_dmax [expr 0.5*(1000/165)-1.8]
set ISL_inputs [get_ports {GPIO[32] GPIO[31] GPIO[30] GPIO[29] GPIO[28] GPIO[27] GPIO[26] GPIO[25] GPIO[24] GPIO[23] GPIO[22] GPIO[21] GPIO[20] GPIO[19] GPIO[18] GPIO[17] GPIO[16] GPIO[15] GPIO[14] GPIO[13] GPIO[12] GPIO[11] GPIO[10] GPIO[9] GPIO[8] GPIO[7] GPIO[6] GPIO[35]}]
set_input_delay -clock pclk_isl -clock_fall -min $ISL_dmin $ISL_inputs -add_delay
set_input_delay -clock pclk_isl -clock_fall -max $ISL_dmax $ISL_inputs -add_delay

# SiI1136
set hdmitx_dmin -0.45
set hdmitx_dmax 1.36
set hdmitx_data_outputs [get_ports {HDMI_TX_HSMC_D* HDMI_TX_HSMC_HS HDMI_TX_HSMC_VS HDMI_TX_HSMC_DE}]
set_output_delay -clock pclk_si_out_hsmc -clock_fall -min $hdmitx_dmin $hdmitx_data_outputs -add_delay
set_output_delay -clock pclk_si_out_hsmc -clock_fall -max $hdmitx_dmax $hdmitx_data_outputs -add_delay

# VGA
set vga_dmin -1.5
set vga_dmax 0.2
set vga_data_outputs [get_ports {VGA_R[*] VGA_G[*] VGA_B[*] VGA_HS VGA_VS VGA_SYNC_N VGA_BLANK_N}]
set_output_delay -clock pclk_si_out_vga -min $vga_dmin $vga_data_outputs -add_delay
set_output_delay -clock pclk_si_out_vga -max $vga_dmax $vga_data_outputs -add_delay

# SDRAM
set_input_delay -clock dram_clk -min 2.7 [get_ports {DRAM_DQ[*]}] -add_delay
set_input_delay -clock dram_clk -max 5.4 [get_ports {DRAM_DQ[*]}] -add_delay
set_output_delay -clock dram_clk -min -0.8 [get_ports {DRAM_ADDR[*] DRAM_BA[*] DRAM_CKE DRAM_CS_N DRAM_CAS_N DRAM_RAS_N DRAM_WE_N DRAM_DQM[*] DRAM_DQ[*]}] -add_delay
set_output_delay -clock dram_clk -max 1.5 [get_ports {DRAM_ADDR[*] DRAM_BA[*] DRAM_CKE DRAM_CS_N DRAM_CAS_N DRAM_RAS_N DRAM_WE_N DRAM_DQM[*] DRAM_DQ[*]}] -add_delay

set_false_path -from [get_ports {GPIO[34] KEY* EX_IO[0] EX_IO[1] EX_IO[2] EX_IO[3]}]
set_false_path -to [get_ports {LED*}]

# TODO: set I2C constraints
set_false_path -from [get_ports {HDMI_TX_HSMC_I2C_SCL HDMI_TX_HSMC_I2C_SDA}]
set_false_path -to [get_ports {HDMI_TX_HSMC_I2C_SCL HDMI_TX_HSMC_I2C_SDA}]
set_false_path -from [get_ports {GPIO[4] GPIO[5] IRDA_RXD}]
set_false_path -to [get_ports {GPIO[4] GPIO[5] HDMI_TX_HSMC_RESET_N HDMI_TX_HSMC_SPDIF}]

# LCD
set_input_delay -clock clk27 0 [get_ports LCD_DATA*]
set_output_delay -clock clk27 0 [get_ports {LCD_DATA* LCD_BLON LCD_EN LCD_ON LCD_RS LCD_RW}]

# EPCQ controller (delays from N25Q128A datasheet)
create_generated_clock -name flash_clk -master_clock clk27 -source {pll_sys|altpll_component|auto_generated|pll1|clk[0]} -multiply_by 1 sys:sys_inst|sys_intel_generic_serial_flash_interface_top_0:intel_generic_serial_flash_interface_top_0|sys_intel_generic_serial_flash_interface_top_0_qspi_inf_inst:qspi_inf_inst|flash_clk_reg
create_generated_clock -name epcq_clk -master_clock clk27 -source {pll_sys|altpll_component|auto_generated|pll1|clk[0]} -multiply_by 1 [get_ports *ALTERA_DCLK]
set_input_delay -clock epcq_clk -clock_fall 5 [get_ports *ALTERA_DATA0]
set_output_delay -clock epcq_clk 4 [get_ports *ALTERA_SCE]
set_output_delay -clock epcq_clk 2 [get_ports **ALTERA_SDO]

#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************
set_clock_groups -asynchronous -group \
                            {clk50} \
                            {clk50_2} \
                            {clk50_3} \
                            {clk100 sd_clk sd_clk_out} \
                            {clk_vip dram_clk} \
                            {pclk_isl} \
                            {pclk_si pclk_si_out_hsmc pclk_si_out_vga} \
                            {clk27 flash_clk epcq_clk}

# max 10MHz JTAG clock
remove_clock altera_reserved_tck
create_clock -name altera_reserved_tck -period "10MHz" [get_ports altera_reserved_tck]
set_clock_groups -exclusive -group [get_clocks altera_reserved_tck]
set_input_delay -clock altera_reserved_tck 20 [get_ports altera_reserved_tdi]
set_input_delay -clock altera_reserved_tck 20 [get_ports altera_reserved_tms]
set_output_delay -clock altera_reserved_tck 20 [get_ports altera_reserved_tdo]
