#**************************************************************
# Create Clock
#**************************************************************

create_clock -period 20 -name clk50 [get_ports FPGA_CLK1_50]
create_clock -period 20 -name clk50_2 [get_ports FPGA_CLK2_50]
create_clock -period 20 -name clk50_3 [get_ports FPGA_CLK3_50]

create_clock -period 108MHz -name pclk_isl [get_ports GPIO_1[2]]
create_clock -period 185MHz -name pclk_si [get_ports GPIO_1[0]]

#**************************************************************
# Create Generated Clock
#**************************************************************
#derive_pll_clocks
create_generated_clock -source {pll_sys|pll_inst|altera_pll_i|general[0].gpll~FRACTIONAL_PLL|refclkin} -divide_by 5 -multiply_by 81 -duty_cycle 50.00 -name pll_vco_clk {pll_sys|pll_inst|altera_pll_i|general[0].gpll~FRACTIONAL_PLL|vcoph[0]}
create_generated_clock -source {pll_sys|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]} -divide_by 30 -duty_cycle 50.00 -name clk27 {pll_sys|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}
create_generated_clock -source {pll_sys|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]} -divide_by 6 -duty_cycle 50.00 -name clk_vip {pll_sys|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}
create_generated_clock -source {pll_sys|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]} -divide_by 8 -duty_cycle 50.00 -name clk100 {pll_sys|pll_inst|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}


create_generated_clock -name sd_clk -divide_by 4 -source [get_ports FPGA_CLK1_50] [get_pins sys:sys_inst|sdc_controller_top:sdc_controller_0|sdc_controller:sdc0|sd_clock_divider:clock_divider0|SD_CLK_O|q]

create_generated_clock -name pclk_si_out -master_clock pclk_si -source [get_ports GPIO_1[0]] -multiply_by 1 [get_ports HDMI_TX_CLK]
create_generated_clock -name sd_clk_out -master_clock sd_clk -source [get_pins sys:sys_inst|sdc_controller_top:sdc_controller_0|sdc_controller:sdc0|sd_clock_divider:clock_divider0|SD_CLK_O|q] -multiply_by 1 [get_ports {HPS_SD_CLK}]

# specify div2 capture and output clocks
set pclk_capture_div_pin [get_pins pclk_capture_div2|q]
create_generated_clock -name pclk_isl_div2 -master_clock pclk_isl -source [get_ports GPIO_1[2]] -divide_by 2 $pclk_capture_div_pin
set pclk_si_div_pin [get_pins pclk_out_div2|q]
create_generated_clock -name pclk_si_div2 -master_clock pclk_si -source [get_ports GPIO_1[0]] -divide_by 2 $pclk_si_div_pin

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

# SD card (IO constraints from Kingston specsheet, default mode)
set_input_delay -clock sd_clk_out -clock_fall -min 0 [get_ports {HPS_SD_CMD HPS_SD_DATA[*]}] -add_delay
set_input_delay -clock sd_clk_out -clock_fall -max 14 [get_ports {HPS_SD_CMD HPS_SD_DATA[*]}] -add_delay
set_output_delay -clock sd_clk_out -min -5 [get_ports {HPS_SD_CMD HPS_SD_DATA[*]}] -add_delay
set_output_delay -clock sd_clk_out -max 5 [get_ports {HPS_SD_CMD HPS_SD_DATA[*]}] -add_delay

# ISL51002
set ISL_dmin [expr 3.4-0.5*(1000/165)]
set ISL_dmax [expr 0.5*(1000/165)-1.8]
set ISL_inputs [get_ports {GPIO_1[32] GPIO_1[31] GPIO_1[30] GPIO_1[29] GPIO_1[28] GPIO_1[27] GPIO_1[26] GPIO_1[25] GPIO_1[24] GPIO_1[23] GPIO_1[22] GPIO_1[21] GPIO_1[20] GPIO_1[19] GPIO_1[18] GPIO_1[17] GPIO_1[16] GPIO_1[15] GPIO_1[14] GPIO_1[13] GPIO_1[12] GPIO_1[11] GPIO_1[10] GPIO_1[9] GPIO_1[8] GPIO_1[7] GPIO_1[6] GPIO_1[35]}]
set_input_delay -clock pclk_isl -clock_fall -min $ISL_dmin $ISL_inputs -add_delay
set_input_delay -clock pclk_isl -clock_fall -max $ISL_dmax $ISL_inputs -add_delay

# ADV7513 (0ns video clock delay adjustment)
set hdmitx_dmin -0.7
set hdmitx_dmax 1.0
set hdmitx_data_outputs [get_ports {HDMI_TX_D* HDMI_TX_HS HDMI_TX_VS HDMI_TX_DE}]
set_output_delay -clock pclk_si_out -min $hdmitx_dmin $hdmitx_data_outputs -add_delay
set_output_delay -clock pclk_si_out -max $hdmitx_dmax $hdmitx_data_outputs -add_delay

set_false_path -from [get_ports {GPIO_1[34] KEY* ARDUINO_IO[7] ARDUINO_IO[6] ARDUINO_IO[5] ARDUINO_IO[4]}]
set_false_path -to [get_ports {LED* HPS_LED}]

# TODO: set I2C constraints
set_false_path -from [get_ports {GPIO_1[4] GPIO_1[5] HDMI_I2C_SCL HDMI_I2C_SDA}]
set_false_path -to [get_ports {GPIO_1[4] GPIO_1[5] HDMI_I2C_SCL HDMI_I2C_SDA}]

# misc
set_false_path -setup -to [get_registers sys:sys_inst|sys_alt_vip_cl_cvo_0:alt_vip_cl_cvo_0|alt_vip_cvo_core:cvo_core|alt_vip_cvo_sync_conditioner:pixel_channel_sync_conditioner|alt_vip_common_sync_generation:sync_generation_generate.sync_generation|sof*]

#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************
set_clock_groups -asynchronous -group \
                            {clk50 sd_clk sd_clk_out} \
                            {clk50_2} \
                            {clk50_3} \
                            {clk100} \
                            {clk_vip} \
                            {pclk_isl} \
                            {pclk_isl_div2} \
                            {pclk_si pclk_si_out} \
                            {pclk_si_div2} \
                            {pll_vco_clk} \
                            {clk27}

# max 25MHz JTAG clock
remove_clock altera_reserved_tck
create_clock -name altera_reserved_tck -period "25MHz" [get_ports altera_reserved_tck]
set_clock_groups -exclusive -group [get_clocks altera_reserved_tck]
set_input_delay -clock altera_reserved_tck -clock_fall 3 [get_ports altera_reserved_tdi]
set_input_delay -clock altera_reserved_tck -clock_fall 3 [get_ports altera_reserved_tms]
set_output_delay -clock altera_reserved_tck 3 [get_ports altera_reserved_tdo]
