#**************************************************************
# Create Clock
#**************************************************************

create_clock -period 20 -name clk50 [get_ports CLOCK_50_B6A]
#create_clock -period 20 -name clk50_2 [get_ports CLOCK_50_B3B]

create_clock -period 108MHz -name pclk_isl [get_ports GPIO[2]]
create_clock -period 200MHz -name pclk_si [get_ports GPIO[0]]

#**************************************************************
# Create Generated Clock
#**************************************************************
#derive_pll_clocks
create_generated_clock -source {pll_sys|pll_inst|altera_pll_i|general[0].gpll~FRACTIONAL_PLL|refclkin} -divide_by 128 -multiply_by 1521 -duty_cycle 50.00 -name pll_vco_clk {pll_sys|pll_inst|altera_pll_i|general[0].gpll~FRACTIONAL_PLL|vcoph[0]}
create_generated_clock -source {pll_sys|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]} -divide_by 22 -duty_cycle 50.00 -name clk27 {pll_sys|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}
create_generated_clock -source {pll_sys|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]} -divide_by 4 -duty_cycle 50.00 -name clk_vip {pll_sys|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}
create_generated_clock -source {pll_sys|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]} -divide_by 6 -duty_cycle 50.00 -name clk100 {pll_sys|pll_inst|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}

create_generated_clock -name pclk_si_out -master_clock pclk_si -source [get_ports GPIO[0]] -multiply_by 1 [get_ports HDMI_TX_CLK]
create_generated_clock -name pclk_si_out_hsmc -master_clock pclk_si -source [get_ports GPIO[0]] -multiply_by 1 [get_ports HDMI_TX_HSMC_CLK]

# specify div2 capture and output clocks
set pclk_capture_div_pin [get_pins pclk_capture_div2|q]
create_generated_clock -name pclk_isl_div2 -master_clock pclk_isl -source [get_ports GPIO[2]] -divide_by 2 $pclk_capture_div_pin
set pclk_si_div_pin [get_pins pclk_out_div2|q]
create_generated_clock -name pclk_si_div2 -master_clock pclk_si -source [get_ports GPIO[0]] -divide_by 2 $pclk_si_div_pin

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

# ISL51002
set ISL_dmin [expr 3.4-0.5*(1000/165)]
set ISL_dmax [expr 0.5*(1000/165)-1.8]
set ISL_inputs [get_ports {GPIO[32] GPIO[31] GPIO[30] GPIO[29] GPIO[28] GPIO[27] GPIO[26] GPIO[25] GPIO[24] GPIO[23] GPIO[22] GPIO[21] GPIO[20] GPIO[19] GPIO[18] GPIO[17] GPIO[16] GPIO[15] GPIO[14] GPIO[13] GPIO[12] GPIO[11] GPIO[10] GPIO[9] GPIO[8] GPIO[7] GPIO[6] GPIO[35]}]
set_input_delay -clock pclk_isl -clock_fall -min $ISL_dmin $ISL_inputs -add_delay
set_input_delay -clock pclk_isl -clock_fall -max $ISL_dmax $ISL_inputs -add_delay

# ADV7513 (0ns video clock delay adjustment)
set hdmitx_dmin -1.9
set hdmitx_dmax -0.2
set hdmitx_data_outputs [get_ports {HDMI_TX_D* HDMI_TX_HS HDMI_TX_VS HDMI_TX_DE}]
set_output_delay -clock pclk_si_out -min $hdmitx_dmin $hdmitx_data_outputs -add_delay
set_output_delay -clock pclk_si_out -max $hdmitx_dmax $hdmitx_data_outputs -add_delay

# SiI1136
set hdmitx_dmin -0.45
set hdmitx_dmax 1.36
set hdmitx_data_outputs [get_ports {HDMI_TX_HSMC_D* HDMI_TX_HSMC_HS HDMI_TX_HSMC_VS HDMI_TX_HSMC_DE}]
set_output_delay -clock pclk_si_out_hsmc -clock_fall -min $hdmitx_dmin $hdmitx_data_outputs -add_delay
set_output_delay -clock pclk_si_out_hsmc -clock_fall -max $hdmitx_dmax $hdmitx_data_outputs -add_delay

set_false_path -from [get_ports {GPIO[34] KEY* SW*}]
set_false_path -to [get_ports {LED* HDMI_TX_HSMC_RESET_N HDMI_TX_HSMC_SPDIF}]

# TODO: set I2C constraints
set_false_path -from [get_ports {GPIO[4] GPIO[5] I2C_SDA HDMI_TX_HSMC_I2C_SCL HDMI_TX_HSMC_I2C_SDA}]
set_false_path -to [get_ports {GPIO[4] GPIO[5] I2C_SDA I2C_SCL HDMI_TX_HSMC_I2C_SCL HDMI_TX_HSMC_I2C_SDA}]

# Misc
set_false_path -from {emif_hwreset_n_sync2_reg emif_swreset_n_sync2_reg}
set_false_path -to {emif_hwreset_n_sync1_reg emif_swreset_n_sync1_reg}
set_false_path -to sys:sys_inst|sys_pio_1:pio_2|readdata[0]
set_false_path -to sys:sys_inst|sys_pio_1:pio_2|readdata[1]
set_false_path -to sys:sys_inst|sys_pio_1:pio_2|readdata[2]
set_false_path -setup -to [get_registers sys:sys_inst|sys_alt_vip_cl_cvo_0:alt_vip_cl_cvo_0|alt_vip_cvo_core:cvo_core|alt_vip_cvo_sync_conditioner:pixel_channel_sync_conditioner|alt_vip_common_sync_generation:sync_generation_generate.sync_generation|sof*]


#**************************************************************
# Set Clock Groups
#**************************************************************
set_clock_groups -asynchronous -group \
                            {clk50} \
                            {clk100} \
                            {clk_vip} \
                            {pclk_isl} \
                            {pclk_isl_div2} \
                            {pclk_si pclk_si_out pclk_si_out_hsmc} \
                            {pclk_si_div2} \
                            {pll_vco_clk} \
                            {clk27}

# max 10MHz JTAG clock
remove_clock altera_reserved_tck
create_clock -name altera_reserved_tck -period "10MHz" [get_ports altera_reserved_tck]
set_clock_groups -exclusive -group [get_clocks altera_reserved_tck]
set_input_delay -clock altera_reserved_tck 20 [get_ports altera_reserved_tdi]
set_input_delay -clock altera_reserved_tck 20 [get_ports altera_reserved_tms]
set_output_delay -clock altera_reserved_tck 20 [get_ports altera_reserved_tdo]
