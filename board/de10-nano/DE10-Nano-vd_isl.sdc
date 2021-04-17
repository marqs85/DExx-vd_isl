#**************************************************************
# Create Clock
#**************************************************************

create_clock -period 20 -name clk50 [get_ports FPGA_CLK1_50]
create_clock -period 20 -name clk50_2 [get_ports FPGA_CLK2_50]
create_clock -period 20 -name clk50_3 [get_ports FPGA_CLK3_50]

create_clock -period 108MHz -name pclk_isl [get_ports GPIO_0[1]]
create_clock -period 185MHz -name pclk_si [get_ports GPIO_0[0]]

#**************************************************************
# Create Generated Clock
#**************************************************************
#derive_pll_clocks
create_generated_clock -source {pll_sys|pll_inst|altera_pll_i|general[0].gpll~FRACTIONAL_PLL|refclkin} -divide_by 5 -multiply_by 54 -duty_cycle 50.00 -name pll_vco_clk {pll_sys|pll_inst|altera_pll_i|general[0].gpll~FRACTIONAL_PLL|vcoph[0]}
create_generated_clock -source {pll_sys|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]} -divide_by 20 -duty_cycle 50.00 -name clk27 {pll_sys|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}
create_generated_clock -source {u0|pll_0|altera_pll_i|general[0].gpll~FRACTIONAL_PLL|refclkin} -divide_by 2 -multiply_by 12 -duty_cycle 50.00 -name pll_0_vco {u0|pll_0|altera_pll_i|general[0].gpll~FRACTIONAL_PLL|vcoph[0]}
create_generated_clock -source {u0|pll_0|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]} -divide_by 2 -duty_cycle 50.00 -name clk150 {u0|pll_0|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}
create_generated_clock -source {u0|pll_0|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]} -divide_by 3 -duty_cycle 50.00 -name clk100 {u0|pll_0|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}

create_generated_clock -name pclk_si_out -master_clock pclk_si -source [get_ports GPIO_0[0]] -multiply_by 1 [get_ports HDMI_TX_CLK]

# specify div2 capture and output clocks
set pclk_capture_div_pin [get_pins pclk_capture_div2|q]
create_generated_clock -name pclk_isl_div2 -master_clock pclk_isl -source [get_ports GPIO_0[1]] -divide_by 2 $pclk_capture_div_pin
set pclk_si_div_pin [get_pins pclk_out_div2|q]
create_generated_clock -name pclk_si_div2 -master_clock pclk_si -source [get_ports GPIO_0[0]] -divide_by 2 $pclk_si_div_pin

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
set ISL_inputs [get_ports {GPIO_0[32] GPIO_0[31] GPIO_0[30] GPIO_0[29] GPIO_0[28] GPIO_0[27] GPIO_0[26] GPIO_0[25] GPIO_0[24] GPIO_0[23] GPIO_0[22] GPIO_0[21] GPIO_0[20] GPIO_0[19] GPIO_0[18] GPIO_0[17] GPIO_0[16] GPIO_0[15] GPIO_0[14] GPIO_0[13] GPIO_0[12] GPIO_0[11] GPIO_0[10] GPIO_0[9] GPIO_0[8] GPIO_0[7] GPIO_0[6] GPIO_0[35]}]
set_input_delay -clock pclk_isl -clock_fall -min $ISL_dmin $ISL_inputs -add_delay
set_input_delay -clock pclk_isl -clock_fall -max $ISL_dmax $ISL_inputs -add_delay

# ADV7513 (0ns video clock delay adjustment)
set hdmitx_dmin -1.9
set hdmitx_dmax -0.2
set hdmitx_data_outputs [get_ports {HDMI_TX_D* HDMI_TX_HS HDMI_TX_VS HDMI_TX_DE}]
set_output_delay -clock pclk_si_out -min $hdmitx_dmin $hdmitx_data_outputs -add_delay
set_output_delay -clock pclk_si_out -max $hdmitx_dmax $hdmitx_data_outputs -add_delay

set_false_path -from [get_ports {GPIO_0[34] KEY* ARDUINO_IO[7] ARDUINO_IO[6] ARDUINO_IO[5] ARDUINO_IO[4]}]
set_false_path -to [get_ports {LED*}]

# TODO: set I2C constraints
set_false_path -from [get_ports {GPIO_0[4] GPIO_0[5] HDMI_I2C_SCL HDMI_I2C_SDA}]
set_false_path -to [get_ports {GPIO_0[4] GPIO_0[5] HDMI_I2C_SCL HDMI_I2C_SDA}]

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
                            {clk100} \
                            {clk150} \
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
