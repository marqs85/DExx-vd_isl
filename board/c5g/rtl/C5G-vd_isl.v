`define ENABLE_GPIO
`define HSMC_HDMI

module C5G_vd_isl (

      ///////// ADC /////////
      output             ADC_CONVST,
      output             ADC_SCK,
      output             ADC_SDI,
      input              ADC_SDO,

      ///////// AUD /////////
      input              AUD_ADCDAT,
      inout              AUD_ADCLRCK,
      inout              AUD_BCLK,
      output             AUD_DACDAT,
      inout              AUD_DACLRCK,
      output             AUD_XCK,

      ///////// CLOCK /////////
      input              CLOCK_125_p,
      input              CLOCK_50_B5B,
      input              CLOCK_50_B6A,
      input              CLOCK_50_B7A,
      input              CLOCK_50_B8A,

      ///////// CPU /////////
      input              CPU_RESET_n,

`ifdef ENABLE_DDR2LP
      ///////// DDR2LP /////////
      output      [9:0]  DDR2LP_CA,
      output      [1:0]  DDR2LP_CKE,
      output             DDR2LP_CK_n,
      output             DDR2LP_CK_p,
      output      [1:0]  DDR2LP_CS_n,
      output      [3:0]  DDR2LP_DM,
      inout       [31:0] DDR2LP_DQ,
      inout       [3:0]  DDR2LP_DQS_n,
      inout       [3:0]  DDR2LP_DQS_p,
      input              DDR2LP_OCT_RZQ,
`endif /*ENABLE_DDR2LP*/

`ifdef ENABLE_GPIO
      ///////// GPIO /////////
      inout       [35:0] GPIO,
`else
      ///////// HEX2 /////////
      output      [6:0]  HEX2,

      ///////// HEX3 /////////
      output      [6:0]  HEX3,
`endif /*ENABLE_GPIO*/

      ///////// HDMI /////////
      output             HDMI_TX_CLK,
      output      [23:0] HDMI_TX_D,
      output             HDMI_TX_DE,
      output             HDMI_TX_HS,
      input              HDMI_TX_INT,
      output             HDMI_TX_VS,

      ///////// HEX0 /////////
      output      [6:0]  HEX0,

      ///////// HEX1 /////////
      output      [6:0]  HEX1,

`ifdef HSMC_GENERIC
      ///////// HSMC /////////
      input              HSMC_CLKIN0,
      input       [2:1]  HSMC_CLKIN_n,
      input       [2:1]  HSMC_CLKIN_p,
      output             HSMC_CLKOUT0,
      output      [2:1]  HSMC_CLKOUT_n,
      output      [2:1]  HSMC_CLKOUT_p,
      inout       [3:0]  HSMC_D,
`ifdef ENABLE_HSMC_XCVR
      input       [3:0]  HSMC_GXB_RX_p,
      output      [3:0]  HSMC_GXB_TX_p,
`endif /*ENABLE_HSMC_XCVR*/
      inout       [16:0] HSMC_RX_n,
      inout       [16:0] HSMC_RX_p,
      inout       [16:0] HSMC_TX_n,
      inout       [16:0] HSMC_TX_p,
`endif /*HSMC_GENERIC*/

`ifdef HSMC_HDMI
      ///////// HSMC_HDMI /////////
      inout              HDMI_TX_HSMC_I2C_SCL,
      inout              HDMI_TX_HSMC_I2C_SDA,
      output             HDMI_TX_HSMC_I2S,
      output             HDMI_TX_HSMC_LRCLK,
      output             HDMI_TX_HSMC_MCLK,
      output             HDMI_TX_HSMC_SCLK,
      output             HDMI_TX_HSMC_SPDIF,
      output             HDMI_TX_HSMC_CLK,
      output reg    [35:0] HDMI_TX_HSMC_D,
      output reg         HDMI_TX_HSMC_DE,
      output reg         HDMI_TX_HSMC_HS,
      input              HDMI_TX_HSMC_INT,
      output reg         HDMI_TX_HSMC_VS,
      output             HDMI_TX_HSMC_RESET_N,
`endif /*HSMC_HDMI*/

      ///////// I2C /////////
      output             I2C_SCL,
      inout              I2C_SDA,

      ///////// KEY /////////
      input       [3:0]  KEY,

      ///////// LEDG /////////
      output      [7:0]  LEDG,

      ///////// LEDR /////////
      output      [9:0]  LEDR,

`ifdef ENABLE_REFCLK
      ///////// REFCLK /////////
      input              REFCLK_p0,
      input              REFCLK_p1,
`endif /*ENABLE_REFCLK*/

      ///////// SD /////////
      output             SD_CLK,
      inout              SD_CMD,
      inout       [3:0]  SD_DAT,

`ifdef ENABLE_SMA
      ///////// SMA /////////
      input              SMA_GXB_RX_p,
      output             SMA_GXB_TX_p,
`endif /*ENABLE_SMA*/

      ///////// SRAM /////////
      output      [17:0] SRAM_A,
      output             SRAM_CE_n,
      inout       [15:0] SRAM_D,
      output             SRAM_LB_n,
      output             SRAM_OE_n,
      output             SRAM_UB_n,
      output             SRAM_WE_n,

      ///////// SW /////////
      input       [9:0]  SW,

      ///////// UART /////////
      input              UART_RX,
      output             UART_TX
);


//=======================================================
//  REG/WIRE declarations
//=======================================================

wire clk27, PCLK_sc, pclk_out;
wire SI_PCLK_i = GPIO[0];
wire ISL_PCLK_i = GPIO[1];
wire sys_reset_n = 1'b1;
wire [7:0] ISL_R_i = {GPIO[22], GPIO[23], GPIO[24], GPIO[25], GPIO[26], GPIO[27], GPIO[28], GPIO[29]};
wire [7:0] ISL_G_i = {GPIO[14], GPIO[15], GPIO[16], GPIO[17], GPIO[18], GPIO[19], GPIO[20], GPIO[21]};
wire [7:0] ISL_B_i = {GPIO[6], GPIO[7], GPIO[8], GPIO[9], GPIO[10], GPIO[11], GPIO[12], GPIO[13]};
wire ISL_HS_i = GPIO[30];
wire ISL_HSYNC_i = GPIO[31];
wire ISL_VSYNC_i = GPIO[32];
wire ISL_FID_i = GPIO[35];
wire ISL_INT_N_i = GPIO[33];
wire IR_RX_i = GPIO[34];
wire pclk_capture = ISL_PCLK_i;
wire SPDIF_EXT_i = SW[9];

// Minimize impendance to GND on FPGA end
assign GPIO[2] = 1'b0;
assign GPIO[3] = 1'b0;

wire [15:0] sys_ctrl;
/*wire sys_poweron = sys_ctrl[0];
wire isl_reset_n = sys_ctrl[1];
wire hdmirx_reset_n = sys_ctrl[2];
wire emif_hwreset_n = sys_ctrl[3];
wire emif_swreset_n = sys_ctrl[4];
wire emif_powerdn_req = sys_ctrl[5];
wire emif_powerdn_mask = sys_ctrl[6];
wire capture_sel = sys_ctrl[7];*/
wire hdmitx_reset_n = sys_ctrl[2];
wire isl_hsync_pol = sys_ctrl[8];
wire isl_vsync_pol = sys_ctrl[9];
wire isl_vsync_type = sys_ctrl[10];
//wire audmux_sel = sys_ctrl[11];
wire testpattern_enable = sys_ctrl[12];
wire csc_enable = sys_ctrl[13];
wire adap_lm = sys_ctrl[14];

assign HDMI_TX_HSMC_RESET_N = hdmitx_reset_n;

//reg [1:0] clk_osc_div = 2'h0;
/*wire SCL = GPIO[5] & HDMI_I2C_SCL;
wire SDA = GPIO[4] & HDMI_I2C_SDA;*/
/*assign GPIO[5] = scl_oe ? 1'b0 : 1'bz;
assign GPIO[4] = sda_oe ? 1'b0 : 1'bz;*/
/*assign HDMI_I2C_SCL = scl_oe ? 1'b0 : 1'bz;
assign HDMI_I2C_SDA = sda_oe ? 1'b0 : 1'bz;*/

reg ir_rx_sync1_reg, ir_rx_sync2_reg;
reg [1:0] btn_sync1_reg, btn_sync2_reg;

wire [15:0] ir_code;
wire [7:0] ir_code_cnt;

wire scl_oe, sda_oe;
wire pll_lock;
wire nios_reset_req;

wire vs_flag = testpattern_enable ? 1'b0 : ~ISL_VSYNC_post;

wire [31:0] controls = {6'h0, btn_sync2_reg, ir_code_cnt, ir_code};
wire [31:0] sys_status = {32'h0};

wire [31:0] hv_in_config, hv_in_config2, hv_in_config3, hv_out_config, hv_out_config2, hv_out_config3, xy_out_config, xy_out_config2;
wire [31:0] misc_config, sl_config, sl_config2;

reg [23:0] resync_led_ctr;
reg resync_strobe_sync1_reg, resync_strobe_sync2_reg, resync_strobe_prev;
wire resync_strobe_i;
wire resync_strobe = resync_strobe_sync2_reg;

assign LEDG = {adap_lm, 6'h0, (ir_code == 0)};
assign LEDR = {10{(resync_led_ctr != 0)}};

wire [11:0] xpos;
wire [10:0] ypos;
wire osd_enable;
wire [1:0] osd_color;


// ISL51002 RGB digitizer
reg [7:0] ISL_R, ISL_G, ISL_B;
reg ISL_HS;
reg ISL_VS_sync1_reg, ISL_VS_sync2_reg;
reg ISL_HSYNC_sync1_reg, ISL_HSYNC_sync2_reg;
reg ISL_VSYNC_sync1_reg, ISL_VSYNC_sync2_reg;
reg ISL_FID_sync1_reg, ISL_FID_sync2_reg;
reg ISL_DE;
always @(posedge ISL_PCLK_i) begin
    ISL_R <= ISL_R_i;
    ISL_G <= ISL_G_i;
    ISL_B <= ISL_B_i;
    ISL_HS <= ISL_HS_i;

    // sync to pclk
    ISL_VS_sync1_reg <= ISL_VSYNC_i;
    ISL_VS_sync2_reg <= ISL_VS_sync1_reg;
end
always @(posedge clk27) begin
    // sync to always-running fixed meas clk
    ISL_HSYNC_sync1_reg <= ISL_HSYNC_i;
    ISL_HSYNC_sync2_reg <= ISL_HSYNC_sync1_reg;
    ISL_VSYNC_sync1_reg <= ISL_VSYNC_i;
    ISL_VSYNC_sync2_reg <= ISL_VSYNC_sync1_reg;
    ISL_FID_sync1_reg <= ISL_FID_i;
    ISL_FID_sync2_reg <= ISL_FID_sync1_reg;
end

wire [7:0] ISL_R_post, ISL_G_post, ISL_B_post;
wire ISL_HSYNC_post, ISL_VSYNC_post, ISL_DE_post, ISL_FID_post;
wire ISL_fe_interlace, ISL_fe_frame_change;
wire [19:0] ISL_fe_pcnt_frame;
wire [10:0] ISL_fe_vtotal, ISL_fe_xpos, ISL_fe_ypos;
isl51002_frontend u_isl_frontend ( 
    .PCLK_i(ISL_PCLK_i),
    .CLK_MEAS_i(clk27),
    .reset_n(sys_reset_n),
    .R_i(ISL_R),
    .G_i(ISL_G),
    .B_i(ISL_B),
    .HS_i(ISL_HS),
    .VS_i(ISL_VS_sync2_reg),
    .HSYNC_i(ISL_HSYNC_sync2_reg),
    .VSYNC_i(ISL_VSYNC_sync2_reg),
    .DE_i(ISL_DE),
    .FID_i(ISL_FID_sync2_reg),
    .hsync_i_polarity(isl_hsync_pol),
    .vsync_i_polarity(isl_vsync_pol),
    .vsync_i_type(isl_vsync_type),
    .csc_enable(csc_enable),
    .csc_cs(misc_config[14]),
    .hv_in_config(hv_in_config),
    .hv_in_config2(hv_in_config2),
    .hv_in_config3(hv_in_config3),
    .R_o(ISL_R_post),
    .G_o(ISL_G_post),
    .B_o(ISL_B_post),
    .HSYNC_o(ISL_HSYNC_post),
    .VSYNC_o(ISL_VSYNC_post),
    .DE_o(ISL_DE_post),
    .FID_o(ISL_FID_post),
    .interlace_flag(ISL_fe_interlace),
    .xpos_o(ISL_fe_xpos),
    .ypos_o(ISL_fe_ypos),
    .vtotal(ISL_fe_vtotal),
    .frame_change(ISL_fe_frame_change),
    .pcnt_frame(ISL_fe_pcnt_frame)
);

// output clock assignment
assign pclk_out = PCLK_sc;
assign HDMI_TX_CLK = pclk_out;
assign HDMI_TX_HSMC_CLK = ~pclk_out;

// output data assignment (2 stages and launch on negedge for timing closure)
reg [7:0] R_out, G_out, B_out;
reg HSYNC_out, VSYNC_out, DE_out;
wire [7:0] R_sc, G_sc, B_sc;
wire HSYNC_sc, VSYNC_sc, DE_sc;

always @(posedge pclk_out) begin
    if (osd_enable) begin
        if (osd_color == 2'h0) begin
            {R_out, G_out, B_out} <= 24'h000000;
        end else if (osd_color == 2'h1) begin
            {R_out, G_out, B_out} <= 24'h0000ff;
        end else if (osd_color == 2'h2) begin
            {R_out, G_out, B_out} <= 24'hffff00;
        end else begin
            {R_out, G_out, B_out} <= 24'hffffff;
        end
    end else begin
        {R_out, G_out, B_out} <= {R_sc, G_sc, B_sc};
    end

    HSYNC_out <= HSYNC_sc;
    VSYNC_out <= VSYNC_sc;
    DE_out <= DE_sc;
end

always @(negedge pclk_out) begin
    HDMI_TX_D[23:16] <= R_out;
    HDMI_TX_D[15:8] <= G_out;
    HDMI_TX_D[7:0] <= B_out;
    HDMI_TX_HS <= HSYNC_out;
    HDMI_TX_VS <= VSYNC_out;
    HDMI_TX_DE <= DE_out;
end

always @(posedge pclk_out) begin
    HDMI_TX_HSMC_D[35:28] <= R_out;
    HDMI_TX_HSMC_D[23:16] <= G_out;
    HDMI_TX_HSMC_D[11:4] <= B_out;
    HDMI_TX_HSMC_HS <= HSYNC_out;
    HDMI_TX_HSMC_VS <= VSYNC_out;
    HDMI_TX_HSMC_DE <= DE_out;
    {HDMI_TX_HSMC_D[27:24], HDMI_TX_HSMC_D[15:12], HDMI_TX_HSMC_D[3:0]} = 12'h0;
end

//audio
assign HDMI_TX_HSMC_SCLK = SW[8];
assign HDMI_TX_HSMC_LRCLK = SW[6];
assign HDMI_TX_HSMC_I2S = SW[7];
assign HDMI_TX_HSMC_SPDIF = SPDIF_EXT_i;

always @(posedge clk27) begin
    if (~resync_strobe_prev & resync_strobe) begin
        resync_led_ctr <= {24{1'b1}};
    end else if (resync_led_ctr > 0) begin
        resync_led_ctr <= resync_led_ctr - 1'b1;
    end

    resync_strobe_sync1_reg <= resync_strobe_i;
    resync_strobe_sync2_reg <= resync_strobe_sync1_reg;
    resync_strobe_prev <= resync_strobe_sync2_reg;
end

// Insert synchronizers to async inputs (synchronize to CPU clock)
always @(posedge clk27 or negedge sys_reset_n) begin
    if (!sys_reset_n) begin
        btn_sync1_reg <= 2'b11;
        btn_sync2_reg <= 2'b11;
        ir_rx_sync1_reg <= 1'b1;
        ir_rx_sync2_reg <= 1'b1;
    end else begin
        btn_sync1_reg <= KEY;
        btn_sync2_reg <= btn_sync1_reg;
        ir_rx_sync1_reg <= IR_RX_i;
        ir_rx_sync2_reg <= ir_rx_sync1_reg;
    end
end

pll pll_sys (
    .refclk(CLOCK_50_B5B),
    .rst(1'b0),
    .outclk_0(clk27),
    .locked(pll_lock)
);

sys u0 (
    .clk_clk                 (clk27),                 //              clk.clk
    .reset_reset_n           (sys_reset_n),            //            reset.reset_n
    /*.i2c_0_i2c_serial_sda_in (HDMI_I2C_SDA), // i2c_0_i2c_serial.sda_in
    .i2c_0_i2c_serial_scl_in (HDMI_I2C_SCL), //                 .scl_in
    .i2c_0_i2c_serial_sda_oe (sda_oe), //                 .sda_oe
    .i2c_0_i2c_serial_scl_oe (scl_oe), //                 .scl_oe*/
    .i2c_opencores_0_export_scl_pad_io      (GPIO[5]),
    .i2c_opencores_0_export_sda_pad_io      (GPIO[4]),
    .i2c_opencores_0_export_spi_miso_pad_i  (1'b0),
    .i2c_opencores_1_export_scl_pad_io      (I2C_SCL),
    .i2c_opencores_1_export_sda_pad_io      (I2C_SDA),
    .i2c_opencores_1_export_spi_miso_pad_i  (1'b0),
    .i2c_opencores_2_export_scl_pad_io      (HDMI_TX_HSMC_I2C_SCL),
    .i2c_opencores_2_export_sda_pad_io      (HDMI_TX_HSMC_I2C_SDA),
    .i2c_opencores_2_export_spi_miso_pad_i  (1'b0),
    .pio_0_sys_ctrl_out_export              (sys_ctrl),
    .pio_1_controls_in_export               (controls),
    .sc_config_0_sc_if_fe_status_i          ({20'h0, ISL_fe_interlace, ISL_fe_vtotal}),
    .sc_config_0_sc_if_fe_status2_i         ({12'h0, ISL_fe_pcnt_frame}),
    .sc_config_0_sc_if_lt_status_i          (32'h00000000),
    .sc_config_0_sc_if_hv_in_config_o       (hv_in_config),
    .sc_config_0_sc_if_hv_in_config2_o      (hv_in_config2),
    .sc_config_0_sc_if_hv_in_config3_o      (hv_in_config3),
    .sc_config_0_sc_if_hv_out_config_o      (hv_out_config),
    .sc_config_0_sc_if_hv_out_config2_o     (hv_out_config2),
    .sc_config_0_sc_if_hv_out_config3_o     (hv_out_config3),
    .sc_config_0_sc_if_xy_out_config_o      (xy_out_config),
    .sc_config_0_sc_if_xy_out_config2_o     (xy_out_config2),
    .sc_config_0_sc_if_misc_config_o        (misc_config),
    .sc_config_0_sc_if_sl_config_o          (sl_config),
    .sc_config_0_sc_if_sl_config2_o         (sl_config2),
    .osd_generator_0_osd_if_vclk            (PCLK_sc),
    .osd_generator_0_osd_if_xpos            (xpos),
    .osd_generator_0_osd_if_ypos            (ypos),
    .osd_generator_0_osd_if_osd_enable      (osd_enable),
    .osd_generator_0_osd_if_osd_color       (osd_color)
);

scanconverter scanconverter_inst (
    .PCLK_CAP_i(pclk_capture),
    .PCLK_OUT_i(SI_PCLK_i),
    .reset_n(sys_reset_n),  //TODO: sync to pclk_capture
    .R_i(ISL_R_post),
    .G_i(ISL_G_post),
    .B_i(ISL_B_post),
    .HSYNC_i(ISL_HSYNC_post),
    .VSYNC_i(ISL_VSYNC_post),
    .DE_i(ISL_DE_post),
    .FID_i(ISL_FID_post),
    .interlaced_in_i(ISL_fe_interlace),
    .frame_change_i(ISL_fe_frame_change),
    .xpos_i(ISL_fe_xpos),
    .ypos_i(ISL_fe_ypos),
    .hv_out_config(hv_out_config),
    .hv_out_config2(hv_out_config2),
    .hv_out_config3(hv_out_config3),
    .xy_out_config(xy_out_config),
    .xy_out_config2(xy_out_config2),
    .misc_config(misc_config),
    .sl_config(sl_config),
    .sl_config2(sl_config2),
    .testpattern_enable(testpattern_enable),
    .PCLK_o(PCLK_sc),
    .R_o(R_sc),
    .G_o(G_sc),
    .B_o(B_sc),
    .HSYNC_o(HSYNC_sc),
    .VSYNC_o(VSYNC_sc),
    .DE_o(DE_sc),
    .xpos_o(xpos),
    .ypos_o(ypos),
    .resync_strobe(resync_strobe_i)
);

ir_rcv ir0 (
    .clk27          (clk27),
    .reset_n        (sys_reset_n),
    .ir_rx          (ir_rx_sync2_reg),
    .ir_code        (ir_code),
    .ir_code_ack    (),
    .ir_code_cnt    (ir_code_cnt)
);

endmodule
