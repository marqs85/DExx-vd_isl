`define ENABLE_HPS
`define VIP
`define PIXPAR2

module DE10_Nano_vd_isl (

      ///////// ADC /////////
      output             ADC_CONVST,
      output             ADC_SCK,
      output             ADC_SDI,
      input              ADC_SDO,

      ///////// ARDUINO /////////
      inout       [15:0] ARDUINO_IO,
      inout              ARDUINO_RESET_N,

      ///////// FPGA /////////
      input              FPGA_CLK1_50,
      input              FPGA_CLK2_50,
      input              FPGA_CLK3_50,

      ///////// GPIO /////////
      inout       [35:0] GPIO_0,
      inout       [35:0] GPIO_1,

      ///////// HDMI /////////
      inout              HDMI_I2C_SCL,
      inout              HDMI_I2C_SDA,
      output             HDMI_I2S,
      output             HDMI_LRCLK,
      output             HDMI_MCLK,
      output             HDMI_SCLK,
      output             HDMI_TX_CLK,
      output reg    [23:0] HDMI_TX_D,
      output reg         HDMI_TX_DE,
      output reg         HDMI_TX_HS,
      input              HDMI_TX_INT,
      output reg         HDMI_TX_VS,

`ifdef ENABLE_HPS
      ///////// HPS /////////
      //inout              HPS_CONV_USB_N,
      output      [14:0] HPS_DDR3_ADDR,
      output      [2:0]  HPS_DDR3_BA,
      output             HPS_DDR3_CAS_N,
      output             HPS_DDR3_CKE,
      output             HPS_DDR3_CK_N,
      output             HPS_DDR3_CK_P,
      output             HPS_DDR3_CS_N,
      output      [3:0]  HPS_DDR3_DM,
      inout       [31:0] HPS_DDR3_DQ,
      inout       [3:0]  HPS_DDR3_DQS_N,
      inout       [3:0]  HPS_DDR3_DQS_P,
      output             HPS_DDR3_ODT,
      output             HPS_DDR3_RAS_N,
      output             HPS_DDR3_RESET_N,
      input              HPS_DDR3_RZQ,
      output             HPS_DDR3_WE_N,
      /*output             HPS_ENET_GTX_CLK,
      inout              HPS_ENET_INT_N,
      output             HPS_ENET_MDC,
      inout              HPS_ENET_MDIO,
      input              HPS_ENET_RX_CLK,
      input       [3:0]  HPS_ENET_RX_DATA,
      input              HPS_ENET_RX_DV,
      output      [3:0]  HPS_ENET_TX_DATA,
      output             HPS_ENET_TX_EN,
      inout              HPS_GSENSOR_INT,
      inout              HPS_I2C0_SCLK,
      inout              HPS_I2C0_SDAT,
      inout              HPS_I2C1_SCLK,
      inout              HPS_I2C1_SDAT,
      inout              HPS_KEY,*/
      inout              HPS_LED,
      /*inout              HPS_LTC_GPIO,*/
      inout              HPS_SD_CLK,
      inout              HPS_SD_CMD,
      inout       [3:0]  HPS_SD_DATA,
      /*output             HPS_SPIM_CLK,
      input              HPS_SPIM_MISO,
      output             HPS_SPIM_MOSI,
      inout              HPS_SPIM_SS,
      input              HPS_UART_RX,
      output             HPS_UART_TX,
      input              HPS_USB_CLKOUT,
      inout       [7:0]  HPS_USB_DATA,
      input              HPS_USB_DIR,
      input              HPS_USB_NXT,
      output             HPS_USB_STP,*/
`endif /*ENABLE_HPS*/

      ///////// KEY /////////
      input       [1:0]  KEY,

      ///////// LED /////////
      output      [7:0]  LED,

      ///////// SW /////////
      input       [3:0]  SW
);


//=======================================================
//  REG/WIRE declarations
//=======================================================

wire clk27, clk_vip, clk_100, PCLK_sc, pclk_out;
wire SI_PCLK_i = GPIO_0[0];
wire ISL_PCLK_i = GPIO_0[2];
wire sys_reset_n = 1'b1;
wire [7:0] ISL_R_i = {GPIO_0[22], GPIO_0[23], GPIO_0[24], GPIO_0[25], GPIO_0[26], GPIO_0[27], GPIO_0[28], GPIO_0[29]};
wire [7:0] ISL_G_i = {GPIO_0[14], GPIO_0[15], GPIO_0[16], GPIO_0[17], GPIO_0[18], GPIO_0[19], GPIO_0[20], GPIO_0[21]};
wire [7:0] ISL_B_i = {GPIO_0[6], GPIO_0[7], GPIO_0[8], GPIO_0[9], GPIO_0[10], GPIO_0[11], GPIO_0[12], GPIO_0[13]};
wire ISL_HS_i = GPIO_0[30];
wire ISL_HSYNC_i = GPIO_0[31];
wire ISL_VSYNC_i = GPIO_0[32];
wire ISL_FID_i = GPIO_0[35];
wire ISL_INT_N_i = GPIO_0[33];
wire IR_RX_i = GPIO_0[34];
wire pclk_capture = ISL_PCLK_i;
wire SPDIF_EXT_i = ARDUINO_IO[4];

// Minimize impendance to GND on FPGA end
// (may improve signal integrity with older revision boards but has opposite effect with v1.3->)
//assign GPIO_0[1] = 1'b0;
//assign GPIO_0[3] = 1'b0;

wire [15:0] sys_ctrl;
/*wire sys_poweron = sys_ctrl[0];
wire isl_reset_n = sys_ctrl[1];
wire hdmirx_reset_n = sys_ctrl[2];
wire emif_hwreset_n = sys_ctrl[3];
wire emif_swreset_n = sys_ctrl[4];
wire emif_powerdn_req = sys_ctrl[5];*/
wire vip_reset_n = sys_ctrl[6];
//wire capture_sel = sys_ctrl[7];
wire isl_hsync_pol = sys_ctrl[8];
wire isl_vsync_pol = sys_ctrl[9];
wire isl_vsync_type = sys_ctrl[10];
//wire audmux_sel = sys_ctrl[11];
wire testpattern_enable = sys_ctrl[12];
wire csc_enable = sys_ctrl[13];
wire framelock = sys_ctrl[14];

//reg [1:0] clk_osc_div = 2'h0;
/*wire SCL = GPIO_0[5] & HDMI_I2C_SCL;
wire SDA = GPIO_0[4] & HDMI_I2C_SDA;*/
/*assign GPIO_0[5] = scl_oe ? 1'b0 : 1'bz;
assign GPIO_0[4] = sda_oe ? 1'b0 : 1'bz;*/
/*assign HDMI_I2C_SCL = scl_oe ? 1'b0 : 1'bz;
assign HDMI_I2C_SDA = sda_oe ? 1'b0 : 1'bz;*/

reg ir_rx_sync1_reg, ir_rx_sync2_reg;
reg [5:0] btn_sync1_reg, btn_sync2_reg;

wire [15:0] ir_code;
wire [7:0] ir_code_cnt;

reg pclk_capture_div2, pclk_out_div2;

/* EMIF IF for LM */
wire emif_br_clk;
wire [27:0] emif_rd_addr, emif_wr_addr;
wire [255:0] emif_rd_rdata, emif_wr_wdata;
wire [5:0] emif_rd_burstcount, emif_wr_burstcount;
wire emif_rd_read, emif_rd_waitrequest, emif_rd_readdatavalid, emif_wr_write, emif_wr_waitrequest;

wire scl_oe, sda_oe;
wire pll_locked;
wire nios_reset_req;

wire cvi_overflow, cvo_underflow;

wire vs_flag = testpattern_enable ? 1'b0 : ~ISL_VSYNC_post;

wire [31:0] controls = {2'h0, btn_sync2_reg, ir_code_cnt, ir_code};
wire [31:0] sys_status = {32'h0};

wire [31:0] hv_in_config, hv_in_config2, hv_in_config3, hv_out_config, hv_out_config2, hv_out_config3, xy_out_config, xy_out_config2;
wire [31:0] misc_config, sl_config, sl_config2, sl_config3;

reg [23:0] resync_led_ctr;
reg resync_strobe_sync1_reg, resync_strobe_sync2_reg, resync_strobe_prev;
wire resync_strobe_i;
wire resync_strobe = resync_strobe_sync2_reg;

wire resync_led;
assign {LED[0], LED[7], resync_led} = {framelock, (ir_code == 0), (resync_led_ctr != 0)};

wire [11:0] xpos_sc;
wire [10:0] ypos_sc;
wire osd_enable;
wire [1:0] osd_color;

wire sdc_clk_o;
wire sdc_clk_oe_o = 1'b1;
wire sdc_dat_oe_o, sdc_cmd_oe_o, sdc_cmd_o, sdc_cmd_i;
wire [3:0] sdc_dat_o, sdc_dat_i;

wire [66:0] hps_h2f_loan_io_in;
wire [66:0] hps_h2f_loan_io_out = {13'h0, resync_led, 5'h0, sdc_dat_o[3:2], sdc_clk_o, 5'h0, sdc_dat_o[1:0], 1'h0, sdc_cmd_o, 36'h0};
wire [66:0] hps_h2f_loan_io_oe = {13'h0, 1'b1, 5'h0, {2{sdc_dat_oe_o}}, sdc_clk_oe_o, 5'h0, {2{sdc_dat_oe_o}}, 1'h0, sdc_cmd_oe_o, 36'h0};

assign sdc_dat_i[3:2] = hps_h2f_loan_io_in[47:46];
assign sdc_dat_i[1:0] = hps_h2f_loan_io_in[39:38];
assign sdc_cmd_i = hps_h2f_loan_io_in[36];

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
wire ISL_HSYNC_post, ISL_VSYNC_post, ISL_DE_post, ISL_FID_post, ISL_datavalid_post;
wire ISL_fe_interlace, ISL_fe_frame_change, ISL_sof_scaler;
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
    .datavalid_o(ISL_datavalid_post),
    .xpos_o(ISL_fe_xpos),
    .ypos_o(ISL_fe_ypos),
    .vtotal(ISL_fe_vtotal),
    .frame_change(ISL_fe_frame_change),
    .sof_scaler(ISL_sof_scaler),
    .pcnt_frame(ISL_fe_pcnt_frame)
);

wire [7:0] R_capt = ISL_R_post;
wire [7:0] G_capt = ISL_G_post;
wire [7:0] B_capt = ISL_B_post;
wire HSYNC_capt = ISL_HSYNC_post;
wire VSYNC_capt = ISL_VSYNC_post;
wire DE_capt = ISL_DE_post;
wire FID_capt = ISL_FID_post;
wire interlace_flag_capt = ISL_fe_interlace;
wire datavalid_capt = ISL_datavalid_post;
wire frame_change_capt = ISL_fe_frame_change;
wire sof_scaler_capt = ISL_sof_scaler;
wire [10:0] xpos_capt = ISL_fe_xpos;
wire [10:0] ypos_capt = ISL_fe_ypos;

// output clock assignment
assign pclk_out = PCLK_sc;
assign HDMI_TX_CLK = pclk_out;


// VIP
wire vip_select = misc_config[15];

always @(posedge pclk_capture) begin
    pclk_capture_div2 <= pclk_capture_div2 ^ 1'b1;
end

always @(posedge pclk_out) begin
    pclk_out_div2 <= pclk_out_div2 ^ 1'b1;
end

`ifdef VIP
`ifdef PIXPAR2
wire [47:0] VIP_DATA_o;
wire [1:0] VIP_HSYNC_o, VIP_VSYNC_o, VIP_DE_o;

`ifdef DIV2_SYNC
reg [47:0] VIP_DATA_i;
reg [1:0] VIP_HSYNC_i, VIP_VSYNC_i, VIP_DE_i, VIP_FID_i;
reg [7:0] R_vip, G_vip, B_vip;
reg HSYNC_vip, VSYNC_vip, DE_vip;

always @(posedge pclk_capture) begin
    if (pclk_capture_div2 == 0) begin
        VIP_DATA_i[47:24] <= {R_capt, G_capt, B_capt};
        {VIP_HSYNC_i[1], VIP_VSYNC_i[1], VIP_DE_i[1], VIP_FID_i[1]} <= {~HSYNC_capt, ~VSYNC_capt, DE_capt & datavalid_capt, ~FID_capt};
    end else begin
        VIP_DATA_i[23:0] <= {R_capt, G_capt, B_capt};
        {VIP_HSYNC_i[0], VIP_VSYNC_i[0], VIP_DE_i[0], VIP_FID_i[0]} <= {~HSYNC_capt, ~VSYNC_capt, DE_capt & datavalid_capt, ~FID_capt};
    end
end

always @(posedge pclk_out) begin
    if (pclk_out_div2 == 0) begin
        {R_vip, G_vip, B_vip} <= VIP_DATA_o[23:0];
        {HSYNC_vip, VSYNC_vip, DE_vip} <= {~VIP_HSYNC_o[0], ~VIP_VSYNC_o[0], VIP_DE_o[0]};
    end else begin
        {R_vip, G_vip, B_vip} <= VIP_DATA_o[47:24];
        {HSYNC_vip, VSYNC_vip, DE_vip} <= {~VIP_HSYNC_o[1], ~VIP_VSYNC_o[1], VIP_DE_o[1]};
    end
end
`else // DIV2_SYNC
wire [47:0] VIP_DATA_i;
wire [1:0] VIP_HSYNC_i, VIP_VSYNC_i, VIP_DE_i, VIP_FID_i;
wire [7:0] R_vip, G_vip, B_vip;
wire HSYNC_vip, VSYNC_vip, DE_vip;
wire [3:0] unused1, unused0;
wire [4:0] unused_out;
wire dc_fifo_in_rdempty, dc_fifo_in_wrfull;
wire dc_fifo_out_rdempty, dc_fifo_out_wrfull;
reg dc_fifo_in_rdempty_prev;

dc_fifo_in  dc_fifo_in_inst (
    .data({R_capt, G_capt, B_capt, ~HSYNC_capt, ~VSYNC_capt, DE_capt, ~FID_capt, 4'h0}),
    .rdclk(pclk_capture_div2),
    .rdreq(!dc_fifo_in_rdempty),
    .rdempty(dc_fifo_in_rdempty),
    .wrclk(pclk_capture),
    .wrreq(datavalid_capt & !dc_fifo_in_wrfull),
    .wrfull(dc_fifo_in_wrfull),
    .q({VIP_DATA_i[47:24], VIP_HSYNC_i[1], VIP_VSYNC_i[1], VIP_DE_i[1], VIP_FID_i[1], unused1, VIP_DATA_i[23:0], VIP_HSYNC_i[0], VIP_VSYNC_i[0], VIP_DE_i[0], VIP_FID_i[0], unused0})
);

dc_fifo_out  dc_fifo_out_inst (
    .data({VIP_DATA_o[47:24], VIP_HSYNC_o[1], VIP_VSYNC_o[1], VIP_DE_o[1], 5'h0, VIP_DATA_o[23:0], VIP_HSYNC_o[0], VIP_VSYNC_o[0], VIP_DE_o[0], 5'h0}),
    .rdclk(pclk_out),
    .rdreq(!dc_fifo_out_rdempty),
    .rdempty(dc_fifo_out_rdempty),
    .wrclk(pclk_out_div2),
    .wrreq(!dc_fifo_out_wrfull),
    .wrfull(dc_fifo_out_wrfull),
    .q({R_vip, G_vip, B_vip, HSYNC_vip, VSYNC_vip, DE_vip, unused_out})
);

always @(posedge pclk_capture_div2) begin
    dc_fifo_in_rdempty_prev <= dc_fifo_in_rdempty;
end
`endif // DIV2_SYNC
`else // PIXPAR2
wire [23:0] VIP_DATA_i = {R_capt, G_capt, B_capt};
wire VIP_HSYNC_i = ~HSYNC_capt;
wire VIP_VSYNC_i = ~VSYNC_capt;
wire VIP_DE_i = DE_capt & datavalid_capt;
wire VIP_FID_i = ~FID_capt;
wire [23:0] VIP_DATA_o;
wire [7:0] R_vip = VIP_DATA_o[23:16];
wire [7:0] G_vip = VIP_DATA_o[15:8];
wire [7:0] B_vip = VIP_DATA_o[7:0];
wire VIP_HSYNC_o, VIP_VSYNC_o, VIP_DE_o;
wire HSYNC_vip = VIP_HSYNC_o;
wire VSYNC_vip = VIP_VSYNC_o;
wire DE_vip = VIP_DE_o;
`endif // PIXPAR2

reg VSYNC_vip_prev;
wire vip_frame_start = VSYNC_vip_prev & ~VSYNC_vip;

always @(posedge pclk_out) begin
    VSYNC_vip_prev <= VSYNC_vip;
end
`endif // VIP

// output data assignment (2 stages and launch on negedge for timing closure)
reg [7:0] R_out, G_out, B_out;
reg HSYNC_out, VSYNC_out, DE_out;
wire [7:0] R_sc, G_sc, B_sc;
wire HSYNC_sc, VSYNC_sc, DE_sc;

always @(negedge pclk_out) begin
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

//audio
assign HDMI_SCLK = ARDUINO_IO[5];
assign HDMI_LRCLK = ARDUINO_IO[7];
assign HDMI_I2S = ARDUINO_IO[6];

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
        btn_sync1_reg <= '1;
        btn_sync2_reg <= '1;
        ir_rx_sync1_reg <= 1'b1;
        ir_rx_sync2_reg <= 1'b1;
    end else begin
        btn_sync1_reg <= {KEY[1], 1'b1, KEY[0], 3'h7};
        btn_sync2_reg <= btn_sync1_reg;
        ir_rx_sync1_reg <= IR_RX_i;
        ir_rx_sync2_reg <= ir_rx_sync1_reg;
    end
end

pll pll_sys (
    .refclk(FPGA_CLK1_50),
    .rst(1'b0),
    .outclk_0(clk27),
    .outclk_1(clk_vip),
    .outclk_2(clk_100),
    .locked(pll_locked)
);

sys sys_inst (
    .clk_clk                 (clk27),
    .clk_1_clk               (FPGA_CLK1_50),
    .clk_2_clk               (clk_vip),
    .clk_3_clk               (clk_100),
    .reset_reset_n           (sys_reset_n),
    .reset_2_reset_n         (vip_reset_n),
    .reset_3_reset_n         (vip_reset_n),
    /*.i2c_0_i2c_serial_sda_in (HDMI_I2C_SDA), // i2c_0_i2c_serial.sda_in
    .i2c_0_i2c_serial_scl_in (HDMI_I2C_SCL), //                 .scl_in
    .i2c_0_i2c_serial_sda_oe (sda_oe), //                 .sda_oe
    .i2c_0_i2c_serial_scl_oe (scl_oe), //                 .scl_oe*/
    .sdc_controller_0_sd_sd_cmd_dat_i       (sdc_cmd_i),
    .sdc_controller_0_sd_sd_cmd_out_o       (sdc_cmd_o),
    .sdc_controller_0_sd_sd_cmd_oe_o        (sdc_cmd_oe_o),
    .sdc_controller_0_sd_sd_dat_dat_i       (sdc_dat_i),
    .sdc_controller_0_sd_sd_dat_out_o       (sdc_dat_o),
    .sdc_controller_0_sd_sd_dat_oe_o        (sdc_dat_oe_o),
    .sdc_controller_0_sd_clk_o_clk          (sdc_clk_o),
    .hps_h2f_loan_io_in                     (hps_h2f_loan_io_in),
    .hps_h2f_loan_io_out                    (hps_h2f_loan_io_out),
    .hps_h2f_loan_io_oe                     (hps_h2f_loan_io_oe),
    .hps_io_hps_io_gpio_inst_LOANIO36       (HPS_SD_CMD), 
    .hps_io_hps_io_gpio_inst_LOANIO38       (HPS_SD_DATA[0]),
    .hps_io_hps_io_gpio_inst_LOANIO39       (HPS_SD_DATA[1]),
    .hps_io_hps_io_gpio_inst_LOANIO45       (HPS_SD_CLK),
    .hps_io_hps_io_gpio_inst_LOANIO46       (HPS_SD_DATA[2]),
    .hps_io_hps_io_gpio_inst_LOANIO47       (HPS_SD_DATA[3]),
    .hps_io_hps_io_gpio_inst_LOANIO53       (HPS_LED),
    .i2c_opencores_0_export_scl_pad_io      (GPIO_0[5]),
    .i2c_opencores_0_export_sda_pad_io      (GPIO_0[4]),
    .i2c_opencores_0_export_spi_miso_pad_i  (1'b0),
    .i2c_opencores_1_export_scl_pad_io      (HDMI_I2C_SCL),
    .i2c_opencores_1_export_sda_pad_io      (HDMI_I2C_SDA),
    .i2c_opencores_1_export_spi_miso_pad_i  (1'b0),
    .pio_0_sys_ctrl_out_export              (sys_ctrl),
    .pio_1_controls_in_export               (controls),
    .pio_2_sys_status_in_export             (sys_status),
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
    .sc_config_0_sc_if_sl_config3_o         (sl_config3),
    .osd_generator_0_osd_if_vclk            (PCLK_sc),
    .osd_generator_0_osd_if_xpos            (xpos_sc),
    .osd_generator_0_osd_if_ypos            (ypos_sc),
    .osd_generator_0_osd_if_osd_enable      (osd_enable),
    .osd_generator_0_osd_if_osd_color       (osd_color),
    .emif_bridge_0_clk_o                    (emif_br_clk),
    .emif_bridge_0_wr_address               ({4'h0, emif_wr_addr}),
    .emif_bridge_0_wr_write                 (emif_wr_write),
    .emif_bridge_0_wr_write_data            (emif_wr_wdata),
    .emif_bridge_0_wr_waitrequest           (emif_wr_waitrequest),
    .emif_bridge_0_wr_burstcount            (emif_wr_burstcount),
    .emif_bridge_0_rd_address               ({4'h0, emif_rd_addr}),
    .emif_bridge_0_rd_read                  (emif_rd_read),
    .emif_bridge_0_rd_read_data             (emif_rd_rdata),
    .emif_bridge_0_rd_waitrequest           (emif_rd_waitrequest),
    .emif_bridge_0_rd_readdatavalid         (emif_rd_readdatavalid),
    .emif_bridge_0_rd_burstcount            (emif_rd_burstcount),
    .memory_mem_a                           ( HPS_DDR3_ADDR),                       //                memory.mem_a
    .memory_mem_ba                          ( HPS_DDR3_BA),                         //                .mem_ba
    .memory_mem_ck                          ( HPS_DDR3_CK_P),                       //                .mem_ck
    .memory_mem_ck_n                        ( HPS_DDR3_CK_N),                       //                .mem_ck_n
    .memory_mem_cke                         ( HPS_DDR3_CKE),                        //                .mem_cke
    .memory_mem_cs_n                        ( HPS_DDR3_CS_N),                       //                .mem_cs_n
    .memory_mem_ras_n                       ( HPS_DDR3_RAS_N),                      //                .mem_ras_n
    .memory_mem_cas_n                       ( HPS_DDR3_CAS_N),                      //                .mem_cas_n
    .memory_mem_we_n                        ( HPS_DDR3_WE_N),                       //                .mem_we_n
    .memory_mem_reset_n                     ( HPS_DDR3_RESET_N),                    //                .mem_reset_n
    .memory_mem_dq                          ( HPS_DDR3_DQ),                         //                .mem_dq
    .memory_mem_dqs                         ( HPS_DDR3_DQS_P),                      //                .mem_dqs
    .memory_mem_dqs_n                       ( HPS_DDR3_DQS_N),                      //                .mem_dqs_n
    .memory_mem_odt                         ( HPS_DDR3_ODT),                        //                .mem_odt
    .memory_mem_dm                          ( HPS_DDR3_DM),                         //                .mem_dm
    .memory_oct_rzqin                       ( HPS_DDR3_RZQ),                        //                .oct_rzqin
`ifdef VIP
    .alt_vip_cl_cvi_0_clocked_video_vid_clk                    (
`ifdef PIXPAR2
    pclk_capture_div2
`else
    pclk_capture
`endif
    ),
    .alt_vip_cl_cvi_0_clocked_video_vid_data                   (VIP_DATA_i),
    .alt_vip_cl_cvi_0_clocked_video_vid_de                     (VIP_DE_i),
    .alt_vip_cl_cvi_0_clocked_video_vid_datavalid              (!dc_fifo_in_rdempty_prev),
    .alt_vip_cl_cvi_0_clocked_video_vid_locked                 (1'b1),
    .alt_vip_cl_cvi_0_clocked_video_vid_f                      (VIP_FID_i),
    .alt_vip_cl_cvi_0_clocked_video_vid_v_sync                 (VIP_VSYNC_i),
    .alt_vip_cl_cvi_0_clocked_video_vid_h_sync                 (VIP_HSYNC_i),
    .alt_vip_cl_cvi_0_clocked_video_vid_color_encoding         (0),
    .alt_vip_cl_cvi_0_clocked_video_vid_bit_width              (0),
    .alt_vip_cl_cvi_0_clocked_video_sof                        (),
    .alt_vip_cl_cvi_0_clocked_video_sof_locked                 (),
    .alt_vip_cl_cvi_0_clocked_video_refclk_div                 (),
    .alt_vip_cl_cvi_0_clocked_video_clipping                   (),
    .alt_vip_cl_cvi_0_clocked_video_padding                    (),
    .alt_vip_cl_cvi_0_clocked_video_overflow                   (cvi_overflow),
    .alt_vip_cl_cvo_0_clocked_video_vid_clk                    (
`ifdef PIXPAR2
    pclk_out_div2
`else
    pclk_out
`endif
    ),
    .alt_vip_cl_cvo_0_clocked_video_vid_data                   (VIP_DATA_o),
    .alt_vip_cl_cvo_0_clocked_video_underflow                  (cvo_underflow),
    .alt_vip_cl_cvo_0_clocked_video_vid_mode_change            (),
    .alt_vip_cl_cvo_0_clocked_video_vid_std                    (),
    .alt_vip_cl_cvo_0_clocked_video_vid_vcoclk_div             (),
    .alt_vip_cl_cvo_0_clocked_video_vid_sof_locked             (),
    .alt_vip_cl_cvo_0_clocked_video_vid_sof                    (),
    .alt_vip_cl_cvo_0_clocked_video_vid_datavalid              (VIP_DE_o),
    .alt_vip_cl_cvo_0_clocked_video_vid_v_sync                 (VIP_VSYNC_o),
    .alt_vip_cl_cvo_0_clocked_video_vid_h_sync                 (VIP_HSYNC_o),
    .alt_vip_cl_cvo_0_clocked_video_vid_f                      (),
    .alt_vip_cl_cvo_0_clocked_video_vid_h                      (),
    .alt_vip_cl_cvo_0_clocked_video_vid_v                      (),
    .alt_vip_cl_cvo_0_genlock_sof_locked                       (1'b1),
    .alt_vip_cl_cvo_0_genlock_sof                              (sof_scaler_capt)
`endif
);

scanconverter #(
    .EMIF_ENABLE(0),
    .NUM_LINE_BUFFERS(40)
  ) scanconverter_inst (
    .PCLK_CAP_i(pclk_capture),
    .PCLK_OUT_i(SI_PCLK_i),
    .reset_n(sys_reset_n),  //TODO: sync to pclk_capture
    .R_i(R_capt),
    .G_i(G_capt),
    .B_i(B_capt),
    .HSYNC_i(HSYNC_capt),
    .VSYNC_i(VSYNC_capt),
    .DE_i(DE_capt),
    .datavalid_i(datavalid_capt),
    .FID_i(FID_capt),
    .interlaced_in_i(interlace_flag_capt),
    .frame_change_i(frame_change_capt),
    .xpos_i(xpos_capt),
    .ypos_i(ypos_capt),
    .h_in_active(hv_in_config[23:12]),
    .hv_out_config(hv_out_config),
    .hv_out_config2(hv_out_config2),
    .hv_out_config3(hv_out_config3),
    .xy_out_config(xy_out_config),
    .xy_out_config2(xy_out_config2),
    .misc_config(misc_config),
    .sl_config(sl_config),
    .sl_config2(sl_config2),
    .sl_config3(sl_config3),
    .testpattern_enable(testpattern_enable),
`ifdef VIP
    .ext_sync_mode(vip_select),
    .ext_frame_change_i(vip_frame_start),
    .ext_R_i(R_vip),
    .ext_G_i(G_vip),
    .ext_B_i(B_vip),
`else
    .ext_sync_mode(0),
    .ext_frame_change_i(0),
    .ext_R_i(0),
    .ext_G_i(0),
    .ext_B_i(0),
`endif
    .PCLK_o(PCLK_sc),
    .R_o(R_sc),
    .G_o(G_sc),
    .B_o(B_sc),
    .HSYNC_o(HSYNC_sc),
    .VSYNC_o(VSYNC_sc),
    .DE_o(DE_sc),
    .xpos_o(xpos_sc),
    .ypos_o(ypos_sc),
    .resync_strobe(resync_strobe_i),
    .emif_br_clk(emif_br_clk),
    .emif_rd_addr(emif_rd_addr),
    .emif_rd_read(emif_rd_read),
    .emif_rd_rdata(emif_rd_rdata),
    .emif_rd_waitrequest(emif_rd_waitrequest),
    .emif_rd_readdatavalid(emif_rd_readdatavalid),
    .emif_rd_burstcount(emif_rd_burstcount),
    .emif_wr_addr(emif_wr_addr),
    .emif_wr_write(emif_wr_write),
    .emif_wr_wdata(emif_wr_wdata),
    .emif_wr_waitrequest(emif_wr_waitrequest),
    .emif_wr_burstcount(emif_wr_burstcount)
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
