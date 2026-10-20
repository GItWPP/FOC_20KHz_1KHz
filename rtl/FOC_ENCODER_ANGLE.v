`timescale 1ns/1ps
//机械相位->电相位;整圈使用32位无符号相位表示,自动模2^32回绕
//标定前须由上层将转子对齐到align_phase_i指定的已知电角度

module FOC_ENCODER_ANGLE
#(
    parameter [31:0] POLE_PAIRS = 32'd4
)
(
    input wire          sys_clk,
    input wire          sys_rst_n,
    input wire [31:0]   phase_m_i,
    input wire          encoder_ready_i,
    input wire          encoder_fault_i,
    input wire          calibrate_flag_i,
    input wire [31:0]   align_phase_i,

    output wire [31:0]  phase_e_o,
    output wire         angle_valid_o,
    output reg          angle_calibrated_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//wire define
wire [63:0] electrical_phase_product;
wire [31:0] electrical_phase_relative;

//reg define
reg [31:0] phase_offset_reg;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign electrical_phase_product = phase_m_i * POLE_PAIRS;

assign electrical_phase_relative = electrical_phase_product[31:0];

assign phase_e_o = electrical_phase_relative + phase_offset_reg;

assign angle_valid_o = angle_calibrated_o && encoder_ready_i && !encoder_fault_i;

//复位、译码未就绪或故障后必须重新进行物理对齐和标定
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        phase_offset_reg <= 32'd0;
        angle_calibrated_o <= 1'b0;
    end else if (!encoder_ready_i || encoder_fault_i) begin
        phase_offset_reg <= 32'd0;
        angle_calibrated_o <= 1'b0;
    end else if (calibrate_flag_i == 1'b1) begin
        phase_offset_reg <= align_phase_i - electrical_phase_relative;
        angle_calibrated_o <= 1'b1;
    end
end

endmodule
