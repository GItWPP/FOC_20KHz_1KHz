`timescale 1ns/1ps
//三相电流->Clarke->Park,电角度与电流由调用者作为同一采样帧提供
//phase_sample_i应在ADC实际采样时锁存,不能在扫描完成时重新读编码器

module FOC_CURRENT_TRANS
#(
    parameter integer CURRENT_WIDTH = 16
)
(
    input wire                                sys_clk,
    input wire                                sys_rst_n,
    input wire                                DATA_VAL,
    input wire signed [CURRENT_WIDTH - 1:0]   Ia,
    input wire signed [CURRENT_WIDTH - 1:0]   Ib,
    input wire signed [CURRENT_WIDTH - 1:0]   Ic,
    input wire [31:0]                         phase_sample_i,
    input wire                                angle_sample_valid_i,
    input wire                                encoder_fault_i,

    output wire                               I_VAL,
    output wire signed [CURRENT_WIDTH - 1:0]  I_alpha,
    output wire signed [CURRENT_WIDTH - 1:0]  I_beta,
    output wire                               data_val,
    output wire signed [CURRENT_WIDTH - 1:0]  I_d,
    output wire signed [CURRENT_WIDTH - 1:0]  I_q
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//wire define
wire input_valid;
wire clarke_valid;

//reg define
reg [31:0] phase_reg0;
reg [31:0] phase_reg1;
reg [31:0] phase_reg2;
reg [2:0] angle_valid_reg;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign input_valid = DATA_VAL && angle_sample_valid_i && !encoder_fault_i;

assign I_VAL = clarke_valid && angle_valid_reg[2]&& !encoder_fault_i;

//与Clarke的三层寄存器逐拍对应,E0输入->E2得到alpha/beta/phase
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        phase_reg0 <= 32'd0;
        phase_reg1 <= 32'd0;
        phase_reg2 <= 32'd0;
        angle_valid_reg <= 3'b000;
    end else if (encoder_fault_i == 1'b1) begin
        angle_valid_reg <= 3'b000;
    end else begin
        if (input_valid == 1'b1)
            phase_reg0 <= phase_sample_i;
        phase_reg1 <= phase_reg0;
        phase_reg2 <= phase_reg1;
        angle_valid_reg <= {angle_valid_reg[1:0], input_valid};
    end
end


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
Clark_trans #(
    .CURRENT_WIDTH(CURRENT_WIDTH)
) clarke_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .DATA_VAL(input_valid),
    .Ia(Ia),
    .Ib(Ib),
    .Ic(Ic),
    .I_VAL(clarke_valid),
    .I_alpha(I_alpha),
    .I_beta(I_beta)
);

Park_trans #(
    .CURRENT_WIDTH(CURRENT_WIDTH)
) park_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .I_VAL1(I_VAL),
    .I_alpha(I_alpha),
    .I_beta(I_beta),
    .phase_e_i(phase_reg2),
    .angle_valid_i(angle_valid_reg[2]),
    .encoder_fault_i(encoder_fault_i),
    .data_val(data_val),
    .I_d(I_d),
    .I_q(I_q)
);

endmodule
