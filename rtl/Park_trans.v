//--------------------------------------------------------------------------------------------------------
// 模块： Park_trans 
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： Park 变换
// 作者： WANG
// 时间： 26.10.13
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps

module Park_trans
#(
    parameter integer CURRENT_WIDTH = 16
)
(
    input wire                               sys_clk,
    input wire                               sys_rst_n,
    input wire                               I_VAL1,
    input wire signed [CURRENT_WIDTH - 1:0]  I_alpha,
    input wire signed [CURRENT_WIDTH - 1:0]  I_beta,
    //与输入电流同一采样时刻的电相位
    input wire [31:0]                        phase_e_i,
    //本组电流的角度有效
    input wire                               angle_valid_i,
    //故障时清空有效流水线
    input wire                               encoder_fault_i,

    output wire                              data_val,
    output reg signed [CURRENT_WIDTH - 1:0]  I_d,
    output reg signed [CURRENT_WIDTH - 1:0]  I_q
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam signed [CURRENT_WIDTH + 18:0] CURRENT_MAX = (64'sd1 <<< (CURRENT_WIDTH - 1)) - 1;
localparam signed [CURRENT_WIDTH + 18:0] CURRENT_MIN = - (64'sd1 <<< (CURRENT_WIDTH - 1));

//wire define
//输入/输出使用相同电流Q格式,仅在定点系数相乘后舍入。
//Id = alpha*cos(theta) + beta*sin(theta)
//Iq = beta*cos(theta) - alpha*sin(theta)
//sin/cos为18位有符号数,16位小数,+1=65536,-1=-65536
wire input_valid;
wire sincos_valid;
wire signed [17:0] sin_coeff;
wire signed [17:0] cos_coeff;
wire signed [CURRENT_WIDTH + 18:0] alpha_cos_ext;
wire signed [CURRENT_WIDTH + 18:0] beta_sin_ext;
wire signed [CURRENT_WIDTH + 18:0] alpha_sin_ext;
wire signed [CURRENT_WIDTH + 18:0] beta_cos_ext;
wire signed [CURRENT_WIDTH + 18:0] d_round;
wire signed [CURRENT_WIDTH + 18:0] q_round;

//reg define
reg signed [CURRENT_WIDTH - 1:0] alpha_reg;
reg signed [CURRENT_WIDTH - 1:0] beta_reg;
reg signed [CURRENT_WIDTH - 1:0] alpha_reg1;
reg signed [CURRENT_WIDTH - 1:0] beta_reg1;
reg signed [CURRENT_WIDTH + 17:0] alpha_cos_mul_reg;
reg signed [CURRENT_WIDTH + 17:0] beta_sin_mul_reg;
reg signed [CURRENT_WIDTH + 17:0] alpha_sin_mul_reg;
reg signed [CURRENT_WIDTH + 17:0] beta_cos_mul_reg;
reg signed [CURRENT_WIDTH + 18:0] d_sum_reg;
reg signed [CURRENT_WIDTH + 18:0] q_sum_reg;
reg mul_valid_reg;
reg sum_valid_reg;
reg output_valid_reg;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign input_valid = I_VAL1 && angle_valid_i && !encoder_fault_i;

//编码器故障在同一时钟沿更新时,立即屏蔽此前的输出valid
assign data_val = output_valid_reg && !encoder_fault_i;

assign alpha_cos_ext = {alpha_cos_mul_reg[CURRENT_WIDTH + 17], alpha_cos_mul_reg};

assign beta_sin_ext = {beta_sin_mul_reg[CURRENT_WIDTH + 17], beta_sin_mul_reg};

assign alpha_sin_ext = {alpha_sin_mul_reg[CURRENT_WIDTH + 17], alpha_sin_mul_reg};

assign beta_cos_ext = {beta_cos_mul_reg[CURRENT_WIDTH + 17], beta_cos_mul_reg};

//就近舍入,恰好半个LSB时远离零
assign d_round = (d_sum_reg + (d_sum_reg[CURRENT_WIDTH + 18] ? 64'sd32767 : 64'sd32768)) >>> 16;

assign q_round = (q_sum_reg + (q_sum_reg[CURRENT_WIDTH + 18] ? 64'sd32767 : 64'sd32768)) >>> 16;

function [CURRENT_WIDTH - 1:0] sat_current;
    input signed [CURRENT_WIDTH + 18:0] value;
    begin
        if (value > CURRENT_MAX)
            sat_current = CURRENT_MAX[CURRENT_WIDTH - 1:0];
        else if (value < CURRENT_MIN)
            sat_current = CURRENT_MIN[CURRENT_WIDTH - 1:0];
        else sat_current = value[CURRENT_WIDTH - 1:0];
    end
endfunction

//E0:采样电流/寄存地址;E1:同步查表;E2:乘法;E3:加减;E4:输出
//流水线可每拍接收一组输入,输出保留输入电流的小数位数
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        mul_valid_reg <= 1'b0;
        sum_valid_reg <= 1'b0;
        output_valid_reg <= 1'b0;
    end else if (encoder_fault_i == 1'b1) begin
        mul_valid_reg <= 1'b0;
        sum_valid_reg <= 1'b0;
        output_valid_reg <= 1'b0;
    end else begin
        mul_valid_reg <= sincos_valid;
        sum_valid_reg <= mul_valid_reg;
        output_valid_reg <= sum_valid_reg;
    end
end

//电流增加一拍,与ROM的地址寄存+同步读两级时序对应
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        alpha_reg1 <= 0;
        beta_reg1 <= 0;
    end else begin
        alpha_reg1 <= alpha_reg;
        beta_reg1 <= beta_reg;
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        alpha_reg <= 0;
        beta_reg <= 0;
    end else if (input_valid == 1'b1) begin
        alpha_reg <= I_alpha;
        beta_reg <= I_beta;
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        alpha_cos_mul_reg <= 0;
        beta_sin_mul_reg <= 0;
        alpha_sin_mul_reg <= 0;
        beta_cos_mul_reg <= 0;
    end else if (sincos_valid && !encoder_fault_i) begin
        alpha_cos_mul_reg <= alpha_reg1 * cos_coeff;
        beta_sin_mul_reg <= beta_reg1 * sin_coeff;
        alpha_sin_mul_reg <= alpha_reg1 * sin_coeff;
        beta_cos_mul_reg <= beta_reg1 * cos_coeff;
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        d_sum_reg <= 0;
        q_sum_reg <= 0;
    end else if (mul_valid_reg && !encoder_fault_i) begin
        d_sum_reg <= alpha_cos_ext + beta_sin_ext;
        q_sum_reg <= beta_cos_ext - alpha_sin_ext;
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        I_d <= 0;
        I_q <= 0;
    end else if (sum_valid_reg && !encoder_fault_i) begin
        I_d <= sat_current(d_round);
        I_q <= sat_current(q_round);
    end
end


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
FOC_SINCOS_LUT sincos_lut_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .valid_i(input_valid),
    .flush_i(encoder_fault_i),
    .phase_addr_i(phase_e_i[31:21]),
    .valid_o(sincos_valid),
    .sin_o(sin_coeff),
    .cos_o(cos_coeff)
);

endmodule
