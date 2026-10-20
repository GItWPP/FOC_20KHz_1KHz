//--------------------------------------------------------------------------------------------------------
// 模块： clark_trans 
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： clark 变换
// 作者： WANG
// 时间： 26.10.13
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps

module Clark_trans
#(
    parameter integer CURRENT_WIDTH = 16
)
(
    input wire                               sys_clk,
    input wire                               sys_rst_n,
    input wire                               DATA_VAL,
    input wire signed [CURRENT_WIDTH - 1:0]  Ia,
    input wire signed [CURRENT_WIDTH - 1:0]  Ib,
    input wire signed [CURRENT_WIDTH - 1:0]  Ic,

    output reg                               I_VAL,
    output reg signed [CURRENT_WIDTH - 1:0]  I_alpha,
    output reg signed [CURRENT_WIDTH - 1:0]  I_beta
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam signed [CURRENT_WIDTH + 20:0] CURRENT_MAX = (64'sd1 <<< (CURRENT_WIDTH - 1)) - 1;
localparam signed [CURRENT_WIDTH + 20:0] CURRENT_MIN = - (64'sd1 <<< (CURRENT_WIDTH - 1));
//输入/输出使用相同电流Q格式,仅在定点系数相乘后舍入。
// 等幅值 Clarke 变换，已包含 2/3：
// alpha = (2*Ia - Ib - Ic)/3
// beta  = (Ib - Ic)/sqrt(3)
//ALPHA_COEFF = 87381
//实际值 = 87381 / 2^18 = 87381 / 262144 ≈ 0.33333206
//BETA_COEFF = 151349
//实际值 = 151349 / 2^18 = 151349 / 262144 ≈ 0.57734680
// Q18 定点系数
localparam signed [18:0] ALPHA_COEFF = 19'sd87381;
// 1/3
localparam signed [18:0] BETA_COEFF = 19'sd151349;

//wire define
// 1/sqrt(3)
wire signed [CURRENT_WIDTH + 1:0] ia_ext;
wire signed [CURRENT_WIDTH + 1:0] ib_ext;
wire signed [CURRENT_WIDTH + 1:0] ic_ext;
wire signed [CURRENT_WIDTH + 20:0] alpha_round;
wire signed [CURRENT_WIDTH + 20:0] beta_round;

//reg define
reg data_val_reg;
reg data_val_reg1;
reg signed [CURRENT_WIDTH + 1:0] alpha_num_reg;
reg signed [CURRENT_WIDTH:0] beta_diff_reg;
reg signed [CURRENT_WIDTH + 20:0] alpha_mul_reg;
reg signed [CURRENT_WIDTH + 20:0] beta_mul_reg;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
// 显式符号扩展，避免中间计算溢出
assign ia_ext = {{2{Ia[CURRENT_WIDTH - 1]}}, Ia};

assign ib_ext = {{2{Ib[CURRENT_WIDTH - 1]}}, Ib};

assign ic_ext = {{2{Ic[CURRENT_WIDTH - 1]}}, Ic};

// Q18 统一舍入：恰好半个 LSB 时向远离零的方向舍入
assign alpha_round = (alpha_mul_reg + (alpha_mul_reg[CURRENT_WIDTH + 20] ? 64'sd131071 : 64'sd131072)) >>> 18;

assign beta_round = (beta_mul_reg + (beta_mul_reg[CURRENT_WIDTH + 20] ? 64'sd131071 : 64'sd131072)) >>> 18;

// 按参数电流位宽饱和
function [CURRENT_WIDTH - 1:0] sat_current;
    input signed [CURRENT_WIDTH + 20:0] value;
    begin
        if (value > CURRENT_MAX)
            sat_current = CURRENT_MAX[CURRENT_WIDTH - 1:0];
        else if (value < CURRENT_MIN)
            sat_current = CURRENT_MIN[CURRENT_WIDTH - 1:0];
        else sat_current = value[CURRENT_WIDTH - 1:0];
    end
endfunction

// 有效信号与数据流水线同步传递
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        data_val_reg <= 1'b0;
        data_val_reg1 <= 1'b0;
        I_VAL <= 1'b0;
    end else begin
        data_val_reg <= DATA_VAL;
        data_val_reg1 <= data_val_reg;
        I_VAL <= data_val_reg1;
    end
end

// 第一级：采样并计算扩位后的分子、差值
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        alpha_num_reg <= 0;
        beta_diff_reg <= 0;
    end else if (DATA_VAL == 1'b1) begin
        alpha_num_reg <= (ia_ext <<< 1) - ib_ext - ic_ext;
        beta_diff_reg <= ib_ext - ic_ext;
    end
end

// 第二级：乘以 Q18 定点系数
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        alpha_mul_reg <= 0;
        beta_mul_reg <= 0;
    end else if (data_val_reg == 1'b1) begin
        alpha_mul_reg <= alpha_num_reg * ALPHA_COEFF;
        beta_mul_reg <= beta_diff_reg * BETA_COEFF;
    end
end

// 第三级：舍入、饱和输出，与 I_VAL 同拍更新
// 无有效数据时保持上一次结果
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        I_alpha <= 0;
        I_beta <= 0;
    end else if (data_val_reg1 == 1'b1) begin
        I_alpha <= sat_current(alpha_round);
        I_beta <= sat_current(beta_round);
    end
end

endmodule
