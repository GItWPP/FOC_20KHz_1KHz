`timescale 1ns/1ps
//逆Clarke+共模注入SVPWM,输入单位为Vbus/2的Q15比例
//half_width=HALF_TICKS*(32768+phase-common)/65536,对称脉冲位于周期中心

module FOC_SVPWM
#(
    parameter integer HALF_TICKS = 1250,
    parameter integer ZERO_MARGIN = 200,
    parameter integer DUTY_FRAC_BITS = 0
)
(
    input wire                          sys_clk,
    input wire                          sys_rst_n,
    input wire                          valid_i,
    input wire                          clear_i,
    input wire signed [15:0]            alpha_i,
    input wire signed [15:0]            beta_i,

    output reg                          valid_o,
    output reg [15 + DUTY_FRAC_BITS:0]  half_a_o,
    output reg [15 + DUTY_FRAC_BITS:0]  half_b_o,
    output reg [15 + DUTY_FRAC_BITS:0]  half_c_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
//半脉宽保留0..16位小数。夹紧后再交给PWM周期误差累积,不缩减安全窗。
localparam signed [47:0] MIN_HALF_FIXED = ZERO_MARGIN * (48'sd1 <<< DUTY_FRAC_BITS);
localparam signed [47:0] MAX_HALF_FIXED = (HALF_TICKS - ZERO_MARGIN) * (48'sd1 <<< DUTY_FRAC_BITS);
localparam [15 + DUTY_FRAC_BITS:0] NEUTRAL_HALF = (HALF_TICKS * (64'd1 << DUTY_FRAC_BITS)) / 2;
//先在21位完成三路电压偏移,避免48位乘积上下文扩大前级加减法。
localparam integer HALF_MUL_WIDTH = (HALF_TICKS > 0) ? $clog2(HALF_TICKS + 1) + 1 : 2;
localparam integer DUTY_PRODUCT_WIDTH = (21 + HALF_MUL_WIDTH < 48) ? 21 + HALF_MUL_WIDTH : 48;
localparam signed [HALF_MUL_WIDTH - 1:0] HALF_COEFFICIENT = HALF_TICKS;

//wire define
wire signed [19:0] common_sum;
wire signed [19:0] common;
wire signed [34:0] alpha_q16;
wire signed [34:0] beta_q16;
wire signed [20:0] duty_a_code;
wire signed [20:0] duty_b_code;
wire signed [20:0] duty_c_code;

//reg define
reg [2:0] valid_pipe;
reg signed [15:0] alpha_reg;
reg signed [33:0] beta_sqrt3;
reg signed [18:0] va, vb, vc;
reg signed [18:0] vmax, vmin;
reg signed [DUTY_PRODUCT_WIDTH - 1:0] duty_a_product, duty_b_product, duty_c_product;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign common_sum = $signed({vmax[18], vmax}) + $signed({vmin[18], vmin});

assign common = common_sum >>> 1;

assign alpha_q16 = $signed(alpha_reg) * 35'sd65536;

assign beta_q16 = {beta_sqrt3[33], beta_sqrt3};

assign duty_a_code = $signed({{2{va[18]}}, va}) - $signed({common[19], common}) + 21'sd32768;

assign duty_b_code = $signed({{2{vb[18]}}, vb}) - $signed({common[19], common}) + 21'sd32768;

assign duty_c_code = $signed({{2{vc[18]}}, vc}) - $signed({common[19], common}) + 21'sd32768;

function [15 + DUTY_FRAC_BITS:0] clip_half;
    input signed [DUTY_PRODUCT_WIDTH - 1:0] product;
    reg signed [47:0] value;
    begin
        value = product >>> (16 - DUTY_FRAC_BITS);
        if (value < MIN_HALF_FIXED)
            clip_half = MIN_HALF_FIXED;
        else if (value > MAX_HALF_FIXED)
            clip_half = MAX_HALF_FIXED;
        else clip_half = value[15 + DUTY_FRAC_BITS:0];
    end
endfunction

always @(*) begin
    vmax = va;
    vmin = va;
    if (vb > vmax)
        vmax = vb;
    if (vc > vmax)
        vmax = vc;
    if (vb < vmin)
        vmin = vb;
    if (vc < vmin)
        vmin = vc;
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        valid_pipe <= 0;
        valid_o <= 0;
        alpha_reg <= 0;
        beta_sqrt3 <= 0;
        va <= 0;
        vb <= 0;
        vc <= 0;
        duty_a_product <= 0;
        duty_b_product <= 0;
        duty_c_product <= 0;
        half_a_o <= NEUTRAL_HALF;
        half_b_o <= NEUTRAL_HALF;
        half_c_o <= NEUTRAL_HALF;
    end else if (clear_i) begin
        valid_pipe <= 0;
        valid_o <= 0;
        half_a_o <= NEUTRAL_HALF;
        half_b_o <= NEUTRAL_HALF;
        half_c_o <= NEUTRAL_HALF;
    end else begin
        valid_pipe <= {valid_pipe[1:0], valid_i};
        valid_o <= valid_pipe[2];
        if (valid_i) begin
            alpha_reg <= alpha_i;
            beta_sqrt3 <= beta_i * 18'sd113512;
        end
        if (valid_pipe[0]) begin
            va <= alpha_reg;
            vb <= (- alpha_q16 + beta_q16) >>> 17;
            vc <= (- alpha_q16 - beta_q16) >>> 17;
        end
        if (valid_pipe[1]) begin
            duty_a_product <= duty_a_code * HALF_COEFFICIENT;
            duty_b_product <= duty_b_code * HALF_COEFFICIENT;
            duty_c_product <= duty_c_code * HALF_COEFFICIENT;
        end
        if (valid_pipe[2]) begin
            half_a_o <= clip_half(duty_a_product);
            half_b_o <= clip_half(duty_b_product);
            half_c_o <= clip_half(duty_c_product);
        end
    end
end

endmodule
