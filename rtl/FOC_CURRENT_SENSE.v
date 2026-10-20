`timescale 1ns/1ps
//野火0.02ohm/8倍电流采样:关断时平均零偏,电流保留FRAC_BITS位mA小数

module FOC_CURRENT_SENSE
#(
    parameter integer OFFSET_LOG2 = 8,
    parameter signed [31:0] GAIN_Q16 = - 32'sd62500,
    parameter integer FRAC_BITS = 0,
    parameter signed [31:0] GAIN_A_Q16 = GAIN_Q16,
    parameter signed [31:0] GAIN_B_Q16 = GAIN_Q16,
    parameter signed [31:0] GAIN_C_Q16 = GAIN_Q16
)
(
    input wire                            sys_clk,
    input wire                            sys_rst_n,
    input wire                            calibrate_en_i,
    input wire                            frame_valid_i,
    input wire signed [15:0]              raw_a_i,
    input wire signed [15:0]              raw_b_i,
    input wire signed [15:0]              raw_c_i,

    output reg                            calibrated_o,
    output reg                            valid_o,
    output reg signed [15 + FRAC_BITS:0]  ia_o,
    output reg signed [15 + FRAC_BITS:0]  ib_o,
    output reg signed [15 + FRAC_BITS:0]  ic_o,
    output reg signed [15:0]              offset_a_o,
    output reg signed [15:0]              offset_b_o,
    output reg signed [15:0]              offset_c_o,
    output wire                           rail_fault_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer CNT_WIDTH = (OFFSET_LOG2 > 0) ? OFFSET_LOG2 : 1;
localparam integer OFFSET_SAMPLES = 1 << OFFSET_LOG2;
localparam integer CURRENT_WIDTH = 16 + FRAC_BITS;
localparam integer PRODUCT_WIDTH = CURRENT_WIDTH + 33;
localparam signed [PRODUCT_WIDTH - 1:0] CURRENT_MAX = (64'sd1 <<< (CURRENT_WIDTH - 1)) - 1;
localparam signed [PRODUCT_WIDTH - 1:0] CURRENT_MIN = - (64'sd1 <<< (CURRENT_WIDTH - 1));

//wire define
wire signed [CURRENT_WIDTH:0] delta_a;
wire signed [CURRENT_WIDTH:0] delta_b;
wire signed [CURRENT_WIDTH:0] delta_c;
//先扩位和移位再平均。256帧/Q8时完整保留平均值,不截成整数ADC码。
wire signed [63:0] average_a;
wire signed [63:0] average_b;
wire signed [63:0] average_c;

//reg define
reg [CNT_WIDTH - 1:0] count;
reg signed [31:0] sum_a, sum_b, sum_c;
reg signed [CURRENT_WIDTH - 1:0] offset_a_fixed, offset_b_fixed, offset_c_fixed;
reg signed [PRODUCT_WIDTH - 1:0] product_a, product_b, product_c;
reg multiply_valid;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign delta_a = $signed({raw_a_i[15], raw_a_i, {FRAC_BITS{1'b0}}}) - $signed({offset_a_fixed[CURRENT_WIDTH - 1],
    offset_a_fixed});

assign delta_b = $signed({raw_b_i[15], raw_b_i, {FRAC_BITS{1'b0}}}) - $signed({offset_b_fixed[CURRENT_WIDTH - 1],
    offset_b_fixed});

assign delta_c = $signed({raw_c_i[15], raw_c_i, {FRAC_BITS{1'b0}}}) - $signed({offset_c_fixed[CURRENT_WIDTH - 1],
    offset_c_fixed});

assign average_a = (($signed({{32{sum_a[31]}}, sum_a}) + $signed(raw_a_i)) <<< FRAC_BITS) >>> OFFSET_LOG2;

assign average_b = (($signed({{32{sum_b[31]}}, sum_b}) + $signed(raw_b_i)) <<< FRAC_BITS) >>> OFFSET_LOG2;

assign average_c = (($signed({{32{sum_c[31]}}, sum_c}) + $signed(raw_c_i)) <<< FRAC_BITS) >>> OFFSET_LOG2;

assign rail_fault_o = frame_valid_i && ((raw_a_i > 16'sd32000) || (raw_a_i < - 16'sd32000) || (raw_b_i > 16'sd32000) ||
    (raw_b_i < - 16'sd32000) || (raw_c_i > 16'sd32000) || (raw_c_i < - 16'sd32000));

function [CURRENT_WIDTH - 1:0] round_sat;
    input signed [PRODUCT_WIDTH - 1:0] product;
    reg signed [PRODUCT_WIDTH - 1:0] rounded;
    begin
        rounded = (product + (product[PRODUCT_WIDTH - 1] ? 64'sd32767 : 64'sd32768)) >>> 16;
        if (rounded > CURRENT_MAX)
            round_sat = CURRENT_MAX[CURRENT_WIDTH - 1:0];
        else if (rounded < CURRENT_MIN)
            round_sat = CURRENT_MIN[CURRENT_WIDTH - 1:0];
        else round_sat = rounded[CURRENT_WIDTH - 1:0];
    end
endfunction

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        count <= 0;
        sum_a <= 0;
        sum_b <= 0;
        sum_c <= 0;
        calibrated_o <= 0;
        offset_a_o <= 0;
        offset_b_o <= 0;
        offset_c_o <= 0;
        offset_a_fixed <= 0;
        offset_b_fixed <= 0;
        offset_c_fixed <= 0;
        product_a <= 0;
        product_b <= 0;
        product_c <= 0;
        multiply_valid <= 0;
        valid_o <= 0;
        ia_o <= 0;
        ib_o <= 0;
        ic_o <= 0;
    end else begin
        multiply_valid <= frame_valid_i && calibrated_o;
        valid_o <= multiply_valid;
        if (frame_valid_i && !calibrated_o && calibrate_en_i) begin
            if (count == OFFSET_SAMPLES - 1) begin
                offset_a_fixed <= average_a[CURRENT_WIDTH - 1:0];
                offset_b_fixed <= average_b[CURRENT_WIDTH - 1:0];
                offset_c_fixed <= average_c[CURRENT_WIDTH - 1:0];
                //调试端口仍为整数ADC码;控制使用上面的带小数零偏。
                offset_a_o <= average_a >>> FRAC_BITS;
                offset_b_o <= average_b >>> FRAC_BITS;
                offset_c_o <= average_c >>> FRAC_BITS;
                calibrated_o <= 1;
            end else begin
                sum_a <= sum_a + $signed(raw_a_i);
                sum_b <= sum_b + $signed(raw_b_i);
                sum_c <= sum_c + $signed(raw_c_i);
                count <= count + 1'b1;
            end
        end
        if (frame_valid_i && calibrated_o) begin
            product_a <= delta_a * GAIN_A_Q16;
            product_b <= delta_b * GAIN_B_Q16;
            product_c <= delta_c * GAIN_C_Q16;
        end
        if (multiply_valid) begin
            ia_o <= round_sat(product_a);
            ib_o <= round_sat(product_b);
            ic_o <= round_sat(product_c);
        end
    end
end

endmodule
