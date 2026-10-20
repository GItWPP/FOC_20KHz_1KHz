`timescale 1ns/1ps
//Kp/Ki为无符号Q16,Ki已含采样周期;电流/积分的小数随CURRENT_FRAC_BITS保留

module FOC_PI
#(
    parameter integer OUTPUT_LIMIT = 14000,
    parameter integer CURRENT_FRAC_BITS = 0
)
(
    input wire                                    sys_clk,
    input wire                                    sys_rst_n,
    input wire                                    clear_i,
    input wire                                    valid_i,
    input wire [31:0]                             kp_i,
    input wire [31:0]                             ki_i,
    input wire signed [15 + CURRENT_FRAC_BITS:0]  reference_i,
    input wire signed [15 + CURRENT_FRAC_BITS:0]  feedback_i,

    output reg                                    valid_o,
    output reg signed [15:0]                      voltage_o,
    output reg                                    saturated_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer CURRENT_WIDTH = 16 + CURRENT_FRAC_BITS;
localparam integer ACCUM_FRAC_BITS = 16 + CURRENT_FRAC_BITS;
localparam signed [63:0] LIMIT_Q16 = OUTPUT_LIMIT * (64'sd1 <<< ACCUM_FRAC_BITS);

//wire define
wire signed [CURRENT_WIDTH:0] error;
wire signed [63:0] i_candidate;
wire signed [63:0] total_candidate;
wire block_integral;
wire signed [63:0] integral_used;
wire signed [63:0] total_used;

//reg define
reg product_valid;
reg signed [CURRENT_WIDTH:0] error_reg;
reg signed [CURRENT_WIDTH + 33:0] p_product, i_product;
reg signed [63:0] integral;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign error = $signed({reference_i[CURRENT_WIDTH - 1], reference_i}) - $signed({feedback_i[CURRENT_WIDTH - 1],
    feedback_i});

assign i_candidate = integral + $signed(i_product);

assign total_candidate = $signed(p_product) + i_candidate;

assign block_integral = ((total_candidate > LIMIT_Q16) && (error_reg > 0)) || ((total_candidate < - LIMIT_Q16) &&
    (error_reg < 0));

assign integral_used = block_integral ? integral : clamp_integral(i_candidate);

assign total_used = $signed(p_product) + integral_used;

function signed [63:0] clamp_integral;
    input signed [63:0] value;
    begin
        if (value > LIMIT_Q16)
            clamp_integral = LIMIT_Q16;
        else if (value < - LIMIT_Q16)
            clamp_integral = - LIMIT_Q16;
        else clamp_integral = value;
    end
endfunction

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        integral <= 0;
        product_valid <= 0;
        valid_o <= 0;
        voltage_o <= 0;
        saturated_o <= 0;
        p_product <= 0;
        i_product <= 0;
        error_reg <= 0;
    end else if (clear_i) begin
        integral <= 0;
        product_valid <= 0;
        valid_o <= 0;
        voltage_o <= 0;
        saturated_o <= 0;
    end else begin
        product_valid <= valid_i;
        valid_o <= product_valid;
        if (valid_i) begin
            error_reg <= error;
            p_product <= error * $signed({1'b0, kp_i});
            i_product <= error * $signed({1'b0, ki_i});
        end
        if (product_valid) begin
            integral <= clamp_integral(integral_used);
            saturated_o <= (total_used > LIMIT_Q16) || (total_used < - LIMIT_Q16);
            if (total_used > LIMIT_Q16)
                voltage_o <= OUTPUT_LIMIT;
            else if (total_used < - LIMIT_Q16)
                voltage_o <= - OUTPUT_LIMIT;
            else voltage_o <= total_used >>> ACCUM_FRAC_BITS;
        end
    end
end

endmodule
