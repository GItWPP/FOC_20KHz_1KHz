`timescale 1ns/1ps
//逆Park等价于对(d,q)施加负角度的Park,复用已验证算术流水线

module FOC_INVERSE_PARK
(
    input wire                 sys_clk,
    input wire                 sys_rst_n,
    input wire                 valid_i,
    input wire                 flush_i,
    input wire signed [15:0]   vd_i,
    input wire signed [15:0]   vq_i,
    input wire [31:0]          phase_i,

    output wire                valid_o,
    output wire signed [15:0]  alpha_o,
    output wire signed [15:0]  beta_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
Park_trans inverse_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .I_VAL1(valid_i),
    .I_alpha(vd_i),
    .I_beta(vq_i),
    .phase_e_i(32'd0 - phase_i),
    .angle_valid_i(1'b1),
    .encoder_fault_i(flush_i),
    .data_val(valid_o),
    .I_d(alpha_o),
    .I_q(beta_o)
);

endmodule
