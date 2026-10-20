`timescale 1ns/1ps
//8192计数/圈的多圈定位P环;AB边沿累计,不受Z单圈清零影响。

module FOC_POSITION_CONTROLLER
#(
    parameter integer MAX_RPM = 3000,
    parameter integer TOLERANCE_COUNTS = 2,
    parameter integer SETTLE_TICKS = 100,
    parameter integer MAX_POSITION = 1073741823
)
(
    input wire                 sys_clk,
    input wire                 sys_rst_n,
    input wire                 tick_i,
    input wire                 active_i,
    input wire                 run_i,
    input wire                 step_flag_i,
    input wire                 step_positive_i,
    input wire signed [31:0]   target_counts_i,
    input wire signed [31:0]   speed_q8_i,
    input wire [31:0]          kp_q16_i,
    input wire [15:0]          speed_limit_rpm_i,

    output reg signed [31:0]   position_counts_o,
    output reg signed [31:0]   target_counts_o,
    output wire signed [31:0]  error_counts_o,
    output reg signed [31:0]   speed_reference_q8_o,
    output reg                 ready_o,
    output reg                 settled_o,
    output reg                 range_fault_o,
    output wire                target_limited_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer SETTLE_WIDTH = (SETTLE_TICKS > 1) ? $clog2(SETTLE_TICKS) : 1;

//wire define
wire signed [31:0] bounded_target;
wire signed [32:0] position_error;
wire in_band;
wire slow;
wire [19:0] gain_limited;
wire [15:0] rpm_limited;
wire signed [57:0] extended_product;
wire signed [38:0] speed_candidate;

//reg define
reg [SETTLE_WIDTH - 1:0] settle_count;
reg product_valid, scale_valid;
reg signed [53:0] p_product;
reg signed [57:0] scaled_product;
reg signed [31:0] product_limit, scale_limit;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign bounded_target = (target_counts_i > MAX_POSITION) ? MAX_POSITION :((target_counts_i < - MAX_POSITION) ? -
    MAX_POSITION : target_counts_i);

assign position_error = $signed({target_counts_o[31], target_counts_o}) - $signed({position_counts_o[31],
    position_counts_o});

assign in_band = (position_error <= TOLERANCE_COUNTS && position_error >= - TOLERANCE_COUNTS);

assign slow = (speed_q8_i <= 32'sd256 && speed_q8_i >= - 32'sd256);

assign gain_limited = (kp_q16_i > 32'd655360) ? 20'd655360 : kp_q16_i[19:0];

assign rpm_limited = (speed_limit_rpm_i > MAX_RPM) ? MAX_RPM : speed_limit_rpm_i;

assign extended_product = {{4{p_product[53]}}, p_product};

assign speed_candidate = scaled_product >>> 19;

assign error_counts_o = position_error[31:0];

assign target_limited_o = (bounded_target != target_counts_i);

//RUN上升后建零,随后每个有效AB边沿只累计一次。停止时仍记录减速位移。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        position_counts_o <= 0;
        ready_o <= 0;
        range_fault_o <= 0;
    end else if (!active_i || !run_i) begin
        position_counts_o <= 0;
        ready_o <= 0;
    end else if (!ready_o) begin
        position_counts_o <= 0;
        ready_o <= 1;
    end else if (step_flag_i && !range_fault_o) begin
        if (step_positive_i) begin
            if (position_counts_o >= MAX_POSITION)
                range_fault_o <= 1;
            else position_counts_o <= position_counts_o + 32'sd1;
        end else begin
            if (position_counts_o <= - MAX_POSITION)
                range_fault_o <= 1;
            else position_counts_o <= position_counts_o - 32'sd1;
        end
    end
end

always @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n)
        target_counts_o <= 0;
    else if (tick_i)
        target_counts_o <= bounded_target;

//误差*Kp: counts/s; *60*256/8192转换成Q8 rpm,即乘15后右移19位。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        product_valid <= 0;
        scale_valid <= 0;
        p_product <= 0;
        scaled_product <= 0;
        product_limit <= 0;
        scale_limit <= 0;
        speed_reference_q8_o <= 0;
    end else if (!active_i || !run_i || !ready_o || range_fault_o) begin
        product_valid <= 0;
        scale_valid <= 0;
        speed_reference_q8_o <= 0;
    end else begin
        product_valid <= tick_i;
        scale_valid <= product_valid;
        if (tick_i) begin
            p_product <= in_band ? 54'sd0 : position_error * $signed({1'b0, gain_limited});
            product_limit <= $signed({8'd0, rpm_limited, 8'd0});
        end
        if (product_valid) begin
            scaled_product <= (extended_product <<< 4) - extended_product;
            scale_limit <= product_limit;
        end
        if (scale_valid) begin
            if (speed_candidate > $signed(scale_limit))
                speed_reference_q8_o <= scale_limit;
            else if (speed_candidate < - $signed(scale_limit))
                speed_reference_q8_o <= - scale_limit;
            else speed_reference_q8_o <= speed_candidate[31:0];
        end
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        settle_count <= 0;
        settled_o <= 0;
    end else if (!active_i || !run_i || !ready_o || range_fault_o || bounded_target != target_counts_o) begin
        settle_count <= 0;
        settled_o <= 0;
    end else if (tick_i) begin
        if (in_band && slow) begin
            if (settle_count >= SETTLE_TICKS - 1)
                settled_o <= 1;
            else settle_count <= settle_count + 1'b1;
        end else begin
            settle_count <= 0;
            settled_o <= 0;
        end
    end
end

endmodule
