`timescale 1ns/1ps
//1kHz速度外环:斜坡->Q8 rpm误差->Q16 PI->带IQ_FRAC_BITS位小数的mA Iq。
//start=0/速度设为0时先减速,连续停稳100ms后关闭;故障立即撤销使能。

module FOC_SPEED_CONTROLLER
#(
    parameter integer MAX_RPM = 3000,
    parameter integer MAX_IQ_MA = 200,
    parameter integer ACCEL_RPM_S = 600,
    parameter integer DECEL_RPM_S = 1000,
    parameter integer CONTROL_HZ = 1000,
    parameter integer STOP_HOLD_TICKS = 100,
    parameter integer STOP_TIMEOUT_TICKS = 6000,
    parameter integer STALL_TICKS = 2000,
    parameter integer IQ_FRAC_BITS = 0,
    parameter integer GAIN_SCHEDULE_ENABLE = 0,
    parameter integer SIGNED_TARGET_ENABLE = 0
)
(
    input wire                               sys_clk,
    input wire                               sys_rst_n,
    input wire                               tick_i,
    input wire                               start_i,
    input wire                               run_i,
    input wire                               fault_i,
    input wire                               permit_i,
    input wire [15:0]                        speed_set_rpm_i,
    input wire [15:0]                        iq_limit_ma_i,
    input wire                               position_mode_i,
    input wire signed [31:0]                 position_speed_q8_i,
    input wire [31:0]                        kp_q16_i,
    input wire [31:0]                        ki_q16_i,
    input wire signed [31:0]                 speed_q8_i,

    output wire                              enable_o,
    output reg signed [15 + IQ_FRAC_BITS:0]  iq_request_o,
    output reg signed [31:0]                 ramp_q8_o,
    output wire [15:0]                       accepted_rpm_o,
    output reg [2:0]                         state_o,
    output reg [7:0]                         fault_code_o,
    output reg                               saturated_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam [2:0] ST_OFF = 0, ST_STARTUP = 1, ST_RUN = 2, ST_STOP = 3, ST_FAULT = 4;
localparam integer ACCEL_STEP = (ACCEL_RPM_S * 256) / CONTROL_HZ;
localparam integer DECEL_STEP = (DECEL_RPM_S * 256) / CONTROL_HZ;
localparam integer INTEGRAL_WIDTH = 25 + $clog2(MAX_IQ_MA + 1);
localparam integer STOP_HOLD_WIDTH = (STOP_HOLD_TICKS > 1) ? $clog2(STOP_HOLD_TICKS) : 1;
localparam integer STOP_WIDTH = (STOP_TIMEOUT_TICKS > 1) ? $clog2(STOP_TIMEOUT_TICKS + 1) : 1;
localparam integer STALL_WIDTH = (STALL_TICKS > 1) ? $clog2(STALL_TICKS) : 1;

//wire define
wire request_run;
wire signed [15 + IQ_FRAC_BITS:0] current_limit_fixed;
wire signed [55:0] limit_q24;
wire position_mode;
wire signed [31:0] positive_limit;
wire signed [31:0] bounded_position_speed;
wire signed [31:0] target_q8;
wire [31:0] ramp_magnitude;
wire [31:0] target_magnitude;
wire reversing;
wire signed [31:0] next_target;
wire signed [32:0] signed_difference;
wire signed [32:0] signed_step;
wire signed [32:0] ramp_difference;
wire signed [32:0] speed_error;
wire [19:0] kp_limited;
wire [16:0] ki_limited;
//64rpm以下Kp为基础值的2倍,64..320rpm连续过渡到原值。
//过渡宽256rpm,用Q8斜坡直接得到Q16权重,无除法器。
wire [16:0] gain_weight;
wire [20:0] gain_kp_sum;
wire [19:0] gain_kp_limited;
wire signed [55:0] i_candidate;
wire signed [55:0] total_candidate;
wire block_integral;
wire signed [55:0] integral_used;
wire signed [55:0] integral_clipped;
wire signed [55:0] total_used;
wire stopped;

//reg define
reg [15:0] current_limit;
reg [STOP_HOLD_WIDTH - 1:0] stop_hold_count;
reg [STOP_WIDTH - 1:0] stop_count;
reg [STALL_WIDTH - 1:0] stall_count;
reg product_valid;
reg gain_valid;
reg signed [32:0] gain_error;
reg [19:0] gain_kp_base;
reg [16:0] gain_ki;
reg [36:0] gain_extra_product;
reg signed [32:0] error_reg;
reg signed [INTEGRAL_WIDTH - 1:0] integral;
reg signed [53:0] p_product;
reg signed [50:0] i_product;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign request_run = start_i && (accepted_rpm_o != 0);

assign current_limit_fixed = $signed({1'b0, current_limit}) <<< IQ_FRAC_BITS;

assign limit_q24 = $signed({1'b0, current_limit, 24'd0});

assign position_mode = SIGNED_TARGET_ENABLE && position_mode_i;

assign positive_limit = $signed({8'd0, accepted_rpm_o, 8'd0});

assign bounded_position_speed = (position_speed_q8_i > positive_limit) ? positive_limit :((position_speed_q8_i < -
    positive_limit) ? - positive_limit : position_speed_q8_i);

assign target_q8 = (request_run && state_o != ST_STOP) ? (position_mode ? bounded_position_speed : positive_limit) :
    32'sd0;

assign ramp_magnitude = ramp_q8_o[31] ? - ramp_q8_o : ramp_q8_o;

assign target_magnitude = target_q8[31] ? - target_q8 : target_q8;

assign reversing = (target_q8 > 0 && ramp_q8_o < 0) || (target_q8 < 0 && ramp_q8_o > 0);

assign next_target = reversing ? 32'sd0 : target_q8;

assign signed_difference = $signed({next_target[31], next_target}) - $signed({ramp_q8_o[31], ramp_q8_o});

assign signed_step = (reversing || target_magnitude < ramp_magnitude) ? DECEL_STEP : ACCEL_STEP;

assign ramp_difference = $signed({target_q8[31], target_q8}) - $signed({ramp_q8_o[31], ramp_q8_o});

assign speed_error = $signed({ramp_q8_o[31], ramp_q8_o}) - $signed({speed_q8_i[31], speed_q8_i});

assign kp_limited = (kp_q16_i > 32'd655360) ? 20'd655360 : kp_q16_i[19:0];

assign ki_limited = (ki_q16_i > 32'd65536) ? 17'd65536 : ki_q16_i[16:0];

assign gain_weight = (ramp_magnitude <= 32'd16384) ? 17'd65536 :((ramp_magnitude >= 32'd81920) ? 17'd0 : 32'd81920 -
    ramp_magnitude);

assign gain_kp_sum = {1'b0, gain_kp_base} + gain_extra_product[36:16];

assign gain_kp_limited = (gain_kp_sum > 21'd655360) ? 20'd655360 : gain_kp_sum[19:0];

assign i_candidate = integral + i_product;

assign total_candidate = p_product + i_candidate;

assign block_integral = ((total_candidate > limit_q24) && error_reg > 0) || ((total_candidate < - limit_q24) &&
    error_reg < 0);

assign integral_used = block_integral ? integral : i_candidate;

assign integral_clipped = (integral_used > limit_q24) ? limit_q24 :((integral_used < - limit_q24) ? - limit_q24 :
    integral_used);

assign total_used = p_product + integral_clipped;

assign stopped = (speed_q8_i < 32'sd768 && speed_q8_i > - 32'sd768);

//限流上限按系统时钟预译码,不把慢速VIO信号串入PI宽加法/比较链。
always @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n)
        current_limit <= 0;
    else current_limit <= (iq_limit_ma_i > MAX_IQ_MA) ? MAX_IQ_MA : iq_limit_ma_i;

assign accepted_rpm_o = (speed_set_rpm_i > MAX_RPM) ? MAX_RPM : speed_set_rpm_i;

assign enable_o = (state_o == ST_STARTUP || state_o == ST_RUN || state_o == ST_STOP) && !fault_i && permit_i &&
    (fault_code_o == 0);

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state_o <= ST_OFF;
        fault_code_o <= 0;
        stop_hold_count <= 0;
        stop_count <= 0;
        stall_count <= 0;
    end else if (fault_i || fault_code_o != 0)
        state_o <= ST_FAULT;
    else if (!permit_i) begin
        state_o <= ST_OFF;
        stop_hold_count <= 0;
        stop_count <= 0;
        stall_count <= 0;
    end else begin
        case (state_o)
            ST_OFF: begin
                stop_hold_count <= 0;
                stop_count <= 0;
                stall_count <= 0;
                if (request_run)
                    state_o <= ST_STARTUP;
            end
            ST_STARTUP:
                if (!request_run)
                    state_o <= ST_OFF;
                else if (run_i)
                    state_o <= ST_RUN;
            ST_RUN:
                if (!request_run) begin
                    state_o <= ST_STOP;
                    stop_count <= 0;
                    stop_hold_count <= 0;
                end
            ST_STOP:
                if (tick_i) begin
                    stop_count <= stop_count + 1'b1;
                    if (stopped && ramp_q8_o == 0) begin
                        if (stop_hold_count >= STOP_HOLD_TICKS - 1)
                            state_o <= ST_OFF;
                        else stop_hold_count <= stop_hold_count + 1'b1;
                    end else stop_hold_count <= 0;
                    if (stop_count >= STOP_TIMEOUT_TICKS - 1)
                        fault_code_o <= 8'h0a;
                end
            default:
                state_o <= ST_FAULT;
        endcase
        if (tick_i) begin
            if (state_o == ST_RUN && ramp_magnitude > 32'd12800 && speed_q8_i < 32'sd1280 && speed_q8_i > - 32'sd1280 &&
                saturated_o && current_limit != 0) begin
                if (stall_count >= STALL_TICKS - 1)
                    fault_code_o <= 8'h09;
                else stall_count <= stall_count + 1'b1;
            end else stall_count <= 0;
        end
    end
end

always @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n)
        ramp_q8_o <= 0;
    else if (!enable_o || !run_i)
        ramp_q8_o <= 0;
    else if (tick_i) begin
        if (position_mode) begin
            if (signed_difference > signed_step)
                ramp_q8_o <= ramp_q8_o + signed_step;
            else if (signed_difference < - signed_step)
                ramp_q8_o <= ramp_q8_o - signed_step;
            else ramp_q8_o <= next_target;
        end else if (ramp_difference > ACCEL_STEP)
            ramp_q8_o <= ramp_q8_o + ACCEL_STEP;
        else if (ramp_difference < - DECEL_STEP)
            ramp_q8_o <= ramp_q8_o - DECEL_STEP;
        else ramp_q8_o <= target_q8;
    end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        integral <= 0;
        product_valid <= 0;
        error_reg <= 0;
        p_product <= 0;
        i_product <= 0;
        gain_valid <= 0;
        gain_error <= 0;
        gain_kp_base <= 0;
        gain_ki <= 0;
        gain_extra_product <= 0;
        iq_request_o <= 0;
        saturated_o <= 0;
    end else if (!enable_o || !run_i) begin
        integral <= 0;
        product_valid <= 0;
        gain_valid <= 0;
        iq_request_o <= 0;
        saturated_o <= 0;
    end else begin
        gain_valid <= GAIN_SCHEDULE_ENABLE ? tick_i : 1'b0;
        product_valid <= GAIN_SCHEDULE_ENABLE ? gain_valid : tick_i;
        if (tick_i && GAIN_SCHEDULE_ENABLE) begin
            //误差与两项VIO增益同拍锁存,额外乘法单独成级。
            gain_error <= speed_error;
            gain_kp_base <= kp_limited;
            gain_ki <= ki_limited;
            gain_extra_product <= kp_limited * gain_weight;
        end
        if (GAIN_SCHEDULE_ENABLE && gain_valid) begin
            error_reg <= gain_error;
            p_product <= gain_error * $signed({1'b0, gain_kp_limited});
            i_product <= gain_error * $signed({1'b0, gain_ki});
        end else if (tick_i && !GAIN_SCHEDULE_ENABLE) begin
            error_reg <= speed_error;
            p_product <= speed_error * $signed({1'b0, kp_limited});
            i_product <= speed_error * $signed({1'b0, ki_limited});
        end
        if (product_valid) begin
            integral <= integral_clipped;
            saturated_o <= (total_used >= limit_q24 || total_used <= - limit_q24);
            if (total_used > limit_q24)
                iq_request_o <= current_limit_fixed;
            else if (total_used < - limit_q24)
                iq_request_o <= - current_limit_fixed;
            else iq_request_o <= total_used >>> (24 - IQ_FRAC_BITS);
        end
    end
end

endmodule
