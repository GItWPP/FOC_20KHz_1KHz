//--------------------------------------------------------------------------------------------------------
// 模块： FOC_SPEED_DRIVE
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： 组合位置、速度与电流闭环，保留原启动、采样及硬故障关断
// 作者： WANG
// 时间： 26.10.20
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps
//速度环与已验证电流环组合;保留core中外部故障到七路驱动的直接关断路径。

module FOC_SPEED_DRIVE
#(
    parameter integer CLK_FREQ = 50000000,
    parameter integer PWM_PERIOD = 2500,
    parameter integer MAX_RPM = 3000,
    parameter integer MAX_IQ_MA = 200,
    parameter integer PEAK_CURRENT_MA = 500,
    parameter integer ALIGN_SAMPLES = 10000,
    parameter integer SEARCH_TIMEOUT_SAMPLES = 200000,
    parameter integer ADC_POWERUP_CLKS = 2500000,
    parameter integer OFFSET_LOG2 = 8,
    parameter integer REVERSE_DIRECTION = 1,
    parameter integer SMOOTH_ALIGN = 1,
    parameter integer ALIGN_CURRENT_MA = 150,
    parameter integer PWM_FRAC_BITS = 16,
    parameter integer SPEED_GAIN_SCHEDULE = 1,
    parameter integer POSITION_ENABLE = 0
)
(
    input wire                 sys_clk,
    input wire                 sys_rst_n,
    input wire                 start_i,
    input wire                 permit_i,
    input wire                 hard_trip_i,
    input wire [15:0]          speed_set_rpm_i,
    input wire [15:0]          iq_limit_ma_i,
    input wire                 position_mode_i,
    input wire signed [31:0]   position_target_i,
    input wire [31:0]          position_kp_q16_i,
    input wire [31:0]          speed_kp_q16_i,
    input wire [31:0]          speed_ki_q16_i,
    input wire [31:0]          current_kp_q16_i,
    input wire [31:0]          current_ki_q16_i,
    input wire                 enc_a_i,
    input wire                 enc_b_i,
    input wire                 enc_z_i,
    input wire                 adc_busy_i,
    input wire [15:0]          adc_data_i,

    output wire [2:0]          adc_os_o,
    output wire                adc_cs_n_o,
    output wire                adc_rd_n_o,
    output wire                adc_reset_o,
    output wire                adc_convst_o,
    output wire [2:0]          pwm_h_o,
    output wire [2:0]          pwm_l_o,
    output wire                motor_sd_a_o,
    output wire signed [31:0]  speed_q8_o,
    output wire signed [15:0]  speed_rpm_o,
    output wire [15:0]         accepted_rpm_o,
    output wire [15:0]         ramp_rpm_o,
    output wire [2:0]          speed_state_o,
    output wire                saturated_o,
    output wire                moving_o,
    output wire                fault_o,
    output wire                run_o,
    output wire [3:0]          state_o,
    output wire [7:0]          fault_code_o,
    output wire signed [15:0]  id_o,
    output wire signed [15:0]  iq_o,
    output wire signed [15:0]  id_reference_o,
    output wire signed [15:0]  iq_reference_o,
    output wire [255:0]        core_debug_o,
    output wire [31:0]         adc_debug_o,
    output wire signed [31:0]  position_counts_o,
    output wire signed [31:0]  position_target_o,
    output wire signed [31:0]  position_error_o,
    output wire [5:0]          position_flags_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer MAX_COUNTS = (64'd8192 * 3300 * PWM_PERIOD + 60 * CLK_FREQ - 1) / (60 * CLK_FREQ);

//wire define
wire step_flag, step_positive, speed_tick, core_enable;
wire [7:0] speed_fault_code;
wire position_ready, position_settled, position_range_fault, position_target_limited;
wire requested_mode;
wire signed [31:0] position_speed_q8;
wire [7:0] control_fault_code;
wire signed [31:0] ramp_q8;
//基础电流Ki沿用高速值;低速乘4,64..320rpm与速度Kp一起连续恢复。
wire [31:0] ramp_magnitude;
wire [16:0] gain_weight;
wire [18:0] current_ki_factor;
//速度PI到电流PI保持Q8 mA，串口显示整数mA。
wire signed [23:0] iq_request;

//reg define
reg position_mode;
reg [50:0] current_ki_product;
reg [31:0] current_ki_effective;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign requested_mode = POSITION_ENABLE && position_mode_i;

assign control_fault_code = (speed_fault_code != 0) ? speed_fault_code :(position_range_fault ? 8'h0c : 8'h00);

assign ramp_magnitude = ramp_q8[31] ? - ramp_q8 : ramp_q8;

assign gain_weight = (ramp_magnitude <= 32'd16384) ? 17'd65536 :((ramp_magnitude >= 32'd81920) ? 17'd0 : 32'd81920 -
    ramp_magnitude);

assign current_ki_factor = SPEED_GAIN_SCHEDULE ? 19'd65536 + gain_weight * 19'd3 : 19'd65536;

assign position_flags_o = {requested_mode != position_mode, position_range_fault, position_target_limited,
    position_settled, position_ready, position_mode};

//运行中修改模式仅挂起;停机后才采纳,不在运动中切换外环。
always @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n)
        position_mode <= 0;
    else if (speed_state_o == 3'd0)
        position_mode <= requested_mode;

assign ramp_rpm_o = ramp_q8[23:8];

//两级流水避免把增益乘法串到20kHz电流PI乘法前;同域50MHz,无派生时钟。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        current_ki_product <= 0;
        current_ki_effective <= 0;
    end else begin
        current_ki_product <= current_ki_q16_i * current_ki_factor;
        current_ki_effective <= (| current_ki_product[50:48]) ? 32'hffffffff : current_ki_product[47:16];
    end
end


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
FOC_POSITION_CONTROLLER #(
    .MAX_RPM(MAX_RPM)
) position_controller_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .tick_i(speed_tick),
    .active_i(position_mode),
    .run_i(run_o),
    .step_flag_i(step_flag),
    .step_positive_i(step_positive),
    .target_counts_i(position_target_i),
    .speed_q8_i(speed_q8_o),
    .kp_q16_i(position_kp_q16_i),
    .speed_limit_rpm_i(speed_set_rpm_i),
    .position_counts_o(position_counts_o),
    .target_counts_o(position_target_o),
    .error_counts_o(position_error_o),
    .speed_reference_q8_o(position_speed_q8),
    .ready_o(position_ready),
    .settled_o(position_settled),
    .range_fault_o(position_range_fault),
    .target_limited_o(position_target_limited)
);

FOC_SPEED_ESTIMATOR #(
    .CLK_FREQ(CLK_FREQ)
) speed_estimator_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .step_flag_i(step_flag),
    .step_positive_i(step_positive),
    .tick_o(speed_tick),
    .speed_q8_o(speed_q8_o),
    .speed_rpm_o(speed_rpm_o),
    .moving_o(moving_o)
);

//原始硬故障只经core独立guard直接关断,不能经速度环组合反馈绕回PWM。
FOC_SPEED_CONTROLLER #(
    .MAX_RPM(MAX_RPM),
    .MAX_IQ_MA(MAX_IQ_MA),
    .IQ_FRAC_BITS(8),
    .GAIN_SCHEDULE_ENABLE(SPEED_GAIN_SCHEDULE),
    .SIGNED_TARGET_ENABLE(POSITION_ENABLE)
) speed_controller_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .tick_i(speed_tick),
    .start_i(start_i),
    .run_i(run_o),
    .fault_i(fault_code_o != 8'd0),
    .permit_i(permit_i),
    .speed_set_rpm_i(speed_set_rpm_i),
    .iq_limit_ma_i(iq_limit_ma_i),
    .position_mode_i(position_mode),
    .position_speed_q8_i(position_speed_q8),
    .kp_q16_i(speed_kp_q16_i),
    .ki_q16_i(speed_ki_q16_i),
    .speed_q8_i(speed_q8_o),
    .enable_o(core_enable),
    .iq_request_o(iq_request),
    .ramp_q8_o(ramp_q8),
    .accepted_rpm_o(accepted_rpm_o),
    .state_o(speed_state_o),
    .fault_code_o(speed_fault_code),
    .saturated_o(saturated_o)
);

FOC_MOTOR_CORE #(
    .PWM_PERIOD(PWM_PERIOD),
    .REVERSE_DIRECTION(REVERSE_DIRECTION),
    .ALIGN_CURRENT_MA(ALIGN_CURRENT_MA),
    .SEARCH_CURRENT_MA(150),
    .REFERENCE_LIMIT_MA(MAX_IQ_MA),
    .PEAK_CURRENT_MA(PEAK_CURRENT_MA),
    .MAX_COUNTS_PER_PERIOD(MAX_COUNTS),
    .ALIGN_SAMPLES(ALIGN_SAMPLES),
    .SEARCH_TIMEOUT_SAMPLES(SEARCH_TIMEOUT_SAMPLES),
    .ADC_POWERUP_CLKS(ADC_POWERUP_CLKS),
    .OFFSET_LOG2(OFFSET_LOG2),
    .ENABLE_CONTROL_FAULT(1),
    .REQUEST_FRAC_BITS(8),
    .SMOOTH_ALIGN(SMOOTH_ALIGN),
    .PWM_FRAC_BITS(PWM_FRAC_BITS)
) core_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .enable_i(core_enable),
    .hard_trip_i(hard_trip_i),
    .id_request_i(24'sd0),
    .iq_request_i(iq_request),
    .kp_i(current_kp_q16_i),
    .ki_i(current_ki_effective),
    .control_fault_code_i(control_fault_code),
    .enc_a_i(enc_a_i),
    .enc_b_i(enc_b_i),
    .enc_z_i(enc_z_i),
    .adc_busy_i(adc_busy_i),
    .adc_data_i(adc_data_i),
    .adc_os_o(adc_os_o),
    .adc_cs_n_o(adc_cs_n_o),
    .adc_rd_n_o(adc_rd_n_o),
    .adc_reset_o(adc_reset_o),
    .adc_convst_o(adc_convst_o),
    .pwm_h_o(pwm_h_o),
    .pwm_l_o(pwm_l_o),
    .motor_sd_a_o(motor_sd_a_o),
    .fault_o(fault_o),
    .run_o(run_o),
    .encoder_ready_o(),
    .state_o(state_o),
    .fault_code_o(fault_code_o),
    .current_valid_o(),
    .id_o(id_o),
    .iq_o(iq_o),
    .id_reference_o(id_reference_o),
    .iq_reference_o(iq_reference_o),
    .electrical_phase_o(),
    .debug_o(core_debug_o),
    .adc_debug_o(adc_debug_o),
    .encoder_step_o(step_flag),
    .encoder_positive_o(step_positive)
);

endmodule
