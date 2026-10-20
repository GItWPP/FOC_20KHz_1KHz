`timescale 1ns/1ps
//完整电流/转矩环;板级按键/VIO和UART位于TOP,此层只处理同域控制

module FOC_MOTOR_CORE
#(
    parameter integer PWM_PERIOD = 2500,
    parameter integer DEAD_CLKS = 50,
    parameter integer SAMPLE_OFFSET = 100,
    parameter integer ZERO_MARGIN = 200,
    parameter integer POLE_PAIRS = 4,
    parameter integer REVERSE_DIRECTION = 0,
    parameter integer AB_FILTER_CLKS = 16,
    parameter integer OFFSET_LOG2 = 8,
    parameter signed [31:0] CURRENT_GAIN_Q16 = - 32'sd62500,
    parameter signed [31:0] CURRENT_GAIN_A_Q16 = CURRENT_GAIN_Q16,
    parameter signed [31:0] CURRENT_GAIN_B_Q16 = CURRENT_GAIN_Q16,
    parameter signed [31:0] CURRENT_GAIN_C_Q16 = CURRENT_GAIN_Q16,
    parameter integer REQUEST_FRAC_BITS = 0,
    parameter integer SMOOTH_ALIGN = 0,
    parameter integer PWM_FRAC_BITS = 0,
    parameter integer PEAK_CURRENT_MA = 1300,
    parameter integer MAX_COUNTS_PER_PERIOD = 18,
    parameter integer ALIGN_SAMPLES = 10000,
    parameter integer SEARCH_TIMEOUT_SAMPLES = 200000,
    parameter integer ALIGN_CURRENT_MA = 600,
    parameter integer SEARCH_CURRENT_MA = 800,
    parameter integer REFERENCE_LIMIT_MA = 1000,
    parameter [31:0] SEARCH_PHASE_STEP = 32'd1717987,
    parameter integer ADC_TIMEOUT_CLKS = 1250,
    parameter integer ADC_POWERUP_CLKS = 2500000,
    parameter integer ENABLE_CONTROL_FAULT = 0
)
(
    input wire                                    sys_clk,
    input wire                                    sys_rst_n,
    input wire                                    enable_i,
    input wire                                    hard_trip_i,
    input wire signed [15 + REQUEST_FRAC_BITS:0]  id_request_i,
    input wire signed [15 + REQUEST_FRAC_BITS:0]  iq_request_i,
    input wire [31:0]                             kp_i,
    input wire [31:0]                             ki_i,
    input wire [7:0]                              control_fault_code_i,
    input wire                                    enc_a_i,
    input wire                                    enc_b_i,
    input wire                                    enc_z_i,
    input wire                                    adc_busy_i,
    input wire [15:0]                             adc_data_i,

    output wire [2:0]                             adc_os_o,
    output wire                                   adc_cs_n_o,
    output wire                                   adc_rd_n_o,
    output wire                                   adc_reset_o,
    output wire                                   adc_convst_o,
    output wire [2:0]                             pwm_h_o,
    output wire [2:0]                             pwm_l_o,
    output wire                                   motor_sd_a_o,
    output wire                                   fault_o,
    output wire                                   run_o,
    output wire                                   encoder_ready_o,
    output wire [3:0]                             state_o,
    output reg [7:0]                              fault_code_o,
    output wire                                   current_valid_o,
    output wire signed [15:0]                     id_o,
    output wire signed [15:0]                     iq_o,
    output wire signed [15:0]                     id_reference_o,
    output wire signed [15:0]                     iq_reference_o,
    output wire [31:0]                            electrical_phase_o,
    output wire [255:0]                           debug_o,
    output wire [31:0]                            adc_debug_o,
    output wire                                   encoder_step_o,
    output wire                                   encoder_positive_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer CURRENT_FRAC_BITS = 8, CURRENT_WIDTH = 24;
localparam signed [23:0] REFERENCE_LIMIT_FIXED = REFERENCE_LIMIT_MA * 256;
localparam signed [23:0] PEAK_CURRENT_FIXED = PEAK_CURRENT_MA * 256;
localparam integer CONTROL_WATCHDOG_WIDTH = $clog2(2 * PWM_PERIOD + 2);

//wire define
wire period_flag, sample_req, adc_ready, adc_sample_flag, adc_frame_valid, adc_fault;
wire [15:0] raw_a, raw_b, raw_c;
wire [15:0] offset_a, offset_b, offset_c;
wire calibrated, sense_valid, rail_fault;
wire signed [15:0] ia_ma, ib_ma, ic_ma;
wire signed [23:0] ia_fixed, ib_fixed, ic_fixed, id_fixed, iq_fixed;
wire signed [15:0] startup_id_reference, startup_iq_reference;
wire signed [15:0] id_request_ma;
wire signed [15:0] iq_request_ma;
wire signed [23:0] id_request_fixed;
wire signed [23:0] iq_request_fixed;
wire signed [23:0] id_reference_fixed;
wire signed [23:0] iq_reference_fixed;
wire encoder_fault, angle_valid, indexed;
wire [12:0] position;
wire [31:0] mechanical_phase, open_phase;
wire drive_request, calibrate_flag, startup_timeout, align_timeout;
wire drive_enable;
wire loop_clear;
wire [31:0] control_phase;
wire d_valid, q_valid, inverse_valid, duty_valid;
wire signed [15:0] vd, vq, voltage_alpha, voltage_beta;
wire [15 + PWM_FRAC_BITS:0] half_a, half_b, half_c;
wire [2:0] pwm_high_internal, pwm_low_internal;
wire all_low;
wire signed [12:0] position_delta;
wire current_trip;
wire speed_trip;
wire adc_deadline;
wire sample_unsafe;
wire control_timeout;
wire angle_fault;
wire control_fault;
wire software_fault;
wire [143:0] current_debug_live;

//reg define
reg [31:0] phase_sample;
reg [3:0] sample_state, previous_state;
reg signed [23:0] sample_id_reference, sample_iq_reference;
reg sample_angle_valid;
reg adc_initialized;
reg trip_latched;
reg [CONTROL_WATCHDOG_WIDTH - 1:0] control_watchdog;
reg [12:0] previous_position;
reg previous_run;
reg trip_snapshot_valid;
reg [143:0] trip_snapshot;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign id_request_ma = id_request_i >>> REQUEST_FRAC_BITS;

assign iq_request_ma = iq_request_i >>> REQUEST_FRAC_BITS;

assign id_request_fixed = $signed(id_request_i) <<< (CURRENT_FRAC_BITS - REQUEST_FRAC_BITS);

assign iq_request_fixed = $signed(iq_request_i) <<< (CURRENT_FRAC_BITS - REQUEST_FRAC_BITS);

assign id_reference_fixed = run_o ? clamp_reference(id_request_fixed) : $signed({startup_id_reference, 8'd0});

assign iq_reference_fixed = run_o ? clamp_reference(iq_request_fixed) : $signed({startup_iq_reference, 8'd0});

assign control_phase = run_o ? electrical_phase_o : open_phase;

assign position_delta = position - previous_position;

assign current_trip = sense_valid && ((ia_fixed > PEAK_CURRENT_FIXED) || (ia_fixed < - PEAK_CURRENT_FIXED) || (ib_fixed
    > PEAK_CURRENT_FIXED) || (ib_fixed < - PEAK_CURRENT_FIXED) || (ic_fixed > PEAK_CURRENT_FIXED) || (ic_fixed < -
    PEAK_CURRENT_FIXED));

assign speed_trip = period_flag && run_o && previous_run && (($signed(position_delta) > MAX_COUNTS_PER_PERIOD) ||
    ($signed(position_delta) < - MAX_COUNTS_PER_PERIOD));

assign adc_deadline = sample_req && adc_initialized && !adc_ready;

assign sample_unsafe = adc_sample_flag && drive_enable && !all_low;

assign control_timeout = drive_enable && (control_watchdog >= 2 * PWM_PERIOD);

assign angle_fault = run_o && !angle_valid;

assign control_fault = ENABLE_CONTROL_FAULT && (control_fault_code_i != 0);

assign software_fault = adc_fault || adc_deadline || encoder_fault || rail_fault || current_trip || startup_timeout ||
    align_timeout || sample_unsafe || control_timeout || speed_trip || angle_fault || control_fault;

assign current_debug_live = {raw_a, raw_b, raw_c, offset_a, offset_b, offset_c, ia_ma, ib_ma, ic_ma};

assign fault_o = trip_latched || hard_trip_i;

assign drive_enable = sys_rst_n && drive_request && !trip_latched;

assign loop_clear = !drive_enable || (state_o != previous_state);

//只在VIO/串口/调试边界舍入到整数mA;电流PI使用完整Q8数据。
assign ia_ma = round_ma(ia_fixed);

assign ib_ma = round_ma(ib_fixed);

assign ic_ma = round_ma(ic_fixed);

assign id_o = round_ma(id_fixed);

assign iq_o = round_ma(iq_fixed);

assign id_reference_o = round_ma(id_reference_fixed);

assign iq_reference_o = round_ma(iq_reference_fixed);

function signed [23:0] clamp_reference;
    input signed [23:0] value;
    begin
        if (value > REFERENCE_LIMIT_FIXED)
            clamp_reference = REFERENCE_LIMIT_FIXED;
        else if (value < - REFERENCE_LIMIT_FIXED)
            clamp_reference = - REFERENCE_LIMIT_FIXED;
        else clamp_reference = value;
    end
endfunction

function signed [15:0] round_ma;
    input signed [23:0] value;
    reg signed [24:0] rounded;
    begin
        rounded = ($signed({value[23], value}) + (value[23] ? 25'sd127 : 25'sd128)) >>> 8;
        if (rounded > 25'sd32767)
            round_ma = 16'sh7fff;
        else if (rounded < - 25'sd32768)
            round_ma = 16'sh8000;
        else round_ma = rounded[15:0];
    end
endfunction

//调试输出未连接时由综合优化;从高位依次为原始值/零偏/电流/位置/相位/状态
assign debug_o = {(trip_snapshot_valid ? trip_snapshot : current_debug_live), position, mechanical_phase,
    electrical_phase_o, state_o, fault_code_o, fault_o, run_o, encoder_ready_o, encoder_fault, indexed, angle_valid,
    calibrated, sense_valid, adc_initialized, adc_ready, adc_fault, adc_busy_i, adc_sample_flag, adc_frame_valid,
    d_valid, q_valid, inverse_valid, duty_valid, drive_enable, enable_i, enc_a_i, enc_b_i, enc_z_i};

//首次04故障锁存三相原始码/零偏/电流,不被随后关断后的正常ADC帧覆盖。
//低112位位置/状态仍实时更新;仅增加诊断存储,不延迟或屏蔽保护路径。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        trip_snapshot_valid <= 0;
        trip_snapshot <= 0;
    end else if (!trip_snapshot_valid && (rail_fault || current_trip)) begin
        trip_snapshot_valid <= 1;
        trip_snapshot <= current_debug_live;
    end
end

//外部故障异步置位且直接组合关断六路PWM/SD;复位仅在故障输入恢复后清除
always @(posedge sys_clk or posedge hard_trip_i) begin
    if (hard_trip_i)
        trip_latched <= 1'b1;
    else if (!sys_rst_n)
        trip_latched <= 1'b0;
    else if (software_fault)
        trip_latched <= 1'b1;
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        adc_initialized <= 0;
        previous_state <= 0;
        phase_sample <= 0;
        sample_state <= 0;
        sample_angle_valid <= 0;
        sample_id_reference <= 0;
        sample_iq_reference <= 0;
        control_watchdog <= 0;
        previous_position <= 0;
        previous_run <= 0;
        fault_code_o <= 0;
    end else begin
        if (adc_ready)
            adc_initialized <= 1;
        previous_state <= state_o;
        if (adc_sample_flag) begin
            phase_sample <= control_phase;
            sample_state <= state_o;
            sample_angle_valid <= drive_enable && (!run_o || angle_valid);
            sample_id_reference <= id_reference_fixed;
            sample_iq_reference <= iq_reference_fixed;
        end
        if (period_flag) begin
            previous_position <= position;
            previous_run <= run_o;
        end
        if (!drive_enable || current_valid_o)
            control_watchdog <= 0;
        else control_watchdog <= control_watchdog + 1'b1;
        if (fault_code_o == 0) begin
            //短于一个时钟的外部故障也由异步锁存状态留下诊断记录
            if (trip_latched && !software_fault)
                fault_code_o <= 8'h01;
            else if (adc_fault || adc_deadline)
                fault_code_o <= 8'h02;
            else if (encoder_fault || angle_fault)
                fault_code_o <= 8'h03;
            else if (rail_fault || current_trip)
                fault_code_o <= 8'h04;
            else if (startup_timeout)
                fault_code_o <= 8'h05;
            else if (align_timeout)
                fault_code_o <= 8'h0b;
            else if (sample_unsafe)
                fault_code_o <= 8'h06;
            else if (control_timeout)
                fault_code_o <= 8'h07;
            else if (speed_trip)
                fault_code_o <= 8'h08;
            else if (control_fault)
                fault_code_o <= control_fault_code_i;
        end
    end
end


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
ADC_DRIVER #(
    .TIMEOUT_CLKS(ADC_TIMEOUT_CLKS),
    .POWERUP_CLKS(ADC_POWERUP_CLKS)
) adc_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .sample_req_i(sample_req && adc_ready),
    .busy(adc_busy_i),
    .adc_data(adc_data_i),
    .ready_o(adc_ready),
    .sample_flag_o(adc_sample_flag),
    .frame_valid_o(adc_frame_valid),
    .fault_o(adc_fault),
    .data_ch1(raw_a),
    .data_ch2(raw_b),
    .data_ch3(raw_c),
    .data_ch4(),
    .data_ch5(),
    .data_ch6(),
    .data_ch7(),
    .data_ch8(),
    .os_o(adc_os_o),
    .cs_o(adc_cs_n_o),
    .rd_o(adc_rd_n_o),
    .reset_o(adc_reset_o),
    .convst_o(adc_convst_o),
    .debug_o(adc_debug_o)
);

FOC_CURRENT_SENSE #(
    .OFFSET_LOG2(OFFSET_LOG2),
    .GAIN_Q16(CURRENT_GAIN_Q16),
    .GAIN_A_Q16(CURRENT_GAIN_A_Q16),
    .GAIN_B_Q16(CURRENT_GAIN_B_Q16),
    .GAIN_C_Q16(CURRENT_GAIN_C_Q16),
    .FRAC_BITS(CURRENT_FRAC_BITS)
) sense_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .calibrate_en_i(!drive_request),
    .frame_valid_i(adc_frame_valid),
    .raw_a_i(raw_a),
    .raw_b_i(raw_b),
    .raw_c_i(raw_c),
    .calibrated_o(calibrated),
    .valid_o(sense_valid),
    .ia_o(ia_fixed),
    .ib_o(ib_fixed),
    .ic_o(ic_fixed),
    .offset_a_o(offset_a),
    .offset_b_o(offset_b),
    .offset_c_o(offset_c),
    .rail_fault_o(rail_fault)
);

HKT56_ENCODER_IF #(
    .REVERSE_DIRECTION(REVERSE_DIRECTION),
    .AB_FILTER_CLKS(AB_FILTER_CLKS)
) encoder_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .enc_a_async(enc_a_i),
    .enc_b_async(enc_b_i),
    .enc_z_async(enc_z_i),
    .position_o(position),
    .phase_o(mechanical_phase),
    .encoder_valid_o(),
    .encoder_fault_o(encoder_fault),
    .encoder_ready_o(encoder_ready_o),
    .encoder_indexed_o(indexed),
    .index_flag_o(),
    .step_flag_o(encoder_step_o),
    .step_positive_o(encoder_positive_o)
);

FOC_ENCODER_ANGLE #(
    .POLE_PAIRS(POLE_PAIRS)
) angle_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .phase_m_i(mechanical_phase),
    .encoder_ready_i(encoder_ready_o),
    .encoder_fault_i(encoder_fault),
    .calibrate_flag_i(calibrate_flag),
    .align_phase_i(32'd0),
    .phase_e_o(electrical_phase_o),
    .angle_valid_o(angle_valid),
    .angle_calibrated_o()
);

FOC_STARTUP #(
    .ALIGN_SAMPLES(ALIGN_SAMPLES),
    .SEARCH_TIMEOUT_SAMPLES(SEARCH_TIMEOUT_SAMPLES),
    .SMOOTH_ALIGN(SMOOTH_ALIGN),
    .ALIGN_CURRENT_MA(ALIGN_CURRENT_MA),
    .SEARCH_CURRENT_MA(SEARCH_CURRENT_MA),
    .REFERENCE_LIMIT_MA(REFERENCE_LIMIT_MA),
    .SEARCH_PHASE_STEP(SEARCH_PHASE_STEP)
) startup_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .period_flag_i(period_flag),
    .enable_i(enable_i),
    .fault_i(trip_latched),
    .current_calibrated_i(calibrated),
    .encoder_ready_i(encoder_ready_o),
    .encoder_position_i(position),
    .id_request_i(id_request_ma),
    .iq_request_i(iq_request_ma),
    .drive_enable_o(drive_request),
    .run_o(run_o),
    .calibrate_flag_o(calibrate_flag),
    .timeout_fault_o(startup_timeout),
    .align_timeout_fault_o(align_timeout),
    .state_o(state_o),
    .open_phase_o(open_phase),
    .id_reference_o(startup_id_reference),
    .iq_reference_o(startup_iq_reference)
);

FOC_CURRENT_TRANS #(
    .CURRENT_WIDTH(CURRENT_WIDTH)
) feedback_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .DATA_VAL(sense_valid && sample_angle_valid && (sample_state == state_o) && !loop_clear),
    .Ia(ia_fixed),
    .Ib(ib_fixed),
    .Ic(ic_fixed),
    .phase_sample_i(phase_sample),
    .angle_sample_valid_i(sample_angle_valid),
    .encoder_fault_i(loop_clear),
    .I_VAL(),
    .I_alpha(),
    .I_beta(),
    .data_val(current_valid_o),
    .I_d(id_fixed),
    .I_q(iq_fixed)
);

FOC_PI #(
    .CURRENT_FRAC_BITS(CURRENT_FRAC_BITS)
) d_pi_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .clear_i(loop_clear),
    .valid_i(current_valid_o),
    .kp_i(kp_i),
    .ki_i(ki_i),
    .reference_i(sample_id_reference),
    .feedback_i(id_fixed),
    .valid_o(d_valid),
    .voltage_o(vd),
    .saturated_o()
);

FOC_PI #(
    .CURRENT_FRAC_BITS(CURRENT_FRAC_BITS)
) q_pi_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .clear_i(loop_clear),
    .valid_i(current_valid_o),
    .kp_i(kp_i),
    .ki_i(ki_i),
    .reference_i(sample_iq_reference),
    .feedback_i(iq_fixed),
    .valid_o(q_valid),
    .voltage_o(vq),
    .saturated_o()
);

FOC_INVERSE_PARK inverse_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .valid_i(d_valid && q_valid),
    .flush_i(loop_clear),
    .vd_i(vd),
    .vq_i(vq),
    .phase_i(control_phase),
    .valid_o(inverse_valid),
    .alpha_o(voltage_alpha),
    .beta_o(voltage_beta)
);

FOC_SVPWM #(
    .HALF_TICKS(PWM_PERIOD / 2),
    .ZERO_MARGIN(ZERO_MARGIN),
    .DUTY_FRAC_BITS(PWM_FRAC_BITS)
) svpwm_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .valid_i(inverse_valid),
    .clear_i(loop_clear),
    .alpha_i(voltage_alpha),
    .beta_i(voltage_beta),
    .valid_o(duty_valid),
    .half_a_o(half_a),
    .half_b_o(half_b),
    .half_c_o(half_c)
);

FOC_PWM_DRIVER #(
    .PWM_PERIOD(PWM_PERIOD),
    .DEAD_CLKS(DEAD_CLKS),
    .SAMPLE_OFFSET(SAMPLE_OFFSET),
    .DUTY_FRAC_BITS(PWM_FRAC_BITS)
) pwm_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .enable_i(drive_enable),
    .duty_valid_i(duty_valid),
    .half_a_i(half_a),
    .half_b_i(half_b),
    .half_c_i(half_c),
    .period_flag_o(period_flag),
    .sample_req_o(sample_req),
    .pwm_h_o(pwm_high_internal),
    .pwm_l_o(pwm_low_internal),
    .all_low_o(all_low)
);

(* DONT_TOUCH = "TRUE" *) FOC_OUTPUT_GUARD guard_inst(.trip_i(hard_trip_i), .drive_i({drive_enable, pwm_high_internal, pwm_low_internal}), .drive_o({motor_sd_a_o, pwm_h_o, pwm_l_o}));

endmodule
