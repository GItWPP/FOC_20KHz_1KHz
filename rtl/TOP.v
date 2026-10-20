//--------------------------------------------------------------------------------------------------------
// 模块： TOP
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： AX7Z020串口控制的速度与多圈角度闭环，保留异步硬故障关断
// 作者： WANG
// 时间： 26.10.20
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps

module TOP
#(
    parameter integer REVERSE_DIRECTION = 1
)
(
    input wire         sys_clk,
    input wire         sys_rst_n,
    input wire         A,
    input wire         B,
    input wire         Z,
    input wire         hard_trip_i,
    input wire         adc_busy_i,
    input wire [15:0]  adc_data_i,
    input wire         uart_rx_i,

    output wire [2:0]  adc_os_o,
    output wire        adc_cs_n_o,
    output wire        adc_rd_n_o,
    output wire        adc_reset_o,
    output wire        adc_convst_o,
    output wire [2:0]  pwm_h_o,
    output wire [2:0]  pwm_l_o,
    output wire        motor_sd_a_o,
    output wire        uart_tx_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//wire define
wire control_rst_n, reset_request_n, rst_n;
wire start, soft_reset, heartbeat, permit, host_alive, remote_armed, control_busy;
wire [15:0] speed_set_rpm, iq_limit_ma;
wire position_mode;
wire signed [31:0] position_target_counts, position_counts, position_target, position_error;
wire [31:0] position_kp_q16, speed_kp_q16, speed_ki_q16, current_kp_q16, current_ki_q16;
wire [5:0] position_flags;
wire signed [31:0] speed_rpm_q8;
wire signed [15:0] id, iq, id_reference, iq_reference;
wire [15:0] accepted_rpm, ramp_rpm;
wire [2:0] speed_state;
wire [3:0] motor_state;
wire [7:0] fault_code;
wire saturated, run;
wire [255:0] core_debug;
wire [7:0] flags, remote_flags, rx_data;
wire rx_valid, rx_error;
wire [15:0] ack_sequence, good_frames, error_frames;
wire [7:0] ack_opcode, ack_key, ack_status;
wire [31:0] ack_value;

//reg define
(* ASYNC_REG = "TRUE" *) reg [2:0] control_reset_sync;
(* ASYNC_REG = "TRUE" *) reg [2:0] reset_sync;
reg [31:0] frame_count;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
//串口命令与遥测不随电机软复位清空，复位回执可连续读到。
assign control_rst_n = control_reset_sync[2];

assign reset_request_n = control_rst_n && !soft_reset;

assign rst_n = reset_sync[2];

assign flags = {saturated, host_alive, 1'b1, core_debug[16], core_debug[18], run, motor_sd_a_o, start};

assign remote_flags = {1'b1, 2'd0, control_busy, start, position_mode, host_alive, remote_armed};

always @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n)
        control_reset_sync <= 3'b000;
    else control_reset_sync <= {control_reset_sync[1:0], 1'b1};

always @(posedge sys_clk or negedge reset_request_n)
    if (!reset_request_n)
        reset_sync <= 3'b000;
    else reset_sync <= {reset_sync[1:0], 1'b1};

always @(posedge sys_clk or negedge rst_n)
    if (!rst_n)
        frame_count <= 32'd0;
    else if (core_debug[9])
        frame_count <= frame_count + 1'b1;


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
FOC_UART_RX rx_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(control_rst_n),
    .rx_i(uart_rx_i),
    .data_o(rx_data),
    .valid_o(rx_valid),
    .frame_error_o(rx_error)
);

FOC_UART_CONTROL control_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(control_rst_n),
    .rx_data_i(rx_data),
    .rx_valid_i(rx_valid),
    .rx_error_i(rx_error),
    .host_alive_i(host_alive),
    .drive_i(motor_sd_a_o),
    .motor_state_i(motor_state),
    .speed_state_i(speed_state),
    .fault_i(fault_code),
    .start_o(start),
    .position_mode_o(position_mode),
    .speed_set_rpm_o(speed_set_rpm),
    .position_target_o(position_target_counts),
    .iq_limit_ma_o(iq_limit_ma),
    .speed_kp_q16_o(speed_kp_q16),
    .speed_ki_q16_o(speed_ki_q16),
    .current_kp_q16_o(current_kp_q16),
    .current_ki_q16_o(current_ki_q16),
    .position_kp_q16_o(position_kp_q16),
    .soft_reset_o(soft_reset),
    .heartbeat_o(heartbeat),
    .remote_armed_o(remote_armed),
    .busy_o(control_busy),
    .ack_sequence_o(ack_sequence),
    .ack_opcode_o(ack_opcode),
    .ack_key_o(ack_key),
    .ack_status_o(ack_status),
    .ack_value_o(ack_value),
    .good_frames_o(good_frames),
    .error_frames_o(error_frames)
);

FOC_HOST_WATCHDOG host_watchdog_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(control_rst_n),
    .watchdog_en_i(1'b1),
    .heartbeat_i(heartbeat),
    .start_i(start),
    .permit_o(permit),
    .alive_o(host_alive)
);

FOC_SPEED_DRIVE #(
    .REVERSE_DIRECTION(REVERSE_DIRECTION),
    .POSITION_ENABLE(1)
) drive_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(rst_n),
    .start_i(start),
    .permit_i(permit),
    .hard_trip_i(hard_trip_i),
    .speed_set_rpm_i(speed_set_rpm),
    .iq_limit_ma_i(iq_limit_ma),
    .position_mode_i(position_mode),
    .position_target_i(position_target_counts),
    .position_kp_q16_i(position_kp_q16),
    .speed_kp_q16_i(speed_kp_q16),
    .speed_ki_q16_i(speed_ki_q16),
    .current_kp_q16_i(current_kp_q16),
    .current_ki_q16_i(current_ki_q16),
    .enc_a_i(A),
    .enc_b_i(B),
    .enc_z_i(Z),
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
    .speed_q8_o(speed_rpm_q8),
    .speed_rpm_o(),
    .accepted_rpm_o(accepted_rpm),
    .ramp_rpm_o(ramp_rpm),
    .speed_state_o(speed_state),
    .saturated_o(saturated),
    .moving_o(),
    .fault_o(),
    .run_o(run),
    .state_o(motor_state),
    .fault_code_o(fault_code),
    .id_o(id),
    .iq_o(iq),
    .id_reference_o(id_reference),
    .iq_reference_o(iq_reference),
    .core_debug_o(core_debug),
    .adc_debug_o(),
    .position_counts_o(position_counts),
    .position_target_o(position_target),
    .position_error_o(position_error),
    .position_flags_o(position_flags)
);

FOC_SPEED_UART_MONITOR #(
    .POSITION_TELEMETRY(1),
    .CONTROL_TELEMETRY(1)
) monitor_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(control_rst_n),
    .flags_i(flags),
    .fault_i(fault_code),
    .state_i(motor_state),
    .speed_state_i(speed_state),
    .accepted_rpm_i(accepted_rpm),
    .ramp_rpm_i(ramp_rpm),
    .iq_limit_ma_i(iq_limit_ma),
    .speed_q8_i(speed_rpm_q8),
    .id_i(id),
    .iq_i(iq),
    .id_reference_i(id_reference),
    .iq_reference_i(iq_reference),
    .position_i(core_debug[111:99]),
    .position_counts_i(position_counts),
    .position_target_i(position_target),
    .position_error_i(position_error),
    .position_flags_i({2'd0, position_flags}),
    .remote_flags_i(remote_flags),
    .ack_sequence_i(ack_sequence),
    .ack_opcode_i(ack_opcode),
    .ack_status_i(ack_status),
    .ack_key_i(ack_key),
    .ack_value_i(ack_value),
    .speed_kp_q16_i(speed_kp_q16),
    .speed_ki_q16_i(speed_ki_q16),
    .current_kp_q16_i(current_kp_q16),
    .current_ki_q16_i(current_ki_q16),
    .position_kp_q16_i(position_kp_q16),
    .adc_frames_i(frame_count),
    .rx_good_i(good_frames),
    .rx_errors_i(error_frames),
    .tx_o(uart_tx_o)
);

endmodule
