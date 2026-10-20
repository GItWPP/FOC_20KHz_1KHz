//--------------------------------------------------------------------------------------------------------
// 模块： tb_foc_uart_control
// Type    : simulation
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： 验证真实串口命令、回执、正负角度、异常帧与失联重新使能
// 作者： WANG
// 时间： 26.10.20
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps

module tb_foc_uart_control
(
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//wire define
wire [7:0] rx_data;
wire rx_valid, rx_error, start, mode, soft_reset, heartbeat, armed, busy, alive, permit;
wire signed [31:0] target;
wire [15:0] rpm, iq_limit, ack_seq, good, errors;
wire [31:0] speed_kp, speed_ki, current_kp, current_ki, position_kp, ack_value;
wire [7:0] ack_op, ack_key, ack_status;

//reg define
reg clk = 1'b0, rst_n = 1'b0, rx = 1'b1, drive = 1'b0;
reg [3:0] motor_state = 4'd1;
reg [2:0] speed_state = 3'd0;
reg [7:0] fault = 8'd0;
reg [7:0] bytes[0:15];
reg [15:0] checksum;
reg [31:0] random_state = 32'h2a34b783;
reg signed [31:0] angle;
reg [63:0] magnitude, expected;
integer index, j, k, reset_pulses = 0;
integer old_good, old_errors, old_heartbeat;
integer seq_number = 1;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
always
    # 10 clk = !clk;

always @(negedge clk)
    if (soft_reset)
        reset_pulses = reset_pulses + 1;

task check;
    input condition;
    input [767:0] message;
    begin
        if (condition !== 1'b1)
            $fatal(1, "FAIL: UART control %0s", message);
    end
endtask

task send_byte;
    input [7:0] data;
    integer bit_no;
    begin
        rx = 1'b0;
        # 666.666667;
        for (bit_no = 0; bit_no < 8; bit_no = bit_no + 1) begin
            rx = data[bit_no];
            # 666.666667;
        end
        rx = 1'b1;
        # 666.666667;
    end
endtask

task packet_send;
    input [15:0] sequence_value;
    input [7:0] opcode, key;
    input [31:0] value;
    input bad_crc;
    begin
        bytes[0]= 8'h5a;
        bytes[1]= 8'ha5;
        bytes[2]= 8'd1;
        bytes[3]= 8'd16;
        bytes[4]= sequence_value[15:8];
        bytes[5]= sequence_value[7:0];
        bytes[6]= opcode;
        bytes[7]= key;
        bytes[8]= value[31:24];
        bytes[9]= value[23:16];
        bytes[10]= value[15:8];
        bytes[11]= value[7:0];
        bytes[12]= 8'd0;
        bytes[13]= 8'd0;
        checksum = 16'hffff;
        for (j = 0; j < 14; j = j + 1) begin
            checksum = checksum ^ {bytes[j], 8'd0};
            for (k = 0; k < 8; k = k + 1)
                checksum = checksum[15] ? (checksum << 1) ^ 16'h1021 : checksum << 1;
        end
        bytes[14]= checksum[15:8];
        bytes[15]= checksum[7:0]^ {7'd0, bad_crc};
        for (j = 0; j < 16; j = j + 1)
            send_byte(bytes[j]);
        repeat (60)
            @(negedge clk);
    end
endtask

task command;
    input [7:0] opcode, key;
    input [31:0] value;
    input [7:0] status_expected;
    begin
        packet_send(seq_number[15:0], opcode, key, value, 1'b0);
        check(ack_seq == seq_number[15:0]&& ack_op == opcode && ack_key == key, "ACK identity");
        if (ack_status !== status_expected)
            $fatal(1, "FAIL: command seq=%0d op=%0d key=%h status=%0d expected=%0d", seq_number, opcode, key, ack_status, status_expected);
        seq_number = seq_number + 1;
    end
endtask

task angle_command;
    input signed [31:0] value;
    reg signed [31:0] expected_signed;
    begin
        magnitude = value < 0 ? - 64'sd1 * value : value;
        expected = (magnitude * 64'd128 + 64'd2812) / 64'd5625;
        expected_signed = value < 0 ? - $signed(expected[31:0]) : $signed(expected[31:0]);
        command(8'd2, 8'h04, value, 8'd0);
        if (target !== expected_signed || ack_value !== value)
            $fatal(1, "FAIL: angle=%0d target=%0d expected=%0d", value, target, expected_signed);
    end
endtask

initial begin
    repeat (5)
        @(negedge clk);
    rst_n = 1'b1;
    repeat (20)
        @(negedge clk);
    check(!start && !armed && rpm == 300 && iq_limit == 200, "safe defaults");
    packet_send(16'd0, 8'd1, 8'd0, 32'd0, 1'b0);
    check(alive && !armed, "heartbeat alone cannot arm");
    command(8'd2, 8'h01, 32'd1, 8'd5);
    command(8'd2, 8'h01, 32'd0, 8'd0);
    check(armed && permit, "STOP establishes remote enable");
    command(8'd2, 8'h03, 32'd3001, 8'd3);
    check(rpm == 300, "reject rpm overflow without changing settings");
    command(8'd2, 8'h05, 32'd201, 8'd3);
    command(8'd2, 8'h10, 32'd655361, 8'd3);
    command(8'd2, 8'h03, 32'd600, 8'd0);
    command(8'd3, 8'h03, 32'd0, 8'd0);
    check(ack_value == 600, "readback rpm");
    command(8'd2, 8'h02, 32'd1, 8'd0);
    angle_command(32'sd90000);
    check(target == 2048, "90degrees =2048counts");
    angle_command(- 32'sd90000);
    angle_command(32'sd720000);
    check(target == 16384, "720degrees =2turns");
    angle_command(32'sh80000000);
    angle_command(32'sh7fffffff);
    for (index = 0; index < 64; index = index + 1) begin
        random_state = random_state * 32'd1664525 + 32'd1013904223;
        angle_command(random_state);
    end
    command(8'd2, 8'h01, 32'd1, 8'd0);
    drive = 1'b1;
    motor_state = 4'd6;
    speed_state = 3'd2;
    command(8'd2, 8'h02, 32'd0, 8'd4);
    command(8'd2, 8'h10, 32'd65536, 8'd4);
    command(8'd4, 8'h00, 32'd0, 8'd4);
    angle_command(32'sd360000);
    old_good = good;
    old_errors = errors;
    old_heartbeat = heartbeat;
    packet_send(16'hfffe, 8'd2, 8'h03, 32'd1800, 1'b1);
    check(good == old_good && errors == old_errors + 1 && rpm == 600 && heartbeat == old_heartbeat, "bad CRC cannot write or refresh link");
    send_byte(8'h5a);
    send_byte(8'ha5);
    send_byte(8'h01);
    # 60000;
    check(errors == old_errors + 2, "partial packet timeout");
    command(8'd2, 8'h01, 32'd0, 8'd0);
    check(!start && target == 0, "STOP clears prior position target");
    drive = 1'b0;
    motor_state = 4'd1;
    speed_state = 3'd0;
    command(8'd4, 8'h00, 32'd0, 8'd0);
    check(reset_pulses == 1 && !armed && !start, "reset is a pulse and disarms start");
    packet_send((seq_number - 1), 8'd4, 8'h00, 32'd0, 1'b0);
    check(reset_pulses == 1, "duplicate reset request not executed twice");
    command(8'd2, 8'h01, 32'd1, 8'd5);
    command(8'd2, 8'h01, 32'd0, 8'd0);
    command(8'd2, 8'h02, 32'd0, 8'd0);
    command(8'd2, 8'h01, 32'd1, 8'd0);
    # 4500000;
    check(!alive && !permit && !start && !armed, "lost host disarms motor");
    packet_send(16'd0, 8'd1, 8'd0, 32'd0, 1'b0);
    check(alive && !start && !armed, "restored heartbeats do not restart");
    command(8'd2, 8'h01, 32'd1, 8'd5);
    command(8'd2, 8'h01, 32'd0, 8'd0);
    command(8'd2, 8'h01, 32'd1, 8'd0);
    fault = 8'h0b;
    repeat (5)
        @(negedge clk);
    check(!start && !armed, "fault clears remote start");
    speed_state = 3'd4;
    command(8'd4, 8'd0, 32'd0, 8'd0);
    check(reset_pulses == 2, "reset accepted in stopped latched-fault state");
    packet_send((seq_number - 1), 8'd4, 8'd0, 32'd0, 1'b0);
    check(reset_pulses == 2, "fault-state reset retry remains idempotent");
    fault = 8'd0;
    speed_state = 3'd0;
    command(8'd2, 8'h01, 32'd1, 8'd5);
    command(8'd3, 8'hfe, 32'd0, 8'd2);
    command(8'd3, 8'h03, 32'd1, 8'd3);
    command(8'hfe, 8'd0, 32'd0, 8'd1);
    $display("PASS: physical UART command CRC/ACK/readback, +/-angle32 extremes, bounds, repeat/reset and host-loss explicit rearm");
    $finish;
end

initial begin
    # 25000000;
    $fatal(1, "FAIL: UART control timeout");
end


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
FOC_UART_RX rx_inst (
    .sys_clk(clk),
    .sys_rst_n(rst_n),
    .rx_i(rx),
    .data_o(rx_data),
    .valid_o(rx_valid),
    .frame_error_o(rx_error)
);

FOC_HOST_WATCHDOG #(
    .TIMEOUT_CLKS(200000)
) watchdog_inst (
    .sys_clk(clk),
    .sys_rst_n(rst_n),
    .watchdog_en_i(1'b1),
    .heartbeat_i(heartbeat),
    .start_i(start),
    .permit_o(permit),
    .alive_o(alive)
);

FOC_UART_CONTROL #(
    .FRAME_TIMEOUT_CLKS(2000)
) dut (
    .sys_clk(clk),
    .sys_rst_n(rst_n),
    .rx_data_i(rx_data),
    .rx_valid_i(rx_valid),
    .rx_error_i(rx_error),
    .host_alive_i(alive),
    .drive_i(drive),
    .motor_state_i(motor_state),
    .speed_state_i(speed_state),
    .fault_i(fault),
    .start_o(start),
    .position_mode_o(mode),
    .speed_set_rpm_o(rpm),
    .position_target_o(target),
    .iq_limit_ma_o(iq_limit),
    .speed_kp_q16_o(speed_kp),
    .speed_ki_q16_o(speed_ki),
    .current_kp_q16_o(current_kp),
    .current_ki_q16_o(current_ki),
    .position_kp_q16_o(position_kp),
    .soft_reset_o(soft_reset),
    .heartbeat_o(heartbeat),
    .remote_armed_o(armed),
    .busy_o(busy),
    .ack_sequence_o(ack_seq),
    .ack_opcode_o(ack_op),
    .ack_key_o(ack_key),
    .ack_status_o(ack_status),
    .ack_value_o(ack_value),
    .good_frames_o(good),
    .error_frames_o(errors)
);

endmodule
