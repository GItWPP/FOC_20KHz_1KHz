//--------------------------------------------------------------------------------------------------------
// 模块： tb_foc_uart_top
// Type    : simulation
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： 真实UART管脚控制TOP、AD7606和ABZ/RL模型，验证启停与掉线保护
// 作者： WANG
// 时间： 26.10.20
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps

module tb_foc_uart_top
(
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer REVERSE_DIRECTION = 1;
//wire define
wire [2:0] high_pwm, low_pwm, os;
wire cs, rd, adc_reset, convst, sd, tx, fault, run, current_valid;
wire [3:0] state;
wire [7:0] fault_code;

//reg define
reg clk = 0, rst_n = 0, hard_trip = 0, rx = 1;
reg [1:0] ab = 0;
reg z = 0, busy = 0;
reg [15:0] adc_data = 0;
integer frames = 0, read_channel = 0, physical_position = 64, edge_clock = 0;
integer busy_mode = 0;
reg encoder_motion = 1, overspeed = 0, overcurrent = 0;
integer raw_samples[0:7];
real current_a = 0.0, current_b = 0.0, current_c = 0.0;
real pole_a, pole_b, pole_c, neutral;
reg [7:0] bytes[0:15];
reg [15:0] checksum;
integer j, k, index, seq_number = 1;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
time rd_start;

always
    # 10 clk = ~clk;

assign state = dut.motor_state;

assign fault_code = dut.fault_code;

assign fault = | fault_code;

assign run = dut.run;

assign current_valid = dut.drive_inst.core_inst.current_valid_o;

task check;
    input condition;
    input [767:0] message;
    begin
        if (condition !== 1'b1)
            $fatal(1, "FAIL: UART TOP %0s state=%0d fault=%h", message, state, fault_code);
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
    input [7:0] expected_status;
    begin
        packet_send(seq_number[15:0], opcode, key, value, 1'b0);
        check(dut.ack_sequence == seq_number[15:0]&& dut.ack_status == expected_status, "command ACK/status through real RX");
        seq_number = seq_number + 1;
    end
endtask

task hold_ms;
    input integer count;
    integer n;
    begin
        for (n = 0; n < count; n = n + 1) begin
            packet_send(16'd0, 8'd1, 8'd0, 32'd0, 1'b0);
            repeat (45000)
                @(negedge clk);
            check(!fault, "no fault during heartbeat and current control");
        end
    end
endtask

task start_wait;
    integer n;
    begin
        command(8'd2, 8'h01, 32'd1, 8'd0);
        n = 0;
        while (!run && !fault && n < 100) begin
            hold_ms(1);
            n = n + 1;
        end
        check(run && sd && !fault, "startup reaches RUN through UART");
    end
endtask

task stop_wait;
    integer n;
    begin
        command(8'd2, 8'h01, 32'd0, 8'd0);
        n = 0;
        while (sd && !fault && n < 200) begin
            hold_ms(1);
            n = n + 1;
        end
        check(!sd && !fault && !dut.start, "UART STOP leaves drive disabled");
    end
endtask

//真实管脚PWM驱动三相星接RL离散模型,采用20ns步长
always @(negedge clk) begin
    if (rst_n) begin
        pole_a = (sd && high_pwm[0]) ? 36.0 : 0.0;
        pole_b = (sd && high_pwm[1]) ? 36.0 : 0.0;
        pole_c = (sd && high_pwm[2]) ? 36.0 : 0.0;
        neutral = (pole_a + pole_b + pole_c) / 3.0;
        current_a = current_a + (pole_a - neutral - 1.2 * current_a) * 0.00000002 / 0.00075;
        current_b = current_b + (pole_b - neutral - 1.2 * current_b) * 0.00000002 / 0.00075;
        current_c = current_c + (pole_c - neutral - 1.2 * current_c) * 0.00000002 / 0.00075;
        //只在寻零时模拟机械运动;进入对齐/RUN后保持已知转子位置
        if ((state == 3 && encoder_motion) || (run && overspeed)) begin
            if (edge_clock == 63) begin
                edge_clock = 0;
                physical_position = (physical_position + (REVERSE_DIRECTION ? 8191 : 1)) % 8192;
                case (physical_position % 4)
                    0:
                        ab = 2'b00;
                    1:
                        ab = 2'b01;
                    2:
                        ab = 2'b11;
                    3:
                        ab = 2'b10;
                endcase
                z = (physical_position < 4);
            end else edge_clock = edge_clock + 1;
        end
    end
end

//ADC在CONVST上升沿同时采样三相;BUSY和数据相对FPGA时钟非对齐
always @(posedge convst)
    if (rst_n && !adc_reset) begin
        if (overcurrent) begin
            raw_samples[0]= 8126 - 2097;
            raw_samples[1]= 8142;
            raw_samples[2]= 8108 + 2097;
        end else begin
            raw_samples[0]= 8126 - $rtoi(current_a * 1048.576);
            raw_samples[1]= 8142 - $rtoi(current_b * 1048.576);
            raw_samples[2]= 8108 - $rtoi(current_c * 1048.576);
        end
        raw_samples[3]= 16'h1234;
        raw_samples[4]= 16'h5678;
        raw_samples[5]= 0;
        raw_samples[6]= 0;
        raw_samples[7]= 0;
        read_channel = 0;
        if (busy_mode != 1) begin
            # 30;
            busy = 1;
            if (busy_mode != 2) begin
                # 4000;
                busy = 0;
            end
        end
    end

always @(negedge rd)
    if (rst_n) begin
        rd_start = $time;
        # 30;
        adc_data = raw_samples[read_channel];
    end

always @(posedge rd)
    if (rst_n && !cs) begin
        check($time - rd_start >= 80, "full-system ADC read meets t14 margin");
        read_channel = read_channel + 1;
    end

always @(posedge clk) begin
    # 1;
    if (rst_n) begin
        check((high_pwm & low_pwm) == 0, "no bridge shoot-through at any system state");
        if (!sd)
            check(high_pwm == 0 && low_pwm == 0, "disabled SD always accompanies six off PWM outputs");
        if (current_valid)
            frames = frames + 1;
    end
end

initial begin
    repeat (8)
        @(negedge clk);
    rst_n = 1;
    wait(state == 1 || fault);
    check(!fault && !sd && !dut.start, "calibrate and idle without VIO or host");
    command(8'd2, 8'h01, 32'd0, 8'd0);
    command(8'd2, 8'h02, 32'd1, 8'd0);
    command(8'd2, 8'h03, 32'd60, 8'd0);
    start_wait;
    //寻Z留下的测速滤波状态需衰减后，才满足原±1rpm到位判据。
    hold_ms(35);
    check(dut.position_flags[2:0]== 3'b111 && dut.position_counts == 0, "position zero and hold after startup");
    command(8'd2, 8'h04, 32'd90000, 8'd0);
    hold_ms(2);
    check(dut.position_target == 2048, "90degree target reaches original position controller");
    for (index = 0; index < 16; index = index + 1) begin
        physical_position = (physical_position + 8191) % 8192;
        case (physical_position % 4)
            0:
                ab = 2'b00;
            1:
                ab = 2'b01;
            2:
                ab = 2'b11;
            3:
                ab = 2'b10;
        endcase
        z = (physical_position < 4);
        repeat (5000)
            @(negedge clk);
    end
    hold_ms(30);
    check(dut.position_counts == 16 && dut.position_error == 2032, "real AB steps update angle/error");
    command(8'd2, 8'h02, 32'd0, 8'd4);
    stop_wait;
    check(dut.position_target_counts == 0, "STOP clears stale position request");
    command(8'd2, 8'h02, 32'd0, 8'd0);
    command(8'd2, 8'h03, 32'd300, 8'd0);
    start_wait;
    hold_ms(3);
    # 12000000;
    check(!sd && !dut.start && !dut.remote_armed && fault_code == 8'h00, "host timeout shuts actual control/PWM and clears start");
    packet_send(16'd0, 8'd1, 8'd0, 32'd0, 1'b0);
    repeat (100)
        @(negedge clk);
    check(!sd && !dut.start, "returning heartbeats cannot restart");
    command(8'd4, 8'd0, 32'd0, 8'd0);
    wait(state == 1 || fault);
    check(!fault && !sd, "serial soft reset recovers stopped drive");
    command(8'd2, 8'h01, 32'd0, 8'd0);
    start_wait;
    @(negedge clk);
    # 3;
    hard_trip = 1;
    # 1;
    check(!sd && high_pwm == 0 && low_pwm == 0, "hard-trip asynchronous output guard preserved");
    repeat (5)
        @(negedge clk);
    check(fault_code == 8'h01 && !dut.start, "fault feedback clears remote request");
    hard_trip = 1'b0;
    command(8'd4, 8'd0, 32'd0, 8'd0);
    wait(state == 1 || fault);
    check(!fault && !sd && !dut.start, "serial RESET clears latched hard-trip fault without restart");
    command(8'd2, 8'h01, 32'd0, 8'd0);
    $display("PASS: UART-only TOP actual AD7606/ABZ/RL/PWM, degree target, mode guard, STOP/reset, host-loss and hard-trip");
    $finish;
end

initial begin
    # 500000000;
    $fatal(1, "FAIL: UART TOP timeout state=%0d fault=%h", state, fault_code);
end

defparam dut.drive_inst.ADC_POWERUP_CLKS = 8;

defparam dut.drive_inst.OFFSET_LOG2 = 3;

defparam dut.drive_inst.ALIGN_SAMPLES = 50;

defparam dut.drive_inst.SEARCH_TIMEOUT_SAMPLES = 3000;

defparam dut.drive_inst.speed_controller_inst.STOP_HOLD_TICKS = 3;

defparam dut.drive_inst.speed_controller_inst.STOP_TIMEOUT_TICKS = 200;

defparam dut.drive_inst.position_controller_inst.SETTLE_TICKS = 3;

defparam dut.host_watchdog_inst.TIMEOUT_CLKS = 500000;


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
TOP dut (
    .sys_clk(clk),
    .sys_rst_n(rst_n),
    .A(ab[1]),
    .B(ab[0]),
    .Z(z),
    .hard_trip_i(hard_trip),
    .adc_busy_i(busy),
    .adc_data_i(adc_data),
    .uart_rx_i(rx),
    .adc_os_o(os),
    .adc_cs_n_o(cs),
    .adc_rd_n_o(rd),
    .adc_reset_o(adc_reset),
    .adc_convst_o(convst),
    .pwm_h_o(high_pwm),
    .pwm_l_o(low_pwm),
    .motor_sd_a_o(sd),
    .uart_tx_o(tx)
);

endmodule
