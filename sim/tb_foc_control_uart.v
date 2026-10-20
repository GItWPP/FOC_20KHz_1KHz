//--------------------------------------------------------------------------------------------------------
// 模块： tb_foc_control_uart
// Type    : simulation
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： 验证V6毫度角度、参数和回执的完整串口快照与CRC
// 作者： WANG
// 时间： 26.10.20
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps

module tb_foc_control_uart
(
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//wire define
wire tx;

//reg define
reg clk = 1'b0, rst_n = 1'b0;
reg signed [31:0] position = - 12345, target = 16384, error = 28729;
reg [15:0] ack_seq = 16'h5678;
reg [31:0] speed_kp = 32'd65536;
reg [7:0] bytes[0:95];
reg [15:0] checksum;
reg signed [63:0] decoded;
integer j, k, fd;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
time first_start, next_start;

always
    # 10 clk = !clk;

function signed [63:0] mdeg;
    input signed [31:0] value;
    reg signed [63:0] magnitude;
    begin
        magnitude = value < 0 ? - 64'sd1 * value : value;
        magnitude = (magnitude * 64'd5625 + 64'd64) >> 7;
        mdeg = value < 0 ? - magnitude : magnitude;
    end
endfunction

task check;
    input condition;
    input [767:0] message;
    begin
        if (condition !== 1'b1)
            $fatal(1, "FAIL: V6 telemetry %0s", message);
    end
endtask

task read_byte;
    output [7:0] data;
    integer bit_no;
    begin
        @(negedge tx);
        if (j == 0)
            next_start = $time;
        # 1000;
        for (bit_no = 0; bit_no < 8; bit_no = bit_no + 1) begin
            data[bit_no]= tx;
            # 666.666667;
        end
        check(tx == 1'b1, "8N1 stop bit");
    end
endtask

task read_frame;
    begin
        for (j = 0; j < 96; j = j + 1) begin
            read_byte(bytes[j]);
            $fwrite(fd, "%c", bytes[j]);
        end
        check({bytes[0], bytes[1], bytes[2], bytes[3]}== 32'ha55a0660, "version/length");
        checksum = 16'hffff;
        for (j = 0; j < 94; j = j + 1) begin
            checksum = checksum ^ {bytes[j], 8'd0};
            for (k = 0; k < 8; k = k + 1)
                checksum = checksum[15] ? (checksum << 1) ^ 16'h1021 : checksum << 1;
        end
        check({bytes[94], bytes[95]}== checksum, "CRC covers all94bytes");
    end
endtask

task check_angles;
    input signed [31:0] actual, goal, difference;
    begin
        decoded = {bytes[30], bytes[31], bytes[32], bytes[33], bytes[34], bytes[35], bytes[36], bytes[37]};
        check(decoded == mdeg(actual), "actual angle signed64mdeg");
        decoded = {bytes[38], bytes[39], bytes[40], bytes[41], bytes[42], bytes[43], bytes[44], bytes[45]};
        check(decoded == mdeg(goal), "target angle signed64mdeg");
        decoded = {bytes[46], bytes[47], bytes[48], bytes[49], bytes[50], bytes[51], bytes[52], bytes[53]};
        check(decoded == mdeg(difference), "error angle signed64mdeg");
    end
endtask

initial begin
    fd = $fopen("control_uart_frames.bin", "wb");
    repeat (5)
        @(negedge clk);
    rst_n = 1'b1;
    fork
        begin
            read_frame;
            first_start = next_start;
        end
        begin
            @(negedge tx);
            # 10000;
            position = 8192;
            target = - 8192;
            error = - 16384;
            ack_seq = 16'h5679;
            speed_kp = 32'd131072;
        end
    join
    check_angles(- 12345, 16384, 28729);
    check({bytes[56], bytes[57]}== 16'h5678 && bytes[58]== 2 && bytes[59]== 0 && bytes[60]== 4, "ACK snapshot");
    check({bytes[66], bytes[67], bytes[68], bytes[69]}== 65536, "gain snapshot");
    check(bytes[54]== 3 && bytes[55]== 8'h83, "mode/link flags");
    read_frame;
    check(next_start - first_start == 1000000, "1000Hz frame cadence");
    check_angles(8192, - 8192, - 16384);
    check({bytes[56], bytes[57]}== 16'h5679, "next ACK frame");
    check({bytes[66], bytes[67], bytes[68], bytes[69]}== 131072, "next gain frame");
    position = - 1073741823;
    target = 1073741823;
    error = 2147483646;
    read_frame;
    check_angles(- 1073741823, 1073741823, 2147483646);
    $fclose(fd);
    $display("PASS: V6 96byte angle64/ACK/gain coherent snapshot, signed extremes, CRC and1kHz actual UART");
    $finish;
end

initial begin
    # 5000000;
    $fatal(1, "FAIL: V6 telemetry timeout");
end


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
FOC_SPEED_UART_MONITOR #(
    .POSITION_TELEMETRY(1),
    .CONTROL_TELEMETRY(1)
) dut (
    .sys_clk(clk),
    .sys_rst_n(rst_n),
    .flags_i(8'h7f),
    .fault_i(8'd0),
    .state_i(4'd6),
    .speed_state_i(3'd2),
    .accepted_rpm_i(16'd300),
    .ramp_rpm_i(- 16'sd120),
    .iq_limit_ma_i(16'd200),
    .speed_q8_i(- 32'sd2688),
    .id_i(- 16'sd2),
    .iq_i(- 16'sd123),
    .id_reference_i(16'd0),
    .iq_reference_i(16'sd100),
    .position_i(13'd1234),
    .position_counts_i(position),
    .position_target_i(target),
    .position_error_i(error),
    .position_flags_i(8'h03),
    .remote_flags_i(8'h83),
    .ack_sequence_i(ack_seq),
    .ack_opcode_i(8'd2),
    .ack_status_i(8'd0),
    .ack_key_i(8'd4),
    .ack_value_i(- 32'sd90000),
    .speed_kp_q16_i(speed_kp),
    .speed_ki_q16_i(32'd655),
    .current_kp_q16_i(32'd131072),
    .current_ki_q16_i(32'd4096),
    .position_kp_q16_i(32'd262144),
    .adc_frames_i(32'h12345678),
    .rx_good_i(16'd33),
    .rx_errors_i(16'd2),
    .tx_o(tx)
);

endmodule
