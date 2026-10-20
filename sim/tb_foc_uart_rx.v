//--------------------------------------------------------------------------------------------------------
// 模块： tb_foc_uart_rx
// Type    : simulation
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： 验证1.5Mbps接收的频差、连续字节、短毛刺及错误停止位恢复
// 作者： WANG
// 时间： 26.10.20
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps

module tb_foc_uart_rx
(
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//wire define
wire [7:0] data;
wire valid, frame_error;

//reg define
reg clk = 1'b0, rst_n = 1'b0, rx = 1'b1;
reg [7:0] expected[0:511];
integer sent = 0, received = 0, frame_errors = 0, index;
real bit_time;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
always
    # 10 clk = !clk;

always @(negedge clk) begin
    if (valid) begin
        if (received >= sent || data !== expected[received])
            $fatal(1, "FAIL: RX index=%0d data=%h expected=%h", received, data, expected[received]);
        received = received + 1;
    end
    if (frame_error)
        frame_errors = frame_errors + 1;
end

task send_byte;
    input [7:0] value;
    input real period;
    input good_stop;
    integer bit_no;
    begin
        if (good_stop) begin
            expected[sent]= value;
            sent = sent + 1;
        end
        rx = 1'b0;
        # (period);
        for (bit_no = 0; bit_no < 8; bit_no = bit_no + 1) begin
            rx = value[bit_no];
            # (period);
        end
        rx = good_stop;
        # (period);
    end
endtask

initial begin
    repeat (5)
        @(negedge clk);
    rst_n = 1'b1;
    # 137;
    rx = 1'b0;
    # 80;
    rx = 1'b1;
    # 1500;
    if (received != 0 || frame_errors != 0)
        $fatal(1, "FAIL: short start glitch accepted");
    for (index = 0; index < 256; index = index + 1)
        send_byte(index[7:0], 666.666667, 1'b1);
    for (index = 0; index < 64; index = index + 1)
        send_byte(index[7:0]^ 8'ha5, 653.333333, 1'b1);
    for (index = 0; index < 64; index = index + 1)
        send_byte(index[7:0]^ 8'h5a, 680.0, 1'b1);
    send_byte(8'h00, 666.666667, 1'b0);
    # 2000;
    rx = 1'b1;
    # 2000;
    send_byte(8'hc3, 666.666667, 1'b1);
    # 2000;
    if (received != sent || frame_errors != 1)
        $fatal(1, "FAIL: RX totals sent=%0d received=%0d frame_errors=%0d", sent, received, frame_errors);
    $display("PASS: UART RX all256 bytes, continuous1.5Mbps, +/-2percent baud skew, glitch and bad-stop recovery");
    $finish;
end

initial begin
    # 4000000;
    $fatal(1, "FAIL: UART RX timeout");
end


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
FOC_UART_RX dut (
    .sys_clk(clk),
    .sys_rst_n(rst_n),
    .rx_i(rx),
    .data_o(data),
    .valid_o(valid),
    .frame_error_o(frame_error)
);

endmodule
