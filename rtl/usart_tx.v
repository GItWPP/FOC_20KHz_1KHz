`timescale  1ns/1ps
// Byte-oriented UART transmitter,8 data bits,no parity,1 stop bit.

module uart_tx_16bit
#(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD_RATE = 115_200
)
(
    //系统时钟
    input wire        clk,
    //系统复位,低电平有效
    input wire        rst_n,
    //单字节发送请求
    input wire        tx_dv,
    //待发送数据
    input wire [7:0]  tx_byte,

    //发送器空闲
    output reg        tx_ready,
    //UART串行输出
    output reg        txd
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
//分数分频:50MHz/1.5Mbps=33+1/3拍,交替使用33/34拍而非截断成33拍。
localparam integer BAUD_THRESHOLD = CLK_FREQ - BAUD_RATE;
localparam [1:0] ST_IDLE = 2'd0;
localparam [1:0] ST_START = 2'd1;
localparam [1:0] ST_DATA = 2'd2;
localparam [1:0] ST_STOP = 2'd3;
localparam integer BAUD_WIDTH = (CLK_FREQ > 1) ? $clog2(CLK_FREQ) : 1;

//reg define
reg [1:0] state;
reg [BAUD_WIDTH - 1:0] baud_cnt;
reg [2:0] bit_cnt;
reg [7:0] data_buf;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
//UART发送状态机:起始位、8位数据和停止位均按波特率计数
always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
        state <= ST_IDLE;
        baud_cnt <= 32'd0;
        bit_cnt <= 3'd0;
        data_buf <= 8'd0;
        tx_ready <= 1'b1;
        txd <= 1'b1;
    end else begin
        case (state)
            ST_IDLE: begin
                tx_ready <= 1'b1;
                txd <= 1'b1;
                baud_cnt <= 32'd0;
                bit_cnt <= 3'd0;
                if (tx_dv == 1'b1) begin
                    tx_ready <= 1'b0;
                    data_buf <= tx_byte;
                    state <= ST_START;
                end
            end
            ST_START: begin
                txd <= 1'b0;
                if (baud_cnt >= BAUD_THRESHOLD) begin
                    baud_cnt <= baud_cnt - BAUD_THRESHOLD;
                    state <= ST_DATA;
                end else begin
                    baud_cnt <= baud_cnt + BAUD_RATE;
                end
            end
            ST_DATA: begin
                txd <= data_buf[bit_cnt];
                if (baud_cnt >= BAUD_THRESHOLD) begin
                    baud_cnt <= baud_cnt - BAUD_THRESHOLD;
                    if (bit_cnt == 3'd7) begin
                        bit_cnt <= 3'd0;
                        state <= ST_STOP;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end else begin
                    baud_cnt <= baud_cnt + BAUD_RATE;
                end
            end
            ST_STOP: begin
                txd <= 1'b1;
                if (baud_cnt >= BAUD_THRESHOLD) begin
                    baud_cnt <= baud_cnt - BAUD_THRESHOLD;
                    state <= ST_IDLE;
                end else begin
                    baud_cnt <= baud_cnt + BAUD_RATE;
                end
            end
            default: begin
                state <= ST_IDLE;
                baud_cnt <= 32'd0;
                bit_cnt <= 3'd0;
                tx_ready <= 1'b1;
                txd <= 1'b1;
            end
        endcase
    end
end

endmodule
