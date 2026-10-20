//--------------------------------------------------------------------------------------------------------
// 模块： FOC_UART_RX
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： 三级同步与分数位周期的8N1串口接收，校验起始和停止位
// 作者： WANG
// 时间： 26.10.20
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps

module FOC_UART_RX
#(
    parameter integer CLK_FREQ = 50000000,
    parameter integer BAUD_RATE = 1500000
)
(
    input wire        sys_clk,
    input wire        sys_rst_n,
    input wire        rx_i,

    output reg [7:0]  data_o,
    output reg        valid_o,
    output reg        frame_error_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer PHASE_WIDTH = (CLK_FREQ > 1) ? $clog2(CLK_FREQ) : 1;
localparam [2:0] IDLE = 3'd0, START_BIT = 3'd1, DATA_BITS = 3'd2, STOP_BIT = 3'd3, WAIT_HIGH = 3'd4;

//wire define
wire start_edge;
wire bit_sample;

//reg define
(* ASYNC_REG = "TRUE" *) reg [2:0] rx_sync;
reg rx_dly;
reg [2:0] state;
reg [PHASE_WIDTH - 1:0] phase;
reg [2:0] bit_count;
reg [7:0] shift_data;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign start_edge = rx_dly && !rx_sync[2];

assign bit_sample = phase >= CLK_FREQ - BAUD_RATE;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        rx_sync <= 3'b111;
        rx_dly <= 1'b1;
    end else begin
        rx_sync <= {rx_sync[1:0], rx_i};
        rx_dly <= rx_sync[2];
    end
end

//下降沿后半位确认起始位，随后在每一位中心采样；停止位错误等待总线恢复高。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state <= IDLE;
        phase <= {PHASE_WIDTH{1'b0}};
        bit_count <= 3'd0;
        shift_data <= 8'd0;
        data_o <= 8'd0;
        valid_o <= 1'b0;
        frame_error_o <= 1'b0;
    end else begin
        valid_o <= 1'b0;
        frame_error_o <= 1'b0;
        if (state != IDLE && state != WAIT_HIGH) begin
            if (bit_sample)
                phase <= phase + BAUD_RATE - CLK_FREQ;
            else phase <= phase + BAUD_RATE;
        end
        case (state)
            IDLE:
                if (start_edge) begin
                    phase <= CLK_FREQ / 2;
                    state <= START_BIT;
                end
            START_BIT:
                if (bit_sample) begin
                    if (!rx_sync[2]) begin
                        bit_count <= 3'd0;
                        state <= DATA_BITS;
                    end else state <= IDLE;
                end
            DATA_BITS:
                if (bit_sample) begin
                    shift_data[bit_count]<= rx_sync[2];
                    if (bit_count == 3'd7)
                        state <= STOP_BIT;
                    else bit_count <= bit_count + 1'b1;
                end
            STOP_BIT:
                if (bit_sample) begin
                    if (rx_sync[2]) begin
                        data_o <= shift_data;
                        valid_o <= 1'b1;
                        state <= IDLE;
                    end else begin
                        frame_error_o <= 1'b1;
                        state <= WAIT_HIGH;
                    end
                end
            WAIT_HIGH:
                if (rx_sync[2])
                    state <= IDLE;
            default:
                state <= IDLE;
        endcase
    end
end

endmodule
