`timescale 1ns/1ps
//AD7606并行八通道:外部触发,确认BUSY完整转换,一帧只发布一次

module ADC_DRIVER
#(
    parameter integer POWERUP_CLKS = 2500000,
    //10us全复位+300us恢复,同时兼容AD7606及AD7606C-16的274us全复位恢复要求
    parameter integer RESET_CLKS = 500,
    parameter integer RECOVERY_CLKS = 15000,
    parameter integer CONVST_CLKS = 5,
    parameter integer RD_LOW_CLKS = 4,
    parameter integer RD_HIGH_CLKS = 4,
    parameter integer TIMEOUT_CLKS = 1250
)
(
    input wire          sys_clk,
    input wire          sys_rst_n,
    input wire          sample_req_i,
    input wire          busy,
    input wire [15:0]   adc_data,

    output wire         ready_o,
    output wire         sample_flag_o,
    output reg          frame_valid_o,
    output reg          fault_o,
    output reg [15:0]   data_ch1,
    output reg [15:0]   data_ch2,
    output reg [15:0]   data_ch3,
    output reg [15:0]   data_ch4,
    output reg [15:0]   data_ch5,
    output reg [15:0]   data_ch6,
    output reg [15:0]   data_ch7,
    output reg [15:0]   data_ch8,
    output wire [2:0]   os_o,
    output reg          cs_o,
    output reg          rd_o,
    output reg          reset_o,
    output reg          convst_o,
    output wire [31:0]  debug_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam [3:0] ST_RESET = 0, ST_RECOVER = 1, ST_IDLE = 2, ST_CONVST = 3, ST_BUSY_HIGH = 4, ST_BUSY_LOW = 5,
    ST_READ_LOW = 6, ST_READ_HIGH = 7, ST_FAULT = 8, ST_POWERUP = 9, ST_CS_SETUP = 10;
localparam integer INIT_MAX = (POWERUP_CLKS > RESET_CLKS) ? POWERUP_CLKS : RESET_CLKS;
localparam integer RECOVER_MAX = (INIT_MAX > RECOVERY_CLKS) ? INIT_MAX : RECOVERY_CLKS;
localparam integer READ_MAX = (RD_LOW_CLKS > RD_HIGH_CLKS) ? RD_LOW_CLKS : RD_HIGH_CLKS;
localparam integer TRANSFER_MAX = (READ_MAX > CONVST_CLKS) ? READ_MAX : CONVST_CLKS;
localparam integer WAIT_MAX = (RECOVER_MAX > TRANSFER_MAX) ? RECOVER_MAX : TRANSFER_MAX;
localparam integer COUNT_MAX = (WAIT_MAX > TIMEOUT_CLKS) ? WAIT_MAX : TIMEOUT_CLKS;
localparam integer COUNT_WIDTH = (COUNT_MAX > 1) ? $clog2(COUNT_MAX) : 1;

//wire define
wire [15:0] count_debug;

//reg define
reg [3:0] state;
reg [COUNT_WIDTH - 1:0] count;
reg [2:0] channel;
reg busy_seen;
reg [3:0] last_state;
reg busy_dly;
reg [15:0] busy_width;
reg conversion_busy_seen;
(* ASYNC_REG = "TRUE" *) reg [1:0] busy_sync;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign count_debug = count;

assign os_o = 3'b000;

assign ready_o = (state == ST_IDLE) && !fault_o;

//故障后保留最后的非故障状态,用于区分未收到BUSY与BUSY不结束
assign debug_o = {busy_seen, 7'd0, last_state, state, (fault_o ? busy_width : count_debug)};

always @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n) begin
        busy_seen <= 0;
        last_state <= ST_RESET;
        busy_dly <= 0;
        busy_width <= 0;
    end else begin
        busy_dly <= busy_sync[1];
        if (busy_sync[1]&& !busy_dly)
            busy_width <= 16'd1;
        else if (busy_sync[1]&& busy_width != 16'hffff)
            busy_width <= busy_width + 1'b1;
        if (busy_sync[1])
            busy_seen <= 1;
        if (state != ST_FAULT)
            last_state <= state;
    end

//预译码使下游在CONVST实际上升的同一时钟沿锁存角度
assign sample_flag_o = sys_rst_n && ready_o && sample_req_i;

always @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n)
        busy_sync <= 0;
    else busy_sync <= {busy_sync[0], busy};

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state <= (POWERUP_CLKS > 0) ? ST_POWERUP : ST_RESET;
        count <= 0;
        channel <= 0;
        cs_o <= 1;
        rd_o <= 1;
        reset_o <= (POWERUP_CLKS == 0);
        convst_o <= 0;
        conversion_busy_seen <= 0;
        frame_valid_o <= 0;
        fault_o <= 0;
        data_ch1 <= 0;
        data_ch2 <= 0;
        data_ch3 <= 0;
        data_ch4 <= 0;
        data_ch5 <= 0;
        data_ch6 <= 0;
        data_ch7 <= 0;
        data_ch8 <= 0;
    end else begin
        frame_valid_o <= 0;
        if (state == ST_IDLE && sample_req_i)
            conversion_busy_seen <= 0;
        else if ((state == ST_CONVST || state == ST_BUSY_HIGH) && busy_sync[1])
            conversion_busy_seen <= 1;
        if (sample_req_i && !ready_o) begin
            fault_o <= 1;
            state <= ST_FAULT;
            cs_o <= 1;
            rd_o <= 1;
            convst_o <= 0;
        end else case (state)
            //OS输出先稳定,再产生新的RESET上升沿;与已验证的FPGA_Motor驱动一致
            ST_POWERUP:
                if (count == POWERUP_CLKS - 1) begin
                    reset_o <= 1;
                    count <= 0;
                    state <= ST_RESET;
                end else count <= count + 1'b1;
            ST_RESET:
                if (count == RESET_CLKS - 1) begin
                    reset_o <= 0;
                    count <= 0;
                    state <= ST_RECOVER;
                end else count <= count + 1'b1;
            ST_RECOVER:
                if (count == RECOVERY_CLKS - 1) begin
                    count <= 0;
                    state <= ST_IDLE;
                end else count <= count + 1'b1;
            ST_IDLE:
                if (sample_req_i) begin
                    if (busy_sync[1]) begin
                        fault_o <= 1;
                        state <= ST_FAULT;
                    end else begin
                        convst_o <= 1;
                        count <= 0;
                        state <= ST_CONVST;
                    end
                end
            ST_CONVST:
                if (count == CONVST_CLKS - 1) begin
                    convst_o <= 0;
                    count <= 0;
                    state <= ST_BUSY_HIGH;
                end else count <= count + 1'b1;
            ST_BUSY_HIGH:
                if (conversion_busy_seen || busy_sync[1]) begin
                    count <= 0;
                    state <= ST_BUSY_LOW;
                end else if (count == TIMEOUT_CLKS - 1) begin
                    fault_o <= 1;
                    state <= ST_FAULT;
                end else count <= count + 1'b1;
            ST_BUSY_LOW:
                if (!busy_sync[1]) begin
                    cs_o <= 0;
                    channel <= 0;
                    count <= 0;
                    state <= ST_CS_SETUP;
                end else if (count == TIMEOUT_CLKS - 1) begin
                    fault_o <= 1;
                    state <= ST_FAULT;
                end else count <= count + 1'b1;
            ST_CS_SETUP: begin
                rd_o <= 0;
                count <= 0;
                state <= ST_READ_LOW;
            end
            ST_READ_LOW:
                if (count == RD_LOW_CLKS - 1) begin
                    case (channel)
                        0:
                            data_ch1 <= adc_data;
                        1:
                            data_ch2 <= adc_data;
                        2:
                            data_ch3 <= adc_data;
                        3:
                            data_ch4 <= adc_data;
                        4:
                            data_ch5 <= adc_data;
                        5:
                            data_ch6 <= adc_data;
                        6:
                            data_ch7 <= adc_data;
                        7:
                            data_ch8 <= adc_data;
                        default:
                            ;
                    endcase
                    rd_o <= 1;
                    count <= 0;
                    state <= ST_READ_HIGH;
                end else count <= count + 1'b1;
            ST_READ_HIGH:
                if (count == RD_HIGH_CLKS - 1) begin
                    count <= 0;
                    if (channel == 7) begin
                        cs_o <= 1;
                        frame_valid_o <= 1;
                        state <= ST_IDLE;
                    end else begin
                        channel <= channel + 1'b1;
                        rd_o <= 0;
                        state <= ST_READ_LOW;
                    end
                end else count <= count + 1'b1;
            default: begin
                state <= ST_FAULT;
                fault_o <= 1;
                cs_o <= 1;
                rd_o <= 1;
                convst_o <= 0;
            end
        endcase
    end
end

endmodule
