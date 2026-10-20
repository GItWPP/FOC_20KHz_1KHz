//--------------------------------------------------------------------------------------------------------
// 模块： FOC_HOST_WATCHDOG
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： 可参数化的主机心跳看门狗与显式重新使能
// 作者： WANG
// 时间： 26.10.20
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps
//通用心跳与停机重新使能；串口TOP固定开启，超时2秒撤销permit。

module FOC_HOST_WATCHDOG
#(
    parameter integer TIMEOUT_CLKS = 100000000
)
(
    input wire   sys_clk,
    input wire   sys_rst_n,
    input wire   watchdog_en_i,
    input wire   heartbeat_i,
    input wire   start_i,

    output wire  permit_o,
    output reg   alive_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer COUNT_WIDTH = (TIMEOUT_CLKS > 0) ? $clog2(TIMEOUT_CLKS + 1) : 1;

//reg define
reg [COUNT_WIDTH - 1:0] count;
reg heartbeat_dly, armed;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign permit_o = !watchdog_en_i || (alive_o && (armed || !start_i));

always @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n) begin
        count <= 0;
        heartbeat_dly <= 0;
        alive_o <= 0;
        armed <= 0;
    end else begin
        heartbeat_dly <= heartbeat_i;
        if (!watchdog_en_i) begin
            count <= 0;
            alive_o <= 0;
            armed <= 0;
        end else begin
            if (heartbeat_i != heartbeat_dly) begin
                count <= 0;
                alive_o <= 1;
            end else if (count < TIMEOUT_CLKS)
                count <= count + 1'b1;
            else alive_o <= 0;
            if (!alive_o)
                armed <= 0;
            else if (!start_i)
                armed <= 1;
        end
    end

endmodule
