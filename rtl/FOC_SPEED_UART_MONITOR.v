//--------------------------------------------------------------------------------------------------------
// 模块： FOC_SPEED_UART_MONITOR
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： 速度、毫度角度、参数和命令回执的1kHz一致快照发送
// 作者： WANG
// 时间： 26.10.20
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps
//1.5Mbps/1kHz一致快照；V6 96字节含毫度角度与命令回执，兼容V3/V5仿真。

module FOC_SPEED_UART_MONITOR
#(
    parameter integer CLK_FREQ = 50000000,
    parameter integer BAUD_RATE = 1500000,
    parameter integer REPORT_CLKS = 50000,
    parameter integer POSITION_TELEMETRY = 0,
    parameter integer CONTROL_TELEMETRY = 0
)
(
    input wire                sys_clk,
    input wire                sys_rst_n,
    input wire [7:0]          flags_i,
    input wire [7:0]          fault_i,
    input wire [3:0]          state_i,
    input wire [2:0]          speed_state_i,
    input wire [15:0]         accepted_rpm_i,
    input wire [15:0]         ramp_rpm_i,
    input wire [15:0]         iq_limit_ma_i,
    input wire signed [31:0]  speed_q8_i,
    input wire signed [15:0]  id_i,
    input wire signed [15:0]  iq_i,
    input wire signed [15:0]  id_reference_i,
    input wire signed [15:0]  iq_reference_i,
    input wire [12:0]         position_i,
    input wire signed [31:0]  position_counts_i,
    input wire signed [31:0]  position_target_i,
    input wire signed [31:0]  position_error_i,
    input wire [7:0]          position_flags_i,
    input wire [7:0]          remote_flags_i,
    input wire [15:0]         ack_sequence_i,
    input wire [7:0]          ack_opcode_i,
    input wire [7:0]          ack_status_i,
    input wire [7:0]          ack_key_i,
    input wire [31:0]         ack_value_i,
    input wire [31:0]         speed_kp_q16_i,
    input wire [31:0]         speed_ki_q16_i,
    input wire [31:0]         current_kp_q16_i,
    input wire [31:0]         current_ki_q16_i,
    input wire [31:0]         position_kp_q16_i,
    input wire [31:0]         adc_frames_i,
    input wire [15:0]         rx_good_i,
    input wire [15:0]         rx_errors_i,

    output wire               tx_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer FRAME_BYTES = CONTROL_TELEMETRY ? 96 :(POSITION_TELEMETRY ? 48 : 32);
localparam integer BYTE_WIDTH = $clog2(FRAME_BYTES);
localparam integer ANGLE_FIELDS_HIGH = CONTROL_TELEMETRY ? (FRAME_BYTES - 2) * 8 - 241 : 191;
localparam integer PAYLOAD_BYTES = FRAME_BYTES - 2;
localparam integer REPORT_WIDTH = (REPORT_CLKS > 1) ? $clog2(REPORT_CLKS) : 1;

//wire define
wire tx_ready;
wire [239:0] base_frame;
wire [PAYLOAD_BYTES * 8 - 1:0] live_frame;
wire [7:0] crc_byte;
wire [751:0] control_frame;

//reg define
reg [REPORT_WIDTH - 1:0] report_count;
reg [2:0] state;
reg [BYTE_WIDTH - 1:0] byte_index;
reg signed [31:0] angle_actual_snapshot, angle_target_snapshot, angle_error_snapshot;
reg signed [45:0] angle_actual_product, angle_target_product, angle_error_product;
reg [PAYLOAD_BYTES * 8 - 1:0] snapshot;
reg [15:0] sequence_count, checksum;
reg tx_valid;
reg [7:0] tx_byte;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign base_frame = {8'ha5, 8'h5a, (CONTROL_TELEMETRY ? 8'h06 :(POSITION_TELEMETRY ? 8'h05 : 8'h03)), (CONTROL_TELEMETRY
    ? 8'd96 :(POSITION_TELEMETRY ? 8'd48 : 8'd32)), sequence_count, flags_i, 4'd0, state_i, fault_i, 5'd0,
    speed_state_i, accepted_rpm_i, ramp_rpm_i, speed_q8_i, id_i, iq_i, id_reference_i, iq_reference_i, iq_limit_ma_i,
    3'd0, position_i};

assign live_frame = POSITION_TELEMETRY ?{base_frame, position_counts_i, position_target_i, position_error_i,
    position_flags_i, 24'd0}: base_frame;

assign control_frame = {base_frame, 192'd0, position_flags_i, remote_flags_i, ack_sequence_i, ack_opcode_i,
    ack_status_i, ack_key_i, 8'd0, ack_value_i, speed_kp_q16_i, speed_ki_q16_i, current_kp_q16_i, current_ki_q16_i,
    position_kp_q16_i, adc_frames_i, rx_good_i, rx_errors_i};

assign crc_byte = snapshot >> ((PAYLOAD_BYTES - 1 - byte_index) * 8);

//8192计数/圈：毫度=计数*5625/128，负数按幅值舍入后恢复符号。
function signed [63:0] rounded_mdeg;
    input signed [45:0] value;
    reg signed [45:0] magnitude;
    begin
        magnitude = value[45] ? - value : value;
        magnitude = (magnitude + 46'sd64) >>> 7;
        rounded_mdeg = value[45] ? - magnitude : magnitude;
    end
endfunction

function [15:0] update_crc;
    input [15:0] old_crc;
    input [7:0] data;
    reg [15:0] crc;
    integer bit_no;
    begin
        crc = old_crc ^ {data, 8'd0};
        for (bit_no = 0; bit_no < 8; bit_no = bit_no + 1)
            crc = crc[15] ? (crc << 1) ^ 16'h1021 :(crc << 1);
        update_crc = crc;
    end
endfunction

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        report_count <= 0;
        state <= 0;
        byte_index <= 0;
        sequence_count <= 0;
        snapshot <= 0;
        checksum <= 0;
        tx_valid <= 0;
        tx_byte <= 0;
        angle_actual_snapshot <= 32'sd0;
        angle_target_snapshot <= 32'sd0;
        angle_error_snapshot <= 32'sd0;
        angle_actual_product <= 46'sd0;
        angle_target_product <= 46'sd0;
        angle_error_product <= 46'sd0;
    end else begin
        tx_valid <= 0;
        if (report_count == REPORT_CLKS - 1)
            report_count <= 0;
        else report_count <= report_count + 1'b1;
        case (state)
            0:
                if (report_count == REPORT_CLKS - 1) begin
                    snapshot <= CONTROL_TELEMETRY ? control_frame : live_frame;
                    angle_actual_snapshot <= position_counts_i;
                    angle_target_snapshot <= position_target_i;
                    angle_error_snapshot <= position_error_i;
                    checksum <= 16'hffff;
                    byte_index <= 0;
                    sequence_count <= sequence_count + 1'b1;
                    state <= CONTROL_TELEMETRY ? 3'd5 : 3'd4;
                end
            1:
                if (tx_ready) begin
                    if (byte_index == PAYLOAD_BYTES)
                        tx_byte <= checksum[15:8];
                    else if (byte_index == FRAME_BYTES - 1)
                        tx_byte <= checksum[7:0];
                    else tx_byte <= snapshot >> ((PAYLOAD_BYTES - 1 - byte_index) * 8);
                    tx_valid <= 1;
                    state <= 2;
                end
            2:
                if (!tx_ready)
                    state <= 3;
            3:
                if (tx_ready) begin
                    if (byte_index == FRAME_BYTES - 1)
                        state <= 0;
                    else begin
                        byte_index <= byte_index + 1'b1;
                        state <= 1;
                    end
                end
            4: begin
                checksum <= update_crc(checksum, crc_byte);
                if (byte_index == PAYLOAD_BYTES - 1) begin
                    byte_index <= 0;
                    state <= 1;
                end else byte_index <= byte_index + 1'b1;
            end
            5: begin
                angle_actual_product <= angle_actual_snapshot * 14'sd5625;
                angle_target_product <= angle_target_snapshot * 14'sd5625;
                angle_error_product <= angle_error_snapshot * 14'sd5625;
                state <= 3'd6;
            end
            6: begin
                snapshot[ANGLE_FIELDS_HIGH -:192]<= {rounded_mdeg(angle_actual_product),
                    rounded_mdeg(angle_target_product), rounded_mdeg(angle_error_product)};
                state <= 3'd4;
            end
            default:
                state <= 0;
        endcase
    end
end


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
uart_tx_16bit #(
    .CLK_FREQ(CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
) tx_inst (
    .clk(sys_clk),
    .rst_n(sys_rst_n),
    .tx_dv(tx_valid),
    .tx_byte(tx_byte),
    .tx_ready(tx_ready),
    .txd(tx_o)
);

endmodule
