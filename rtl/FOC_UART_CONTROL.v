//--------------------------------------------------------------------------------------------------------
// 模块： FOC_UART_CONTROL
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： CRC命令解析、控制参数与回执、毫度目标换算和远端重新使能
// 作者： WANG
// 时间： 26.10.20
//--------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps

module FOC_UART_CONTROL
#(
    parameter integer FRAME_TIMEOUT_CLKS = 100000
)
(
    input wire                sys_clk,
    input wire                sys_rst_n,
    input wire [7:0]          rx_data_i,
    input wire                rx_valid_i,
    input wire                rx_error_i,
    input wire                host_alive_i,
    input wire                drive_i,
    input wire [3:0]          motor_state_i,
    input wire [2:0]          speed_state_i,
    input wire [7:0]          fault_i,

    output reg                start_o,
    output reg                position_mode_o,
    output reg [15:0]         speed_set_rpm_o,
    output reg signed [31:0]  position_target_o,
    output reg [15:0]         iq_limit_ma_o,
    output reg [31:0]         speed_kp_q16_o,
    output reg [31:0]         speed_ki_q16_o,
    output reg [31:0]         current_kp_q16_o,
    output reg [31:0]         current_ki_q16_o,
    output reg [31:0]         position_kp_q16_o,
    output reg                soft_reset_o,
    output reg                heartbeat_o,
    output reg                remote_armed_o,
    output wire               busy_o,
    output reg [15:0]         ack_sequence_o,
    output reg [7:0]          ack_opcode_o,
    output reg [7:0]          ack_key_o,
    output reg [7:0]          ack_status_o,
    output reg [31:0]         ack_value_o,
    output reg [15:0]         good_frames_o,
    output wire [15:0]        error_frames_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer TIMEOUT_WIDTH = (FRAME_TIMEOUT_CLKS > 1) ? $clog2(FRAME_TIMEOUT_CLKS) : 1;
localparam [1:0] SEEK_FIRST = 2'd0, SEEK_SECOND = 2'd1, RECEIVE = 2'd2;
localparam [7:0] OP_HEARTBEAT = 8'd1, OP_WRITE = 8'd2, OP_READ = 8'd3, OP_RESET = 8'd4;
localparam [7:0] KEY_START = 8'h01, KEY_MODE = 8'h02, KEY_RPM = 8'h03, KEY_ANGLE = 8'h04, KEY_IQ = 8'h05, KEY_SPEED_KP =
    8'h10, KEY_SPEED_KI = 8'h11, KEY_CURRENT_KP = 8'h12, KEY_CURRENT_KI = 8'h13, KEY_POSITION_KP = 8'h14;
localparam [7:0] OK = 8'd0, BAD_OPCODE = 8'd1, BAD_KEY = 8'd2, BAD_VALUE = 8'd3, STOP_REQUIRED = 8'd4, NOT_ARMED = 8'd5,
    NOT_READY = 8'd6, MOTOR_FAULT = 8'd7, BUSY = 8'd8;

//wire define
wire [15:0] request_sequence;
wire [7:0] request_opcode, request_key;
wire [31:0] request_value;
wire stopped;
wire duplicate_request;
wire valid_heartbeat;
wire [31:0] angle_magnitude;
wire [13:0] division_trial;
wire division_take;
wire [38:0] division_next_quotient;

//reg define
reg [1:0] parse_state;
reg [3:0] byte_count;
reg [111:0] packet;
reg [15:0] crc;
reg [7:0] crc_high;
reg [TIMEOUT_WIDTH - 1:0] timeout_count;
reg command_valid;
reg [15:0] parse_errors, command_errors;
reg [7:0] response_status;
reg [31:0] response_value;
reg known_key;
reg signed [31:0] target_mdeg;
reg [79:0] last_request;
reg last_request_valid;
reg [7:0] last_result;
reg host_alive_dly;
reg fault_active_dly;
reg division_busy, division_negative;
reg [38:0] division_numerator, division_quotient;
reg [12:0] division_remainder;
reg [5:0] division_count;
reg [15:0] division_sequence;
reg [31:0] division_value;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign request_sequence = packet[79:64];

assign request_opcode = packet[63:56];

assign request_key = packet[55:48];

assign request_value = packet[47:16];

assign stopped = !start_o && !drive_i && speed_state_i == 3'd0;

assign duplicate_request = last_request_valid && last_request == packet[79:0];

assign valid_heartbeat = request_opcode == OP_HEARTBEAT && request_key == 8'd0 && request_value == 32'd0 &&
    packet[15:0]== 16'd0;

assign angle_magnitude = request_value[31] ? (~request_value + 32'd1) : request_value;

assign division_trial = {division_remainder, division_numerator[38]};

assign division_take = division_trial >= 14'd5625;

assign division_next_quotient = {division_quotient[37:0], division_take};

assign busy_o = division_busy;

assign error_frames_o = parse_errors + command_errors;

function [15:0] update_crc;
    input [15:0] old_crc;
    input [7:0] data;
    reg [15:0] value;
    integer bit_no;
    begin
        value = old_crc ^ {data, 8'd0};
        for (bit_no = 0; bit_no < 8; bit_no = bit_no + 1)
            value = value[15] ? (value << 1) ^ 16'h1021 :(value << 1);
        update_crc = value;
    end
endfunction

//只在完整CRC通过后发布单拍command_valid，半帧超时及停止位错误回到搜头。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        parse_state <= SEEK_FIRST;
        byte_count <= 4'd0;
        packet <= 112'd0;
        crc <= 16'hffff;
        crc_high <= 8'd0;
        timeout_count <= {TIMEOUT_WIDTH{1'b0}};
        command_valid <= 1'b0;
        good_frames_o <= 16'd0;
        parse_errors <= 16'd0;
    end else begin
        command_valid <= 1'b0;
        if (rx_error_i) begin
            parse_state <= SEEK_FIRST;
            timeout_count <= {TIMEOUT_WIDTH{1'b0}};
            parse_errors <= parse_errors + 1'b1;
        end else if (rx_valid_i) begin
            timeout_count <= {TIMEOUT_WIDTH{1'b0}};
            case (parse_state)
                SEEK_FIRST:
                    if (rx_data_i == 8'h5a) begin
                        packet <= {104'd0, rx_data_i};
                        crc <= update_crc(16'hffff, rx_data_i);
                        parse_state <= SEEK_SECOND;
                    end
                SEEK_SECOND:
                    if (rx_data_i == 8'ha5) begin
                        packet <= {packet[103:0], rx_data_i};
                        crc <= update_crc(crc, rx_data_i);
                        byte_count <= 4'd2;
                        parse_state <= RECEIVE;
                    end else if (rx_data_i != 8'h5a)
                        parse_state <= SEEK_FIRST;
                RECEIVE: begin
                    if ((byte_count == 4'd2 && rx_data_i != 8'd1) || (byte_count == 4'd3 && rx_data_i != 8'd16)) begin
                        parse_state <= SEEK_FIRST;
                        parse_errors <= parse_errors + 1'b1;
                    end else if (byte_count == 4'd15) begin
                        parse_state <= SEEK_FIRST;
                        if ({crc_high, rx_data_i}== crc) begin
                            command_valid <= 1'b1;
                            good_frames_o <= good_frames_o + 1'b1;
                        end else parse_errors <= parse_errors + 1'b1;
                    end else begin
                        byte_count <= byte_count + 1'b1;
                        if (byte_count == 4'd14)
                            crc_high <= rx_data_i;
                        else begin
                            packet <= {packet[103:0], rx_data_i};
                            crc <= update_crc(crc, rx_data_i);
                        end
                    end
                end
                default:
                    parse_state <= SEEK_FIRST;
            endcase
        end else if (parse_state != SEEK_FIRST) begin
            if (timeout_count == FRAME_TIMEOUT_CLKS - 1) begin
                parse_state <= SEEK_FIRST;
                timeout_count <= {TIMEOUT_WIDTH{1'b0}};
                parse_errors <= parse_errors + 1'b1;
            end else timeout_count <= timeout_count + 1'b1;
        end
    end
end

//读回与边界译码；高级系数及模式只允许完全停机时写入。
always @(*) begin
    response_status = OK;
    response_value = request_value;
    known_key = 1'b1;
    case (request_key)
        KEY_START:
            response_value = {31'd0, start_o};
        KEY_MODE:
            response_value = {31'd0, position_mode_o};
        KEY_RPM:
            response_value = {16'd0, speed_set_rpm_o};
        KEY_ANGLE:
            response_value = target_mdeg;
        KEY_IQ:
            response_value = {16'd0, iq_limit_ma_o};
        KEY_SPEED_KP:
            response_value = speed_kp_q16_o;
        KEY_SPEED_KI:
            response_value = speed_ki_q16_o;
        KEY_CURRENT_KP:
            response_value = current_kp_q16_o;
        KEY_CURRENT_KI:
            response_value = current_ki_q16_o;
        KEY_POSITION_KP:
            response_value = position_kp_q16_o;
        default:
            known_key = 1'b0;
    endcase
    if (packet[15:0]!= 16'd0)
        response_status = BAD_VALUE;
    else if (request_opcode == OP_READ) begin
        if (!known_key)
            response_status = BAD_KEY;
        else if (request_value != 32'd0)
            response_status = BAD_VALUE;
    end else if (request_opcode == OP_RESET) begin
        response_value = 32'd0;
        if (request_key != 8'd0 || request_value != 32'd0)
            response_status = BAD_VALUE;
        else if (start_o || drive_i)
            response_status = STOP_REQUIRED;
    end else if (request_opcode == OP_WRITE) begin
        response_value = request_value;
        if (!known_key)
            response_status = BAD_KEY;
        else case (request_key)
            KEY_START:
                if (request_value > 32'd1)
                    response_status = BAD_VALUE;
                else if (request_value == 32'd1) begin
                    if (!remote_armed_o || !host_alive_i)
                        response_status = NOT_ARMED;
                    else if (fault_i != 8'd0)
                        response_status = MOTOR_FAULT;
                    else if (!start_o && (motor_state_i != 4'd1 || drive_i))
                        response_status = NOT_READY;
                    else if (speed_set_rpm_o == 16'd0 || iq_limit_ma_o == 16'd0)
                        response_status = BAD_VALUE;
                end
            KEY_MODE:
                if (request_value > 32'd1)
                    response_status = BAD_VALUE;
                else if (!stopped)
                    response_status = STOP_REQUIRED;
            KEY_RPM:
                if (request_value > 32'd3000)
                    response_status = BAD_VALUE;
            KEY_IQ:
                if (request_value > 32'd200)
                    response_status = BAD_VALUE;
            KEY_SPEED_KP, KEY_POSITION_KP:
                if (request_value > 32'd655360)
                    response_status = BAD_VALUE;
                else if (!stopped)
                    response_status = STOP_REQUIRED;
            KEY_SPEED_KI, KEY_CURRENT_KI:
                if (request_value > 32'd65536)
                    response_status = BAD_VALUE;
                else if (!stopped)
                    response_status = STOP_REQUIRED;
            KEY_CURRENT_KP:
                if (request_value > 32'd1048576)
                    response_status = BAD_VALUE;
                else if (!stopped)
                    response_status = STOP_REQUIRED;
            default:
                response_status = OK;
        endcase
    end else response_status = BAD_OPCODE;
end

//命令提交和角度顺序除法。39拍小于一个UART字节周期，位置目标只在换算完成后更新。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        start_o <= 1'b0;
        position_mode_o <= 1'b0;
        speed_set_rpm_o <= 16'd300;
        position_target_o <= 32'sd0;
        target_mdeg <= 32'sd0;
        iq_limit_ma_o <= 16'd200;
        speed_kp_q16_o <= 32'd65536;
        speed_ki_q16_o <= 32'd655;
        current_kp_q16_o <= 32'd131072;
        current_ki_q16_o <= 32'd4096;
        position_kp_q16_o <= 32'd262144;
        soft_reset_o <= 1'b0;
        heartbeat_o <= 1'b0;
        remote_armed_o <= 1'b0;
        ack_sequence_o <= 16'd0;
        ack_opcode_o <= 8'd0;
        ack_key_o <= 8'd0;
        ack_status_o <= 8'hff;
        ack_value_o <= 32'd0;
        command_errors <= 16'd0;
        last_request <= 80'd0;
        last_request_valid <= 1'b0;
        last_result <= OK;
        host_alive_dly <= 1'b0;
        fault_active_dly <= 1'b0;
        division_busy <= 1'b0;
        division_negative <= 1'b0;
        division_numerator <= 39'd0;
        division_quotient <= 39'd0;
        division_remainder <= 13'd0;
        division_count <= 6'd0;
        division_sequence <= 16'd0;
        division_value <= 32'd0;
    end else begin
        soft_reset_o <= 1'b0;
        host_alive_dly <= host_alive_i;
        fault_active_dly <= fault_i != 8'd0;
        if (division_busy) begin
            division_numerator <= {division_numerator[37:0], 1'b0};
            division_remainder <= division_take ? division_trial - 14'd5625 : division_trial;
            division_quotient <= division_next_quotient;
            division_count <= division_count - 1'b1;
            if (division_count == 6'd1) begin
                position_target_o <= division_negative ? - $signed(division_next_quotient[31:0]) :
                    $signed(division_next_quotient[31:0]);
                target_mdeg <= division_value;
                division_busy <= 1'b0;
                ack_sequence_o <= division_sequence;
                ack_opcode_o <= OP_WRITE;
                ack_key_o <= KEY_ANGLE;
                ack_status_o <= OK;
                ack_value_o <= division_value;
            end
        end
        if (command_valid) begin
            if (valid_heartbeat)
                heartbeat_o <= !heartbeat_o;
            else if (duplicate_request) begin
                if (last_result == OK)
                    heartbeat_o <= !heartbeat_o;
            end else if (division_busy && !(request_opcode == OP_WRITE && request_key == KEY_START && request_value ==
                32'd0)) begin
                ack_sequence_o <= request_sequence;
                ack_opcode_o <= request_opcode;
                ack_key_o <= request_key;
                ack_status_o <= BUSY;
                ack_value_o <= request_value;
                command_errors <= command_errors + 1'b1;
            end else begin
                last_request <= packet[79:0];
                last_request_valid <= 1'b1;
                last_result <= response_status;
                if (response_status == OK)
                    heartbeat_o <= !heartbeat_o;
                else command_errors <= command_errors + 1'b1;
                if (response_status == OK && request_opcode == OP_WRITE && request_key == KEY_ANGLE) begin
                    division_busy <= 1'b1;
                    division_negative <= request_value[31];
                    division_numerator <= {angle_magnitude, 7'd0} + 39'd2812;
                    division_quotient <= 39'd0;
                    division_remainder <= 13'd0;
                    division_count <= 6'd39;
                    division_sequence <= request_sequence;
                    division_value <= request_value;
                end else begin
                    ack_sequence_o <= request_sequence;
                    ack_opcode_o <= request_opcode;
                    ack_key_o <= request_key;
                    ack_status_o <= response_status;
                    ack_value_o <= response_value;
                    if (response_status == OK) begin
                        if (request_opcode == OP_RESET) begin
                            soft_reset_o <= 1'b1;
                            start_o <= 1'b0;
                            remote_armed_o <= 1'b0;
                            position_target_o <= 32'sd0;
                            target_mdeg <= 32'sd0;
                            division_busy <= 1'b0;
                        end else if (request_opcode == OP_WRITE) begin
                            case (request_key)
                                KEY_START: begin
                                    start_o <= request_value[0];
                                    if (!request_value[0]) begin
                                        remote_armed_o <= 1'b1;
                                        position_target_o <= 32'sd0;
                                        target_mdeg <= 32'sd0;
                                        division_busy <= 1'b0;
                                    end
                                end
                                KEY_MODE: begin
                                    position_mode_o <= request_value[0];
                                    position_target_o <= 32'sd0;
                                    target_mdeg <= 32'sd0;
                                end
                                KEY_RPM:
                                    speed_set_rpm_o <= request_value[15:0];
                                KEY_IQ:
                                    iq_limit_ma_o <= request_value[15:0];
                                KEY_SPEED_KP:
                                    speed_kp_q16_o <= request_value;
                                KEY_SPEED_KI:
                                    speed_ki_q16_o <= request_value;
                                KEY_CURRENT_KP:
                                    current_kp_q16_o <= request_value;
                                KEY_CURRENT_KI:
                                    current_ki_q16_o <= request_value;
                                KEY_POSITION_KP:
                                    position_kp_q16_o <= request_value;
                                default: begin
                                end
                            endcase
                        end
                    end
                end
            end
        end
        //掉线撤销旧启动及位置目标；恢复心跳后必须重新STOP和显式START。
        if (host_alive_dly && !host_alive_i) begin
            start_o <= 1'b0;
            remote_armed_o <= 1'b0;
            position_target_o <= 32'sd0;
            target_mdeg <= 32'sd0;
            division_busy <= 1'b0;
            last_request_valid <= 1'b0;
        end
        if (fault_i != 8'd0) begin
            start_o <= 1'b0;
            remote_armed_o <= 1'b0;
            //只在故障进入时撤销旧回执缓存，故障中的复位重传仍保持幂等。
            if (!fault_active_dly && !(command_valid && request_opcode == OP_RESET && response_status == OK))
                last_request_valid <= 1'b0;
        end
    end
end

endmodule
