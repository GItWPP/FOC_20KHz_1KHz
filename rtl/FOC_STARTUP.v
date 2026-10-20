`timescale 1ns/1ps
//按PWM周期运行启动控制;先预对齐,再低速寻Z,最后对齐并建立电角度偏移

module FOC_STARTUP
#(
    parameter integer ALIGN_SAMPLES = 10000,
    parameter integer SEARCH_TIMEOUT_SAMPLES = 200000,
    parameter [31:0] SEARCH_PHASE_STEP = 32'd1717987,
    parameter integer ALIGN_CURRENT_MA = 600,
    parameter integer SEARCH_CURRENT_MA = 800,
    parameter integer REFERENCE_LIMIT_MA = 1000,
    parameter integer RAMP_MA = 5,
    parameter integer SMOOTH_ALIGN = 0,
    parameter integer ALIGN_TOL_COUNTS = 2
)
(
    input wire                sys_clk,
    input wire                sys_rst_n,
    input wire                period_flag_i,
    input wire                enable_i,
    input wire                fault_i,
    input wire                current_calibrated_i,
    input wire                encoder_ready_i,
    input wire [12:0]         encoder_position_i,
    input wire signed [15:0]  id_request_i,
    input wire signed [15:0]  iq_request_i,

    output wire               drive_enable_o,
    output wire               run_o,
    output reg                calibrate_flag_o,
    output reg                timeout_fault_o,
    output reg                align_timeout_fault_o,
    output reg [3:0]          state_o,
    output reg [31:0]         open_phase_o,
    output reg signed [15:0]  id_reference_o,
    output reg signed [15:0]  iq_reference_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam [3:0] ST_OFFSET = 0, ST_IDLE = 1, ST_PREALIGN = 2, ST_SEARCH = 3, ST_ALIGN = 4, ST_CALIBRATE = 5, ST_RUN = 6,
    ST_FAULT = 7;
localparam integer APPROACH_SAMPLES = (ALIGN_SAMPLES > 1) ? ALIGN_SAMPLES / 2 : 1;
localparam integer STABLE_SAMPLES = (ALIGN_SAMPLES >= 5) ? ALIGN_SAMPLES / 5 : 1;
localparam integer ALIGN_TIMEOUT_SAMPLES = 6 * ALIGN_SAMPLES;
localparam [31:0] APPROACH_STEP = 32'd1073741824 / APPROACH_SAMPLES;
localparam [1:0] ALIGN_BEHIND = 0, ALIGN_APPROACH = 1, ALIGN_HOLD = 2;

//wire define
wire signed [12:0] align_delta;
wire align_stable;

//reg define
reg [31:0] sample_count;
reg [1:0] align_stage;
reg [31:0] align_elapsed, stable_count;
reg [12:0] align_anchor;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign align_delta = encoder_position_i - align_anchor;

assign align_stable = ($signed(align_delta) <= ALIGN_TOL_COUNTS) && ($signed(align_delta) >= - ALIGN_TOL_COUNTS);

assign drive_enable_o = (state_o >= ST_PREALIGN) && (state_o <= ST_RUN) && enable_i && !fault_i && !timeout_fault_o &&
    !align_timeout_fault_o;

assign run_o = (state_o == ST_RUN);

function signed [15:0] ramp_reference;
    input signed [15:0] old_value, request;
    reg signed [16:0] target;
    reg signed [16:0] difference;
    begin
        if (request > REFERENCE_LIMIT_MA)
            target = REFERENCE_LIMIT_MA;
        else if (request < - REFERENCE_LIMIT_MA)
            target = - REFERENCE_LIMIT_MA;
        else target = request;
        difference = target - $signed(old_value);
        if (difference > RAMP_MA)
            ramp_reference = old_value + RAMP_MA;
        else if (difference < - RAMP_MA)
            ramp_reference = old_value - RAMP_MA;
        else ramp_reference = target[15:0];
    end
endfunction

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state_o <= ST_OFFSET;
        sample_count <= 0;
        open_phase_o <= 0;
        id_reference_o <= 0;
        iq_reference_o <= 0;
        calibrate_flag_o <= 0;
        timeout_fault_o <= 0;
        align_timeout_fault_o <= 0;
        align_stage <= ALIGN_BEHIND;
        align_elapsed <= 0;
        stable_count <= 0;
        align_anchor <= 0;
    end else begin
        calibrate_flag_o <= 0;
        if (fault_i || timeout_fault_o || align_timeout_fault_o) begin
            state_o <= ST_FAULT;
            id_reference_o <= 0;
            iq_reference_o <= 0;
        end else if (!current_calibrated_i) begin
            state_o <= ST_OFFSET;
            sample_count <= 0;
        end else if (!enable_i) begin
            state_o <= ST_IDLE;
            sample_count <= 0;
            open_phase_o <= 0;
            id_reference_o <= 0;
            iq_reference_o <= 0;
            align_stage <= ALIGN_BEHIND;
            align_elapsed <= 0;
            stable_count <= 0;
            align_anchor <= encoder_position_i;
        end else if (state_o == ST_CALIBRATE) begin
            //该状态的前一拍已发出校准脉冲,此拍FOC_ENCODER_ANGLE消费脉冲
            state_o <= ST_RUN;
            id_reference_o <= 0;
            iq_reference_o <= 0;
        end else if (period_flag_i) begin
            case (state_o)
                ST_OFFSET, ST_IDLE: begin
                    state_o <= ST_PREALIGN;
                    sample_count <= 0;
                    open_phase_o <= 0;
                    id_reference_o <= 0;
                    iq_reference_o <= 0;
                end
                ST_PREALIGN: begin
                    id_reference_o <= ramp_reference(id_reference_o, ALIGN_CURRENT_MA);
                    iq_reference_o <= 0;
                    if (sample_count == ALIGN_SAMPLES - 1) begin
                        state_o <= ST_SEARCH;
                        sample_count <= 0;
                    end else sample_count <= sample_count + 1'b1;
                end
                ST_SEARCH: begin
                    id_reference_o <= ramp_reference(id_reference_o, SEARCH_CURRENT_MA);
                    open_phase_o <= open_phase_o + SEARCH_PHASE_STEP;
                    if (encoder_ready_i) begin
                        state_o <= ST_ALIGN;
                        sample_count <= 0;
                        open_phase_o <= SMOOTH_ALIGN ? 32'hc0000000 : 32'd0;
                        align_stage <= ALIGN_BEHIND;
                        align_elapsed <= 0;
                        stable_count <= 0;
                        align_anchor <= encoder_position_i;
                    end else if (sample_count == SEARCH_TIMEOUT_SAMPLES - 1)
                        timeout_fault_o <= 1;
                    else sample_count <= sample_count + 1'b1;
                end
                ST_ALIGN: begin
                    id_reference_o <= ramp_reference(id_reference_o, ALIGN_CURRENT_MA);
                    iq_reference_o <= 0;
                    if (SMOOTH_ALIGN == 0) begin
                        open_phase_o <= 0;
                        if (sample_count == ALIGN_SAMPLES - 1) begin
                            state_o <= ST_CALIBRATE;
                            calibrate_flag_o <= 1;
                        end else sample_count <= sample_count + 1'b1;
                    end else if (align_elapsed >= ALIGN_TIMEOUT_SAMPLES - 1) begin
                        align_timeout_fault_o <= 1;
                    end else begin
                        align_elapsed <= align_elapsed + 1'b1;
                        case (align_stage)
                            ALIGN_BEHIND: begin
                                //先在目标后方90度停稳,固定最后逼近方向。
                                open_phase_o <= 32'hc0000000;
                                if (sample_count >= APPROACH_SAMPLES - 1) begin
                                    align_stage <= ALIGN_APPROACH;
                                    sample_count <= 0;
                                end else sample_count <= sample_count + 1'b1;
                            end
                            ALIGN_APPROACH: begin
                                if (sample_count >= APPROACH_SAMPLES - 1) begin
                                    open_phase_o <= 0;
                                    align_stage <= ALIGN_HOLD;
                                    sample_count <= 0;
                                    stable_count <= 0;
                                    align_anchor <= encoder_position_i;
                                end else begin
                                    open_phase_o <= open_phase_o + APPROACH_STEP;
                                    sample_count <= sample_count + 1'b1;
                                end
                            end
                            ALIGN_HOLD: begin
                                open_phase_o <= 0;
                                if (sample_count < ALIGN_SAMPLES)
                                    sample_count <= sample_count + 1'b1;
                                if (!align_stable) begin
                                    stable_count <= 0;
                                    align_anchor <= encoder_position_i;
                                end else if (stable_count < STABLE_SAMPLES)
                                    stable_count <= stable_count + 1'b1;
                                if (sample_count >= ALIGN_SAMPLES - 1 && stable_count >= STABLE_SAMPLES - 1 &&
                                    align_stable && encoder_ready_i) begin
                                    state_o <= ST_CALIBRATE;
                                    calibrate_flag_o <= 1;
                                end
                            end
                            default:
                                align_timeout_fault_o <= 1;
                        endcase
                    end
                end
                ST_RUN: begin
                    id_reference_o <= ramp_reference(id_reference_o, id_request_i);
                    iq_reference_o <= ramp_reference(iq_reference_o, iq_request_i);
                end
                default:
                    state_o <= ST_FAULT;
            endcase
        end
    end
end

endmodule
