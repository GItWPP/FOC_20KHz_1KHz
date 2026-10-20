// HKT56 single-ended A/B/Z incremental encoder interface.
//
// The external 5V encoder signals must be converted to 3.3V before entering
// the FPGA. A/B/Z are synchronized and filtered independently. A/B use x4
// decoding. A 2048-PPR encoder therefore
// produces 8192=2^13 counts/revolution, allowing direct mapping from the
// 13-bit position to a 32-bit mechanical phase without division.
//
// Z establishes the mechanical origin before encoder_ready_o is asserted.
// Use the forward rising / reverse falling edge of Z to reference the SAME
// physical boundary in both directions; Z need not coincide with an A/B edge.
// Illegal A/B transitions or inconsistent index intervals latch a fault.
// Fault recovery requires reset, a new index search and FOC calibration.
// ENABLE_Z_INDEX=0 preserves the legacy reset-relative A/B-only behavior.
`timescale  1ns/1ps

module HKT56_ENCODER_IF
#(
    parameter integer REVERSE_DIRECTION = 0,
    parameter integer AB_FILTER_CLKS = 16,
    parameter [31:0] REV_TIMEOUT_CLKS = 32'hffff_ffff,
    parameter integer ENABLE_Z_INDEX = 1,
    parameter integer Z_FILTER_CLKS = AB_FILTER_CLKS,
    parameter integer INDEX_COUNT_TOLERANCE = 1
)
(
    //系统时钟
    input wire          sys_clk,
    //系统复位,低电平有效
    input wire          sys_rst_n,
    //编码器A相,异步输入
    input wire          enc_a_async,
    //编码器B相,异步输入
    input wire          enc_b_async,
    //编码器Z相,高电平有效
    input wire          enc_z_async,

    //模8192机械位置,Z为零位
    output reg [12:0]   position_o,
    //32位机械相位
    output wire [31:0]  phase_o,
    //整圈测量有效
    output reg          encoder_valid_o,
    //AB/Z异常锁存
    output reg          encoder_fault_o,
    //Z定位完成且无故障
    output wire         encoder_ready_o,
    //本次复位后已找到Z零位
    output reg          encoder_indexed_o,
    //接受Z定位,单时钟脉冲
    output reg          index_flag_o,
    //有效AB步进,不受Z位置清零影响
    output wire         step_flag_o,
    //经REVERSE_DIRECTION修正后的方向
    output wire         step_positive_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam [12:0] POSITION_MAX = 13'd8_191;
localparam [13:0] QUAD_COUNTS_PER_REV = 14'd8_192;
localparam [13:0] EDGE_COUNT_MAX = 14'h3fff;
//REV_TIMEOUT_CLKS允许范围:1..32'hffff_ffff;超时只清除整圈有效状态
localparam integer FILTER_CNT_WIDTH = (AB_FILTER_CLKS > 1) ? $clog2(AB_FILTER_CLKS) : 1;
localparam [FILTER_CNT_WIDTH - 1:0] FILTER_COUNT_MAX = AB_FILTER_CLKS - 1;
localparam integer Z_FILTER_CNT_WIDTH = (Z_FILTER_CLKS > 1) ? $clog2(Z_FILTER_CLKS) : 1;
localparam [Z_FILTER_CNT_WIDTH - 1:0] Z_FILTER_COUNT_MAX = Z_FILTER_CLKS - 1;
//Z与AB独立同步可能有一拍采样差;默认允许8192 +/- 1个计数
//INDEX_COUNT_TOLERANCE允许范围:0..8191,实际推荐0或1
localparam [14:0] INDEX_COUNT_MIN = 15'd8_192 - INDEX_COUNT_TOLERANCE;
localparam [14:0] INDEX_COUNT_MAX = 15'd8_192 + INDEX_COUNT_TOLERANCE;
localparam [12:0] INDEX_POSITION_LOW = INDEX_COUNT_TOLERANCE;
localparam [13:0] INDEX_POSITION_HIGH = 14'd8_192 - INDEX_COUNT_TOLERANCE;
//同步、初始电平确认和译码基线建立所需的启动等待时间
localparam integer STARTUP_FILTER_CLKS = (ENABLE_Z_INDEX && (Z_FILTER_CLKS > AB_FILTER_CLKS)) ? Z_FILTER_CLKS :
    AB_FILTER_CLKS;
localparam integer STARTUP_CLKS = (STARTUP_FILTER_CLKS > 8) ? STARTUP_FILTER_CLKS + 8 : 16;
localparam integer STARTUP_CNT_WIDTH = $clog2(STARTUP_CLKS);
localparam [STARTUP_CNT_WIDTH - 1:0] STARTUP_COUNT_MAX = STARTUP_CLKS - 1;

//wire define
wire [1:0] ab_current;
wire step_valid;
wire step_positive;
wire position_wrap;
wire revolution_timeout;
wire z_current;
wire index_event;
wire index_direction;
wire [12:0] position_after_step;
wire [14:0] index_edge_count;
wire index_count_ok;
wire index_position_ok;

//reg define
(* ASYNC_REG = "TRUE" *) reg [2:0] enc_a_sync;
(* ASYNC_REG = "TRUE" *) reg [2:0] enc_b_sync;
(* ASYNC_REG = "TRUE" *) reg [2:0] enc_z_sync;
reg [STARTUP_CNT_WIDTH - 1:0] startup_count;
reg decode_ready;
reg [1:0] ab_previous;
reg z_previous;
reg have_direction;
reg raw_forward_reg;
reg step_forward_decoded;
reg step_reverse_decoded;
reg transition_invalid_decoded;
//最近步进方向,用于换向时清除整圈有效状态
reg direction_reg;
reg have_revolution_boundary;
reg boundary_direction;
reg [13:0] edge_count_since_boundary;
reg [31:0] revolution_timer;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign step_valid = step_forward_decoded || step_reverse_decoded;

assign step_positive = REVERSE_DIRECTION ? step_reverse_decoded : step_forward_decoded;

assign step_flag_o = decode_ready && !encoder_fault_o && step_valid && !transition_invalid_decoded;

assign step_positive_o = step_positive;

assign position_wrap = step_valid && ((step_positive && (position_o == POSITION_MAX)) || (!step_positive && (position_o
    == 13'd0)));

//8192计数/圈,每个计数对应2^19个32位相位单位
assign phase_o = {position_o, 19'd0};

assign encoder_ready_o = decode_ready && !encoder_fault_o && (!ENABLE_Z_INDEX || encoder_indexed_o);

//方向反转参数只改变计数符号,不改变所选Z机械边界
assign index_event = decode_ready && (have_direction || step_valid) && ((step_valid ? step_forward_decoded :
    raw_forward_reg) ? (z_current && !z_previous) :(!z_current && z_previous));

assign index_direction = step_valid ? step_positive : direction_reg;

assign position_after_step = !step_valid ? position_o :(step_positive ? position_o + 13'd1 : position_o - 13'd1);

//与Z同拍的AB边沿计入上一圈,Z定位的清零优先于普通加减计数
assign index_edge_count = {1'b0, edge_count_since_boundary} + (step_valid ? 15'd1 : 15'd0);

assign index_count_ok = (index_edge_count >= INDEX_COUNT_MIN) && (index_edge_count <= INDEX_COUNT_MAX);

assign index_position_ok = (position_after_step <= INDEX_POSITION_LOW) || ((INDEX_COUNT_TOLERANCE != 0) && ({1'b0,
    position_after_step}>= INDEX_POSITION_HIGH));

//达到超时阈值的这一拍也不能发布有效周期,避免32位加一回绕
assign revolution_timeout = have_revolution_boundary && (revolution_timer >= REV_TIMEOUT_CLKS - 1'b1);

//enc_a_sync,enc_b_sync,enc_z_sync:异步A/B/Z输入三级同步
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        enc_a_sync <= 3'b000;
        enc_b_sync <= 3'b000;
        enc_z_sync <= 3'b000;
    end else begin
        enc_a_sync <= {enc_a_sync[1:0], enc_a_async};
        enc_b_sync <= {enc_b_sync[1:0], enc_b_async};
        enc_z_sync <= {enc_z_sync[1:0], enc_z_async};
    end
end

//四倍频Gray状态译码,正向序列:00->01->11->10->00
always @(*) begin
    step_forward_decoded = 1'b0;
    step_reverse_decoded = 1'b0;
    transition_invalid_decoded = 1'b0;
    case ({ab_previous, ab_current})
        4'b0000, 4'b0101, 4'b1010, 4'b1111: begin
            //无位移
        end
        4'b0001, 4'b0111, 4'b1110, 4'b1000:
            step_forward_decoded = 1'b1;
        4'b0010, 4'b1011, 4'b1101, 4'b0100:
            step_reverse_decoded = 1'b1;
        default:
            transition_invalid_decoded = 1'b1;
    endcase
end

//位置、故障状态和整圈有效判断
//Z模式:同方向相邻两次Z间约8192计数才确认整圈有效
//AB兼容模式:连续两次同方向计数回绕确认整圈有效
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (sys_rst_n == 1'b0) begin
        startup_count <= {STARTUP_CNT_WIDTH{1'b0}};
        decode_ready <= 1'b0;
        ab_previous <= 2'b00;
        z_previous <= 1'b0;
        have_direction <= 1'b0;
        raw_forward_reg <= 1'b1;
        encoder_indexed_o <= 1'b0;
        index_flag_o <= 1'b0;
        position_o <= 13'd0;
        encoder_valid_o <= 1'b0;
        encoder_fault_o <= 1'b0;
        direction_reg <= 1'b1;
        have_revolution_boundary <= 1'b0;
        boundary_direction <= 1'b1;
        edge_count_since_boundary <= 14'd0;
        revolution_timer <= 32'd0;
    end else begin
        index_flag_o <= 1'b0;
        if (decode_ready == 1'b0) begin
            position_o <= 13'd0;
            encoder_valid_o <= 1'b0;
            encoder_fault_o <= 1'b0;
            have_revolution_boundary <= 1'b0;
            edge_count_since_boundary <= 14'd0;
            revolution_timer <= 32'd0;
            ab_previous <= ab_current;
            z_previous <= z_current;
            have_direction <= 1'b0;
            encoder_indexed_o <= 1'b0;
            if (startup_count == STARTUP_COUNT_MAX)
                decode_ready <= 1'b1;
            else startup_count <= startup_count + 1'b1;
        end else begin
            //故障后继续采样A/B/Z,但冻结相位数据直到系统复位
            ab_previous <= ab_current;
            z_previous <= z_current;
            if (encoder_fault_o == 1'b1) begin
                encoder_valid_o <= 1'b0;
                have_revolution_boundary <= 1'b0;
                edge_count_since_boundary <= 14'd0;
                revolution_timer <= 32'd0;
            end else begin
                if (have_revolution_boundary && (revolution_timer < REV_TIMEOUT_CLKS))
                    revolution_timer <= revolution_timer + 1'b1;
                if (revolution_timeout == 1'b1)
                    encoder_valid_o <= 1'b0;
                if (transition_invalid_decoded == 1'b1) begin
                    encoder_valid_o <= 1'b0;
                    encoder_fault_o <= 1'b1;
                    have_revolution_boundary <= 1'b0;
                    edge_count_since_boundary <= 14'd0;
                    revolution_timer <= 32'd0;
                end else if (ENABLE_Z_INDEX != 0) begin
                    if (step_valid == 1'b1) begin
                        direction_reg <= step_positive;
                        raw_forward_reg <= step_forward_decoded;
                        have_direction <= 1'b1;
                    end
                    if (index_event == 1'b1) begin
                        if (encoder_indexed_o && (!index_position_ok || (have_revolution_boundary && (index_direction ==
                            boundary_direction) && !index_count_ok))) begin
                            //异常Z位置/圈计数不强行校零,防止带错角度进入FOC
                            encoder_valid_o <= 1'b0;
                            encoder_fault_o <= 1'b1;
                            have_revolution_boundary <= 1'b0;
                            edge_count_since_boundary <= 14'd0;
                            revolution_timer <= 32'd0;
                        end else begin
                            position_o <= 13'd0;
                            encoder_indexed_o <= 1'b1;
                            index_flag_o <= 1'b1;
                            encoder_valid_o <= encoder_indexed_o && have_revolution_boundary && (index_direction ==
                                boundary_direction) && index_count_ok && !revolution_timeout;
                            have_revolution_boundary <= 1'b1;
                            boundary_direction <= index_direction;
                            edge_count_since_boundary <= 14'd0;
                            revolution_timer <= 32'd0;
                        end
                    end else if (step_valid == 1'b1) begin
                        position_o <= position_after_step;
                        if (step_positive != direction_reg)
                            encoder_valid_o <= 1'b0;
                        if (have_revolution_boundary == 1'b1) begin
                            if (step_positive != boundary_direction) begin
                                encoder_valid_o <= 1'b0;
                                have_revolution_boundary <= 1'b0;
                                edge_count_since_boundary <= 14'd0;
                                revolution_timer <= 32'd0;
                            end else if (index_edge_count > INDEX_COUNT_MAX) begin
                                //多于一圈仍没有Z:缺失Z或计数错误,锁存故障
                                position_o <= position_o;
                                encoder_valid_o <= 1'b0;
                                encoder_fault_o <= 1'b1;
                                have_revolution_boundary <= 1'b0;
                                edge_count_since_boundary <= 14'd0;
                                revolution_timer <= 32'd0;
                            end else begin
                                edge_count_since_boundary <= index_edge_count[13:0];
                            end
                        end
                    end
                end else if (step_valid == 1'b1) begin
                    direction_reg <= step_positive;
                    //换向后清除旧方向的整圈有效状态
                    if (step_positive != direction_reg)
                        encoder_valid_o <= 1'b0;
                    if (step_positive == 1'b1)
                        position_o <= position_o + 1'b1;
                    else position_o <= position_o - 1'b1;
                    if (position_wrap == 1'b1) begin
                        if ((have_revolution_boundary == 1'b0) || (step_positive != boundary_direction)) begin
                            have_revolution_boundary <= 1'b1;
                            boundary_direction <= step_positive;
                            edge_count_since_boundary <= 14'd0;
                            revolution_timer <= 32'd0;
                            encoder_valid_o <= 1'b0;
                        end else if (edge_count_since_boundary == QUAD_COUNTS_PER_REV - 1'b1) begin
                            if (revolution_timeout == 1'b0) begin
                                encoder_valid_o <= 1'b1;
                            end else begin
                                //超时只清除整圈有效状态;位置继续累计
                                encoder_valid_o <= 1'b0;
                            end
                            edge_count_since_boundary <= 14'd0;
                            revolution_timer <= 32'd0;
                        end else begin
                            //缺少Z相时无法恢复丢失的机械相位
                            encoder_valid_o <= 1'b0;
                            encoder_fault_o <= 1'b1;
                            have_revolution_boundary <= 1'b0;
                            edge_count_since_boundary <= 14'd0;
                            revolution_timer <= 32'd0;
                        end
                    end else if (have_revolution_boundary == 1'b1) begin
                        if (step_positive != boundary_direction) begin
                            encoder_valid_o <= 1'b0;
                            have_revolution_boundary <= 1'b0;
                            edge_count_since_boundary <= 14'd0;
                            revolution_timer <= 32'd0;
                        end else if (edge_count_since_boundary != EDGE_COUNT_MAX) begin
                            edge_count_since_boundary <= edge_count_since_boundary + 1'b1;
                        end
                    end
                end
            end
        end
    end
end


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
//A/B两相独立去毛刺,短脉冲不会在两个通道间互相累积
generate
    if (AB_FILTER_CLKS > 1) begin : g_ab_filter
        reg enc_a_filtered;
        reg enc_b_filtered;
        reg [FILTER_CNT_WIDTH - 1:0] a_filter_count;
        reg [FILTER_CNT_WIDTH - 1:0] b_filter_count;
        assign ab_current = {enc_a_filtered, enc_b_filtered};
        always @(posedge sys_clk or negedge sys_rst_n) begin
            if (sys_rst_n == 1'b0) begin
                enc_a_filtered <= 1'b0;
                enc_b_filtered <= 1'b0;
                a_filter_count <= {FILTER_CNT_WIDTH{1'b0}};
                b_filter_count <= {FILTER_CNT_WIDTH{1'b0}};
            end else begin
                if (enc_a_sync[2]== enc_a_filtered)
                    a_filter_count <= {FILTER_CNT_WIDTH{1'b0}};
                else if (a_filter_count == FILTER_COUNT_MAX) begin
                    enc_a_filtered <= enc_a_sync[2];
                    a_filter_count <= {FILTER_CNT_WIDTH{1'b0}};
                end else begin
                    a_filter_count <= a_filter_count + 1'b1;
                end
                if (enc_b_sync[2]== enc_b_filtered)
                    b_filter_count <= {FILTER_CNT_WIDTH{1'b0}};
                else if (b_filter_count == FILTER_COUNT_MAX) begin
                    enc_b_filtered <= enc_b_sync[2];
                    b_filter_count <= {FILTER_CNT_WIDTH{1'b0}};
                end else begin
                    b_filter_count <= b_filter_count + 1'b1;
                end
            end
        end
    end else begin : g_ab_filter_bypass
        assign ab_current = {enc_a_sync[2], enc_b_sync[2]};
    end
endgenerate

//Z独立去毛刺;上电时的高电平只建立基线,不会伪造零位事件
generate
    if (ENABLE_Z_INDEX && (Z_FILTER_CLKS > 1)) begin : g_z_filter
        reg enc_z_filtered;
        reg [Z_FILTER_CNT_WIDTH - 1:0] z_filter_count;
        assign z_current = enc_z_filtered;
        always @(posedge sys_clk or negedge sys_rst_n) begin
            if (sys_rst_n == 1'b0) begin
                enc_z_filtered <= 1'b0;
                z_filter_count <= {Z_FILTER_CNT_WIDTH{1'b0}};
            end else if (enc_z_sync[2]== enc_z_filtered) begin
                z_filter_count <= {Z_FILTER_CNT_WIDTH{1'b0}};
            end else if (z_filter_count == Z_FILTER_COUNT_MAX) begin
                enc_z_filtered <= enc_z_sync[2];
                z_filter_count <= {Z_FILTER_CNT_WIDTH{1'b0}};
            end else begin
                z_filter_count <= z_filter_count + 1'b1;
            end
        end
    end else begin : g_z_filter_bypass
        assign z_current = ENABLE_Z_INDEX ? enc_z_sync[2] : 1'b0;
    end
endgenerate

endmodule
