`timescale 1ns/1ps
//M/T混合测速:8边沿周期预测相邻计数内的位置,每1ms对预测位置作差并低通。
//预测最多领先1计数,停顿时自然保持位置而非旧速度;速度积分与实际步数有界一致。

module FOC_SPEED_ESTIMATOR
#(
    parameter integer CLK_FREQ = 50000000,
    parameter integer CONTROL_HZ = 1000,
    parameter integer COUNTS_PER_REV = 8192,
    parameter integer MEASURE_EDGES = 8,
    parameter integer ZERO_TIMEOUT_CLKS = CLK_FREQ / 10,
    parameter integer FILTER_SHIFT = 2
)
(
    input wire                 sys_clk,
    input wire                 sys_rst_n,
    input wire                 step_flag_i,
    input wire                 step_positive_i,

    output wire                tick_o,
    output reg signed [31:0]   speed_q8_o,
    output wire signed [15:0]  speed_rpm_o,
    output reg                 moving_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer TICK_CLKS = CLK_FREQ / CONTROL_HZ;
localparam [31:0] RPM_PER_COUNT_Q24 = (64'd60 * CONTROL_HZ * 256 * 65536) / COUNTS_PER_REV;
localparam [2:0] ST_IDLE = 0, ST_FRACTION = 1, ST_POSITION = 2, ST_PRODUCT = 3, ST_FILTER = 4;
localparam integer TICK_WIDTH = (TICK_CLKS > 1) ? $clog2(TICK_CLKS) : 1;
localparam integer QUIET_WIDTH = (ZERO_TIMEOUT_CLKS > 0) ? $clog2(ZERO_TIMEOUT_CLKS + 1) : 1;

//wire define
wire [63:0] scaled_age;
wire [32:0] shifted_remainder;
wire divide_bit;
wire [31:0] sample_phase;
wire signed [64:0] rounded_product;
wire signed [32:0] rate_wide;
wire signed [31:0] rate_q8;
wire signed [40:0] filter_target;
wire signed [40:0] filter_error;
wire signed [40:0] filter_next;
wire signed [40:0] rounded_filter;
wire signed [31:0] rounded_speed;

//reg define
reg [TICK_WIDTH - 1:0] tick_count;
reg [QUIET_WIDTH - 1:0] quiet_count;
reg [31:0] edge_timer, measured_period;
reg [7:0] edge_count;
reg have_edge, direction_reg, period_valid;
reg [15:0] position_count, sample_position;
reg sample_positive;
reg [2:0] state;
reg [31:0] divisor;
reg [32:0] remainder;
reg [3:0] divide_count;
reg [16:0] fraction;
reg [31:0] previous_phase;
reg signed [31:0] position_difference;
reg signed [64:0] speed_product;
reg signed [39:0] filtered_q16;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign scaled_age = {32'd0, quiet_count}* MEASURE_EDGES;

assign shifted_remainder = {remainder[31:0], 1'b0};

assign divide_bit = (shifted_remainder >= {1'b0, divisor});

assign sample_phase = sample_positive ?{sample_position, 16'd0} + {15'd0, fraction}:{sample_position, 16'd0} - {15'd0,
    fraction};

assign rounded_product = speed_product + (speed_product[64] ? 65'sd2147483647 : 65'sd2147483648);

assign rate_wide = rounded_product >>> 32;

assign rate_q8 = (rate_wide > 1536000) ? 32'sd1536000 :((rate_wide < - 1536000) ? - 32'sd1536000 : rate_wide[31:0]);

assign filter_target = $signed({rate_q8, 8'd0});

assign filter_error = filter_target - $signed(filtered_q16);

assign filter_next = (FILTER_SHIFT == 0) ? filter_target : $signed(filtered_q16) + (filter_error >>> FILTER_SHIFT);

assign rounded_filter = filter_next + (filter_next[40] ? 41'sd127 : 41'sd128);

assign rounded_speed = (speed_q8_o < 0) ? - (((- speed_q8_o) + 128) >>> 8) :((speed_q8_o + 128) >>> 8);

assign tick_o = (tick_count == TICK_CLKS - 1);

assign speed_rpm_o = rounded_speed[15:0];

always @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n)
        tick_count <= 0;
    else if (tick_o)
        tick_count <= 0;
    else tick_count <= tick_count + 1'b1;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        edge_timer <= 0;
        quiet_count <= 0;
        edge_count <= 0;
        have_edge <= 0;
        direction_reg <= 1;
        moving_o <= 0;
        period_valid <= 0;
        measured_period <= 1;
        position_count <= 0;
    end else begin
        if (edge_timer != 32'hffffffff)
            edge_timer <= edge_timer + 1'b1;
        if (quiet_count < ZERO_TIMEOUT_CLKS)
            quiet_count <= quiet_count + 1'b1;
        if (quiet_count >= ZERO_TIMEOUT_CLKS - 1 && !step_flag_i) begin
            //只重启周期测量锚点,保留上次周期使预测位置不会在超时点倒退1计数。
            have_edge <= 0;
            edge_count <= 0;
            moving_o <= 0;
        end else begin
            if (step_flag_i) begin
                position_count <= step_positive_i ? position_count + 1'b1 : position_count - 1'b1;
                quiet_count <= 0;
                moving_o <= 1;
                direction_reg <= step_positive_i;
                if (!have_edge || direction_reg != step_positive_i) begin
                    have_edge <= 1;
                    edge_timer <= 0;
                    edge_count <= 0;
                    if (direction_reg != step_positive_i)
                        period_valid <= 0;
                end else if (edge_count == MEASURE_EDGES - 1) begin
                    edge_timer <= 0;
                    edge_count <= 0;
                    measured_period <= edge_timer + 1'b1;
                    period_valid <= 1;
                end else edge_count <= edge_count + 1'b1;
            end
        end
    end
end

//分数位串行除法只需16拍;周期快照在新边沿到来时也保持同一采样时刻。
//内部位置模65536回绕,相邻1ms的有符号差远小于32768计数。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state <= ST_IDLE;
        sample_position <= 0;
        sample_positive <= 1;
        divisor <= 1;
        remainder <= 0;
        divide_count <= 0;
        fraction <= 0;
        previous_phase <= 0;
        position_difference <= 0;
        speed_product <= 0;
        filtered_q16 <= 0;
        speed_q8_o <= 0;
    end else begin
        case (state)
            ST_IDLE:
                if (tick_o) begin
                    sample_position <= position_count;
                    sample_positive <= direction_reg;
                    fraction <= 0;
                    divide_count <= 0;
                    divisor <= measured_period;
                    if (!period_valid)
                        state <= ST_POSITION;
                    else if (scaled_age >= measured_period) begin
                        fraction <= 17'd65536;
                        state <= ST_POSITION;
                    end else begin
                        remainder <= {1'b0, scaled_age[31:0]};
                        state <= ST_FRACTION;
                    end
                end
            ST_FRACTION: begin
                remainder <= divide_bit ? shifted_remainder - {1'b0, divisor}: shifted_remainder;
                fraction <= {fraction[15:0], divide_bit};
                if (divide_count == 15)
                    state <= ST_POSITION;
                else divide_count <= divide_count + 1'b1;
            end
            ST_POSITION: begin
                position_difference <= $signed(sample_phase - previous_phase);
                previous_phase <= sample_phase;
                state <= ST_PRODUCT;
            end
            ST_PRODUCT: begin
                speed_product <= position_difference * $signed({1'b0, RPM_PER_COUNT_Q24});
                state <= ST_FILTER;
            end
            ST_FILTER: begin
                filtered_q16 <= filter_next[39:0];
                speed_q8_o <= rounded_filter >>> 8;
                state <= ST_IDLE;
            end
            default:
                state <= ST_IDLE;
        endcase
    end
end

endmodule
