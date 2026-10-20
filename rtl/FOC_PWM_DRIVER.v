`timescale 1ns/1ps
//中心对齐PWM,三相占空比仅在周期边界共同更新;每次换相先关断后延时导通

module FOC_PWM_DRIVER
#(
    parameter integer PWM_PERIOD = 2500,
    parameter integer DEAD_CLKS = 50,
    parameter integer SAMPLE_OFFSET = 100,
    parameter integer DUTY_FRAC_BITS = 0
)
(
    input wire                          sys_clk,
    input wire                          sys_rst_n,
    input wire                          enable_i,
    input wire                          duty_valid_i,
    input wire [15 + DUTY_FRAC_BITS:0]  half_a_i,
    input wire [15 + DUTY_FRAC_BITS:0]  half_b_i,
    input wire [15 + DUTY_FRAC_BITS:0]  half_c_i,

    output wire                         period_flag_o,
    output wire                         sample_req_o,
    output wire [2:0]                   pwm_h_o,
    output wire [2:0]                   pwm_l_o,
    output wire                         all_low_o
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
localparam integer HALF = PWM_PERIOD / 2;
localparam integer CNT_WIDTH = $clog2(PWM_PERIOD);
localparam integer DEAD_WIDTH = (DEAD_CLKS > 1) ? $clog2(DEAD_CLKS + 1) : 1;
localparam integer DUTY_WIDTH = 16 + DUTY_FRAC_BITS;
localparam integer ERROR_WIDTH = (DUTY_FRAC_BITS > 0) ? DUTY_FRAC_BITS : 1;
localparam [DUTY_WIDTH - 1:0] FRAC_MASK = (64'd1 << DUTY_FRAC_BITS) - 1;
localparam [DUTY_WIDTH - 1:0] NEUTRAL_HALF = (HALF * (64'd1 << DUTY_FRAC_BITS)) / 2;

//wire define
wire [ERROR_WIDTH:0] fraction_a;
wire [ERROR_WIDTH:0] fraction_b;
wire [ERROR_WIDTH:0] fraction_c;
wire [15:0] next_a;
wire [15:0] next_b;
wire [15:0] next_c;
wire [2:0] raw_high;

//reg define
reg [CNT_WIDTH - 1:0] carrier;
reg [DUTY_WIDTH - 1:0] shadow_a, shadow_b, shadow_c;
reg [15:0] active_a, active_b, active_c;
reg [ERROR_WIDTH - 1:0] error_a, error_b, error_c;
genvar phase;


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
assign fraction_a = {1'b0, error_a} + (shadow_a & FRAC_MASK);

assign fraction_b = {1'b0, error_b} + (shadow_b & FRAC_MASK);

assign fraction_c = {1'b0, error_c} + (shadow_c & FRAC_MASK);

assign next_a = (shadow_a >> DUTY_FRAC_BITS) + (fraction_a >> DUTY_FRAC_BITS);

assign next_b = (shadow_b >> DUTY_FRAC_BITS) + (fraction_b >> DUTY_FRAC_BITS);

assign next_c = (shadow_c >> DUTY_FRAC_BITS) + (fraction_c >> DUTY_FRAC_BITS);

assign period_flag_o = (carrier == PWM_PERIOD - 1);

assign sample_req_o = (carrier == SAMPLE_OFFSET);

assign raw_high[0]= (carrier >= HALF - active_a) && (carrier < HALF + active_a);

assign raw_high[1]= (carrier >= HALF - active_b) && (carrier < HALF + active_b);

assign raw_high[2]= (carrier >= HALF - active_c) && (carrier < HALF + active_c);

assign all_low_o = (pwm_h_o == 3'b000) && (pwm_l_o == 3'b111);

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        carrier <= 0;
        shadow_a <= NEUTRAL_HALF;
        shadow_b <= NEUTRAL_HALF;
        shadow_c <= NEUTRAL_HALF;
        active_a <= HALF / 2;
        active_b <= HALF / 2;
        active_c <= HALF / 2;
    end else begin
        if (period_flag_o)
            carrier <= 0;
        else carrier <= carrier + 1'b1;
        if (!enable_i) begin
            shadow_a <= NEUTRAL_HALF;
            shadow_b <= NEUTRAL_HALF;
            shadow_c <= NEUTRAL_HALF;
        end else if (duty_valid_i) begin
            shadow_a <= half_a_i;
            shadow_b <= half_b_i;
            shadow_c <= half_c_i;
        end
        if (period_flag_o) begin
            active_a <= enable_i ? next_a : HALF / 2;
            active_b <= enable_i ? next_b : HALF / 2;
            active_c <= enable_i ? next_c : HALF / 2;
        end
    end
end

//误差只按实际锁存的整周期累积,累计半脉宽误差始终小于1个时钟。
//关断清零,不把上一轮启动的分数误差带到下一轮。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        error_a <= 0;
        error_b <= 0;
        error_c <= 0;
    end else if (!enable_i) begin
        error_a <= 0;
        error_b <= 0;
        error_c <= 0;
    end else if (period_flag_o) begin
        error_a <= fraction_a[ERROR_WIDTH - 1:0];
        error_b <= fraction_b[ERROR_WIDTH - 1:0];
        error_c <= fraction_c[ERROR_WIDTH - 1:0];
    end
end


//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//
generate
    for (phase = 0; phase < 3; phase = phase + 1) begin : g_deadtime
        reg desired;
        reg [DEAD_WIDTH - 1:0] dead_count;
        reg high_reg, low_reg;
        //使能/故障组合屏蔽,即使主时钟停止也不会继续输出旧导通状态
        assign pwm_h_o[phase]= sys_rst_n && enable_i && high_reg;
        assign pwm_l_o[phase]= sys_rst_n && enable_i && low_reg;
        always @(posedge sys_clk or negedge sys_rst_n) begin
            if (!sys_rst_n) begin
                desired <= 0;
                dead_count <= DEAD_CLKS;
                high_reg <= 0;
                low_reg <= 0;
            end else if (!enable_i) begin
                desired <= raw_high[phase];
                dead_count <= DEAD_CLKS;
                high_reg <= 0;
                low_reg <= 0;
            end else if (raw_high[phase]!= desired) begin
                desired <= raw_high[phase];
                dead_count <= DEAD_CLKS - 1;
                high_reg <= 0;
                low_reg <= 0;
            end else if (dead_count != 0) begin
                dead_count <= dead_count - 1'b1;
                high_reg <= 0;
                low_reg <= 0;
            end else begin
                high_reg <= desired;
                low_reg <= !desired;
            end
        end
    end
endgenerate

endmodule
