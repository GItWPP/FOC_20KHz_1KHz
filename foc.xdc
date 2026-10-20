# AX7Z020 用户提供J20表 + 已确认沿用的旧PZT ADC/AB/50MHz接线
# xc7z020clg400-2 / Vivado 2024.2 / Bank默认3.3V
create_clock -name sys_clk -period 20.000 [get_ports sys_clk]
set_property PACKAGE_PIN U18 [get_ports {sys_clk}]
set_property PACKAGE_PIN M15 [get_ports {sys_rst_n}]
set_property PACKAGE_PIN B20 [get_ports {adc_busy_i}]
set_property PACKAGE_PIN K19 [get_ports {adc_convst_o}]
set_property PACKAGE_PIN C20 [get_ports {adc_cs_n_o}]
set_property PACKAGE_PIN B19 [get_ports {adc_rd_n_o}]
set_property PACKAGE_PIN A20 [get_ports {adc_reset_o}]
set_property PACKAGE_PIN T10 [get_ports uart_tx_o]
set_property DRIVE 4 [get_ports uart_tx_o]
set_property SLEW SLOW [get_ports uart_tx_o]
set_property PACKAGE_PIN V16 [get_ports {hard_trip_i}]
set_property PACKAGE_PIN P18 [get_ports {motor_sd_a_o}]
set_property PACKAGE_PIN M17 [get_ports {adc_os_o[0]}]
set_property PACKAGE_PIN M18 [get_ports {adc_os_o[1]}]
set_property PACKAGE_PIN J19 [get_ports {adc_os_o[2]}]
set_property PACKAGE_PIN D20 [get_ports {adc_data_i[0]}]
set_property PACKAGE_PIN D19 [get_ports {adc_data_i[1]}]
set_property PACKAGE_PIN L20 [get_ports {adc_data_i[2]}]
set_property PACKAGE_PIN L19 [get_ports {adc_data_i[3]}]
set_property PACKAGE_PIN F16 [get_ports {adc_data_i[4]}]
set_property PACKAGE_PIN F17 [get_ports {adc_data_i[5]}]
set_property PACKAGE_PIN H20 [get_ports {adc_data_i[6]}]
set_property PACKAGE_PIN J20 [get_ports {adc_data_i[7]}]
set_property PACKAGE_PIN G18 [get_ports {adc_data_i[8]}]
set_property PACKAGE_PIN G17 [get_ports {adc_data_i[9]}]
set_property PACKAGE_PIN H17 [get_ports {adc_data_i[10]}]
set_property PACKAGE_PIN H16 [get_ports {adc_data_i[11]}]
set_property PACKAGE_PIN G15 [get_ports {adc_data_i[12]}]
set_property PACKAGE_PIN H15 [get_ports {adc_data_i[13]}]
set_property PACKAGE_PIN K18 [get_ports {adc_data_i[14]}]
set_property PACKAGE_PIN K17 [get_ports {adc_data_i[15]}]
set_property PACKAGE_PIN P19 [get_ports {pwm_h_o[0]}]
set_property PACKAGE_PIN R17 [get_ports {pwm_h_o[1]}]
set_property PACKAGE_PIN P15 [get_ports {pwm_h_o[2]}]
set_property PACKAGE_PIN N18 [get_ports {pwm_l_o[0]}]
set_property PACKAGE_PIN R16 [get_ports {pwm_l_o[1]}]
set_property PACKAGE_PIN P16 [get_ports {pwm_l_o[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sys_clk sys_rst_n adc_* uart_tx_o hard_trip_i pwm_* motor_sd_a_o}]
set_property SLEW SLOW [get_ports {pwm_h_o[*] pwm_l_o[*] motor_sd_a_o}]
set_property DRIVE 8 [get_ports {pwm_h_o[*] pwm_l_o[*] motor_sd_a_o}]
# 外部故障接口开路即关断 健康状态必须由外部电路拉低
set_property PULLUP true [get_ports hard_trip_i]
# 本工程不实例化PS7 顶层仅包含PL连接器信号

# AD7606并行读：RD低80ns后采集 数据最坏到达预算54ns
# 预算=RD主时钟边沿到引脚20ns+去程2ns+t14最大30ns+回程2ns
# 最小到达6ns依据t15；实物需核对VDRIVE>=2.7V及线缆时延
set_input_delay -clock sys_clk -max 54.000 [get_ports {adc_data_i[*]}]
set_input_delay -clock sys_clk -min 6.000 [get_ports {adc_data_i[*]}]
set_multicycle_path -setup 4 -from [get_ports {adc_data_i[*]}]
set_multicycle_path -hold 3 -from [get_ports {adc_data_i[*]}]
# ADC/PWM接收端无50MHz同步时钟：输出按一个系统周期20ns的引脚到达预算
# ADC读回预算含这20ns；不能重复扣除虚构的接收端setup时间
set_output_delay -clock sys_clk -max 0.000 [get_ports {adc_convst_o adc_reset_o adc_cs_n_o adc_rd_n_o adc_os_o[*] uart_tx_o pwm_h_o[*] pwm_l_o[*] motor_sd_a_o}]
set_output_delay -clock sys_clk -min 0.000 [get_ports {adc_convst_o adc_reset_o adc_cs_n_o adc_rd_n_o adc_os_o[*] uart_tx_o pwm_h_o[*] pwm_l_o[*] motor_sd_a_o}]
# 异步输入仅到第一级同步触发器作例外 保留后级50MHz时序
set_input_delay -clock sys_clk -max 0.000 [get_ports {adc_busy_i}]
set_input_delay -clock sys_clk -min 0.000 [get_ports {adc_busy_i}]
# 这些端口在RTL中仅连接各自第一级同步器 D，不存在其它功能路径
set_false_path -from [get_ports {adc_busy_i}]
set_false_path -from [get_ports sys_rst_n]
set_input_delay -clock sys_clk -max 0.000 [get_ports hard_trip_i]
set_input_delay -clock sys_clk -min 0.000 [get_ports hard_trip_i]
set_false_path -from [get_ports hard_trip_i] -to [get_pins -hier -filter {NAME =~ *trip_latched_reg/PRE}]
# 外部硬件故障到七路驱动输出的独立引脚间数据路径预算10ns
set_max_delay -datapath_only 10.000 -from [get_ports hard_trip_i] -to [get_ports {pwm_h_o[*] pwm_l_o[*] motor_sd_a_o}]
# 异步组合关断路径补齐最小传播延迟；不建立外部接收时钟关系。
set_min_delay 0.000 -from [get_ports hard_trip_i] -to [get_ports {pwm_h_o[*] pwm_l_o[*] motor_sd_a_o}]
set_property BITSTREAM.GENERAL.COMPRESS true [current_design]

# HKT56 ABZ: J20 pin3/pin4/pin5; use level conversion before FPGA
set_property PACKAGE_PIN R14 [get_ports A]
set_property PACKAGE_PIN P14 [get_ports B]
set_property PACKAGE_PIN U12 [get_ports Z]
set_property IOSTANDARD LVCMOS33 [get_ports {A B Z}]
set_input_delay -clock sys_clk -max 0.000 [get_ports {A B Z}]
set_input_delay -clock sys_clk -min 0.000 [get_ports {A B Z}]
set_false_path -from [get_ports {A B Z}]

# USB转串口TX -> J20-9/T11；与现有J20-10/T10的FPGA TX组成全双工。
set_property PACKAGE_PIN T11 [get_ports uart_rx_i]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx_i]
set_property PULLUP true [get_ports uart_rx_i]
set_input_delay -clock sys_clk -max 0.000 [get_ports uart_rx_i]
set_input_delay -clock sys_clk -min 0.000 [get_ports uart_rx_i]
# RX仅进入三级同步器第一级D端，后级继续按50MHz计时。
set_false_path -from [get_ports uart_rx_i]
