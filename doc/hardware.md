# 当前硬件与管脚

本表直接从根目录foc.xdc提取，方向相对于FPGA。当前器件xc7z020clg400-2，逻辑LVCMOS33，系统时钟50MHz；具体约束以foc.xdc为准。

| TOP端口/位 | PACKAGE_PIN | 方向 |
|---|---|---|
| sys_clk | U18 | input |
| sys_rst_n | M15 | input |
| adc_busy_i | B20 | input |
| adc_convst_o | K19 | output |
| adc_cs_n_o | C20 | output |
| adc_rd_n_o | B19 | output |
| adc_reset_o | A20 | output |
| uart_tx_o | T10 | output |
| hard_trip_i | V16 | input |
| motor_sd_a_o | P18 | output |
| adc_os_o[0] | M17 | output |
| adc_os_o[1] | M18 | output |
| adc_os_o[2] | J19 | output |
| adc_data_i[0] | D20 | input |
| adc_data_i[1] | D19 | input |
| adc_data_i[2] | L20 | input |
| adc_data_i[3] | L19 | input |
| adc_data_i[4] | F16 | input |
| adc_data_i[5] | F17 | input |
| adc_data_i[6] | H20 | input |
| adc_data_i[7] | J20 | input |
| adc_data_i[8] | G18 | input |
| adc_data_i[9] | G17 | input |
| adc_data_i[10] | H17 | input |
| adc_data_i[11] | H16 | input |
| adc_data_i[12] | G15 | input |
| adc_data_i[13] | H15 | input |
| adc_data_i[14] | K18 | input |
| adc_data_i[15] | K17 | input |
| pwm_h_o[0] | P19 | output |
| pwm_h_o[1] | R17 | output |
| pwm_h_o[2] | P15 | output |
| pwm_l_o[0] | N18 | output |
| pwm_l_o[1] | R16 | output |
| pwm_l_o[2] | P16 | output |
| A | R14 | input |
| B | P14 | input |
| Z | U12 | input |
| uart_rx_i | T11 | input |

## 已确认接口

- USB串口RX←J20-10/T10，USB串口TX→J20-9/T11，共地，1500000/8N1。
- HKT56实际为2048线，A/B/Z经接口送J20-3/4/5（R14/P14/U12），四倍频8192计数/圈；编码器资料中的线数与实物不同。
- AD7606采用16位并行模式，CONVST A/B短接并一起接J21-5；J21-8 RESET、J21-9 BUSY、J21-10 CS、J21-7 RD。
- OS0/1/2接J21-4/3/6，FPGA输出000关闭过采样；前三通道读取三相电流。
- 六路PWM高有效；motor_sd_a_o高允许驱动，hard_trip_i高直接关断，故障输入内部上拉。

IO_J20.png、IO_J21.png为原连接器资料。HKT56.pdf为编码器资料，1NSPC136001D_APUS2.pdf及DRAWING_140324.pdf为原电机资料；本目录不保留空白Visio或历史测量数据。

原理图、电平转换和现有接线应保持当前已验证配置。FPGA端为3.3V逻辑，编码器输出必须符合其输入电平要求。ADC的电流符号、比例和保护阈值已写在readme及RTL参数中。
