# 基于rt-thread studio的stm32 demo
- pwm demo upload
- WIFI demo upload
- OLED demo upload
- stm32 demo based Clion upload
- Encoder model
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    通过stm32cubemx进行配置，引脚为encoder model T1 and T2 模式
  加入“pulse_encoder_config.h”和“pulse_encoder_config.c”文件
  
board中并无此句，需添加
 <p align="center">
<img src="figure/encoder_config1.png" width="80%" >
</p>
添加pulse_encoder_config.h文件到索引列表
<p align="center">
<img src="figure/encoder_config2.png" width="80%" >
</p>
可以获取所定义的编码器信息
<p align="center">
<img src="figure/encoder_config.png" width="100%" >
</p>
添加编码器软件框架
<p align="center">
<img src="figure/encoder_config3.png" width="100%" >
</p>