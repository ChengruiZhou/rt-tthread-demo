- STM32CUBEMX初始化接口
- SCCH协议
- DCMI

PIX_CLK 像素同步时钟
HSYNC  行同步信号(水平同步信号)
VSYNC  帧同步信号(垂直同步信号)

- 初始化DCMI 时钟，I2C 时钟；
- 使用I2C 接口向OV5640 写入寄存器配置；
- 初始化DCMI 工作模式；
- 初始化DMA，用于搬运DCMI 的数据到显存空间进行显示；
- 编写测试程序，控制采集图像数据并显示。