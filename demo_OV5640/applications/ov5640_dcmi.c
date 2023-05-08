/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-08     Mr.zhou       the first version
 */
#include <rtthread.h>
#include <board.h>
#include <drv_common.h>
#include "ov5640_dcmi.h"
#include "ov5640_scch.h"

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;
UART_HandleTypeDef *UartHandle;
/*
* Enable DMA controller clock
*/
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}
/*
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/*
* @brief DCMI MSP Initialization
* This function configures the hardware resources used in this example
* @param hdcmi: DCMI handle pointer
* @retval None
*/
void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdcmi->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspInit 0 */

  /* USER CODE END DCMI_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DCMI_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DCMI GPIO Configuration
    PG10     ------> DCMI_D2
    PG11     ------> DCMI_D3
    PD3     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PE5     ------> DCMI_D6
    PE4     ------> DCMI_D4
    PE6     ------> DCMI_D7
    PC7     ------> DCMI_D1
    PC6     ------> DCMI_D0
    PA6     ------> DCMI_PIXCLK
    PA4     ------> DCMI_HSYNC
    */
    GPIO_InitStruct.Pin = DCMI_D2_Pin|DCMI_D3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_D5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_D6_Pin|DCMI_D4_Pin|DCMI_D7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_D1_Pin|DCMI_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|DCMI_HSYNC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* DCMI DMA Init */
    /* DCMI Init */
    hdma_dcmi.Instance = DMA1_Stream0;
    hdma_dcmi.Init.Request = DMA_REQUEST_DCMI;
    hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dcmi.Init.Mode = DMA_CIRCULAR;
    hdma_dcmi.Init.Priority = DMA_PRIORITY_LOW;
    hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_dcmi.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hdcmi,DMA_Handle,hdma_dcmi);

    /* DCMI interrupt Init */
    HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);
  /* USER CODE BEGIN DCMI_MspInit 1 */


  /* USER CODE END DCMI_MspInit 1 */
  }

}


/*
* @brief DCMI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hdcmi: DCMI handle pointer
* @retval None
*/
void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi)
{
  if(hdcmi->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspDeInit 0 */

  /* USER CODE END DCMI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DCMI_CLK_DISABLE();

    /**DCMI GPIO Configuration
    PG10     ------> DCMI_D2
    PG11     ------> DCMI_D3
    PD3     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PE5     ------> DCMI_D6
    PE4     ------> DCMI_D4
    PE6     ------> DCMI_D7
    PC7     ------> DCMI_D1
    PC6     ------> DCMI_D0
    PA6     ------> DCMI_PIXCLK
    PA4     ------> DCMI_HSYNC
    */
    HAL_GPIO_DeInit(GPIOG, DCMI_D2_Pin|DCMI_D3_Pin);

    HAL_GPIO_DeInit(DCMI_D5_GPIO_Port, DCMI_D5_Pin);

    HAL_GPIO_DeInit(DCMI_VSYNC_GPIO_Port, DCMI_VSYNC_Pin);

    HAL_GPIO_DeInit(GPIOE, DCMI_D6_Pin|DCMI_D4_Pin|DCMI_D7_Pin);

    HAL_GPIO_DeInit(GPIOC, DCMI_D1_Pin|DCMI_D0_Pin);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|DCMI_HSYNC_Pin);
    /* DCMI DMA DeInit */
    HAL_DMA_DeInit(hdcmi->DMA_Handle);

    /* DCMI interrupt DeInit */
    HAL_NVIC_DisableIRQ(DCMI_IRQn);
  /* USER CODE BEGIN DCMI_MspDeInit 1 */

  /* USER CODE END DCMI_MspDeInit 1 */
  }

}
unsigned short RGB565_Init[][2]=
{

    //15fps VGA RGB565 output

    // 24MHz input clock, 24MHz PCLK

    {0x3103, 0x11}, // system clock from pad, bit[1]

    {0x3008, 0x82}, // software reset, bit[7]

    // delay 5ms

    {0x3008, 0x42}, // software power down, bit[6]

    {0x3103, 0x03}, // system clock from PLL, bit[1]

    {0x3017, 0xff}, // FREX, Vsync, HREF, PCLK, D[9:6] output enable

    {0x3018, 0xff}, // D[5:0], GPIO[1:0] output enable

    {0x3034, 0x1a}, // MIPI 10-bit

    {0x3037, 0x13}, // PLL root divider, bit[4], PLL pre-divider, bit[3:0]

    {0x3108, 0x01}, // PCLK root divider, bit[5:4], SCLK2x root divider, bit[3:2]

    // SCLK root divider, bit[1:0]

    {0x3630, 0x36},

    {0x3631, 0x0e},

    {0x3632, 0xe2},

    {0x3633, 0x12},

    {0x3621, 0xe0},

    {0x3704, 0xa0},

    {0x3703, 0x5a},

    {0x3715, 0x78},

    {0x3717, 0x01},

    {0x370b, 0x60},

    {0x3705, 0x1a},

    {0x3905, 0x02},

    {0x3906, 0x10},

    {0x3901, 0x0a},

    {0x3731, 0x12},

    {0x3600, 0x08}, // VCM control

    {0x3601, 0x33}, // VCM control

    {0x302d, 0x60}, // system control

    {0x3620, 0x52},

    {0x371b, 0x20},

    {0x471c, 0x50},

    {0x3a13, 0x43}, // pre-gain = 1.047x

    {0x3a18, 0x00}, // gain ceiling

    {0x3a19, 0xf8}, // gain ceiling = 15.5x

    {0x3635, 0x13},

    {0x3636, 0x03},

    {0x3634, 0x40},

    {0x3622, 0x01},

    // 50/60Hz detection 50/60Hz 灯光条纹过滤

    {0x3c01, 0x34}, // Band auto, bit[7]

    {0x3c04, 0x28}, // threshold low sum

    {0x3c05, 0x98}, // threshold high sum

    {0x3c06, 0x00}, // light meter 1 threshold[15:8]

    {0x3c07, 0x08}, // light meter 1 threshold[7:0]

    {0x3c08, 0x00}, // light meter 2 threshold[15:8]

    {0x3c09, 0x1c}, // light meter 2 threshold[7:0]

    {0x3c0a, 0x9c}, // sample number[15:8]

    {0x3c0b, 0x40}, // sample number[7:0]

    {0x3810, 0x00}, // Timing Hoffset[11:8]

    {0x3811, 0x10}, // Timing Hoffset[7:0]

    {0x3812, 0x00}, // Timing Voffset[10:8]

    {0x3708, 0x64},

    {0x4001, 0x02}, // BLC start from line 2

    {0x4005, 0x1a}, // BLC always update

    {0x3000, 0x00}, // enable blocks

    {0x3004, 0xff}, // enable clocks

    {0x300e, 0x58}, // MIPI power down, DVP enable

    {0x302e, 0x00},



    {0x4300, 0x6f}, // RGB565

    {0x501f, 0x01}, // RGB565



    {0x440e, 0x00},

    {0x5000, 0xa7}, // Lenc on, raw gamma on, BPC on, WPC on, CIP on

    // AEC target 自动曝光控制

    {0x3a0f, 0x30}, // stable range in high

    {0x3a10, 0x28}, // stable range in low

    {0x3a1b, 0x30}, // stable range out high

    {0x3a1e, 0x26}, // stable range out low

    {0x3a11, 0x60}, // fast zone high

    {0x3a1f, 0x14}, // fast zone low

    // Lens correction for ? 镜头补偿

    {0x5800, 0x23},

    {0x5801, 0x14},

    {0x5802, 0x0f},

    {0x5803, 0x0f},

    {0x5804, 0x12},

    {0x5805, 0x26},

    {0x5806, 0x0c},

    {0x5807, 0x08},

    {0x5808, 0x05},

    {0x5809, 0x05},

    {0x580a, 0x08},

    {0x580b, 0x0d},

    {0x580c, 0x08},

    {0x580d, 0x03},

    {0x580e, 0x00},

    {0x580f, 0x00},

    {0x5810, 0x03},

    {0x5811, 0x09},

    {0x5812, 0x07},

    {0x5813, 0x03},

    {0x5814, 0x00},

    {0x5815, 0x01},

    {0x5816, 0x03},

    {0x5817, 0x08},

    {0x5818, 0x0d},

    {0x5819, 0x08},

    {0x581a, 0x05},

    {0x581b, 0x06},

    {0x581c, 0x08},

    {0x581d, 0x0e},

    {0x581e, 0x29},

    {0x581f, 0x17},

    {0x5820, 0x11},

    {0x5821, 0x11},

    {0x5822, 0x15},

    {0x5823, 0x28},

    {0x5824, 0x46},

    {0x5825, 0x26},

    {0x5826, 0x08},

    {0x5827, 0x26},

    {0x5828, 0x64},

    {0x5829, 0x26},

    {0x582a, 0x24},

    {0x582b, 0x22},

    {0x582c, 0x24},

    {0x582d, 0x24},

    {0x582e, 0x06},

    {0x582f, 0x22},

    {0x5830, 0x40},

    {0x5831, 0x42},

    {0x5832, 0x24},

    {0x5833, 0x26},

    {0x5834, 0x24},

    {0x5835, 0x22},

    {0x5836, 0x22},

    {0x5837, 0x26},

    {0x5838, 0x44},

    {0x5839, 0x24},

    {0x583a, 0x26},

    {0x583b, 0x28},

    {0x583c, 0x42},

    {0x583d, 0xce}, // lenc BR offset

    // AWB 自动白平衡

    {0x5180, 0xff}, // AWB B block

    {0x5181, 0xf2}, // AWB control

    {0x5182, 0x00}, // [7:4] max local counter, [3:0] max fast counter

    {0x5183, 0x14}, // AWB advanced

    {0x5184, 0x25},

    {0x5185, 0x24},

    {0x5186, 0x09},

    {0x5187, 0x09},

    {0x5188, 0x09},

    {0x5189, 0x75},

    {0x518a, 0x54},

    {0x518b, 0xe0},

    {0x518c, 0xb2},

    {0x518d, 0x42},

    {0x518e, 0x3d},

    {0x518f, 0x56},

    {0x5190, 0x46},

    {0x5191, 0xf8}, // AWB top limit

    {0x5192, 0x04}, // AWB bottom limit

    {0x5193, 0x70}, // red limit

    {0x5194, 0xf0}, // green limit

    {0x5195, 0xf0}, // blue limit

    {0x5196, 0x03}, // AWB control

    {0x5197, 0x01}, // local limit

    {0x5198, 0x04},

    {0x5199, 0x12},

    {0x519a, 0x04},

    {0x519b, 0x00},

    {0x519c, 0x06},

    {0x519d, 0x82},

    {0x519e, 0x38}, // AWB control

    // Gamma 伽玛曲线

    {0x5480, 0x01}, // Gamma bias plus on, bit[0]

    {0x5481, 0x08},

    {0x5482, 0x14},

    {0x5483, 0x28},

    {0x5484, 0x51},

    {0x5485, 0x65},

    {0x5486, 0x71},

    {0x5487, 0x7d},

    {0x5488, 0x87},

    {0x5489, 0x91},

    {0x548a, 0x9a},

    {0x548b, 0xaa},

    {0x548c, 0xb8},

    {0x548d, 0xcd},

    {0x548e, 0xdd},

    {0x548f, 0xea},

    {0x5490, 0x1d},

    // color matrix 色彩矩阵

    {0x5381, 0x1e}, // CMX1 for Y

    {0x5382, 0x5b}, // CMX2 for Y

    {0x5383, 0x08}, // CMX3 for Y

    {0x5384, 0x0a}, // CMX4 for U

    {0x5385, 0x7e}, // CMX5 for U

    {0x5386, 0x88}, // CMX6 for U

    {0x5387, 0x7c}, // CMX7 for V

    {0x5388, 0x6c}, // CMX8 for V

    {0x5389, 0x10}, // CMX9 for V

    {0x538a, 0x01}, // sign[9]

    {0x538b, 0x98}, // sign[8:1]

    // UV adjust UV 色彩饱和度调整

    {0x5580, 0x06}, // saturation on, bit[1]

    {0x5583, 0x40},

    {0x5584, 0x10},

    {0x5589, 0x10},

    {0x558a, 0x00},

    {0x558b, 0xf8},

    {0x501d, 0x40}, // enable manual offset of contrast

    // CIP 锐化和降噪

    {0x5300, 0x08}, // CIP sharpen MT threshold 1

    {0x5301, 0x30}, // CIP sharpen MT threshold 2

    {0x5302, 0x10}, // CIP sharpen MT offset 1

    {0x5303, 0x00}, // CIP sharpen MT offset 2

    {0x5304, 0x08}, // CIP DNS threshold 1

    {0x5305, 0x30}, // CIP DNS threshold 2

    {0x5306, 0x08}, // CIP DNS offset 1

    {0x5307, 0x16}, // CIP DNS offset 2

    {0x5309, 0x08}, // CIP sharpen TH threshold 1

    {0x530a, 0x30}, // CIP sharpen TH threshold 2

    {0x530b, 0x04}, // CIP sharpen TH offset 1

    {0x530c, 0x06}, // CIP sharpen TH offset 2

    {0x5025, 0x00},

    {0x3008, 0x02} // wake up from standby, bit[6]



//    0x503d, 0x80,//测试彩条

//    0x4741, 0x00,

};

unsigned short RGB565_QVGA[][2]=
{

    // input clock 24Mhz, PCLK 45.6Mhz

    {0x3035, 0x41}, // PLL

    {0x3036, 0x72}, // PLL

    {0x3c07, 0x08}, // light meter 1 threshold[7:0]

    {0x3820, 0x42}, // flip

    {0x3821, 0x00}, // mirror

    {0x3814, 0x31}, // timing X inc

    {0x3815, 0x31}, // timing Y inc

    {0x3800, 0x00}, // HS

    {0x3801, 0x00}, // HS

    {0x3802, 0x00}, // VS

    {0x3803, 0xbe}, // VS

    {0x3804, 0x0a}, // HW (HE)

    {0x3805, 0x3f}, // HW (HE)

    {0x3806, 0x06}, // VH (VE)

    {0x3807, 0xe4}, // VH (VE)



    {0x3808, 0x01}, // DVPHO = 320

    {0x3809, 0x40}, // DVP HO

    {0x380a, 0x00}, // DVPVO = 240

    {0x380b, 0xf0}, // DVPVO

    {0x3810, 0x00}, // H offset = 16

    {0x3811, 0x10}, // H offset

    {0x3812, 0x00}, // V offset = 4

    {0x3813, 0x04}, // V offset



    {0x380c, 0x07}, // HTS

    {0x380d, 0x69}, // HTS

    {0x380e, 0x03}, // VTS

    {0x380f, 0x21}, // VTS



    {0x3618, 0x00},

    {0x3612, 0x29},

    {0x3709, 0x52},

    {0x370c, 0x03},

    {0x3a02, 0x09}, // 60Hz max exposure, night mode 5fps

    {0x3a03, 0x63}, // 60Hz max exposure



    {0x3a14, 0x09}, // 50Hz max exposure, night mode 5fps

    {0x3a15, 0x63}, // 50Hz max exposure

    {0x4004, 0x02}, // BLC line number

    {0x3002, 0x1c}, // reset JFIFO, SFIFO, JPG

    {0x3006, 0xc3}, // disable clock of JPEG2x, JPEG

    {0x4713, 0x03}, // JPEG mode 3

    {0x4407, 0x04}, // Quantization sacle

    {0x460b, 0x35},

    {0x460c, 0x22},

    {0x4837, 0x22}, // MIPI global timing

    {0x3824, 0x02}, // PCLK manual divider

    {0x5001, 0xa3}, // SDE on, CMX on, AWB on

    {0x3503, 0x00} // AEC/AGC on//     /* Initialize OV5640 */

};

unsigned short RGB565_VGA[][2]=

{

    // input clock 24Mhz, PCLK 45.6Mhz

    {0x3035, 0x41}, // PLL

    {0x3036, 0x72}, // PLL

    {0x3c07, 0x08}, // light meter 1 threshold[7:0]

    {0x3820, 0x42}, // flip

    {0x3821, 0x00}, // mirror

    {0x3814, 0x31}, // timing X inc

    {0x3815, 0x31}, // timing Y inc

    {0x3800, 0x00}, // HS

    {0x3801, 0x00}, // HS

    {0x3802, 0x00}, // VS

    {0x3803, 0xbe}, // VS

    {0x3804, 0x0a}, // HW (HE)

    {0x3805, 0x3f}, // HW (HE)

    {0x3806, 0x06}, // VH (VE)

    {0x3807, 0xe4}, // VH (VE)



    {0x3808, 0x02}, // DVPHO = 640

    {0x3809, 0x80}, // DVP HO

    {0x380a, 0x01}, // DVPVO = 480

    {0x380b, 0xe0}, // DVPVO

    {0x3810, 0x00}, // H offset = 16

    {0x3811, 0x10}, // H offset

    {0x3812, 0x00}, // V offset = 4

    {0x3813, 0x04}, // V offset



    {0x380c, 0x07}, // HTS

    {0x380d, 0x69}, // HTS

    {0x380e, 0x03}, // VTS

    {0x380f, 0x21}, // VTS

//    0x3813, 0x06, // timing V offset

    {0x3618, 0x00},

    {0x3612, 0x29},

    {0x3709, 0x52},

    {0x370c, 0x03},

    {0x3a02, 0x09}, // 60Hz max exposure, night mode 5fps

    {0x3a03, 0x63}, // 60Hz max exposure



    {0x3a14, 0x09}, // 50Hz max exposure, night mode 5fps

    {0x3a15, 0x63}, // 50Hz max exposure

    {0x4004, 0x02}, // BLC line number

    {0x3002, 0x1c}, // reset JFIFO, SFIFO, JPG

    {0x3006, 0xc3}, // disable clock of JPEG2x, JPEG

    {0x4713, 0x03}, // JPEG mode 3

    {0x4407, 0x04}, // Quantization sacle

    {0x460b, 0x35},

    {0x460c, 0x22},

    {0x4837, 0x22}, // MIPI global timing

    {0x3824, 0x02}, // PCLK manual divider

    {0x5001, 0xa3}, // SDE on, CMX on, AWB on

    {0x3503, 0x00} // AEC/AGC on//     /* Initialize OV5640 */



};



unsigned short RGB565_WVGA[][2]=

{

    // 800x480 15fps, night mode 5fps

    // input clock 24Mhz, PCLK 45.6Mhz

    {0x3035, 0x41}, // PLL

    {0x3036, 0x72}, // PLL

    {0x3c07, 0x08}, // light meter 1 threshold[7:0]

    {0x3820, 0x42}, // flip

    {0x3821, 0x00}, // mirror

    {0x3814, 0x31}, // timing X inc

    {0x3815, 0x31}, // timing Y inc

    {0x3800, 0x00}, // HS

    {0x3801, 0x00}, // HS

    {0x3802, 0x00}, // VS

    {0x3803, 0xbe}, // VS

    {0x3804, 0x0a}, // HW (HE)

    {0x3805, 0x3f}, // HW (HE)

    {0x3806, 0x06}, // VH (VE)

    {0x3807, 0xe4}, // VH (VE)

    {0x3808, 0x03}, // DVPHO

    {0x3809, 0x20}, // DVPHO

    {0x380a, 0x01}, // DVPVO

    {0x380b, 0xe0}, // DVPVO

    {0x380c, 0x07}, // HTS

    {0x380d, 0x69}, // HTS

    {0x380e, 0x03}, // VTS

    {0x380f, 0x21}, // VTS

    {0x3813, 0x06}, // timing V offset

    {0x3618, 0x00},

    {0x3612, 0x29},

    {0x3709, 0x52},

    {0x370c, 0x03},

    {0x3a02, 0x09}, // 60Hz max exposure, night mode 5fps

    {0x3a03, 0x63}, // 60Hz max exposure

    // banding filters are calculated automatically in camera driver

    //0x3a08, 0x00, // B50 step

    //0x3a09, 0x78, // B50 step

    //0x3a0a, 0x00, // B60 step

    //0x3a0b, 0x64, // B60 step

    //0x3a0e, 0x06, // 50Hz max band

    //0x3a0d, 0x08, // 60Hz max band

    {0x3a14, 0x09}, // 50Hz max exposure, night mode 5fps

    {0x3a15, 0x63}, // 50Hz max exposure

    {0x4004, 0x02}, // BLC line number

    {0x3002, 0x1c}, // reset JFIFO, SFIFO, JPG

    {0x3006, 0xc3}, // disable clock of JPEG2x, JPEG

    {0x4713, 0x03}, // JPEG mode 3

    {0x4407, 0x04}, // Quantization sacle

    {0x460b, 0x35},

    {0x460c, 0x22},

    {0x4837, 0x22}, // MIPI global timing

    {0x3824, 0x02}, // PCLK manual divider

    {0x5001, 0xa3}, // SDE on, CMX on, AWB on

    {0x3503, 0x00} // AEC/AGC on//     /* Initialize OV5640 */


//    0x503d, 0x80,//测试彩条

//    0x4741, 0x00,

};
void OV5640_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_CLK_GPIO_Port, DCMI_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DCMI_CLKIN_Pin */
  GPIO_InitStruct.Pin = DCMI_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DCMI_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_RST_Pin */
  GPIO_InitStruct.Pin = DCMI_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DCMI_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_PWDN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DCMI_PWDN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void OV5640_init(void)
{
    OV5640_GPIO_Init();
    MX_DMA_Init();
    MX_DCMI_Init();
    //使能DCMI采集数据
//    HAL_DCMI_Start_DMA(&hdma_dcmi,DCMI_MODE_CONTINUOUS, (uint32_t)DMA_Memory0BaseAddr,DMA_BufferSize);

}
/*
* @brief  Resets the OV5640 camera.
* @param  None
* @retval None
*/
void OV5640_Reset(void)
{
    /*OV5640软件复位*/
  OV5640_WriteReg(0x3008, 0x80);
}
/*
* @brief  读取摄像头的ID.
* @param  OV5640ID: 存储ID的结构体
* @retval None
*/
void OV5640_ReadID(OV5640_IDTypeDef *OV5640ID)
{

    /*读取寄存芯片ID*/
  OV5640ID->PIDH = OV5640_ReadReg(OV5640_SENSOR_PIDH);
  OV5640ID->PIDL = OV5640_ReadReg(OV5640_SENSOR_PIDL);
}

unsigned short sensor_reg[(sizeof(RGB565_Init)/4)];
/*
* @brief  Configures the OV5640 camera in BMP mode.
* @param  BMP ImageSize: BMP image size
* @retval None
*/
void OV5640_RGB565Config(void)
{

}

//__align(32) uint32_t buf[65535] __attribute((at(0X24040000)));

static uint32_t JpegBuffer[pictureBufferLength];

void StartOV5640(void)
{
    __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IER_FRAME_IE);                                                //使用帧中断
    memset((void *)JpegBuffer,0,sizeof(JpegBuffer));                                                //把接收BUF清空
    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)JpegBuffer, pictureBufferLength);    //启动拍照
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{

    HAL_DCMI_Suspend(hdcmi);//拍照完成，挂起DCMI
    HAL_DCMI_Stop(hdcmi);//拍照完成，停止DMA传输
    int pictureLength =pictureBufferLength;
    while(pictureLength > 0)//循环计算出接收的JPEG的大小
    {
        if(JpegBuffer[pictureLength-1] != 0x00000000)
        {
            break;
        }
        pictureLength--;
    }
    pictureLength*=4;//buf是uint32_t，下面发送是uint8_t,所以长度要*4
    if(UartHandle != NULL)
        HAL_UART_Transmit(UartHandle, (uint8_t*)JpegBuffer, pictureLength, 0XFFFFF);//将jpeg原始数据传出

    StartOV5640();
}