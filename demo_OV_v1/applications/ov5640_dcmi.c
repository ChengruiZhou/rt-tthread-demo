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
#include "drv_ov5640_config.h"
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
  hdcmi.Init.SynchroMode = DCMI_MODE_CONTINUOUS;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
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
  HAL_NVIC_SetPriority(DCMI_IRQn, 0, 5);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);
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
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_D5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_D6_Pin|DCMI_D4_Pin|DCMI_D7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_D1_Pin|DCMI_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|DCMI_HSYNC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    HAL_GPIO_WritePin(, GPIO_Pin, PinState)
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

void OV5640_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);

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
  HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);
  /*PWDN引脚，高电平关闭电源，低电平供电*/
  HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin,GPIO_PIN_SET);

  HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);
  rt_thread_mdelay(10);
  HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_SET);
  //必须延时50ms,模块才会正常工作
  rt_thread_mdelay(50);
/* USER CODE END MX_GPIO_Init_2 */
}

void OV5640_init(void)
{
    MX_I2C1_Init();
    OV5640_GPIO_Init();
    MX_DMA_Init();
    MX_DCMI_Init();
    rt_kprintf("OV5640_init is ok!\n");
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
ImageFormat_TypeDef ImageFormat;
/*
* @brief  Configures the OV5640 camera in BMP mode.
* @param  BMP ImageSize: BMP image size
* @retval None
*/
void OV5640_RGB565Config(void)
{
    uint32_t i;

    /*摄像头复位*/
    OV5640_Reset();
    /* 写入寄存器配置 */
    /* Initialize OV5640   Set to output RGB565 */
    for(i=0; i<(sizeof(RGB565_Init)/4); i++)
    {
    OV5640_WriteReg(RGB565_Init[i][0], RGB565_Init[i][1]);
    rt_thread_mdelay(10);
    sensor_reg[i] = OV5640_ReadReg(RGB565_Init[i][0]);
    if(RGB565_Init[i][1] != sensor_reg[i])
        rt_kprintf("sensor_reg[0x%x]:%x-%x\n",RGB565_Init[i][0],RGB565_Init[i][1],sensor_reg[i]);
    }
    if(img_width == 320)

        ImageFormat=BMP_320x240;

    else if(img_width == 640)

        ImageFormat=BMP_640x480;

    else if(img_width == 800)

        ImageFormat=BMP_800x480;
    switch(ImageFormat)
    {
        case BMP_320x240:
        {
          for(i=0; i<(sizeof(RGB565_QVGA)/4); i++)
          {
            OV5640_WriteReg(RGB565_QVGA[i][0], RGB565_QVGA[i][1]);
          }
          break;
        }
        case BMP_640x480:
        {
           for(i=0; i<(sizeof(RGB565_VGA)/4); i++)
          {
            OV5640_WriteReg(RGB565_VGA[i][0], RGB565_VGA[i][1]);
          }
          break;
        }
        case BMP_800x480:
        {
          for(i=0; i<(sizeof(RGB565_WVGA)/4); i++)
          {
            OV5640_WriteReg(RGB565_WVGA[i][0], RGB565_WVGA[i][1]);
            sensor_reg[i] = OV5640_ReadReg(RGB565_WVGA[i][0]);
            if(RGB565_WVGA[i][1] != sensor_reg[i])
                rt_kprintf("sensor_reg[0x%x]:%x-%x\n",RGB565_Init[i][0],RGB565_Init[i][1],sensor_reg[i]);
          }
          rt_kprintf("OK\r\n");
          break;
        }
        default:
        {
          for(i=0; i<(sizeof(RGB565_WVGA)/4); i++)
          {
            OV5640_WriteReg(RGB565_WVGA[i][0], RGB565_WVGA[i][1]);
          }
          break;
        }
    }
    OV5640_ReadReg(RGB565_WVGA[i][0]);
}

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
