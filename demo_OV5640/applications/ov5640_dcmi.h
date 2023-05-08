/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-08     Mr.zhou       the first version
 */



#ifndef APPLICATIONS_OV5640_DCMI_H_
#define APPLICATIONS_OV5640_DCMI_H_

#define DCMI_D0_Pin GPIO_PIN_6
#define DCMI_D0_GPIO_Port GPIOC

#define DCMI_D1_Pin GPIO_PIN_7
#define DCMI_D1_GPIO_Port GPIOC

#define DCMI_D2_Pin GPIO_PIN_10
#define DCMI_D2_GPIO_Port GPIOG

#define DCMI_D3_Pin GPIO_PIN_11
#define DCMI_D3_GPIO_Port GPIOG

#define DCMI_D5_Pin GPIO_PIN_3
#define DCMI_D5_GPIO_Port GPIOD

#define DCMI_D4_Pin GPIO_PIN_4
#define DCMI_D4_GPIO_Port GPIOE

#define DCMI_D6_Pin GPIO_PIN_5
#define DCMI_D6_GPIO_Port GPIOE

#define DCMI_D7_Pin GPIO_PIN_6
#define DCMI_D7_GPIO_Port GPIOE

#define DCMI_VSYNC_Pin GPIO_PIN_7
#define DCMI_VSYNC_GPIO_Port GPIOB

#define DCMI_CLK_Pin GPIO_PIN_8
#define DCMI_CLK_GPIO_Port GPIOA

#define DCMI_RST_Pin GPIO_PIN_10
#define DCMI_RST_GPIO_Port GPIOF

#define DCMI_PWDN_Pin GPIO_PIN_10
#define DCMI_PWDN_GPIO_Port GPIOB

#define DCMI_HSYNC_Pin GPIO_PIN_4
#define DCMI_HSYNC_GPIO_Port GPIOA

#define OV5640_JPEG_WIDTH     1600   //JPEG拍照的宽度
#define OV5640_JPEG_HEIGHT 1200     //JPEG拍照的高度
#define pictureBufferLength OV5640_JPEG_WIDTH*OV5640_JPEG_HEIGHT

//存储摄像头ID的结构体
typedef struct
{
  uint8_t PIDH;
  uint8_t PIDL;
}OV5640_IDTypeDef;

#define OV5640_DEVICE_WRITE_ADDRESS    0x78
#define OV5640_DEVICE_READ_ADDRESS     0x79

#define OV5640_SENSOR_PIDH       0x300A
#define OV5640_SENSOR_PIDL       0x300B

void MX_DMA_Init(void);
void MX_DCMI_Init(void);
void OV5640_GPIO_Init(void);
void OV5640_init(void);
//dma_memory 以16位数据为单位， dma_bufsize以32位数据为单位(即像素个数/2)
//  OV5640_DMA_Config(LCD_FB_START_ADDRESS,LCD_GetXSize()*LCD_GetYSize()/2);
void OV5640_Reset(void);
void OV5640_ReadID(OV5640_IDTypeDef *OV5640ID);
void OV5640_RGB565Config(void);
void StartOV5640(void);
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi);

#endif /* APPLICATIONS_OV5640_DCMI_H_ */
