        /*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-14     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "ov5640_scch.h"
#include "bsp_ov5640.h"
#include "ov5640_AF.h"

#include "car_speed.h"
#include "Encoder.h"


extern DCMI_HandleTypeDef DCMI_Handle;
extern DMA_HandleTypeDef DMA_Handle_dcmi;

HAL_StatusTypeDef dcmi_dma_status = HAL_OK;

#define RGB_WIDTH 192
#define RGB_HEIGHT 192
#define BUFF_SIZE RGB_WIDTH*RGB_HEIGHT
uint32_t dcmi_buffer[BUFF_SIZE];

uint32_t DCMI_RN = 0;  //row number
uint32_t DCMI_CN = 0;  //column number
uint32_t DCMI_RS = 0;  //row start
uint32_t DCMI_CS = 0;  //column start
uint8_t dcmi_flag = 1;
//uint8_t fps;
//extern uint32_t image[img_width * img_height] __attribute__((section(".SDRAM")));

void DCMI_DMA_MemInc_En(void);
void DCMI_DMA_MemInc_Den(void);
void USER_DCMI_Full_Init(void);
//摄像头识别处理函数
void DCMI_ov5640_process(void);

int main(void)
{


    OV5640_IDTypeDef OV5640_Camera_ID;
    MX_I2C1_Init();
    OV5640_HW_Init();

    OV5640_ReadID(&OV5640_Camera_ID);

    if(OV5640_Camera_ID.PIDH  == 0x56)
    {
        LOG_D("Find %x%x\n",OV5640_Camera_ID.PIDH ,OV5640_Camera_ID.PIDL);
    }
    else {
        LOG_D("can not find ov5640\n");
        while(1);
    }
    // 配置摄像头输出像素格式
    OV5640_RGB565Config();
    // 初始化摄像头，捕获并显示图像
    OV5640_Init();
    OV5640_AUTO_FOCUS();
    OV5640_OutSize_Set(16,4,320,244);
    rt_kprintf("STM32H7A3ZIT-Q DCMI Init... \r\n");
    HAL_DCMI_DeInit(&DCMI_Handle);
    rt_kprintf("STM32H7A3ZIT-Q DCMI Init OK!\r\n");
    OV5640_Focus_Single();
    rt_kprintf("Frame START\r\n");
    rt_kprintf("ov5640 cpture \r\n");
    rt_kprintf("ov5640 cpture 224*224\r\n");

    hw_car_speed_init();
//    car_speed_control(500000,500000);

    Encoder_init();
    while (1)
    {
        speed_thread_entry();
        /*
         // 输出 Fps
        LOG_D("fps = %d",fps);

        LOG_D("Adress: 0x%08X, Data: 0x%08X\n", (uint32_t)&image[0] , image[0]);
        */
//        LOG_D("TIM");

        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}


//DMA地址自增使能
void DCMI_DMA_MemInc_En(void)
{
    HAL_DMA_DeInit(&DMA_Handle_dcmi);

    DMA_Handle_dcmi.Init.MemInc = DMA_MINC_ENABLE;
    if (HAL_DMA_Init(&DMA_Handle_dcmi) != HAL_OK)
    {
        Error_Handler();
    }
}
//DMA地址自增失能
void DCMI_DMA_MemInc_Den(void)
{
    HAL_DMA_DeInit(&DMA_Handle_dcmi);
    DMA_Handle_dcmi.Init.MemInc = DMA_MINC_DISABLE;
    if (HAL_DMA_Init(&DMA_Handle_dcmi) != HAL_OK)
    {
        Error_Handler();
    }
}

//DCMI重新配置，并赋值标志位
void USER_DCMI_Full_Init(void)
{
    DCMI_Handle.Instance = DCMI;
    DCMI_Handle.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
    DCMI_Handle.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
    DCMI_Handle.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
    DCMI_Handle.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
    DCMI_Handle.Init.CaptureRate = DCMI_CR_ALL_FRAME;
    DCMI_Handle.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
    DCMI_Handle.Init.JPEGMode = DCMI_JPEG_ENABLE;
    DCMI_Handle.Init.ByteSelectMode = DCMI_BSM_ALL;
    DCMI_Handle.Init.ByteSelectStart = DCMI_OEBS_ODD;
    DCMI_Handle.Init.LineSelectMode = DCMI_LSM_ALL;
    DCMI_Handle.Init.LineSelectStart = DCMI_OELS_ODD;
    if (HAL_DCMI_Init(&DCMI_Handle) != HAL_OK)
    {
        Error_Handler();
    }
//  __HAL_DCMI_DISABLE_IT(&hdcmi,DCMI_IT_FRAME|DCMI_IT_LINE | DCMI_IT_VSYNC | DCMI_IT_ERR | DCMI_IT_OVR);
//  __HAL_DCMI_ENABLE_IT(&hdcmi,DCMI_IT_FRAME);      //使能帧中断
    dcmi_flag=1;//使能识别
}
//摄像头识别处理函数
void DCMI_ov5640_process(void)
{

    if (dcmi_flag==1)
    {
        DCMI_DMA_MemInc_En();
        HAL_DCMI_DisableCrop (&DCMI_Handle);
        HAL_Delay(100);

        dcmi_dma_status = HAL_DCMI_Init(&DCMI_Handle);
        HAL_DCMI_DisableCrop (&DCMI_Handle);
        DCMI_RN = 192;
        DCMI_CN = 192;
        DCMI_RS = 0;
        DCMI_CS = 0;
        HAL_DCMI_ConfigCrop (&DCMI_Handle, DCMI_CS, DCMI_RS, DCMI_CN, DCMI_RN);//裁剪输出图像
        rt_thread_mdelay(1);
        HAL_DCMI_EnableCrop (&DCMI_Handle);
        rt_thread_mdelay(1);

        __HAL_DCMI_ENABLE_IT(&DCMI_Handle,DCMI_IT_FRAME);
        HAL_DCMI_Start_DMA(&DCMI_Handle, DCMI_MODE_SNAPSHOT,(uint32_t)dcmi_buffer, DCMI_CN*DCMI_RN/4);//开始通过DCMI接口采集ov5640拍摄的图片数据。
    }
}
