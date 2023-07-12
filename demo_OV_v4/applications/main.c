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

uint8_t fps;
extern uint32_t image[img_width * img_height] __attribute__((section(".SDRAM")));

int main(void)
{

    /*
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
    */

    hw_car_speed_init();
    car_speed_control(-100000,200000);

    while (1)
    {
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


