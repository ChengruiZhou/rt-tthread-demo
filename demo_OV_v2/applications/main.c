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
#include "ov5640_dcmi.h"
////
extern volatile uint32_t image[img_width * img_height] __attribute__((section(".SDRAM")));
int main(void)
{
    OV5640_IDTypeDef OV5640_Camera_ID;
    OV5640_init();
    OV5640_Reset();
    OV5640_ReadID(&OV5640_Camera_ID);

    if(OV5640_Camera_ID.PIDH  == 0x56)
    {
        LOG_D("Find %x%x\n",OV5640_Camera_ID.PIDH ,OV5640_Camera_ID.PIDL);
    }
    else {
        LOG_D("can not find ov5640\n");
        while(1);
    }

    OV5640_RGB565Config();

    for (uint32_t i = 0; i < 128; i++) {
        volatile uint32_t *address = (volatile uint32_t *)&image[i];
        volatile uint32_t data = *address;
        LOG_D("address: 0x%08X, data: %d\n", (uint32_t)address, data);
    }

    while (1)
    {
        LOG_D("c ov5640");
//        HAL_Delay(1000);
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}


