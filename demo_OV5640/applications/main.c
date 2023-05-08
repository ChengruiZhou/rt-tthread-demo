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

#include "ov5640_scch.h"
#include "ov5640_dcmi.h"


int main(void)
{
    OV5640_IDTypeDef OV5640_Camera_ID;
//    uint32_t index = 1;
    SCB_EnableICache();//使能I-Cache
    SCB_EnableDCache();//使能D-Cache
    SCB->CACR|=1<<2;   //强制D-Cache透写,如不开启,实际使用中可能遇到各种问题
    MX_I2C1_Init();
    OV5640_init();

    /* 读取摄像头芯片ID，确定摄像头正常连接 */
    OV5640_ReadID(&OV5640_Camera_ID);
    if(OV5640_Camera_ID.PIDH  == 0x56){
        rt_kprintf("检测到OV5640摄像头。");
    }else {
        rt_kprintf("没有检测到OV5640摄像头，请重新检查连接。");
    }
//    OV2640_JPEG_Mode(); //切换为JPEG模式
//    rt_kprintf("切换为JPEG模式 !\r\n");
//    OV2640_OutSize_Set(OV5640_JPEG_WIDTH,OV5640_JPEG_HEIGHT);
//    rt_kprintf("设置图像大小 !\r\n");
//    SCCB_WR_Reg(0XFF,0X00);
//    SCCB_WR_Reg(0XD3,0X10);
//    SCCB_WR_Reg(0XFF,0X01);
//    SCCB_WR_Reg(0X11,0X8);
//    OV5640_WriteReg
//    StartOV5640();
    while (1)
    {

    }

    return RT_EOK;
}
