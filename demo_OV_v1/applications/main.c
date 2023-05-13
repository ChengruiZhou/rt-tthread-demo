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
void OV5640_CreatColor();
int main(void)
{
    OV5640_IDTypeDef OV5640_Camera_ID;
    OV5640_init();
    OV5640_Reset();
    OV5640_ReadID(&OV5640_Camera_ID);

    if(OV5640_Camera_ID.PIDH  == 0x56)
    {
        rt_kprintf("Find %x%x\n",OV5640_Camera_ID.PIDH ,OV5640_Camera_ID.PIDL);
    }
    else {
        rt_kprintf("can not find ov5640\n");
        while(1);
    }
    OV5640_RGB565Config();

    OV5640_CreatColor();
    while (1)
    {

    }

    return RT_EOK;
}
//单片机生成彩条测试
void OV5640_CreatColor()
{
    uint16_t i,j,k = 0;
    uint16_t color = 0;
    //列缓存区
    uint16_t buff[320];

    while(1)
    {
        //数据开始(从上往下，从左往右)
        rt_kprintf("data:\n");

        for(i=0;i<240;i++)
        {
            rt_kprintf("L");//列有效
            for(j=0;j < 320;j++)//一列
            {
                //生成彩条  9E F7,8D EF ,9E 3F,83 1F,FF F0,43 D9,95 28,82 10
                if(i < 30)
                {
                    color = 0x9EF7;
                }else if(i < 60)
                {
                    color = 0x8DEF;
                }else if(i < 90)
                {
                    color = 0x9E3F;
                }else if(i < 120)
                {
                    color = 0x831F;
                }else if(i < 150)
                {
                    color = 0xFFF0;
                }else if(i < 180)
                {
                    color = 0x43D9;
                }else if(i < 210)
                {
                    color = 0x9528;
                }else
                {
                    color = 0x8210;
                }
                buff[j] = color;
            }
            //准备好一列数据
            for(k=0;k<320;k++)
            {
                rt_kprintf("%04X", buff[k]);//打印色条
            }
            rt_kprintf("\n");
        }
    }
}
