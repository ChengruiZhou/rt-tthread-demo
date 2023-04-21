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

/* 定义 PWM3 设备对象 */
struct rt_device_pwm *pwm3_device;

#define PWM_DEV_NAME        "pwm3"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL1     3       /* PWM通道 */
struct rt_device_pwm *pwm_dev1;      /* PWM设备句柄 */

extern void MX_TIM3_Init(void);
int main(void)
{
    MX_TIM3_Init();
    rt_uint32_t period, pulse, dir;
    period = 1000000;    /* 1KHz周期为1ms,这里单位是纳秒ns，1ms等于10的6次方纳秒ns*/
    pulse = 500000;          /* PWM脉冲宽度值，单位为纳秒ns */
    dir = 0;
           /* 查找设备 */
    pwm_dev1 = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev1 == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
    }
    rt_kprintf("pwm sample run ! find %s device!\n", PWM_DEV_NAME);
   /* 设置PWM周期和脉冲宽度 */
    rt_pwm_set(pwm_dev1, PWM_DEV_CHANNEL1, period, pulse);
   /* 使能设备 */
    rt_pwm_enable(pwm_dev1, PWM_DEV_CHANNEL1);
    while (1)
    {
        rt_thread_mdelay(50);
        if (dir)
        {
            pulse += 5000;      /* 从0值开始每次增加5000ns */
        }
        else
        {
            pulse -= 5000;      /* 从最大值开始每次减少5000ns */
        }
        if (pulse >= period)
        {
            dir = 0;
        }
        if (0 == pulse)
        {
            dir = 1;
        }

        /* 设置PWM周期和脉冲宽度 */
        rt_pwm_set(pwm_dev1, PWM_DEV_CHANNEL1, period, pulse);


    }

    return RT_EOK;
}
