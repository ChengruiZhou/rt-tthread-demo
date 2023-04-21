/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-02     RT-Thread    first version
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <oled.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

int main(void)
{
    // Initialization
    oled_init();
    while (1)
    {
        // //显示字符、显示实心方块
        // oled_str(1, 18, "OLED 0.96");
        // oled_DrawBox(48,30,25,15);
        // //延时1s
        // rt_thread_mdelay(1000);
        // //清除数据缓冲
        // oled_clear();
        // //延时1s
        rt_thread_mdelay(1000);
        LOG_D("aas\n");
    }

    return RT_EOK;
}





//static void u8g2_ssd1306_12864_sw_i2c_example(int argc,char *argv[])
//{
//    u8g2_t u8g2;
//
//    // Initialization
//    u8g2_Setup_ssd1306_i2c_128x64_noname_f( &u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_rtthread);
//    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_CLOCK, OLED_I2C_PIN_SCL);
//    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_DATA, OLED_I2C_PIN_SDA);
//
//    u8g2_InitDisplay(&u8g2);
//    u8g2_SetPowerSave(&u8g2, 0);
//
//    // Draw Graphics
//    /* full buffer example, setup procedure ends in _f */
//    u8g2_ClearBuffer(&u8g2);
//    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
//    u8g2_DrawStr(&u8g2, 1, 18, "OLED on RTT");
//    u8g2_SendBuffer(&u8g2);
//
//    u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
//    u8g2_DrawGlyph(&u8g2, 112, 56, 0x2603 );
//    u8g2_SendBuffer(&u8g2);
//}


