/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-04     18099       the first version
 */
#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <u8g2_port.h>
#include <oled.h>

u8g2_t u8g2;

void oled_init(void)
{

    u8g2_Setup_ssd1306_i2c_128x64_noname_f( &u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_rtthread);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_CLOCK, OLED_I2C_PIN_SCL);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_DATA, OLED_I2C_PIN_SDA);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
}

void oled_clear(void)
{
    u8g2_ClearBuffer(&u8g2);
}

void oled_str(uint16_t x, uint16_t y, const char *str)
{
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
    u8g2_DrawStr(&u8g2, x, y, str);
    u8g2_SendBuffer(&u8g2);
}

void oled_DrawBox(uint16_t x,uint16_t y, uint16_t w,uint16_t h)
{
    u8g2_DrawBox(&u8g2,x,y,w,h);
    u8g2_SendBuffer(&u8g2);
}



