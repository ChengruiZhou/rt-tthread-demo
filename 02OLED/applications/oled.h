/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-04     18099       the first version
 */
#ifndef APPLICATIONS_OLED_H_
#define APPLICATIONS_OLED_H_



#define OLED_I2C_PIN_SCL                    22  // PB6
#define OLED_I2C_PIN_SDA                    23  // PB7

void oled_init(void);
void oled_clear(void);
void oled_str(uint16_t x, uint16_t y, const char *str);
void oled_DrawBox(uint16_t x,uint16_t y, uint16_t w,uint16_t h);



#endif /* APPLICATIONS_OLED_H_ */
