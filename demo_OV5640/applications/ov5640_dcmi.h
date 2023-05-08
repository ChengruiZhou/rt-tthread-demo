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

void MX_DCMI_Init(void);

#endif /* APPLICATIONS_OV5640_DCMI_H_ */
