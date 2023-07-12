/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-08     Mr.zhou       the first version
 */

#define I2C_OWN_ADDRESS           0x00
#define OV5640_DEVICE_ADDRESS     0x78

#define DCMI_SDA_Pin GPIO_PIN_9
#define DCMI_SDA_GPIO_Port GPIOB
#define DCMI_SCL_Pin GPIO_PIN_8
#define DCMI_SCL_GPIO_Port GPIOB

void MX_I2C1_Init(void);
uint8_t OV5640_WriteReg(uint16_t Addr, uint8_t Data);
uint8_t OV5640_ReadReg(uint16_t Addr);
uint8_t OV5640_WriteFW(uint8_t *pBuffer ,uint16_t BufferSize);
