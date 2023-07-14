/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-12     Mr.zhou       the first version
 */


#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_TAG "encoder"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "Encoder.h"
#include "car_speed.h"
/*
 * TIM2
 *       PA15 (JTDI)     ------> TIM2_CH1
 *       PB3 (JTDO/TRACESWO)     ------> TIM2_CH2
 * TIM4
 *       PB6     ------> TIM4_CH1
 *       PD13     ------> TIM4_CH2
 */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

#define L_ENCODE_DEV    "pulse2"    /* 脉冲编码器名称 */
#define R_ENCODE_DEV    "pulse4"    /* 脉冲编码器名称 */

rt_device_t L_encode;   /* 左脉冲编码器设备句柄 */
rt_device_t R_encode;   /* 右脉冲编码器设备句柄 */



void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(tim_encoderHandle->Instance==TIM2)
    {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
      /* TIM2 clock enable */
      __HAL_RCC_TIM2_CLK_ENABLE();

      __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();
      /**TIM2 GPIO Configuration
      PA15 (JTDI)     ------> TIM2_CH1
      PB3 (JTDO/TRACESWO)     ------> TIM2_CH2
      */
      GPIO_InitStruct.Pin = GPIO_PIN_15;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = GPIO_PIN_3;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
    }
    else if(tim_encoderHandle->Instance==TIM4)
    {
        /* USER CODE BEGIN TIM4_MspInit 0 */

        /* USER CODE END TIM4_MspInit 0 */
          /* TIM4 clock enable */
          __HAL_RCC_TIM4_CLK_ENABLE();

          __HAL_RCC_GPIOB_CLK_ENABLE();
          __HAL_RCC_GPIOD_CLK_ENABLE();
          /**TIM4 GPIO Configuration
          PB6     ------> TIM4_CH1
          PD13     ------> TIM4_CH2
          */
          GPIO_InitStruct.Pin = GPIO_PIN_6;
          GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull = GPIO_NOPULL;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
          GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
          HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

          GPIO_InitStruct.Pin = GPIO_PIN_13;
          GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull = GPIO_NOPULL;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
          GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
          HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM4_MspInit 1 */

        /* USER CODE END TIM4_MspInit 1 */
    }
}


int Encoder_init(void){
    MX_TIM2_Init();
    MX_TIM4_Init();

    /* 查找脉冲编码器设备 */
    L_encode = rt_device_find(L_ENCODE_DEV);
    R_encode = rt_device_find(R_ENCODE_DEV);

    if (L_encode == RT_NULL||R_encode == RT_NULL)
    {
        if(L_encode == RT_NULL){
            LOG_D("L_encode not find! ");
        }
        else {
            LOG_D("R_encode not find! ");
        }
        return RT_ERROR;
    }

    /* 以只读方式打开设备 */
    rt_device_open(L_encode, RT_DEVICE_OFLAG_RDONLY);
    rt_device_open(R_encode, RT_DEVICE_OFLAG_RDONLY);
    LOG_D("Encode_init_successful!");

    return RT_EOK;
}


int Read_Encoder(rt_int8_t encoder)
{
    int speed=0;
    switch (encoder){
        case 2:     rt_device_open(L_encode, RT_DEVICE_OFLAG_RDONLY);
                    rt_device_control(L_encode, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
                    rt_thread_mdelay(ENCODE_TIME_DT);
                    rt_device_read(L_encode, 0, &speed, 1);
                    rt_device_close(L_encode);
                    break;

        case 4:     rt_device_open(R_encode, RT_DEVICE_OFLAG_RDONLY);
                    rt_device_control(R_encode, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
                    rt_thread_mdelay(ENCODE_TIME_DT);
                    rt_device_read(R_encode, 0, &speed, 1);
                    rt_device_close(R_encode);
                    break;
        default :break;
    }
    return speed;
}
void speed_thread_entry(void)//PID控制
//void speed_thread_entry(void *parameter)//PID控制
{
    int encoder_L = 0;
    int encoder_R = 0;

    while (1)
    {
        encoder_L = -Read_Encoder(2);
        encoder_R = Read_Encoder(4);
        rt_kprintf("encoder_R = %d \r\n encoder_L = %d\r\n",encoder_R,encoder_L);
        rt_thread_mdelay(20);

    }
}


