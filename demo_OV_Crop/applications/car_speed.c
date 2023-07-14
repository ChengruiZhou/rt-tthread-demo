#include <rtthread.h>
#include <board.h>
#include <drv_common.h>
#include <stm32h7xx.h>

#include <rtdevice.h>
//#include "drv_common.h"
#include "car_speed.h"

#include <stdlib.h>

#define DBG_TAG "car_speed"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define PWM_DEV_NAME        "pwm3"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL_L     3       /* PWM通道 */
#define PWM_DEV_CHANNEL_R     4       /* PWM通道 */
struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */


TIM_HandleTypeDef htim3;

rt_uint32_t T = 500000;  /* 2KHz */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB0     ------> TIM3_CH3
    PB1     ------> TIM3_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 200;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}



void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}




int hw_car_speed_init(void)
{
    MX_TIM3_Init();
    rt_pin_mode(L_engine_direction1, PIN_MODE_OUTPUT);
    rt_pin_mode(L_engine_direction2, PIN_MODE_OUTPUT);
    rt_pin_mode(R_engine_direction1, PIN_MODE_OUTPUT);
    rt_pin_mode(R_engine_direction2, PIN_MODE_OUTPUT);
    /* 查找设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);

    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm device %s not found!\n", PWM_DEV_NAME);
        return -RT_ERROR;
    }

    LOG_D("hw_car_speed_init is ok!");
    return RT_EOK;
}

/*
 * 1 向前
 * 0 停止
 * -1向后
 */
void car_dir_control(rt_int8_t L_dwheel , rt_int8_t R_dwheel)
{
    switch (L_dwheel)
    {
        case (-1) : rt_pin_write(L_engine_direction1, PIN_LOW); rt_pin_write(L_engine_direction2, PIN_HIGH); break;
        case (0) : rt_pin_write(L_engine_direction1, PIN_LOW); rt_pin_write(L_engine_direction2, PIN_LOW); break;
        case (1): rt_pin_write(L_engine_direction1, PIN_HIGH); rt_pin_write(L_engine_direction2, PIN_LOW); break;
        default : break;
    }
    switch (R_dwheel)
    {
        case (-1) : rt_pin_write(R_engine_direction1, PIN_LOW); rt_pin_write(R_engine_direction2, PIN_HIGH); break;
        case (0) : rt_pin_write(R_engine_direction1, PIN_LOW); rt_pin_write(R_engine_direction2, PIN_LOW); break;
        case (1) : rt_pin_write(R_engine_direction1, PIN_HIGH); rt_pin_write(R_engine_direction2, PIN_LOW); break;
        default : break;
    }
}
/*
 * pulse_CH3 --> PB0  pulse_CH4 --> PB1
 * 0 -- 500000范围
 * 2khz
 */
void car_speed_control(rt_int32_t pulse_CH3,rt_int32_t pulse_CH4){

    if(pulse_CH3 > 0 && pulse_CH4 > 0){
        car_dir_control(1, 1);
    }else if (pulse_CH3 > 0 && pulse_CH4 < 0) {
        car_dir_control(1, -1);
    }else if (pulse_CH3 < 0 && pulse_CH4 > 0) {
        car_dir_control(-1, 1);
    }else if (pulse_CH3 < 0 && pulse_CH4 < 0) {
        car_dir_control(-1, -1);
    }else if (pulse_CH3 == 0 && pulse_CH4 < 0) {
        car_dir_control(0, -1);
    }else if (pulse_CH3 == 0 && pulse_CH4 > 0) {
        car_dir_control(0, 1);
    }else if (pulse_CH3 < 0 && pulse_CH4 == 0) {
        car_dir_control(1, 0);
    }else if (pulse_CH3 < 0 && pulse_CH4 == 0) {
        car_dir_control(-1, 0);
    }else{
        car_dir_control(0, 0);
    }

    pulse_CH3 = abs(pulse_CH3);
    pulse_CH4 = abs(pulse_CH4);
    /* 设置PWM周期和脉冲宽度 */
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_L, T, pulse_CH3);
    /* 使能设备 */
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL_L);

    /* 设置PWM周期和脉冲宽度 */
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_R, T, pulse_CH4);
    /* 使能设备 */
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL_R);
}

