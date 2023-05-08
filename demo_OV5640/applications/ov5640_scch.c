/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-08     Mr.zhou       the first version
 */

#include <rtthread.h>
#include <board.h>
#include <drv_common.h>
#include "ov5640_scch.h"
//#include <stm32h7xx.h>

I2C_HandleTypeDef hi2c1;

/*
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C0ECFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}


/*
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB9     ------> I2C1_SDA
    PB8     ------> I2C1_SCL
    */
    GPIO_InitStruct.Pin = DCMI_SDA_Pin|DCMI_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

}

/*
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB9     ------> I2C1_SDA
    PB8     ------> I2C1_SCL
    */
    HAL_GPIO_DeInit(DCMI_SDA_GPIO_Port, DCMI_SDA_Pin);

    HAL_GPIO_DeInit(DCMI_SCL_GPIO_Port, DCMI_SCL_Pin);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }

}
/*
* @brief  写一字节数据到OV2640寄存器
* @param  Addr: OV2640 的寄存器地址
* @param  Data: 要写入的数据
* @retval 返回0表示写入正常，0xFF表示错误
*/
uint8_t OV5640_WriteReg(uint16_t Addr, uint8_t Data)
{
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_I2C_Mem_Write(&hi2c1, OV5640_DEVICE_ADDRESS, (uint16_t)Addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&Data, 1, 1000);

    if (status != HAL_OK)
    {
        Error_Handler();
    }

    return status;
}

/*
* @brief  从OV2640寄存器中读取一个字节的数据
* @param  Addr: 寄存器地址
* @retval 返回读取得的数据
*/
uint8_t OV5640_ReadReg(uint16_t Addr)
{
    uint8_t Data = 0;

    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_I2C_Mem_Read(&hi2c1, OV5640_DEVICE_ADDRESS, (uint16_t)Addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&Data, 1, 1000);

    /* Check the communication status */
    if(status != HAL_OK)
    {
      Error_Handler();
    }
    /* return the read data */
    return Data;
}

/*
* @brief  将固件写入到OV5640片内MCU
* @param  Addr: OV5640 的MCU基地址0x8000
* @param  Data: 要写入的数据
* @retval 返回0表示写入正常，0xFF表示错误
*/
uint8_t OV5640_WriteFW(uint8_t *pBuffer ,uint16_t BufferSize)
{
  uint16_t Addr=0x8000;
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&hi2c1, OV5640_DEVICE_ADDRESS, (uint16_t)Addr, I2C_MEMADD_SIZE_16BIT, pBuffer, BufferSize, 1000);

  /* 检查通信状态 */
  if(status != HAL_OK)
  {
      Error_Handler();
  }
  return status;
}

