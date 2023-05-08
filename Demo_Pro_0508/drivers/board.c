/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-01     RealThread   first version
 */

#include <rtthread.h>
#include <board.h>
#include <drv_common.h>


TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

static uint32_t FMC_Initialized = 0;

QSPI_HandleTypeDef hqspi;
SDRAM_HandleTypeDef hsdram1;

void MX_GPIO_Init(void)
{
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

      /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();


    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* QUADSPI init function */
void MX_QUADSPI_Init(void)
{

    /* USER CODE BEGIN QUADSPI_Init 0 */

    /* USER CODE END QUADSPI_Init 0 */

    /* USER CODE BEGIN QUADSPI_Init 1 */

    /* USER CODE END QUADSPI_Init 1 */
    hqspi.Instance = QUADSPI;
    hqspi.Init.ClockPrescaler = 1;
    hqspi.Init.FifoThreshold = 8;
    hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
    hqspi.Init.FlashSize = 50;
    hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_4_CYCLE;
    hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
    hqspi.Init.DualFlash = QSPI_DUALFLASH_ENABLE;
    if (HAL_QSPI_Init(&hqspi) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN QUADSPI_Init 2 */

    /* USER CODE END QUADSPI_Init 2 */

}
void HAL_QSPI_MspInit(QSPI_HandleTypeDef* qspiHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
     RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
     if(qspiHandle->Instance==QUADSPI)
     {
     /* USER CODE BEGIN QUADSPI_MspInit 0 */

     /* USER CODE END QUADSPI_MspInit 0 */

     /** Initializes the peripherals clock
     */
       PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
       PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
       if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
       {
         Error_Handler();
       }

       /* QUADSPI clock enable */
       __HAL_RCC_QSPI_CLK_ENABLE();

       __HAL_RCC_GPIOG_CLK_ENABLE();
       __HAL_RCC_GPIOF_CLK_ENABLE();
       __HAL_RCC_GPIOH_CLK_ENABLE();
       __HAL_RCC_GPIOB_CLK_ENABLE();
       /**QUADSPI GPIO Configuration
       PG9     ------> QUADSPI_BK2_IO2
       PG14     ------> QUADSPI_BK2_IO3
       PG6     ------> QUADSPI_BK1_NCS
       PF6     ------> QUADSPI_BK1_IO3
       PF7     ------> QUADSPI_BK1_IO2
       PF8     ------> QUADSPI_BK1_IO0
       PF9     ------> QUADSPI_BK1_IO1
       PH2     ------> QUADSPI_BK2_IO0
       PH3     ------> QUADSPI_BK2_IO1
       PB2     ------> QUADSPI_CLK
       */
       GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_14;
       GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
       GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
       HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

       GPIO_InitStruct.Pin = GPIO_PIN_6;
       GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
       GPIO_InitStruct.Pull = GPIO_PULLUP;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
       GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
       HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

       GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
       GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
       GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
       HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

       GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
       GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
       GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
       HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

       GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
       GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
       GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
       HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

       GPIO_InitStruct.Pin = GPIO_PIN_2;
       GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
       GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
       HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

     /* USER CODE BEGIN QUADSPI_MspInit 1 */

     /* USER CODE END QUADSPI_MspInit 1 */
     }
}
INIT_BOARD_EXPORT(MX_QUADSPI_Init);
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

      /** Supply configuration update enable
      */
      HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

      /** Configure the main internal regulator output voltage
      */
      __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

      while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

      /** Initializes the RCC Oscillators according to the specified parameters
      * in the RCC_OscInitTypeDef structure.
      */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
      RCC_OscInitStruct.HSEState = RCC_HSE_ON;
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
      RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
      RCC_OscInitStruct.PLL.PLLM = 6;
      RCC_OscInitStruct.PLL.PLLN = 192;
      RCC_OscInitStruct.PLL.PLLP = 2;
      RCC_OscInitStruct.PLL.PLLQ = 2;
      RCC_OscInitStruct.PLL.PLLR = 2;
      RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
      RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
      RCC_OscInitStruct.PLL.PLLFRACN = 0;
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      {
        Error_Handler();
      }

      /** Initializes the CPU, AHB and APB buses clocks
      */
      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                  |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
      RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
      RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
      RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
      RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
      {
        Error_Handler();
      }
}


/* FMC initialization function */
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

void MX_FMC_Init(void)
{
    /* USER CODE BEGIN FMC_Init 0 */

    /* USER CODE END FMC_Init 0 */

    FMC_SDRAM_TimingTypeDef SdramTiming = {0};

    /* USER CODE BEGIN FMC_Init 1 */

    /* USER CODE END FMC_Init 1 */

    /** Perform the SDRAM1 memory initialization sequence
    */
    hsdram1.Instance = FMC_SDRAM_DEVICE;
    /* hsdram1.Init */
    hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
    hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
    hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
    hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
    hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
    hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
    hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
    hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_3;
    hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
    hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
    /* SdramTiming */
    SdramTiming.LoadToActiveDelay = 2;
    SdramTiming.ExitSelfRefreshDelay = 8;
    SdramTiming.SelfRefreshTime = 6;
    SdramTiming.RowCycleDelay = 7;
    SdramTiming.WriteRecoveryTime = 8;
    SdramTiming.RPDelay = 2;
    SdramTiming.RCDDelay = 2;
  //  rt_kprintf("HAL_SDRAM_Init s");
    if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
    {
       // rt_kprintf("HAL_SDRAM_Initerrpr ");
      Error_Handler( );
    }

    /* USER CODE BEGIN FMC_Init 2 */

    /* USER CODE END FMC_Init 2 */
}

void HAL_FMC_MspInit(void){

    /* USER CODE BEGIN FMC_MspInit 0 */

      /* USER CODE END FMC_MspInit 0 */
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      if (FMC_Initialized) {
        return;
      }
      FMC_Initialized = 1;
      RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

      /** Initializes the peripherals clock
      */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FMC;
        PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
          Error_Handler();
        }

      /* Peripheral clock enable */
      __HAL_RCC_FMC_CLK_ENABLE();

      /** FMC GPIO Configuration
      PI6   ------> FMC_D28
      PI5   ------> FMC_NBL3
      PI4   ------> FMC_NBL2
      PI1   ------> FMC_D25
      PI0   ------> FMC_D24
      PI7   ------> FMC_D29
      PE1   ------> FMC_NBL1
      PI2   ------> FMC_D26
      PH15   ------> FMC_D23
      PH14   ------> FMC_D22
      PE0   ------> FMC_NBL0
      PI3   ------> FMC_D27
      PG15   ------> FMC_SDNCAS
      PD0   ------> FMC_D2
      PH13   ------> FMC_D21
      PI9   ------> FMC_D30
      PD1   ------> FMC_D3
      PI10   ------> FMC_D31
      PG8   ------> FMC_SDCLK
      PF2   ------> FMC_A2
      PF1   ------> FMC_A1
      PF0   ------> FMC_A0
      PG5   ------> FMC_BA1
      PF3   ------> FMC_A3
      PG4   ------> FMC_BA0
      PG2   ------> FMC_A12
      PF5   ------> FMC_A5
      PF4   ------> FMC_A4
      PC0   ------> FMC_SDNWE
      PE10   ------> FMC_D7
      PF13   ------> FMC_A7
      PF14   ------> FMC_A8
      PE9   ------> FMC_D6
      PE11   ------> FMC_D8
      PH10   ------> FMC_D18
      PH11   ------> FMC_D19
      PD15   ------> FMC_D1
      PD14   ------> FMC_D0
      PF12   ------> FMC_A6
      PF15   ------> FMC_A9
      PE12   ------> FMC_D9
      PE15   ------> FMC_D12
      PH9   ------> FMC_D17
      PH12   ------> FMC_D20
      PF11   ------> FMC_SDNRAS
      PG0   ------> FMC_A10
      PE8   ------> FMC_D5
      PE13   ------> FMC_D10
      PH6   ------> FMC_SDNE1
      PH8   ------> FMC_D16
      PD10   ------> FMC_D15
      PD9   ------> FMC_D14
      PG1   ------> FMC_A11
      PE7   ------> FMC_D4
      PE14   ------> FMC_D11
      PH7   ------> FMC_SDCKE1
      PD8   ------> FMC_D13
      */
      /* GPIO_InitStruct */
      GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_1
                              |GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_2|GPIO_PIN_3
                              |GPIO_PIN_9|GPIO_PIN_10;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

      HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

      /* GPIO_InitStruct */
      GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_9
                              |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_8
                              |GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_14;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

      HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

      /* GPIO_InitStruct */
      GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_10
                              |GPIO_PIN_11|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_6
                              |GPIO_PIN_8|GPIO_PIN_7;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

      HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

      /* GPIO_InitStruct */
      GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_5|GPIO_PIN_4
                              |GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_1;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

      HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

      /* GPIO_InitStruct */
      GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_15|GPIO_PIN_14
                              |GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_8;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

      /* GPIO_InitStruct */
      GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_3
                              |GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_13|GPIO_PIN_14
                              |GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_11;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

      HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

      /* GPIO_InitStruct */
      GPIO_InitStruct.Pin = GPIO_PIN_0;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

      /* USER CODE BEGIN FMC_MspInit 1 */

      /* USER CODE END FMC_MspInit 1 */
  }


void HAL_SDRAM_MspInit(SDRAM_HandleTypeDef* sdramHandle){
  /* USER CODE BEGIN SDRAM_MspInit 0 */

  /* USER CODE END SDRAM_MspInit 0 */
  HAL_FMC_MspInit();
  /* USER CODE BEGIN SDRAM_MspInit 1 */

  /* USER CODE END SDRAM_MspInit 1 */
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
  htim4.Init.Prescaler = 200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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

void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 200;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_encoderHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
  else if(tim_encoderHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM5 GPIO Configuration
    PA1     ------> TIM5_CH2
    PA0     ------> TIM5_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder)
{
  if(htim_encoder->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
  else if(htim_encoder->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /**TIM5 GPIO Configuration
    PA1     ------> TIM5_CH2
    PA0     ------> TIM5_CH1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_0);

  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }

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

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */



RT_WEAK void rt_hw_board_init()
{
    extern void hw_board_init(char *clock_src, int32_t clock_src_freq, int32_t clock_target_freq);

    /* Heap initialization */
#if defined(RT_USING_HEAP)
    rt_system_heap_init((void *) HEAP_BEGIN, (void *) HEAP_END);
#endif

    hw_board_init(BSP_CLOCK_SOURCE, BSP_CLOCK_SOURCE_FREQ_MHZ, BSP_CLOCK_SYSTEM_FREQ_MHZ);

    /* Set the shell console output device */
#if defined(RT_USING_DEVICE) && defined(RT_USING_CONSOLE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

    /* Board underlying hardware initialization */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

}
