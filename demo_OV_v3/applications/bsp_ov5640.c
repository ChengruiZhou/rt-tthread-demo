


/* Includes ------------------------------------------------------------------*/
#include "bsp_ov5640.h"
#include "ov5640_scch.h"
#include "drv_ov5640_config.h"
DCMI_HandleTypeDef DCMI_Handle;
DMA_HandleTypeDef DMA_Handle_dcmi;
/** @addtogroup DCMI_Camera
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  TIMEOUT  2

ImageFormat_TypeDef ImageFormat;

uint32_t image[img_width * img_height] __attribute__((section(".SDRAM")));

/**
  * @brief  初始化控制摄像头使用的GPIO(I2C/DCMI)
  * @param  None
  * @retval None
  */
void OV5640_HW_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /***DCMI引脚配置***/
    /* 使能DCMI时钟 */
    DCMI_PWDN_GPIO_CLK_ENABLE();
    DCMI_RST_GPIO_CLK_ENABLE();
    DCMI_VSYNC_GPIO_CLK_ENABLE();
    DCMI_HSYNC_GPIO_CLK_ENABLE();
    DCMI_PIXCLK_GPIO_CLK_ENABLE();    
    DCMI_D0_GPIO_CLK_ENABLE();
    DCMI_D1_GPIO_CLK_ENABLE();
    DCMI_D2_GPIO_CLK_ENABLE();
    DCMI_D3_GPIO_CLK_ENABLE();    
    DCMI_D4_GPIO_CLK_ENABLE();
    DCMI_D5_GPIO_CLK_ENABLE();
    DCMI_D6_GPIO_CLK_ENABLE();
    DCMI_D7_GPIO_CLK_ENABLE();

    /*控制/同步信号线*/
    GPIO_InitStructure.Pin = DCMI_VSYNC_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLUP ;
    GPIO_InitStructure.Alternate = DCMI_VSYNC_AF;
    HAL_GPIO_Init(DCMI_VSYNC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_HSYNC_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_VSYNC_AF;
    HAL_GPIO_Init(DCMI_HSYNC_GPIO_PORT, &GPIO_InitStructure);


    GPIO_InitStructure.Pin = DCMI_PIXCLK_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_PIXCLK_AF;
    HAL_GPIO_Init(DCMI_PIXCLK_GPIO_PORT, &GPIO_InitStructure);

    /*数据信号*/
    GPIO_InitStructure.Pin = DCMI_D0_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D0_AF;
    HAL_GPIO_Init(DCMI_D0_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D1_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D1_AF;
    HAL_GPIO_Init(DCMI_D1_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D2_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D2_AF;
    HAL_GPIO_Init(DCMI_D2_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D3_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D3_AF;
    HAL_GPIO_Init(DCMI_D3_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D4_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D4_AF;
    HAL_GPIO_Init(DCMI_D4_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D5_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D5_AF;
    HAL_GPIO_Init(DCMI_D5_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D6_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D6_AF;
    HAL_GPIO_Init(DCMI_D6_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D7_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D7_AF;
    HAL_GPIO_Init(DCMI_D7_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_PWDN_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;    
    HAL_GPIO_Init(DCMI_PWDN_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_RST_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;    
    HAL_GPIO_Init(DCMI_RST_GPIO_PORT, &GPIO_InitStructure);

    HAL_GPIO_WritePin(DCMI_RST_GPIO_PORT,DCMI_RST_GPIO_PIN,GPIO_PIN_RESET);
    /*PWDN引脚，高电平关闭电源，低电平供电*/
    HAL_GPIO_WritePin(DCMI_PWDN_GPIO_PORT,DCMI_PWDN_GPIO_PIN,GPIO_PIN_SET);
   
    HAL_GPIO_WritePin(DCMI_PWDN_GPIO_PORT,DCMI_PWDN_GPIO_PIN,GPIO_PIN_RESET);
    rt_thread_mdelay(10);
    HAL_GPIO_WritePin(DCMI_RST_GPIO_PORT,DCMI_RST_GPIO_PIN,GPIO_PIN_SET);
    //必须延时50ms,模块才会正常工作
    rt_thread_mdelay(50);
}
/**
  * @brief  Resets the OV5640 camera.
  * @param  None
  * @retval None
  */
void OV5640_Reset(void)
{
    /*OV5640软件复位*/
  OV5640_WriteReg(0x3008, 0x80);
}

/**
  * @brief  读取摄像头的ID.
  * @param  OV5640ID: 存储ID的结构体
  * @retval None
  */
void OV5640_ReadID(OV5640_IDTypeDef *OV5640ID)
{

    /*读取寄存芯片ID*/
  OV5640ID->PIDH = OV5640_ReadReg(OV5640_SENSOR_PIDH);
  OV5640ID->PIDL = OV5640_ReadReg(OV5640_SENSOR_PIDL);
}

/**
  * @brief  配置 DCMI/DMA 以捕获摄像头数据
  * @param  None
  * @retval None
  */
void OV5640_Init(void) 
{
  /*** 配置DCMI接口 ***/
  /* 使能DCMI时钟 */
  __HAL_RCC_DCMI_CLK_ENABLE();

  /* DCMI 配置*/
  //DCMI外设寄存器基地址
  DCMI_Handle.Instance              = DCMI;    
  //连续采集模式
  DCMI_Handle.Init.SynchroMode      = DCMI_MODE_CONTINUOUS;
  //连续采集模式
  DCMI_Handle.Init.SynchroMode      = DCMI_SYNCHRO_HARDWARE;
  //像素时钟上升沿有效
  DCMI_Handle.Init.PCKPolarity      = DCMI_PCKPOLARITY_RISING;
  //VSP高电平有效
  DCMI_Handle.Init.VSPolarity       = DCMI_VSPOLARITY_HIGH;
  //HSP低电平有效
  DCMI_Handle.Init.HSPolarity       = DCMI_HSPOLARITY_LOW;
  //全采集
  DCMI_Handle.Init.CaptureRate      = DCMI_CR_ALL_FRAME;
  //8位数据宽度
  DCMI_Handle.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  HAL_DCMI_Init(&DCMI_Handle);
    
    /* 配置中断 */
  HAL_NVIC_SetPriority(DCMI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);
  //dma_memory 以16位数据为单位， dma_bufsize以32位数据为单位(即像素个数/2)
  OV5640_DMA_Config((uint32_t)image,(uint32_t)(img_width*img_height));
}


/**
  * @brief  配置 DCMI/DMA 以捕获摄像头数据
    * @param  DMA_Memory0BaseAddr:本次传输的目的首地址
  * @param DMA_BufferSize：本次传输的数据量(单位为字,即4字节)
  */
void OV5640_DMA_Config(uint32_t DMA_Memory0BaseAddr,uint32_t DMA_BufferSize)
{
  /* 配置DMA从DCMI中获取数据*/
  /* 使能DMA*/
  __HAL_RCC_DMA2_CLK_ENABLE(); 
  DMA_Handle_dcmi.Instance = DMA2_Stream1;
  DMA_Handle_dcmi.Init.Request = DMA_REQUEST_DCMI; 
  DMA_Handle_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
  DMA_Handle_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
  DMA_Handle_dcmi.Init.MemInc = DMA_MINC_ENABLE;    //寄存器地址自增
  DMA_Handle_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  DMA_Handle_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  DMA_Handle_dcmi.Init.Mode = DMA_CIRCULAR;         //循环模式
  DMA_Handle_dcmi.Init.Priority = DMA_PRIORITY_HIGH;
  DMA_Handle_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  DMA_Handle_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  DMA_Handle_dcmi.Init.MemBurst = DMA_MBURST_INC4;
  DMA_Handle_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;

  /*DMA中断配置 */
  __HAL_LINKDMA(&DCMI_Handle, DMA_Handle, DMA_Handle_dcmi);
  
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  
  HAL_DMA_Init(&DMA_Handle_dcmi);
  
  //使能DCMI采集数据
  HAL_DCMI_Start_DMA(&DCMI_Handle, DCMI_MODE_CONTINUOUS, (uint32_t)DMA_Memory0BaseAddr,DMA_BufferSize);

}


unsigned short sensor_reg[(sizeof(RGB565_Init)/4)];
/**
  * @brief  Configures the OV5640 camera in BMP mode.
  * @param  BMP ImageSize: BMP image size
  * @retval None
  */
void OV5640_RGB565Config(void)
{
  uint32_t i;
  
    /*摄像头复位*/
  OV5640_Reset();
  /* 写入寄存器配置 */
  /* Initialize OV5640   Set to output RGB565 */
  for(i=0; i<(sizeof(RGB565_Init)/4); i++)
  {
    OV5640_WriteReg(RGB565_Init[i][0], RGB565_Init[i][1]);
    rt_thread_mdelay(10);
    sensor_reg[i] = OV5640_ReadReg(RGB565_Init[i][0]);
    if(RGB565_Init[i][1] != sensor_reg[i])
      CAMERA_DEBUG("sensor_reg[0x%x]:%x-%x\n",RGB565_Init[i][0],RGB565_Init[i][1],sensor_reg[i]);

  }

//  Delay(500);
    
  if(img_width == 320)
       
    ImageFormat=BMP_320x240;
   
  else if(img_width == 640)
      
    ImageFormat=BMP_640x480;
  
  else if(img_width == 800)
      
    ImageFormat=BMP_800x480;
  
  switch(ImageFormat)
  {
    case BMP_320x240:
    {
      for(i=0; i<(sizeof(RGB565_QVGA)/4); i++)
      {
        OV5640_WriteReg(RGB565_QVGA[i][0], RGB565_QVGA[i][1]);
      }
      break;
    }
    case BMP_640x480:
    {
       for(i=0; i<(sizeof(RGB565_VGA)/4); i++)
      {
        OV5640_WriteReg(RGB565_VGA[i][0], RGB565_VGA[i][1]);
      }
      break;
    }
    case BMP_800x480:
    {
      for(i=0; i<(sizeof(RGB565_WVGA)/4); i++)
      {
        OV5640_WriteReg(RGB565_WVGA[i][0], RGB565_WVGA[i][1]);
        sensor_reg[i] = OV5640_ReadReg(RGB565_WVGA[i][0]);
        if(RGB565_WVGA[i][1] != sensor_reg[i])
            CAMERA_DEBUG("sensor_reg[0x%x]:%x-%x\n",RGB565_Init[i][0],RGB565_Init[i][1],sensor_reg[i]);
      }
      rt_kprintf("OK\r\n");
      break;
    }
    default:
    {
      for(i=0; i<(sizeof(RGB565_WVGA)/4); i++)
      {
        OV5640_WriteReg(RGB565_WVGA[i][0], RGB565_WVGA[i][1]);
      }
      break;
    }
  }
  
  OV5640_ReadReg(RGB565_WVGA[i][0]);
}

/**
  * @brief  DMA中断服务函数
  * @param  None
  * @retval None
  */
void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&DMA_Handle_dcmi);
}

/**
  * @brief  DCMI中断服务函数
  * @param  None
  * @retval None
  */
void DCMI_IRQHandler(void)
{
    SCB_InvalidateDCache_by_Addr((uint32_t*)image,(uint32_t)(img_width*img_height));
  HAL_DCMI_IRQHandler(&DCMI_Handle);
}
extern uint8_t fps;
/**
  * @brief  Line event callback.
  * @param  None
  * @retval None
  */
void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
//  SCB_InvalidateDCache_by_Addr((uint32_t*)LCD_FB_START_ADDRESS,LCD_GetXSize()*LCD_GetYSize()/2);
    fps++; //帧率计数
//    OV5640_DMA_Config(LCD_FB_START_ADDRESS,LCD_GetXSize()*LCD_GetYSize()/2);
    OV5640_DMA_Config((uint32_t)image,(uint32_t)(img_width*img_height));


}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
