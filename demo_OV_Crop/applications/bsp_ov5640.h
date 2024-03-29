#ifndef __OV5640_H
#define __OV5640_H

/**
  ******************************************************************************
  * @file    DCMI/Camera/dcmi_ov5640.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Header for dcmi_ov5640.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DCMI_OV5640_H
#define __DCMI_OV5640_H
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"

#include "stm32h7xx_hal.h"

extern DCMI_HandleTypeDef DCMI_Handle;

#define CAMERA_INSTANCES_NBR           1U

#define CAMERA_MODE_CONTINUOUS         DCMI_MODE_CONTINUOUS
#define CAMERA_MODE_SNAPSHOT           DCMI_MODE_SNAPSHOT

#define BSP_ERROR_NONE                    0
#define BSP_ERROR_NO_INIT                -1
#define BSP_ERROR_WRONG_PARAM            -2
#define BSP_ERROR_BUSY                   -3
#define BSP_ERROR_PERIPH_FAILURE         -4
#define BSP_ERROR_COMPONENT_FAILURE      -5
#define BSP_ERROR_UNKNOWN_FAILURE        -6
#define BSP_ERROR_UNKNOWN_COMPONENT      -7
#define BSP_ERROR_BUS_FAILURE            -8
#define BSP_ERROR_CLOCK_FAILURE          -9
#define BSP_ERROR_MSP_FAILURE            -10
#define BSP_ERROR_FEATURE_NOT_SUPPORTED  -11

/*摄像头采集图像的大小，改变这两个值可以改变数据量，
但不会加快采集速度，要加快采集速度需要改成SVGA模式*/
#define img_width  320
#define img_height 240

//uint32_t image[img_width * img_height] __attribute__((section(".SDRAM")));

/* Exported constants --------------------------------------------------------*/
#define DCMI_DR_ADDRESS       0x50050028


/* Image Sizes enumeration */
typedef enum   
{
  BMP_320x240          =   0x00,      /* BMP Image 320x240 Size */
  BMP_352x288          =   0x01,      /* BMP Image 352x288 Size */
  BMP_640x480          =   0x02,      /* BMP Image 640x480 Size */
  BMP_800x480          =   0x03,      /* BMP Image 800x480 Size */
}ImageFormat_TypeDef;


/*摄像头接口 */
//IIC SCCB
#define CAMERA_I2C                          I2C1
#define CAMERA_I2C_CLK_ENABLE()             __HAL_RCC_I2C1_CLK_ENABLE()

#define CAMERA_I2C_SCL_PIN                  GPIO_PIN_8
#define CAMERA_I2C_SCL_GPIO_PORT            GPIOB
#define CAMERA_I2C_SCL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define CAMERA_I2C_SCL_AF                   GPIO_AF4_I2C1

#define CAMERA_I2C_SDA_PIN                  GPIO_PIN_9
#define CAMERA_I2C_SDA_GPIO_PORT            GPIOB
#define CAMERA_I2C_SDA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define CAMERA_I2C_SDA_AF                   GPIO_AF4_I2C1

//VSYNC
#define DCMI_VSYNC_GPIO_PORT                GPIOB
#define DCMI_VSYNC_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define DCMI_VSYNC_GPIO_PIN                 GPIO_PIN_7
#define DCMI_VSYNC_AF                           GPIO_AF13_DCMI
// HSYNC
#define DCMI_HSYNC_GPIO_PORT                GPIOA
#define DCMI_HSYNC_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define DCMI_HSYNC_GPIO_PIN                 GPIO_PIN_4
#define DCMI_HSYNC_AF                           GPIO_AF13_DCMI
//PIXCLK
#define DCMI_PIXCLK_GPIO_PORT               GPIOA
#define DCMI_PIXCLK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define DCMI_PIXCLK_GPIO_PIN                GPIO_PIN_6
#define DCMI_PIXCLK_AF                        GPIO_AF13_DCMI
//PWDN
#define DCMI_PWDN_GPIO_PORT               GPIOB
#define DCMI_PWDN_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define DCMI_PWDN_GPIO_PIN                GPIO_PIN_10

//RST
#define DCMI_RST_GPIO_PORT                GPIOF
#define DCMI_RST_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOF_CLK_ENABLE()
#define DCMI_RST_GPIO_PIN                 GPIO_PIN_10

//数据信号线
#define DCMI_D0_GPIO_PORT                   GPIOC
#define DCMI_D0_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define DCMI_D0_GPIO_PIN                    GPIO_PIN_6
#define DCMI_D0_AF                            GPIO_AF13_DCMI

#define DCMI_D1_GPIO_PORT                   GPIOC
#define DCMI_D1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define DCMI_D1_GPIO_PIN                    GPIO_PIN_7
#define DCMI_D1_AF                            GPIO_AF13_DCMI

#define DCMI_D2_GPIO_PORT                   GPIOG
#define DCMI_D2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOG_CLK_ENABLE()
#define DCMI_D2_GPIO_PIN                    GPIO_PIN_10
#define DCMI_D2_AF                            GPIO_AF13_DCMI

#define DCMI_D3_GPIO_PORT                   GPIOG
#define DCMI_D3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOG_CLK_ENABLE()
#define DCMI_D3_GPIO_PIN                    GPIO_PIN_11
#define DCMI_D3_AF                            GPIO_AF13_DCMI

#define DCMI_D4_GPIO_PORT                   GPIOE
#define DCMI_D4_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define DCMI_D4_GPIO_PIN                    GPIO_PIN_4
#define DCMI_D4_AF                            GPIO_AF13_DCMI

#define DCMI_D5_GPIO_PORT                   GPIOD
#define DCMI_D5_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOD_CLK_ENABLE()
#define DCMI_D5_GPIO_PIN                    GPIO_PIN_3
#define DCMI_D5_AF                            GPIO_AF13_DCMI

#define DCMI_D6_GPIO_PORT                   GPIOE
#define DCMI_D6_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOE_CLK_ENABLE()
#define DCMI_D6_GPIO_PIN                    GPIO_PIN_5
#define DCMI_D6_AF                            GPIO_AF13_DCMI

#define DCMI_D7_GPIO_PORT                   GPIOE
#define DCMI_D7_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOE_CLK_ENABLE()
#define DCMI_D7_GPIO_PIN                    GPIO_PIN_6
#define DCMI_D7_AF                            GPIO_AF13_DCMI


/*debug*/

#define CAMERA_DEBUG_ON          1
#define CAMERA_DEBUG_ARRAY_ON   1
#define CAMERA_DEBUG_FUNC_ON    1
   
   
// Log define
#define CAMERA_INFO(fmt,arg...)           rt_kprintf("<<-CAMERA-INFO->> "fmt"\n",##arg)
#define CAMERA_ERROR(fmt,arg...)          rt_kprintf("<<-CAMERA-ERROR->> "fmt"\n",##arg)
#define CAMERA_DEBUG(fmt,arg...)          do{\
                                         if(CAMERA_DEBUG_ON)\
                                         rt_kprintf("<<-CAMERA-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)
#define CAMERA_DEBUG_ARRAY(array, num)    do{\
                                         int32_t i;\
                                         uint8_t* a = array;\
                                         if(CAMERA_DEBUG_ARRAY_ON)\
                                         {\
                                             rt_kprintf("<<-CAMERA-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                rt_kprintf("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    rt_kprintf("\n");\
                                                }\
                                            }\
                                            rt_kprintf("\n");\
                                        }\
                                       }while(0)
#define CAMERA_DEBUG_FUNC()               do{\
                                         if(CAMERA_DEBUG_FUNC_ON)\
                                         rt_kprintf("<<-CAMERA-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)


/* Exported types ------------------------------------------------------------*/
//存储摄像头ID的结构体
typedef struct
{
  uint8_t PIDH;
  uint8_t PIDL;
}OV5640_IDTypeDef;


#define OV5640_DEVICE_WRITE_ADDRESS    0x78
#define OV5640_DEVICE_READ_ADDRESS     0x79

#define OV5640_SENSOR_PIDH       0x300A
#define OV5640_SENSOR_PIDL       0x300B

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void OV5640_HW_Init(void);
void OV5640_I2C_Init(void);
void OV5640_Reset(void);
void OV5640_ReadID(OV5640_IDTypeDef *OV5640ID);
void OV5640_Init(void);
void OV5640_QQVGAConfig(void);
void OV5640_WVGAConfig(void);
void OV5640_RGB565Config(void);

uint8_t OV5640_WriteReg(uint16_t Addr, uint8_t Data);
uint8_t OV5640_ReadReg(uint16_t Addr);

uint8_t OV5640_ImageWin_Set(uint16_t offx,uint16_t offy,uint16_t width,uint16_t height);
uint8_t OV5640_OutSize_Set(uint16_t offx,uint16_t offy,uint16_t width,uint16_t height);

void OV5640_DMA_Config(void);


int32_t BSP_CAMERA_Suspend(uint32_t Instance);
int32_t BSP_CAMERA_Stop(uint32_t Instance);
int32_t BSP_CAMERA_Start(uint32_t Instance, uint8_t *pBff, uint32_t Mode);
int32_t BSP_CAMERA_Resume(uint32_t Instance);

void I2Cx_Error(I2C_HandleTypeDef *i2c_handler);
#endif /* __DCMI_OV5640_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#endif /* __LED_H */
