#ifndef __OV5640_AF_H
#define	__OV5640_AF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "bsp_ov5640.h"
//#define Delay(ms)  Delay_ms(ms)

void OV5640_AUTO_FOCUS(void);
uint8_t OV5640_Focus_Single(void);
#endif /* __OV5640_AF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

