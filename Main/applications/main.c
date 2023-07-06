/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-01     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <board.h>
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "move.h"
#include "drv_qspi.h"
#include "drv_spi.h"
#include "fal.h"
#include <inv_mpu.h>
#include <drv_common.h>
//#include "w25qxx.h"

#define QSPI_BUS_NAME "qspi1"
#define QSPI_DEVICE_NAME "qspi10"
#define QSPI_FLASH_NAME "W25Q256F"
#define QSPI_FLASH_PIN GET_PIN(G,6)

/**************************************************************************/

/**************************************************************************/




/* 导出到自动初始化 */
void spi_flash_W25Q256FV_init(void){
    rt_uint8_t ret = 0;
    rt_thread_mdelay(1000);/*等待QSPI设备启用*/
    stm32_qspi_bus_attach_device(QSPI_BUS_NAME, QSPI_DEVICE_NAME, QSPI_FLASH_PIN, 8, RT_NULL, RT_NULL);
    if(!ret ){
            if(rt_sfud_flash_probe(QSPI_FLASH_NAME, QSPI_DEVICE_NAME)==RT_NULL){
                    rt_kprintf("spi flash sfud init error\n");
            }
    }
    fal_init();
}
INIT_COMPONENT_EXPORT(spi_flash_W25Q256FV_init);

//#define VECT_TAB_OFFSET      0x00000000UL
//#define APPLICATION_ADDRESS  (uint32_t)0x90000000

//typedef void (*pFunction)(void);
//pFunction JumpToApplication;





int main(void)
{
    rt_thread_mdelay(3000);/*dmp 加载等待时长*/
   // dmp_int_init();
   // rt_thread_mdelay(3000);
    engine_init();
    dmp_set_enable(0);//dm初始化完成后，一定先关闭dmp。

//    car_walk_direction = 1;
//    car_walk_thread_init();
//
// rt_thread_mdelay(20000);
// car_walk_flag = 0;
//
// rt_thread_mdelay(4000);
// car_walk_flag = 1;
// car_walk_thread_init();
// rt_thread_mdelay(10000);
// car_walk_flag = 0;


   while(1){
       car_pwm_set(10000,50);
        rt_thread_mdelay(3000);
//        l=Read_Encoder(2);
       // rt_thread_mdelay(500);
//        r=Read_Encoder(4);
        rt_kprintf(" ");
    }
    return RT_EOK;
}
