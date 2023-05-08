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
#define QSPI_BUS_NAME "qspi1"
#define QSPI_DEVICE_NAME "qspi10"
#define QSPI_FLASH_NAME "W25Q256FV"
#define QSPI_FLASH_PIN GET_PIN(G,6)

/**************************************************************************/




/* 导出到自动初始化 */
//INIT_COMPONENT_EXPORT(rt_hw_spi_flash_init);

void spi_flash_W25Q256FV_sfud_init(void){
    rt_uint8_t ret = 0;
    rt_thread_mdelay(1000);
    stm32_qspi_bus_attach_device(QSPI_BUS_NAME, QSPI_DEVICE_NAME, QSPI_FLASH_PIN, 8, RT_NULL, RT_NULL);
    if(!ret ){
            if(rt_sfud_flash_probe(QSPI_FLASH_NAME, QSPI_DEVICE_NAME)==RT_NULL){
                    rt_kprintf("spi flash sfud init error\n");
            }
    }
}

int main(void)
{   uint8_t data = 0;
    //MX_FMC_Init();
  // rt_pin_mode(PINX, PIN_MODE_OUTPUT);
/*engine_encode_init();
engine_init();*/
int speed ;

    MX_TIM5_Init();
    int ret = 0;
    /* 查找脉冲编码器设备 */
    L_encode = rt_device_find(L_ENCODE_DEV);
    R_encode = rt_device_find("pulse5");
    if (L_encode == RT_NULL||R_encode == RT_NULL)
    {
        if(L_encode == RT_NULL){
            rt_kprintf("L_encode not find! \n");
        }
        else {
            rt_kprintf("R_encode not find! \n");
        }
        return RT_ERROR;
    }
    //rt_device_open(R_encode, RT_DEVICE_OFLAG_RDONLY);
    rt_kprintf("rt_device_open %d\n",rt_device_open(R_encode, RT_DEVICE_OFLAG_RDONLY));
    rt_kprintf("rt_device_control %d\n", rt_device_control(R_encode, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL));
    rt_thread_mdelay(ENCODE_TIME_DT);
    rt_device_read(R_encode, 0, &speed, 1);
    //rt_device_close(R_encode);
    rt_kprintf("Encode_init_successful!\n");
//engine_pwm_dev = (struct rt_device_pwm *)rt_device_find(ENGINE_PWM_DEV_NAME);

  //  rt_pin_write(PINX, PIN_HIGH);
  //  spi_flash_W25Q256FV_sfud_init();
    //rt_thread_mdelay(200);
   // fal_init();
//stm32_qspi_bus_attach_device( *bus_name, *device_name, rt_uint32_t pin,  data_line_width, void (*enter_qspi_mode)(), void (*exit_qspi_mode)

  // ret= stm32_qspi_bus_attach_device(QSPI_BUS_NAME, QSPI_DEVICE_NAME2, QSPI_FLASH_PIN, 4, RT_NULL, RT_NULL);
    int i =0;
    char *ax;
    //car_speed_conrtorl(1,500);

 while (1){

    // rt_device_control(R_encode, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
     rt_thread_mdelay(ENCODE_TIME_DT);
     rt_kprintf("rt_device_read %d\n",rt_device_read(R_encode, 0, &speed, 1));
     rt_kprintf("%d\n",speed);

 }
//
   /* char *ax2 = rt_malloc(1900400);
    //HAL_FMC_MspInit();
    rt_thread_mdelay(6000);
    rt_free(ax);
    rt_free(ax2);
    rt_kprintf("free\n");*/
    //rt_free(&ax);
  /*  for (i = 0; i < 1600 /16; i++){
        data = *(__IO uint8_t *)(((uint32_t)0XC0000000) + i * 16);
        rt_kprintf("data %d !\n", data);
    }*/
   // sdram_test();
   // rt_thread_mdelay(1000);
    //engine_init();
    //i2c_bus = (struct mpu6xxx_device *)mpu6xxx_init("i2c1", MPU6050_ADDR);   //初始化MPU6050，测量单位为角速度，加速度    while(count++)
   // engine_encode_init();
   // i2c1_bus = (struct rt_i2c_bus_device *)rt_device_find(I2C1_BUS_NAME);
 /*   if (i2c1_bus == RT_NULL)
    {
        rt_kprintf("can't find %lx device!\n", I2C1_BUS_NAME);
    }*/
  // motion_init();
  // motion_loop(drive_pitch,drive_roll,drive_yaw);
  // rt_thread_mdelay(1000);
  // rt_pin_irq_enable(MPU6050_INT_PIN, PIN_IRQ_DISABLE);
 //  rt_thread_mdelay(2000);
  //  char cx=rt_malloc (1024);
      /* while(1){
           rt_thread_mdelay(800);
           *(__IO uint16_t *)(((uint32_t)0XD0000300) + 6 * 16) = (uint16_t)(6 % 1000);
       }
*/
   // MPU6050_initialize();
  //  rt_kprintf("MPU6XXX_SLEEP return %d\n", mpu6xxx_set_param(mpu6050_dev1, MPU6XXX_SLEEP, MPU6XXX_SLEEP_DISABLE   ));
  //  rt_kprintf("MPU6XXX_GYRO_RANGE return %d\n", mpu6xxx_set_param(mpu6050_dev1, MPU6XXX_GYRO_RANGE, MPU6XXX_GYRO_RANGE_500DPS  ));
  //  rt_kprintf("MPU6XXX_ACCEL_RANGE return %d\n", mpu6xxx_set_param(mpu6050_dev1, MPU6XXX_ACCEL_RANGE, MPU6XXX_ACCEL_RANGE_16G  ));
   // MPU6050_setSleepEnabled(0);
   //  MPU6050_setI2CMasterModeEnabled(0);
   //  MPU6050_setI2CBypassEnabled(0);
    //rt_kprintf("MPU6XXX_DLPF_CONFIG  return %d\n", mpu6xxx_set_param(mpu6050_dev1, MPU6XXX_DLPF_CONFIG, MPU6XXX_DLPF_42HZ    ));
    //("MPU6XXX_SAMPLE_RATE return %d\n", mpu6xxx_set_param(mpu6050_dev1, MPU6XXX_SAMPLE_RATE, 100));
   // rt_kprintf("MPU6XXX_SAMPLE_RATE return %d\n", mpu6xxx_get_gyro(mpu6050_dev, axes3data));

    /*rt_kprintf("mpu_init return %d\n", mpu_init());
    rt_kprintf("mmpu_set_sensors return %d\n", mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
    rt_kprintf("mpu_configure_fifo return %d\n", mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL));
    rt_kprintf("dmp_load_motion_driver_firmware() return %d\n", dmp_load_motion_driver_firmware());
    rt_kprintf("dmp_set_orientation return %d\n", dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)));
    rt_kprintf("dmp_enable_feature return %d\n", dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL));
    rt_kprintf("dmp_set_fifo_rate return %d\n", dmp_set_fifo_rate(DEFAULT_MPU_HZ));
    run_self_test();
    rt_kprintf("mpu_set_dmp_state return %d\n", mpu_set_dmp_state(1));*/
       //uint16_t testsram[250000] __attribute__((at(0XD0000300)));//测试用数组
//int ax[128] __attribute__ ((section("sdram")));
   // mpu6xxx_set_param(mpu6050_dev, MPU6XXX_GYRO_RANGE, MPU6XXX_GYRO_RANGE_250DPS  );
    while (0){
        rt_kprintf("read encoder 4: %d!\n",Read_Encoder(4));
        rt_thread_mdelay(500);
        rt_kprintf("time ++!\n");
        rt_kprintf("read encoder 5: %d!\n",Read_Encoder(5));
        rt_thread_mdelay(500);
        rt_kprintf("time ++!\n");
    }
    return RT_EOK;
}
