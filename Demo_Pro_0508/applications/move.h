/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-05     Dell       the first version
 */
#ifndef APPLICATIONS_MOVE_H_
#define APPLICATIONS_MOVE_H_



#endif /* APPLICATIONS_MOVE_H_ */

#include <board.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>

#define ENCODE_TIME_DT 300  //dt ms
#define PWM_PERIOD 2000000


#define L_engine_direction1    GET_PIN(J,1)
#define L_engine_direction2    GET_PIN(J,2)
#define R_engine_direction1    GET_PIN(J,3)
#define R_engine_direction2    GET_PIN(J,4)

#define I2C1_BUS_NAME          "i2c1"  /* 传感器连接的I2C总线设备名称 */
static struct rt_i2c_bus_device *i2c1_bus = RT_NULL;     /* I2C总线设备句柄 */
//--------------------------
#define ENGINE_PWM_DEV_NAME        "pwm3"  /* 电机PWM设备名称*/
#define L_engine_channel     3       /*左电机PWM通道  */
#define R_engine_channel     4       /*  */
struct rt_device_pwm *engine_pwm_dev;      /* 电机PWM设备句柄 */

#define L_ENCODE_DEV    "pulse4"    /* 脉冲编码器名称 */
#define R_ENCODE_DEV    "pulse5"    /* 脉冲编码器名称 */
rt_device_t L_encode;   /* 左脉冲编码器设备句柄 */
rt_device_t R_encode;   /* 右脉冲编码器设备句柄 */

int engine_encode_init(void);
int Read_Encoder(rt_int8_t encoder_number);
int engine_init(void);
rt_uint8_t set_wheel_direction(rt_uint8_t L_dwheel , rt_uint8_t R_dwheel);
rt_uint8_t turn_angle_control(rt_uint8_t target_angle);
rt_uint8_t car_turn(rt_uint8_t target_angle);
rt_uint8_t car_speed_conrtorl(rt_uint8_t direction ,rt_uint8_t speed);
