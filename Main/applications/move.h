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

int Velocity_Pwm;
float velocity_KP=-90;     // 小车速度环PI参数
float velocity_KI=-0.45;

int l_speed,r_speed;


#define ENCODE_TIME_DT 200  //dt ms
#define PWM_PERIOD 2000000
#define TURN_ALLOW_ERROR_ANGLE 2
#define TURN_ANGLE_OFFSET -5
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
int dmp_clear_flag;
rt_uint8_t car_walk_direction;
int car_walk_flag;
#define L_ENCODE_DEV    "pulse2"    /* 脉冲编码器名称 */
#define R_ENCODE_DEV    "pulse4"    /* 脉冲编码器名称 */
rt_device_t L_encode;   /* 左脉冲编码器设备句柄 */
rt_device_t R_encode;   /* 右脉冲编码器设备句柄 */
void car_walk(void);
void car_pwm_set(long int L , long int R);
int engine_encode_init(void);
int Read_Encoder(rt_int8_t encoder_number);
int engine_init(void);
rt_int8_t set_wheel_direction(rt_int8_t L_dwheel , rt_int8_t R_dwheel);
rt_int16_t turn_angle_control(rt_int16_t target_angle);
rt_int16_t car_turn(rt_int16_t target_angle);
rt_int16_t car_speed_conrtorl(rt_int16_t direction ,rt_int16_t speed);
void car_pwm_set(long int L, long int R);
