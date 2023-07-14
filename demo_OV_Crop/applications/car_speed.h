/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-12     chengrui       the first version
 */
#ifndef APPLICATIONS_CAR_SPEED_H_
#define APPLICATIONS_CAR_SPEED_H_



#define L_engine_direction1    GET_PIN(J,1)
#define L_engine_direction2    GET_PIN(J,2)
#define R_engine_direction1    GET_PIN(J,3)
#define R_engine_direction2    GET_PIN(J,4)


int hw_car_speed_init(void);
void car_speed_control(rt_int32_t pulse_CH3,rt_int32_t pulse_CH4); /* pulse_CH3 --> PB0  pulse_CH4 --> PB1 */

#endif /* APPLICATIONS_CAR_SPEED_H_ */
