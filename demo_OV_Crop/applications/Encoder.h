/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-12     Mr.zhou       the first version
 */
#ifndef APPLICATIONS_ENCODER_H_
#define APPLICATIONS_ENCODER_H_

#define ENCODE_TIME_DT 500  //dt ms
int Read_Encoder(rt_int8_t encoder);
int Encoder_init(void);
//rt_int32_t getOriginalCounter(rt_int16_t cnt);
//rt_int16_t getCounter(rt_int16_t cnt);
void speed_thread_entry(void);

#endif /* APPLICATIONS_ENCODER_H_ */
