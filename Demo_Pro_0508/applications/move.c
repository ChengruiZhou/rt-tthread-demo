/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-05     Dell       the first version
 */
#include "move.h"

float Velocity_KP=10,Velocity_KI=10;
long int Motor_Left_PWM=0,Motor_Right_PWM=0; //电机PWM值

rt_uint8_t tof_init()
{

}

int Incremental_PI_Left (int Encoder,int Target)
{
     static int Bias,Pwm,Last_bias;
     Bias=Encoder-Target;
     Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
     if(Pwm>7200)Pwm=7200;
     if(Pwm<-7200)Pwm=-7200;
     Last_bias=Bias;
     return Pwm;
}

int Incremental_PI_Right (int Encoder,int Target)
{
     static int Bias,Pwm,Last_bias;
     Bias=Encoder-Target;                //¼ÆËãÆ«²î
     Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //ÔöÁ¿Ê½PI¿ØÖÆÆ÷
     if(Pwm>7200)Pwm=7200;
     if(Pwm<-7200)Pwm=-7200;
     Last_bias=Bias;                       //±£´æÉÏÒ»´ÎÆ«²î
     return Pwm;                         //ÔöÁ¿Êä³ö
}


void dmp_int_init(void)
{
    motion_init();
    dmp_set_enable(0);
   // rt_pin_irq_enable(MPU6050_INT_PIN, PIN_IRQ_ENABLE);
}
/*中断开关*/
void dmp_set_enable( rt_uint8_t temp )
{
    if(temp)
        rt_pin_irq_enable(MPU6050_INT_PIN, PIN_IRQ_ENABLE);
    else
        rt_pin_irq_enable(MPU6050_INT_PIN, PIN_IRQ_DISABLE);
}


rt_uint8_t car_turn(rt_uint8_t target_angle)
{
    rt_thread_t turn_thread = RT_NULL;
    turn_thread = rt_thread_create("turn_thread", turn_angle_control(target_angle), RT_NULL, 1024, 10, 5);
    rt_thread_mdelay(5000);
    rt_thread_delete(turn_thread);// 超时删除线程
    set_wheel_direction(0 , 0);//停转
    return RT_EOK;
}

/*大于0 右转， 小于0左转  精度±5°*/
rt_uint8_t turn_angle_control(rt_uint8_t target_angle)
{
    rt_uint8_t first_angle,now_angle;
    dmp_set_enable(1);
    first_angle = drive_yaw;
    if (target_angle >0)
        set_wheel_direction(1,-1);
    else
        set_wheel_direction(-1,1);
    while (1)
    {
        now_angle = drive_yaw;
        if((now_angle-first_angle)>(target_angle-5)&&(now_angle-first_angle)<(target_angle+5))
        {
            dmp_set_enable(0);
            set_wheel_direction(0 , 0);
            break;
            return RT_EOK;
        }
        if((now_angle-first_angle)>target_angle)
        {
            dmp_set_enable(0);
            set_wheel_direction(0 , 0);
            break;
            return RT_ERROR;
        }
    }
}
int engine_encode_init(void)
{
   MX_TIM4_Init();
    MX_TIM5_Init();
    int ret = 0;
    /* 查找脉冲编码器设备 */
    L_encode = rt_device_find(L_ENCODE_DEV);
    R_encode = rt_device_find(R_ENCODE_DEV);
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
    rt_device_open(R_encode, RT_DEVICE_OFLAG_RDONLY);
    rt_kprintf("Encode_init_successful!\n");
    return ret;
}
//INIT_BOARD_EXPORT(engine_encode_init);

int Read_Encoder(rt_int8_t encoder_number)
{
    int speed=0;
    switch (encoder_number){
        case 4:     rt_device_open(L_encode, RT_DEVICE_OFLAG_RDONLY);
                    rt_device_control(L_encode, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
                    rt_thread_mdelay(ENCODE_TIME_DT);
                    rt_device_read(L_encode, 0, &speed, 1);
                    rt_device_close(L_encode);
                    break;

        case 5:     rt_device_open(R_encode, RT_DEVICE_OFLAG_RDONLY);
                    rt_device_control(R_encode, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
                    rt_thread_mdelay(ENCODE_TIME_DT);
                    rt_device_read(R_encode, 0, &speed, 1);
                    rt_device_close(R_encode);
                    break;
        default :break;
    }
    return speed;
}

int engine_init(void)
{
    MX_TIM3_Init();
    int ret = 0;
    engine_pwm_dev = (struct rt_device_pwm *)rt_device_find(ENGINE_PWM_DEV_NAME);
    if (engine_pwm_dev == RT_NULL)
    {
        rt_kprintf("Engine PWM device(%s) not find!\n", ENGINE_PWM_DEV_NAME);
        ret = RT_ERROR ;
        return ret;
    }
    rt_pwm_set(engine_pwm_dev, L_engine_channel, PWM_PERIOD, PWM_PERIOD/2);
    rt_pwm_set(engine_pwm_dev, R_engine_channel, PWM_PERIOD, PWM_PERIOD/2);
    rt_pwm_enable(engine_pwm_dev, L_engine_channel);
    rt_pwm_enable(engine_pwm_dev, R_engine_channel);

    rt_pin_mode(L_engine_direction1, PIN_MODE_OUTPUT);
    rt_pin_mode(L_engine_direction2, PIN_MODE_OUTPUT);
    rt_pin_mode(R_engine_direction1, PIN_MODE_OUTPUT);
    rt_pin_mode(R_engine_direction2, PIN_MODE_OUTPUT);
    rt_pin_write(L_engine_direction1, PIN_LOW);
    rt_pin_write(L_engine_direction2, PIN_LOW);
    rt_pin_write(R_engine_direction1, PIN_LOW);
    rt_pin_write(R_engine_direction2, PIN_LOW);
    rt_kprintf("Engine init!\n");
    return ret;
}

//INIT_BOARD_EXPORT(engine_init);
/*1 向前，0 停止，-1向后*/
rt_uint8_t set_wheel_direction(rt_uint8_t L_dwheel , rt_uint8_t R_dwheel)
{
    if((L_dwheel !=1 &&L_dwheel !=0 &&L_dwheel !=-1) ||R_dwheel !=1 &&R_dwheel !=0 &&R_dwheel !=-1){
        rt_kprintf("ERROR set_wheel_direction value error ");
        return RT_ERROR;
    }

    else {
    switch (L_dwheel)
    {
        case (-1) : rt_pin_write(L_engine_direction1, PIN_LOW); rt_pin_write(L_engine_direction2, PIN_HIGH); break;
        case (0) : rt_pin_write(L_engine_direction1, PIN_LOW); rt_pin_write(L_engine_direction2, PIN_LOW); break;
        case (1): rt_pin_write(L_engine_direction1, PIN_HIGH); rt_pin_write(L_engine_direction2, PIN_LOW); break;
        default : break;
    }
    switch (R_dwheel)
    {
        case (-1) : rt_pin_write(R_engine_direction1, PIN_LOW); rt_pin_write(R_engine_direction2, PIN_HIGH); break;
        case (0) : rt_pin_write(R_engine_direction1, PIN_LOW); rt_pin_write(R_engine_direction2, PIN_LOW); break;
        case (1) : rt_pin_write(R_engine_direction1, PIN_HIGH); rt_pin_write(R_engine_direction2, PIN_LOW); break;
        default : break;
    }
    }
    return RT_EOK;
}
/*rt_uint8_t car_move(rt_uint8_t direction,rt_uint8_t distence,rt_uint8_t speed)
{
    rt_thread_t car_move_t = RT_NULL;
    car_move_t = rt_thread_create("car_move_t", car_speed_conrtorl(direction,speed), RT_NULL, 1024, 10, 5);
    rt_thread_startup(car_move_t);
    rt_thread_mdelay(10000);
    rt_thread_delete(car_move_t);// 超时删除线程
    set_wheel_direction(0 , 0);//停转
}*/
rt_uint8_t car_speed_conrtorl(rt_uint8_t direction ,rt_uint8_t speed)
{
    rt_uint8_t i =0;
    int l_speed,r_speed;
    if (direction>0)
        set_wheel_direction(1,1);
    else
        set_wheel_direction(-1,-1);
    rt_pwm_set(engine_pwm_dev, L_engine_channel, PWM_PERIOD, PWM_PERIOD/2);
    rt_pwm_set(engine_pwm_dev, R_engine_channel, PWM_PERIOD, PWM_PERIOD/2);
    rt_pwm_enable(engine_pwm_dev, L_engine_channel);
    rt_pwm_enable(engine_pwm_dev, R_engine_channel);

    for(i=0;i<30;i++)
    {
        l_speed=Read_Encoder(4);
        r_speed=Read_Encoder(5);
        rt_kprintf("\n%d %d ",l_speed,r_speed);
        if((l_speed - r_speed)<100&&(l_speed - r_speed)>-100){
            if( ((l_speed+r_speed)/2)<speed +100 &&((l_speed+r_speed)/2)>speed-100 )
                break;
        }
        Motor_Left_PWM = Incremental_PI_Left(l_speed,speed);
        Motor_Left_PWM = Incremental_PI_Left(r_speed,speed);
        rt_pwm_set(engine_pwm_dev, L_engine_channel, PWM_PERIOD, Motor_Left_PWM);
        rt_pwm_set(engine_pwm_dev, R_engine_channel, PWM_PERIOD, Motor_Left_PWM);
        rt_pwm_enable(engine_pwm_dev, L_engine_channel);
        rt_pwm_enable(engine_pwm_dev, R_engine_channel);
    }
    return RT_EOK;
}

