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
#include<math.h>
#include<stdio.h>
static int  Velocity_LP=8,Velocity_RI=2;
static long int Motor_Left_PWM=598608,Motor_Right_PWM=604544; //电机PWM值
#define  MOTOR_TURN_PWM 598608                  //转向电机PWM值
int Incremental_PI_Left (int Encoder,int Target)
{
     int Bias;
     rt_kprintf("encode [%d] speed :[%d]",Encoder,Target);
     Bias=Target - Encoder;
     Motor_Left_PWM+=(Velocity_LP*Bias);
     rt_kprintf("[\nLeft_PWM+[%d]",(Velocity_LP*Bias));
    // if(Pwm>7200)Pwm=7200;
     //if(Pwm<-7200)Pwm=-7200;
    return 0;
}

int Incremental_PI_Right (int Encoder,int Target)
{
     int Bias;
     rt_kprintf("encode [%d] speed :[%d]",Encoder,Target);
     Bias=Target - Encoder;            //¼ÆËãÆ«²î
     Motor_Right_PWM +=(Velocity_RI*Bias);
     rt_kprintf("[\nRight_PWM+[%d]",(Velocity_RI*Bias));
   //ÔöÁ¿Ê½PI¿ØÖÆÆ÷
    // if(Pwm>7200)Pwm=7200;
    // if(Pwm<-7200)Pwm=-7200;                 //±£´æÉÏÒ»´ÎÆ«²î
     return 0;                         //ÔöÁ¿Êä³ö
}


void dmp_int_init(void)
{
    if(motion_init()!=0)
        rt_kprintf("[/move]motion_init ERROR!\n");
    else
        rt_kprintf("[/move]motion_init Success!\n");
    //dmp_set_enable(0);
   // rt_pin_irq_enable(MPU6050_INT_PIN, PIN_IRQ_ENABLE);
}
INIT_COMPONENT_EXPORT(dmp_int_init);
/*中断开关*/
void dmp_set_enable( rt_uint8_t temp )
{
    if(temp){
       // rt_pin_mode(MPU6050_INT_PIN, PIN_MODE_INPUT_PULLUP);
       // rt_pin_attach_irq(MPU6050_INT_PIN, PIN_IRQ_MODE_FALLING, mpu6050_int_callback, RT_NULL);
        rt_kprintf("dmp_IRQ_ENABLE !\n");
        rt_pin_irq_enable(MPU6050_INT_PIN, PIN_IRQ_ENABLE);
    }
    else{
        rt_pin_irq_enable(MPU6050_INT_PIN, PIN_IRQ_DISABLE);
        rt_kprintf("dmp_IRQ_DIENABLE !\n");
    }
}


rt_int16_t  car_turn(rt_int16_t  target_angle)
{
    rt_thread_t turn_thread = RT_NULL;
    turn_thread = rt_thread_create("turn_thread", turn_angle_control(target_angle), RT_NULL, 1024, 10, 5);
    rt_thread_startup(turn_angle_control);
    rt_thread_mdelay(5000);
    dmp_set_enable(0);
    rt_thread_delete(turn_thread);// 超时删除线程
    set_wheel_direction(0 , 0);//停转
    return RT_EOK;
}
void car_pwm_close (void){
    rt_pwm_set(engine_pwm_dev, L_engine_channel, PWM_PERIOD, 0);
    rt_pwm_set(engine_pwm_dev, R_engine_channel, PWM_PERIOD, 0);
    rt_pwm_enable(engine_pwm_dev, L_engine_channel);
    rt_pwm_enable(engine_pwm_dev, R_engine_channel);

}


void car_walk(void){

   if (dmp_clear_flag == 0) {
        dmp_clear();
        rt_thread_mdelay(200);
    }

    int offset_r = 0;
    int offset_l = 0;
    int enr=0;
    int enl=0;
    long int lpwm = Motor_Left_PWM;
    long int rpwm = Motor_Right_PWM;
    float offset_angle = 1.1;
    //drive_yaw;
    set_wheel_direction(car_walk_direction,car_walk_direction);
    car_pwm_set(lpwm,rpwm);
    dmp_set_enable(1);
    car_walk_flag = 1;
    while(car_walk_flag){
        enr =  Read_Encoder(4);//获取电机实际转速
        enl = Read_Encoder(2);

        offset_angle= drive_yaw;    //获取旋转角度
        if(fabs(offset_angle)>1){

           if(fabs(offset_angle)>3){
               offset_r  += (int)offset_angle*500;
               offset_l  += -((int)offset_angle*500);
           }
           else {
               offset_r  += (int)offset_angle*1000;
               offset_l  += -((int)offset_angle*1000);
          }
            lpwm =Motor_Left_PWM+offset_l;
            rpwm = Motor_Right_PWM+offset_r;
            car_pwm_set(lpwm,rpwm);

            rt_kprintf("L[%ld]R[%ld]\n",lpwm,rpwm);
        }
        if(lpwm>740000||rpwm>740000){
            lpwm -= 1200;
            rpwm -= 1200;
        }
        rt_thread_mdelay(400);
    }

    dmp_clear_flag = 0;
    dmp_set_enable(0);
    set_wheel_direction(0,0);
    car_pwm_close();


}

int car_walk_thread_init(void)
{
    rt_thread_t thread = RT_NULL;

    thread = rt_thread_create("car_walk", car_walk, RT_NULL, 1024, 10, 5);

    if(thread == RT_NULL)
    {
        return RT_ERROR;
    }
    rt_thread_startup(thread);

    return RT_EOK;
}
/*大于0 右转， 小于0左转  精度±5°*/
rt_int16_t turn_angle_control(rt_int16_t target_angle)
{
    float first_angle,now_angle;
    if (dmp_clear_flag == 0) {
        dmp_clear();
    }
    dmp_clear_flag =0;
    dmp_set_enable(1);
    rt_thread_mdelay(10);
    first_angle = drive_yaw;
//    if((first_angle+target_angle)>180){
//        target_angle=(first_angle+target_angle)-360;
//    }
//    if((first_angle+target_angle)<-180){
//        target_angle=(first_angle+target_angle)+360;
//    }


    rt_kprintf("first_angle = %.1f target_angle=%.1f\n",first_angle,target_angle);
    if (target_angle >0)
        set_wheel_direction(1,-1);
    else
        {
        set_wheel_direction(-1,1);
        target_angle = -target_angle;
        }
    rt_kprintf("target_angle=%.1f\n",target_angle);
    if(target_angle == 90){
        target_angle -= 16;
    }
    if(target_angle == 45){
        target_angle -= 10;
    }
    if(target_angle == 180){
        target_angle -= 17;
    }
    rt_pwm_set(engine_pwm_dev, L_engine_channel, PWM_PERIOD, MOTOR_TURN_PWM);
    rt_pwm_set(engine_pwm_dev, R_engine_channel, PWM_PERIOD, MOTOR_TURN_PWM);
    rt_pwm_enable(engine_pwm_dev, L_engine_channel);
    rt_pwm_enable(engine_pwm_dev, R_engine_channel);
    int times =0;
    while (1)
    {
        rt_thread_mdelay(50);
        now_angle = drive_yaw;
        rt_kprintf("[%d]first_angle = %.1f now_angle=%.1f\n",times,first_angle,now_angle);

        if((fabs(now_angle-first_angle)<=(target_angle+TURN_ALLOW_ERROR_ANGLE+TURN_ANGLE_OFFSET))&&\
                (fabs(now_angle-first_angle)>=(target_angle-TURN_ALLOW_ERROR_ANGLE+TURN_ANGLE_OFFSET)))
        {

            set_wheel_direction(0 , 0);
            //rt_thread_mdelay(100);
            car_pwm_close();
            rt_kprintf("target_angle enough\n");
            rt_kprintf("first_angle = %.1f now_angle%d\n",first_angle,now_angle);
            dmp_set_enable(0);
            rt_kprintf("dmp_clear:%d\n",dmp_clear());
            break;
            return RT_EOK;
        }
        if(fabs(now_angle-first_angle)>target_angle)
        {
            rt_kprintf(">target_angle\n");
            dmp_set_enable(0);
            set_wheel_direction(0 , 0);
            dmp_set_enable(0);
            rt_kprintf("dmp_clear:%d\n",dmp_clear());
            //car_pwm_close();
            break;
            return RT_ERROR;
        }
        times++;
    }
    rt_kprintf("turn_angle_control target_angle[%d] END\n",target_angle);
}
int engine_encode_init(void)
{
    rt_thread_mdelay(1000);
    MX_TIM2_Init();
    MX_TIM4_Init();
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
//INIT_COMPONENT_EXPORT(engine_encode_init);

int Read_Encoder(rt_int8_t encoder_number)
{
    int speed=0;
    switch (encoder_number){
        case 2:     rt_device_open(L_encode, RT_DEVICE_OFLAG_RDONLY);
                    rt_device_control(L_encode, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
                    rt_thread_mdelay(ENCODE_TIME_DT);
                    rt_device_read(L_encode, 0, &speed, 1);
                    rt_device_close(L_encode);
                    break;

        case 4:     rt_device_open(R_encode, RT_DEVICE_OFLAG_RDONLY);
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
    //rt_thread_mdelay(1000);
    MX_TIM3_Init();
    int ret = 0;
    car_walk_flag = 0;
    car_walk_direction =0;
    engine_pwm_dev = (struct rt_device_pwm *)rt_device_find(ENGINE_PWM_DEV_NAME);
    if (engine_pwm_dev == RT_NULL)
    {
        rt_kprintf("Engine PWM device(%s) not find!\n", ENGINE_PWM_DEV_NAME);
        ret = RT_ERROR ;
        return ret;
    }
    rt_pwm_set(engine_pwm_dev, L_engine_channel, PWM_PERIOD, 0);
    rt_pwm_set(engine_pwm_dev, R_engine_channel, PWM_PERIOD, 0);
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
    rt_kprintf("[/move]Engine init!\n");
    return ret;
}

INIT_COMPONENT_EXPORT(engine_init);

/*1 向前，0 停止，-1向后*/
rt_int8_t set_wheel_direction(rt_int8_t L_dwheel , rt_int8_t R_dwheel)
{
    rt_kprintf("set_wheel_direction(%d,%d)\n",L_dwheel,R_dwheel);
    if ((L_dwheel != 1 && L_dwheel != 0 && L_dwheel != -1) || (R_dwheel != 1 && R_dwheel != 0 && R_dwheel != -1)) {
        rt_kprintf("ERROR set_wheel_direction value erro\n ");
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
void car_pwm_set(long int L, long int R){
    rt_kprintf("\n SET Lpwm:%d Rpwm:%d!", Motor_Left_PWM,Motor_Right_PWM);
    if (L>PWM_PERIOD/2)
        L = PWM_PERIOD/2;
    if (R>PWM_PERIOD/2)
        R = PWM_PERIOD/2;
    rt_pwm_set(engine_pwm_dev, L_engine_channel, PWM_PERIOD, L);
    rt_pwm_set(engine_pwm_dev, R_engine_channel, PWM_PERIOD, R);
    rt_pwm_enable(engine_pwm_dev, L_engine_channel);
    rt_pwm_enable(engine_pwm_dev, R_engine_channel);
}

rt_int16_t car_run(rt_int8_t directon ,rt_int8_t distance){
    if (directon){
        set_wheel_direction(1, 1);
    }
    else {
        set_wheel_direction(-1, -1);
    }
    return RT_EOK;
}

/*
 * 函数功能：速度PI控制
 * 入口参数：电机编码器的值
 * 返回  值：速度控制PWM
 */

int velocity(int Encoder_Left,int Encoder_Right)
{
    static float Velocity,Encoder_Least,Encoder;
    static float Encoder_Integral;

   //=============速度PI控制器=======================//
    Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
    Encoder *= 0.8;                                                     //===一阶低通滤波器
    Encoder += Encoder_Least*0.2;                                       //===一阶低通滤波器
    Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
    Encoder_Integral=Encoder_Integral;                       //===接收遥控器数据，控制前进后退
    if(Encoder_Integral>10000)      Encoder_Integral=10000;             //===积分限幅
    if(Encoder_Integral<-10000)     Encoder_Integral=-10000;            //===积分限幅
    Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //===速度控制
    if(pitch<-40||pitch>40)           Encoder_Integral=0;                             //===电机关闭后清除积分
    return Velocity;
}

rt_int16_t car_speed_conrtorl(rt_int16_t direction ,rt_int16_t speed)
{
    rt_kprintf("\ndirection:%d target speed:%d ",direction,speed);
    rt_uint8_t i =0;
    int offset = 2;
    int l_speed,r_speed;
    if (direction>0)
        set_wheel_direction(1,1);
    else
        set_wheel_direction(-1,-1);
    car_pwm_set(Motor_Left_PWM,Motor_Right_PWM);

        l_speed=Read_Encoder(2);
        r_speed=Read_Encoder(4);
        rt_kprintf("\n%d %d [%d %d]",l_speed,r_speed,Motor_Left_PWM,Motor_Right_PWM);

        car_pwm_set(Motor_Left_PWM,Motor_Right_PWM);

    rt_thread_mdelay(800);
    car_pwm_close();
    //car_pwm_set(Motor_Left_PWM,Motor_Right_PWM);
    return RT_EOK;
}


