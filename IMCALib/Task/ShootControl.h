#ifndef __SHOOT_CONTROL_
#define __SHOOT_CONTROL_


#include "pid.h"
#include "CanBus_Task.h"


//摩擦轮电机速度环PID参数，编码器反馈速度
#define FRICTIONGEAR_ENCODER_SPD_PID_KP  12.0f
#define FRICTIONGEAR_ENCODER_SPD_PID_KI  0.15f
#define FRICTIONGEAR_ENCODER_SPD_PID_KD  2.0f
#define FRICTIONGEAR_ENCODER_SPD_PID_MAX_OUT  10000.0f
#define FRICTIONGEAR_ENCODER_SPD_PID_MAX_IOUT  500.0f


//拨弹轮电机速度环PID参数，编码器反馈速度
#define TRIGGER_ENCODER_SPD_PID_KP  4.0f
#define TRIGGER_ENCODER_SPD_PID_KI  0.0f
#define TRIGGER_ENCODER_SPD_PID_KD  0.0f
#define TRIGGER_ENCODER_SPD_PID_MAX_OUT  6000.0f
#define TRIGGER_ENCODER_SPD_PID_MAX_IOUT  1000.0f

//拨弹轮电机位置环PID参数，编码器反馈角度
#define TRIGGER_ENCODER_POS_PID_KP  20.0f
#define TRIGGER_ENCODER_POS_PID_KI  0.01f
#define TRIGGER_ENCODER_POS_PID_KD  0.0f
#define TRIGGER_ENCODER_POS_PID_MAX_OUT  3650.0f
#define TRIGGER_ENCODER_POS_PID_MAX_IOUT  10.0f


typedef struct{
    float set_frictiongear_speed;   //摩擦轮电机速度设定值
    float set_trigger_angle;   //拨弹轮电机位置设定值
    
    moto_measure_t Friction_Motor_Data[2];   //编码器反馈的摩擦轮电机数据
    moto_measure_t Trigger_Motor_Data;   //编码器反馈的拨弹轮电机数据
    pid_t Friction_Motor_Pid_spd[2];   //3508作为摩擦轮，速度环PID
    pid_t Trigger_Motor_Pid_pos;   //2006作为拨弹轮，位置环PID
    pid_t Trigger_Motor_Pid_spd;   //2006作为拨弹轮，速度环PID
    
}shoot_t;


extern shoot_t Shoot;



void FrictiongearSpeedControl(void);
void PluckControl(void);


#endif
