#ifndef __SHOOT_CONTROL_
#define __SHOOT_CONTROL_


#include "pid.h"
#include "CanBus_Task.h"


//Ħ���ֵ���ٶȻ�PID�����������������ٶ�
#define FRICTIONGEAR_ENCODER_SPD_PID_KP  12.0f
#define FRICTIONGEAR_ENCODER_SPD_PID_KI  0.15f
#define FRICTIONGEAR_ENCODER_SPD_PID_KD  2.0f
#define FRICTIONGEAR_ENCODER_SPD_PID_MAX_OUT  10000.0f
#define FRICTIONGEAR_ENCODER_SPD_PID_MAX_IOUT  500.0f


//�����ֵ���ٶȻ�PID�����������������ٶ�
#define TRIGGER_ENCODER_SPD_PID_KP  4.0f
#define TRIGGER_ENCODER_SPD_PID_KI  0.0f
#define TRIGGER_ENCODER_SPD_PID_KD  0.0f
#define TRIGGER_ENCODER_SPD_PID_MAX_OUT  6000.0f
#define TRIGGER_ENCODER_SPD_PID_MAX_IOUT  1000.0f

//�����ֵ��λ�û�PID�����������������Ƕ�
#define TRIGGER_ENCODER_POS_PID_KP  20.0f
#define TRIGGER_ENCODER_POS_PID_KI  0.01f
#define TRIGGER_ENCODER_POS_PID_KD  0.0f
#define TRIGGER_ENCODER_POS_PID_MAX_OUT  3650.0f
#define TRIGGER_ENCODER_POS_PID_MAX_IOUT  10.0f


typedef struct{
    float set_frictiongear_speed;   //Ħ���ֵ���ٶ��趨ֵ
    float set_trigger_angle;   //�����ֵ��λ���趨ֵ
    
    moto_measure_t Friction_Motor_Data[2];   //������������Ħ���ֵ������
    moto_measure_t Trigger_Motor_Data;   //�����������Ĳ����ֵ������
    pid_t Friction_Motor_Pid_spd[2];   //3508��ΪĦ���֣��ٶȻ�PID
    pid_t Trigger_Motor_Pid_pos;   //2006��Ϊ�����֣�λ�û�PID
    pid_t Trigger_Motor_Pid_spd;   //2006��Ϊ�����֣��ٶȻ�PID
    
}shoot_t;


extern shoot_t Shoot;



void FrictiongearSpeedControl(void);
void PluckControl(void);


#endif
