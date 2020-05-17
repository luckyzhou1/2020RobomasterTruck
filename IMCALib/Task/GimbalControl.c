#include "GimbalControl.h"
#include "CanBus_Task.h"
#include "pid.h"
#include "RC_Task.h"
#include "ShootControl.h"
#include "ControlTask.h"


uint8_t Gimbal_Ctrl;
gimbal_t Gimbal[2];  //云台结构体
uint16_t Yaw_Encoder_Mid[2] = {CHASSIS_YAW_MID_FRONT, CHASSIS_YAW_MID_BACK};



void GimbalTask(void)
{
    FrictiongearSpeedControl();
    PluckControl();
    GimbalControlSetValue();
    
    GimbalPidCalc();
    
    /*CAN数据发送*/
    SetGimbalMotorVoltage(&hcan2, Gimbal[YAW].can_send, Gimbal[PITCH].can_send);
    SetShootMotorCurrent(&hcan2, Shoot.Trigger_Motor_Pid_spd.pos_out, Shoot.Friction_Motor_Pid_spd[0].pos_out, Shoot.Friction_Motor_Pid_spd[1].pos_out);
}


void GimbalPidCalc(void)
{
    
    /*yaw轴*/
    pid_calc(&Gimbal[YAW].Pid_Pos, Gimbal[YAW].Motor_Data.total_angle, Gimbal[YAW].set_angle);  //位置环
    pid_calc(&Gimbal[YAW].Pid_Spd, Gimbal[YAW].Motor_Data.speed_rpm, Gimbal[YAW].Pid_Pos.pos_out);  //速度环
    Gimbal[YAW].can_send = Gimbal[YAW].Pid_Spd.pos_out; //赋值给CAN发送
    
    /*pitch轴*/
    pid_calc(&Gimbal[PITCH].Pid_Pos, Gimbal[PITCH].Motor_Data.total_angle, Gimbal[PITCH].set_angle);  //位置环
    pid_calc(&Gimbal[PITCH].Pid_Spd, Gimbal[PITCH].Motor_Data.speed_rpm, Gimbal[PITCH].Pid_Pos.pos_out);  //速度环
    Gimbal[PITCH].can_send = Gimbal[PITCH].Pid_Spd.pos_out; //赋值给CAN发送
    
    /*3508摩擦轮*/
    pid_calc(&Shoot.Friction_Motor_Pid_spd[0], Shoot.Friction_Motor_Data[0].speed_rpm, +Shoot.set_frictiongear_speed*REDUCTION_RATIO_3508);
    pid_calc(&Shoot.Friction_Motor_Pid_spd[1], Shoot.Friction_Motor_Data[1].speed_rpm, -Shoot.set_frictiongear_speed*REDUCTION_RATIO_3508);
    
    /*2006拨弹轮*/
    pid_calc(&Shoot.Trigger_Motor_Pid_pos, Shoot.Trigger_Motor_Data.total_angle/36, Shoot.set_trigger_angle);
    pid_calc(&Shoot.Trigger_Motor_Pid_spd, Shoot.Trigger_Motor_Data.speed_rpm, Shoot.Trigger_Motor_Pid_pos.pos_out);
    
}


void GimbalControlSetValue(void)
{
    int16_t yaw_channel_output, pitch_channel_output;
    /*遥控器死区限制*/
    yaw_channel_output = RcDeadlineLimit(remote_control.ch3, 5);
    pitch_channel_output = RcDeadlineLimit(remote_control.ch4, 5);
    
    Gimbal[YAW].set_angle += yaw_channel_output*0.01;
    Gimbal[PITCH].set_angle -= pitch_channel_output*0.008;
}


/*************************************************************************************
  * @brief 改进版，输出范围为-4096 ~ 4096，底盘在云台的左边为正，右边为负
  * @note 待回校上车测试
*************************************************************************************/
int16_t GetEncoderRelativeAngle(uint16_t encoder_angle, uint16_t encoder_middle[])
{
    int16_t relative_angle = 0, relative_angle_out = 0;
    int8_t direction_flag = 0;
    
    relative_angle = encoder_angle - encoder_middle[FRONT]; /*直接作差*/
    
    /*方向选定*/
    if(encoder_middle[FRONT] < encoder_middle[BACK])
    {
        if((encoder_middle[FRONT] < encoder_angle)&&(encoder_angle < encoder_middle[BACK])) 
            direction_flag = -1;
        else
            direction_flag = 1;
    }
    else
    {
        if((encoder_middle[FRONT] < encoder_angle)||(encoder_angle < encoder_middle[BACK]))
            direction_flag = -1;
        else
            direction_flag = 1;
    }
    
    /*取正数*/
    if(relative_angle < 0) 
        relative_angle = -relative_angle;
    
    /*角度范围选取为：-180 ~ 180度*/
    if(relative_angle > 8192/2) 
        relative_angle_out = 8192 - relative_angle;
    else
        relative_angle_out = relative_angle;
    
    return relative_angle_out*direction_flag;
}


/********************************************************测试**********************************************************/
#ifdef FUNCTIONAL_TEST

int32_t set_yaw_speed;
int32_t set_yaw_angle;
int32_t set_pitch_speed;
int32_t set_pitch_angle;

void YawTest(void)
{
//    set_yaw_angle += remote_control.ch3*0.005;
    
    if(RC_UPPER_LEFT_SW_UP)
    {
        set_yaw_speed = 40;
//        set_yaw_angle = 2000;
    }
    else if(RC_UPPER_LEFT_SW_DOWN)
    {
        set_yaw_speed = -60;
//        set_yaw_angle = -4500;
    }
    else if(RC_UPPER_LEFT_SW_MID)
    {
        set_yaw_speed = 0;
//        set_yaw_angle = 0;
    }
    
//    pid_calc(&Gimbal[YAW].Pid_Pos, Gimbal[YAW].Motor_Data.total_angle, set_yaw_angle);  //位置环
    pid_calc(&Gimbal[YAW].Pid_Spd, Gimbal[YAW].Motor_Data.speed_rpm, /*Gimbal[YAW].Pid_Pos.pos_out*/ set_yaw_speed);  //速度环
    Gimbal[YAW].can_send = Gimbal[YAW].Pid_Spd.pos_out; //赋值给CAN发送
    
    //CAN数据发送
    if(RC_UPPER_RIGHT_SW_MID)
        SetGimbalMotorVoltage(&hcan1, Gimbal[YAW].can_send, 0);
    else
        SetGimbalMotorVoltage(&hcan1, 0, 0);

}


void PitchTest(void)
{
//    set_pitch_angle += remote_control.ch4*0.005;
    
    if(RC_UPPER_LEFT_SW_UP)
    {
//        set_pitch_speed = 10;
        set_pitch_angle = 350;
    }
    else if(RC_UPPER_LEFT_SW_DOWN)
    {
//        set_pitch_speed = -20;
        set_pitch_angle = -400;
    }
    else if(RC_UPPER_LEFT_SW_MID)
    {
//        set_pitch_speed = 0;
        set_pitch_angle = 0;
    }
    
//    pid_calc(&Gimbal[PITCH].Pid_Pos, Gimbal[PITCH].Motor_Data.total_angle, set_pitch_angle);  //位置环
    pid_calc(&Gimbal[PITCH].Pid_Spd, Gimbal[PITCH].Motor_Data.speed_rpm, /*Gimbal[PITCH].Pid_Pos.pos_out*/ set_pitch_speed);  //速度环
    Gimbal[PITCH].can_send = Gimbal[PITCH].Pid_Spd.pos_out; //赋值给CAN发送
    
    //CAN数据发送
    if(RC_UPPER_RIGHT_SW_MID)
        SetGimbalMotorVoltage(&hcan1, 0, Gimbal[PITCH].can_send);
    else
        SetGimbalMotorVoltage(&hcan1, 0, 0);
    
}

#endif
/*********************************************************END**********************************************************/



