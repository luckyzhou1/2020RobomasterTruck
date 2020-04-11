/*************************************************************************************************************************
 * @File   ChassisControl.c        
 * @Brief  机器人底盘控制，从底盘右上角电机逆时针开始数，底盘对应电机的ID为1 ~ 4，底盘四个电机的控制是挂在CAN1上，
 *         PID对应的参数是在带有轮子负载的情况下调试得到的
 *
*************************************************************************************************************************/

#include "ChassisControl.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "ramp.h"
#include "stdio.h"


uint8_t Chassis_Ctrl; //底盘控制标志位
pid_t  Moto_Chassis_Pid_Pos[8];  //CAN1位置环PID结构体
pid_t  Moto_Chassis_Pid_Spd[8];  //CAN1速度环PID结构体
chassis_t  Chassis;  /*用于底盘*/

/**********************************J_Scope波形显示*********************************************/
float J_Scope_rcinput_y;
float J_Scope_rcinput_x;
float Jscope_set_speed;
float Jscope_get_speed;
float Jscope_pid_out;

/***************************************END****************************************************/


void ChassisTask(void)
{
//    ChassisSpeedTest();  //测试用
    ChassisDataUpdate(); 
    ChassisPidCalc(); 
    ChassisDataCanSend();
    
}



void ChassisDataUpdate(void)
{
    int16_t chassis_vx_channel, chassis_vy_channel, chassis_vw_channel;
    
    /*遥控器死区限制*/
    chassis_vx_channel = RcDeadlineLimit(remote_control.ch1, 10)*CHASSIS_MAXSPEED_RPM/660;
    chassis_vy_channel = RcDeadlineLimit(remote_control.ch2, 10)*CHASSIS_MAXSPEED_RPM/660;
    chassis_vw_channel = RcDeadlineLimit(remote_control.ch3, 10)*CHASSIS_MAXSPEED_RPM/660;

    
    #if defined (SKYGUARD_CHASSIS)
    
        Chassis.vx = remote_control.ch1*GUARD_CHASSIS_MAXSPEED_RPM/660; //左右平移
    
        Chassis.fr_motor_rpm_201 = -Chassis.vx;
        Chassis.fl_motor_rpm_202 = -Chassis.vx;
    
    #else
    
/****************************************斜坡测试*********************************************************/
        //一阶低通滤波代替斜坡作为底盘速度输入
        FirstOrderFilterCali(&chassis_cmd_slow_set_vx, chassis_vx_channel);
        FirstOrderFilterCali(&chassis_cmd_slow_set_vy, chassis_vy_channel);

        Chassis.vx = chassis_cmd_slow_set_vx.out; //左右平移
        Chassis.vy = chassis_cmd_slow_set_vy.out; //前后运动
        Chassis.vw = chassis_vw_channel; //自转

/*******************************************END***********************************************************/
        
        J_Scope_rcinput_x = Chassis.vx;
        J_Scope_rcinput_y = Chassis.vy;
        
        Chassis.fr_motor_rpm_201 = +Chassis.vx - Chassis.vy + Chassis.vw;
        Chassis.fl_motor_rpm_202 = +Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rl_motor_rpm_203 = -Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rr_motor_rpm_204 = -Chassis.vx - Chassis.vy + Chassis.vw;
        
    #endif
    
}



void ChassisPidCalc(void)
{
    Jscope_set_speed = Chassis.fr_motor_rpm_201;
    Jscope_get_speed = Chassis_Motor[FRON_RIGH_201].speed_rpm/19.2;
    
    pid_calc(&Moto_Chassis_Pid_Spd[FRON_RIGH_201], Chassis_Motor[FRON_RIGH_201].speed_rpm, Chassis.fr_motor_rpm_201*REDUCTION_RATIO_3508);
    pid_calc(&Moto_Chassis_Pid_Spd[FRON_LEFT_202], Chassis_Motor[FRON_LEFT_202].speed_rpm, Chassis.fl_motor_rpm_202*REDUCTION_RATIO_3508);
    pid_calc(&Moto_Chassis_Pid_Spd[REAR_LEFT_203], Chassis_Motor[REAR_LEFT_203].speed_rpm, Chassis.rl_motor_rpm_203*REDUCTION_RATIO_3508);
    pid_calc(&Moto_Chassis_Pid_Spd[REAR_RIGH_204], Chassis_Motor[REAR_RIGH_204].speed_rpm, Chassis.rr_motor_rpm_204*REDUCTION_RATIO_3508);
    
    Jscope_pid_out = Moto_Chassis_Pid_Spd[FRON_RIGH_201].pos_out;
    
}


void ChassisDataCanSend(void)
{
   
        SetChassisMotorCurrent(&hcan1, Moto_Chassis_Pid_Spd[FRON_RIGH_201].pos_out, Moto_Chassis_Pid_Spd[FRON_LEFT_202].pos_out, 
                     Moto_Chassis_Pid_Spd[REAR_LEFT_203].pos_out, Moto_Chassis_Pid_Spd[REAR_RIGH_204].pos_out);
    
}


/************************************************测试用*************************************************************/
int32_t set_speed_test;

void ChassisSpeedTest(void)
{
    if(RC_UPPER_RIGHT_SW_MID)
    {
        set_speed_test =0;
        
    }
    else if(RC_UPPER_RIGHT_SW_UP)
    {
        set_speed_test = 400;
        
    }
    else if(RC_UPPER_RIGHT_SW_DOWN)
    {
        set_speed_test = -150;
        
    }
//    Jscope_set_speed = set_speed_test;
//    Jscope_get_speed = Chassis_Motor[FRON_RIGH_201].speed_rpm/19.2;
    
     pid_calc(&Moto_Chassis_Pid_Spd[FRON_RIGH_201], Chassis_Motor[FRON_RIGH_201].speed_rpm, set_speed_test*REDUCTION_RATIO_3508);
    
//    Jscope_pid_out = Moto_Chassis_Pid_Spd[FRON_RIGH_201].pos_out;
}






/*************************************************END***************************************************************/



