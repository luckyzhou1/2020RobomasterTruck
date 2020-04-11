#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "pid.h"
//用于测试哨兵时，去掉下面的注释
//#define SKYGUARD_CHASSIS       //哨兵底盘控制

//3508在带有麦轮负载且悬空的情况下的最大转速为430rpm左右，设置最大速度尽量不要超过这个数值
#define GUARD_CHASSIS_MAXSPEED_RPM    400    //哨兵底盘电机最大转速，单位：rpm
#define CHASSIS_MAXSPEED_RPM          400    //步兵底盘电机最大转速，单位：rpm
#define CHASSIS_VX_MAXSPEED_RPM       350    //底盘左右平移时，底盘电机的最大速度，单位：rpm
#define REDUCTION_RATIO_3508          19.2f   //3508电机减速比


/*底盘电机序号*/
enum{
    
    FRON_RIGH_201 = 0, //前右
    FRON_LEFT_202 = 1, //前左
    REAR_LEFT_203 = 2, //后左
    REAR_RIGH_204 = 3, //后右
    
};


/*底盘结构体*/
typedef struct{
    
    int32_t  fr_motor_rpm_201; //前右电机
    int32_t  fl_motor_rpm_202; //前左电机
    int32_t  rl_motor_rpm_203; //后左电机
    int32_t  rr_motor_rpm_204; //后右电机
    

    float vx; //左右平移
    float vy; //前后
    float vw; //自转
    
    //车体坐标系中的速度
    float car_vx; //单位：m/s
    float car_vy;
    float car_vw; //rad/s
    
    //轮子对应车体坐标系的速度
    float wheel_rad_201; //轮子的转速，单位：rad/s
    float wheel_rad_202;
    float wheel_rad_203;
    float wheel_rad_204;
    
}chassis_t;


extern uint8_t Chassis_Ctrl;
extern pid_t  Moto_Chassis_Pid_Pos[8];  //位置环PID结构体
extern pid_t  Moto_Chassis_Pid_Spd[8];  //速度环PID结构体
extern chassis_t  Chassis;


void ChassisTask(void);
void ChassisDataUpdate(void);
void ChassisPidCalc(void);
void ParamInit(void);
void ChassisDataCanSend(void);

/****************************************测试用****************************************/
void ChassisSpeedTest(void);



/*****************************************END******************************************/



#endif
