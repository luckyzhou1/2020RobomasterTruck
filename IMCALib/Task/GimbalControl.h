#ifndef __GIMBAL_CONTROL_
#define __GIMBAL_CONTROL_

#include "pid.h"
#include "CanBus_Task.h"
#include "ControlTask.h"


//YAW轴电机速度环PID参数，编码器反馈速度
#define YAW_ENCODER_SPD_PID_KP  200.0f
#define YAW_ENCODER_SPD_PID_KI  6.0f
#define YAW_ENCODER_SPD_PID_KD  20.0f
#define YAW_ENCODER_SPD_PID_MAX_OUT  30000.0f
#define YAW_ENCODER_SPD_PID_MAX_IOUT  10000.0f

//YAW轴电机位置环PID参数，编码器反馈角度
#define YAW_ENCODER_POS_PID_KP  0.2f
#define YAW_ENCODER_POS_PID_KI  0.0f
#define YAW_ENCODER_POS_PID_KD  2.2f
#define YAW_ENCODER_POS_PID_MAX_OUT  200.0f
#define YAW_ENCODER_POS_PID_MAX_IOUT  30.0f


//PITCH轴电机速度环PID参数，编码器反馈速度
#define PITCH_ENCODER_SPD_PID_KP  140.0f
#define PITCH_ENCODER_SPD_PID_KI  12.0f
#define PITCH_ENCODER_SPD_PID_KD  30.0f
#define PITCH_ENCODER_SPD_PID_MAX_OUT  30000.0f
#define PITCH_ENCODER_SPD_PID_MAX_IOUT  15000.0f

//PITCH轴电机位置环PID参数，编码器反馈角度
#define PITCH_ENCODER_POS_PID_KP  0.4f
#define PITCH_ENCODER_POS_PID_KI  0.0f
#define PITCH_ENCODER_POS_PID_KD  1.7f
#define PITCH_ENCODER_POS_PID_MAX_OUT  50.0f
#define PITCH_ENCODER_POS_PID_MAX_IOUT  20.0f


#define REDUCTION_RATIO_3508        19.2f   //3508电机减速比
#define CHASSIS_YAW_MID_FRONT       6000  //云台YAW轴回中时（即对应底盘前面正中）编码器对应的机械角度值
#define CHASSIS_YAW_MID_BACK        (((0 <= CHASSIS_YAW_MID_FRONT)&&    \
                                     (CHASSIS_YAW_MID_FRONT < 4096))    \
                                    ? (CHASSIS_YAW_MID_FRONT + 4096)    \
                                    : (CHASSIS_YAW_MID_FRONT - 4096))



/*云台电机序号*/
enum{
    YAW   = 0,
    PITCH = 1,
    
};

enum{
    FRONT = 0,
    BACK = 1,
    
};


typedef struct{
    int32_t set_angle;              //设定角度
    int32_t offset_relative_angle;  //上电时yaw轴相对于底盘前面正中心的相对机械角度值
    int32_t relative_angle;         //相对于底盘前面正中心的相对机械角度值
    float relative_angle_rad;       //云台与底盘的相对角度，单位：rad
    float can_send;                 //CAN发送的速度环PID输出值
    float set_angle_imu;            //IMU反馈时，角度设定值
    
    moto_measure_t Motor_Data;   //编码器反馈的电机数据
    pid_t Pid_Spd;   //速度环PID，编码器反馈速度
    pid_t Pid_Pos;   //位置环PID，编码器反馈位置
    pid_t Pid_Spd_IMU;   //速度环PID，IMU反馈速度
    pid_t pid_pos_IMU;   //位置环PID，IMU反馈位置
    
}gimbal_t;


extern uint8_t Gimbal_Ctrl;
extern gimbal_t Gimbal[2]; //云台结构体
extern uint16_t Yaw_Encoder_Mid[2]; //云台电机编码器前后中值


void GimbalTask(void);
void GimbalPidCalc(void);
void GimbalControlSetValue(void);
int16_t GetEncoderRelativeAngle(uint16_t encoder_angle, uint16_t encoder_middle[]);


/********************************************************测试**********************************************************/
#ifdef FUNCTIONAL_TEST

extern int32_t set_yaw_speed;
extern int32_t set_yaw_angle;
extern int32_t set_pitch_speed;
extern int32_t set_pitch_angle;

void YawTest(void);
void PitchTest(void);

#endif
/*********************************************************END**********************************************************/

#endif
