#ifndef __GIMBAL_CONTROL_
#define __GIMBAL_CONTROL_

#include "pid.h"
#include "CanBus_Task.h"
#include "ControlTask.h"


//YAW�����ٶȻ�PID�����������������ٶ�
#define YAW_ENCODER_SPD_PID_KP  200.0f
#define YAW_ENCODER_SPD_PID_KI  6.0f
#define YAW_ENCODER_SPD_PID_KD  20.0f
#define YAW_ENCODER_SPD_PID_MAX_OUT  30000.0f
#define YAW_ENCODER_SPD_PID_MAX_IOUT  10000.0f

//YAW����λ�û�PID�����������������Ƕ�
#define YAW_ENCODER_POS_PID_KP  0.2f
#define YAW_ENCODER_POS_PID_KI  0.0f
#define YAW_ENCODER_POS_PID_KD  2.2f
#define YAW_ENCODER_POS_PID_MAX_OUT  200.0f
#define YAW_ENCODER_POS_PID_MAX_IOUT  30.0f


//PITCH�����ٶȻ�PID�����������������ٶ�
#define PITCH_ENCODER_SPD_PID_KP  140.0f
#define PITCH_ENCODER_SPD_PID_KI  12.0f
#define PITCH_ENCODER_SPD_PID_KD  30.0f
#define PITCH_ENCODER_SPD_PID_MAX_OUT  30000.0f
#define PITCH_ENCODER_SPD_PID_MAX_IOUT  15000.0f

//PITCH����λ�û�PID�����������������Ƕ�
#define PITCH_ENCODER_POS_PID_KP  0.4f
#define PITCH_ENCODER_POS_PID_KI  0.0f
#define PITCH_ENCODER_POS_PID_KD  1.7f
#define PITCH_ENCODER_POS_PID_MAX_OUT  50.0f
#define PITCH_ENCODER_POS_PID_MAX_IOUT  20.0f


#define REDUCTION_RATIO_3508        19.2f   //3508������ٱ�
#define CHASSIS_YAW_MID_FRONT       6000  //��̨YAW�����ʱ������Ӧ����ǰ�����У���������Ӧ�Ļ�е�Ƕ�ֵ
#define CHASSIS_YAW_MID_BACK        (((0 <= CHASSIS_YAW_MID_FRONT)&&    \
                                     (CHASSIS_YAW_MID_FRONT < 4096))    \
                                    ? (CHASSIS_YAW_MID_FRONT + 4096)    \
                                    : (CHASSIS_YAW_MID_FRONT - 4096))



/*��̨������*/
enum{
    YAW   = 0,
    PITCH = 1,
    
};

enum{
    FRONT = 0,
    BACK = 1,
    
};


typedef struct{
    int32_t set_angle;              //�趨�Ƕ�
    int32_t offset_relative_angle;  //�ϵ�ʱyaw������ڵ���ǰ�������ĵ���Ի�е�Ƕ�ֵ
    int32_t relative_angle;         //����ڵ���ǰ�������ĵ���Ի�е�Ƕ�ֵ
    float relative_angle_rad;       //��̨����̵���ԽǶȣ���λ��rad
    float can_send;                 //CAN���͵��ٶȻ�PID���ֵ
    float set_angle_imu;            //IMU����ʱ���Ƕ��趨ֵ
    
    moto_measure_t Motor_Data;   //�����������ĵ������
    pid_t Pid_Spd;   //�ٶȻ�PID�������������ٶ�
    pid_t Pid_Pos;   //λ�û�PID������������λ��
    pid_t Pid_Spd_IMU;   //�ٶȻ�PID��IMU�����ٶ�
    pid_t pid_pos_IMU;   //λ�û�PID��IMU����λ��
    
}gimbal_t;


extern uint8_t Gimbal_Ctrl;
extern gimbal_t Gimbal[2]; //��̨�ṹ��
extern uint16_t Yaw_Encoder_Mid[2]; //��̨���������ǰ����ֵ


void GimbalTask(void);
void GimbalPidCalc(void);
void GimbalControlSetValue(void);
int16_t GetEncoderRelativeAngle(uint16_t encoder_angle, uint16_t encoder_middle[]);


/********************************************************����**********************************************************/
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
