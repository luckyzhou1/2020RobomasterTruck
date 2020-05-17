#include "Paraminit.h"
#include "ChassisControl.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "Ano_Dt.h"
#include "arm_math.h"
#include "stdio.h"
#include "ramp.h"
#include "PrintfInfo.h"
#include "SW_Wave.h"
#include "CatchingTask.h"
#include "GimbalControl.h"
#include "ShootControl.h"



void ParamInit(void)
{
    memset(&Chassis, 0, sizeof(chassis_t));
//    memset(&remote_control, 0, sizeof(RC_Type));
    
    ChassisRampInit(); //����б�³�ʼ��
    
    /*�����ĸ�����ٶȻ���ʼ��PID����*/
    for(int i=0; i<4; i++)
    {
        PID_struct_init(&Moto_Chassis_Pid_Spd[i], POSITION_PID, 15000, 500, 12.0f, 0.15f, 2.0f);  //��������ʱ��I = 0.002
    }
    
    
    PID_struct_init(&Moto_Chassis_Pid_Pos[4], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
    PID_struct_init(&Moto_Chassis_Pid_Spd[4], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
    
	PID_struct_init(&Moto_Chassis_Pid_Pos[5], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
    PID_struct_init(&Moto_Chassis_Pid_Spd[5], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
		
	PID_struct_init(&Moto_Chassis_Pid_Pos[6], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
    PID_struct_init(&Moto_Chassis_Pid_Spd[6], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
		
	PID_struct_init(&Moto_Chassis_Pid_Pos[7], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
    PID_struct_init(&Moto_Chassis_Pid_Spd[7], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
    
    /*��̨yaw����PID������ʼ����������������Ϣ*/
    PID_struct_init(&Gimbal[YAW].Pid_Pos, POSITION_PID, 200.0f, 30.0f, 0.2f, 0.0f, 2.2f); //λ�û�
    PID_struct_init(&Gimbal[YAW].Pid_Spd, POSITION_PID, 30000.0f, 10000.0f, 200.0f, 6.0f, 20.0f); //�ٶȻ�
    
    /*��̨pitch����PID������ʼ����������������Ϣ*/
    PID_struct_init(&Gimbal[PITCH].Pid_Pos, POSITION_PID, 50.0f, 20.0f, 0.4f, 0.0f, 1.7f); //λ�û�
    PID_struct_init(&Gimbal[PITCH].Pid_Spd, POSITION_PID, 30000.0f, 15000.0f, 140.0f, 12.0f, 30.0f); //�ٶȻ�
    
    /*��������Ħ���ֵ��PID������ʼ��*/
    PID_struct_init(&Shoot.Friction_Motor_Pid_spd[0], POSITION_PID, 10000.0f, 500.0f, 12.0f, 0.15f, 2.0f);
    PID_struct_init(&Shoot.Friction_Motor_Pid_spd[1], POSITION_PID, 10000.0f, 500.0f, 12.0f, 0.15f, 2.0f);
    
    /*�����ֵ��PID������ʼ��*/
    PID_struct_init(&Shoot.Trigger_Motor_Pid_pos, POSITION_PID, 3650.0f, 10.0f, 20.0f, 0.01f, 0.0f); //λ�û�
    PID_struct_init(&Shoot.Trigger_Motor_Pid_spd, POSITION_PID, 6000.0f, 1000.0f, 4.0f, 0.0f, 0.0f); //�ٶȻ�
    
}
