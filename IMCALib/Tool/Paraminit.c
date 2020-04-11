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

void ParamInit(void)
{
    memset(&Chassis, 0, sizeof(chassis_t));
//    memset(&remote_control, 0, sizeof(RC_Type));
    
    ChassisRampInit(); //底盘斜坡初始化
    
    /*底盘四个电机速度环初始化PID参数*/
    for(int i=0; i<4; i++)
    {
        PID_struct_init(&Moto_Chassis_Pid_Spd[i], POSITION_PID, 15000, 500, 12.0f, 0.15f, 2.0f);  //麦轮悬空时，I = 0.002
    }
    
    
    PID_struct_init(&Moto_Chassis_Pid_Pos[4], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
    PID_struct_init(&Moto_Chassis_Pid_Spd[4], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
    
		PID_struct_init(&Moto_Chassis_Pid_Pos[5], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
    PID_struct_init(&Moto_Chassis_Pid_Spd[5], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
		
		PID_struct_init(&Moto_Chassis_Pid_Pos[6], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
    PID_struct_init(&Moto_Chassis_Pid_Spd[6], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
		
		PID_struct_init(&Moto_Chassis_Pid_Pos[7], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
    PID_struct_init(&Moto_Chassis_Pid_Spd[7], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
}
