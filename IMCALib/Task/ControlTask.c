#include "ControlTask.h"
#include "ChassisControl.h"
#include "Paraminit.h"
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


int catch_count;//catch_count变量用于计量夹取步骤

void AllTask(void)
{
    if(Chassis_Ctrl) //频率1kHz
    {
			
        Chassis_Ctrl = 0;
//        ChassisTask();
        GimbalTask();
        
//        CatchingControlOne1();
//        CatchingControlOne2();		
//        CatchingReset();
    }		
    if(Data_Send_ANO_DT) //频率500Hz
    {
              Data_Send_ANO_DT = 0;
//        ANO_DT_DataUpdate(); /*发送数据到匿名地面站*/
//        SwDataWaveUpdate(); /*发送数据到山外多功能调试助手进行波形显示*/
           
     }

}
/*************************************************PID参数调试函数***********************************************************/
//在线调参时上位机发过来的参数是已经乘以1000的，这里要除以1000
/*速度环PID参数重置*/
void PidResetSpeed(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd)
{
    
    Moto_Chassis_Pid_Spd[0].MaxOutput = maxout/1000.0f;
    Moto_Chassis_Pid_Spd[0].IntegralLimit = intergral_limit/1000.0f;
    Moto_Chassis_Pid_Spd[0].p = kp/1000.0f;
    Moto_Chassis_Pid_Spd[0].i = ki/1000.0f;
    Moto_Chassis_Pid_Spd[0].d = kd/1000.0f;
    
}

/*位置环PID参数重置*/
void PidResetPosition(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd)
{
    
    Moto_Chassis_Pid_Pos[0].MaxOutput = maxout/1000.0f;
    Moto_Chassis_Pid_Pos[0].IntegralLimit = intergral_limit/1000.0f;
    Moto_Chassis_Pid_Pos[0].p = kp/1000.0f;
    Moto_Chassis_Pid_Pos[0].i = ki/1000.0f;
    Moto_Chassis_Pid_Pos[0].d = kd/1000.0f;
    
}

/********************************************************END**********************************************************/



