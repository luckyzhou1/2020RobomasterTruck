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


int catch_count;//catch_count�������ڼ�����ȡ����

void AllTask(void)
{
    if(Chassis_Ctrl) //Ƶ��1kHz
    {
			
        Chassis_Ctrl = 0;
//        ChassisTask();
        GimbalTask();
        
//        CatchingControlOne1();
//        CatchingControlOne2();		
//        CatchingReset();
    }		
    if(Data_Send_ANO_DT) //Ƶ��500Hz
    {
              Data_Send_ANO_DT = 0;
//        ANO_DT_DataUpdate(); /*�������ݵ���������վ*/
//        SwDataWaveUpdate(); /*�������ݵ�ɽ��๦�ܵ������ֽ��в�����ʾ*/
           
     }

}
/*************************************************PID�������Ժ���***********************************************************/
//���ߵ���ʱ��λ���������Ĳ������Ѿ�����1000�ģ�����Ҫ����1000
/*�ٶȻ�PID��������*/
void PidResetSpeed(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd)
{
    
    Moto_Chassis_Pid_Spd[0].MaxOutput = maxout/1000.0f;
    Moto_Chassis_Pid_Spd[0].IntegralLimit = intergral_limit/1000.0f;
    Moto_Chassis_Pid_Spd[0].p = kp/1000.0f;
    Moto_Chassis_Pid_Spd[0].i = ki/1000.0f;
    Moto_Chassis_Pid_Spd[0].d = kd/1000.0f;
    
}

/*λ�û�PID��������*/
void PidResetPosition(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd)
{
    
    Moto_Chassis_Pid_Pos[0].MaxOutput = maxout/1000.0f;
    Moto_Chassis_Pid_Pos[0].IntegralLimit = intergral_limit/1000.0f;
    Moto_Chassis_Pid_Pos[0].p = kp/1000.0f;
    Moto_Chassis_Pid_Pos[0].i = ki/1000.0f;
    Moto_Chassis_Pid_Pos[0].d = kd/1000.0f;
    
}

/********************************************************END**********************************************************/



