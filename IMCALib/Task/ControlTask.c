#include "ControlTask.h"
#include "ChassisControl.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "Ano_Dt.h"
#include "arm_math.h"
#include "stdio.h"
#include "ramp.h"
#include "PrintfInfo.h"
#include "SW_Wave.h"


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
    
}


void AllTask(void)
{
    
    if(Chassis_Ctrl) //Ƶ��1kHz
    {
        Chassis_Ctrl = 0;
        ChassisTask();
    }
    if(Data_Send_ANO_DT) //Ƶ��500Hz
    {
        Data_Send_ANO_DT = 0;
//        ANO_DT_DataUpdate(); /*�������ݵ���������վ*/
//        SwDataWaveUpdate(); /*�������ݵ�ɽ��๦�ܵ������ֽ��в�����ʾ*/
//        PrintfInfo();
//        printf("\n\r The sin and cos value:%f  , %f  \n\r",arm_sin_f32(PI/6), arm_cos_f32(PI/6));
//        printf("\n\r %d \n\r", remote_control.ch1);
//        printf("\n\r Hello Robomaster! \n\r");
        
//        sin_test = arm_sin_f32(PI/3);
       
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

//����
void LEDTest(int flag)
{
    if(flag == 1)
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
}


