#include "CatchingTask.h"
#include "ramp.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "ChassisControl.h"
#include "ControlTask.h"
#include "gpio.h"

SinRampState Sin_Test;
int32_t lift_angle;
int32_t rotate_angle0;
int32_t rotate_angle1;
int32_t set_speed; 
int Delay_100ms; 
int SinSign=1;

int Flip_Box_Delay;
int Flip_Box_Sign;
int Even_Take_Sign=0;

int rotational_delay;
int Rotational_Delay1;
int Rotational_Delay2;
int Rotational_Delay3;

int Rotational_Sign;
/************************J-Scope������ʾ����*************************/
float Jscope_set_angle;
float Jscope_get_angle;
float Jscope_real_speed;

/******************************END***********************************/
void CatchingControlOne1(void)//��ȡ����1�ķֽ⶯��1
{
  int32_t change_angle1;
  
  if(KEY_C)
	{
		if(SinSign==1)
			{
				SinRampInit(&Sin_Test);
				SinSign=0;
			}
		 Sin_Test.sin_ramp_switch = 1;
		 lift_angle = -313000;
	
	}
 
  if(KEY_R)
	{
		if(SinSign==1)
			{
				SinRampInit(&Sin_Test);
				SinSign=0;
			}
		 Sin_Test.sin_ramp_switch = 1;
		 lift_angle = 313000;//����2
	
	}
 
  if(KEY_Q)
	{		
      catch_count = 2;
      Even_Take_Sign=1;//��ʼ��
	}
		 
	 if((catch_count==2)&&(Even_Take_Sign==1))
	 {
			rotate_angle0 = 1111;//��ȡʱ��ת�ĽǶȣ���û����
     rotate_angle1 = (-rotate_angle0);
			catch_count++;
			rotational_delay = 0;//��һ����ʱ��3508��ת��ʱ��
	 }
	 else if((catch_count==3)&&(rotational_delay>5)&&(rotational_delay<10))
	 {
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);	//�н�	
      catch_count++;
      rotational_delay = 0;//��һ����ʱ�Լн�
	 }
	 else if((catch_count==4)&&(rotational_delay ==1))
	 { 
		 if(SinSign==1)
			{
				SinRampInit(&Sin_Test);
				SinSign=0;
			}
		 Sin_Test.sin_ramp_switch = 1;
		 lift_angle = 156400;//����1
	 }
	 else if((catch_count==5)&&(Delay_100ms==1 ))
	 {
      rotate_angle0 = 0;//��ȡʱ��ת�ĽǶȣ���û����
      rotate_angle1 = (-rotate_angle0);
      catch_count++;
      rotational_delay = 0;

	 }
	 else if((rotational_delay<10)&&(rotational_delay>5)&&(catch_count==6))
	 {
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);//���ɼо�
      catch_count++;
	 }
	 else if(catch_count==7)
	{
		  if(SinSign==1)
			{
				SinRampInit(&Sin_Test);
				SinSign=0;
			}
      Sin_Test.sin_ramp_switch = 1;
      lift_angle = -156400;//�½�1
			Even_Take_Sign=2;
		}

    change_angle1 = SinRampCalc(&Sin_Test, lift_angle, 3, 3);
    
    Jscope_set_angle = lift_angle;
    Jscope_get_angle = Chassis_Motor[4].total_angle;
    Jscope_real_speed = Chassis_Motor[4].speed_rpm/REDUCTION_RATIO_3508;
	
    pid_calc(&Moto_Chassis_Pid_Pos[4], Chassis_Motor[4].total_angle, change_angle1);
    pid_calc(&Moto_Chassis_Pid_Spd[4], Chassis_Motor[4].speed_rpm, Moto_Chassis_Pid_Pos[4].pos_out*REDUCTION_RATIO_3508);
		 
		pid_calc(&Moto_Chassis_Pid_Pos[5], Chassis_Motor[5].total_angle, change_angle1);
    pid_calc(&Moto_Chassis_Pid_Spd[5], Chassis_Motor[5].speed_rpm, Moto_Chassis_Pid_Pos[5].pos_out*REDUCTION_RATIO_3508);
		
		pid_calc(&Moto_Chassis_Pid_Pos[6], Chassis_Motor[6].total_angle, rotate_angle0);
    pid_calc(&Moto_Chassis_Pid_Spd[6], Chassis_Motor[6].speed_rpm, Moto_Chassis_Pid_Pos[6].pos_out*REDUCTION_RATIO_3508);
		
		pid_calc(&Moto_Chassis_Pid_Pos[7], Chassis_Motor[7].total_angle, rotate_angle1);
    pid_calc(&Moto_Chassis_Pid_Spd[7], Chassis_Motor[7].speed_rpm, Moto_Chassis_Pid_Pos[7].pos_out*REDUCTION_RATIO_3508);
     
		
		if(RC_UPPER_RIGHT_SW_MID)
		{
			SetMotorValue(&hcan1, Moto_Chassis_Pid_Spd[4].pos_out, Moto_Chassis_Pid_Spd[5].pos_out,  Moto_Chassis_Pid_Spd[6].pos_out,  Moto_Chassis_Pid_Spd[7].pos_out); 
		}  
}

void CatchingControlOne2(void)//��ȡ����1�ķֽ⶯��2
{
  int32_t change_angle2;
    
  if(Even_Take_Sign==2)
  {
    {
        catch_count = 2;
    }
       
      if((catch_count==2)&&(Even_Take_Sign==2))
     {
        rotate_angle0 = 1111;//��ȡʱ��ת�ĽǶȣ���û����
       rotate_angle1 = (-rotate_angle0);
      //	catch_count++;
        rotational_delay = 0;//��һ����ʱ��3508��ת��ʱ��
     }
    else if((	rotational_delay >2)&&(rotational_delay<5)&&(catch_count==2))
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);//������
        Flip_Box_Delay=0;
        Flip_Box_Sign=1;
    }
    if((1< Flip_Box_Delay)&&(Flip_Box_Delay<4)&&(Flip_Box_Sign==1))
    {
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);//�ջص�����
      
        Flip_Box_Sign=0;
        catch_count++;
    }
     else if((catch_count==3)&&(rotational_delay>5)&&(rotational_delay<10))
     {
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);	//�н�	
       catch_count++;
       rotational_delay = 0;//��һ����ʱ�Լн�
     }
     else if((catch_count==4)&&(rotational_delay ==1))
     { 
       if(SinSign==1)
        {
          SinRampInit(&Sin_Test);
          SinSign=0;
        }
       Sin_Test.sin_ramp_switch = 1;
       lift_angle = 156400;//����1
     }
     else if((catch_count==5)&&(Delay_100ms==1 ))
     {
        rotate_angle0 = 0;//��ȡʱ��ת�ĽǶȣ���û����
        rotate_angle1 = (-rotate_angle0);
        catch_count++;
        rotational_delay = 0;
     }
     else if((rotational_delay<10)&&(rotational_delay>5)&&(catch_count==6))
     {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);//���ɼо�
        catch_count++;
       
     }
     else if(catch_count==7)
     { 
        if(SinSign==1)
        {
          SinRampInit(&Sin_Test);
          SinSign=0;
        }
        Sin_Test.sin_ramp_switch = 1;
        lift_angle = -156400;//�½�1
        Even_Take_Sign=3;      
     }

    
      change_angle2 = SinRampCalc(&Sin_Test, lift_angle, 3, 3);
      
      Jscope_set_angle = lift_angle;
      Jscope_get_angle = Chassis_Motor[4].total_angle;
      Jscope_real_speed = Chassis_Motor[4].speed_rpm/REDUCTION_RATIO_3508;
        
      pid_calc(&Moto_Chassis_Pid_Pos[4], Chassis_Motor[4].total_angle, change_angle2);
      pid_calc(&Moto_Chassis_Pid_Spd[4], Chassis_Motor[4].speed_rpm, Moto_Chassis_Pid_Pos[4].pos_out*REDUCTION_RATIO_3508);
       
      pid_calc(&Moto_Chassis_Pid_Pos[5], Chassis_Motor[5].total_angle, change_angle2);
      pid_calc(&Moto_Chassis_Pid_Spd[5], Chassis_Motor[5].speed_rpm, Moto_Chassis_Pid_Pos[5].pos_out*REDUCTION_RATIO_3508);
      
      pid_calc(&Moto_Chassis_Pid_Pos[6], Chassis_Motor[6].total_angle, rotate_angle0);
      pid_calc(&Moto_Chassis_Pid_Spd[6], Chassis_Motor[6].speed_rpm, Moto_Chassis_Pid_Pos[6].pos_out*REDUCTION_RATIO_3508);
      
      pid_calc(&Moto_Chassis_Pid_Pos[7], Chassis_Motor[7].total_angle, rotate_angle1);
      pid_calc(&Moto_Chassis_Pid_Spd[7], Chassis_Motor[7].speed_rpm, Moto_Chassis_Pid_Pos[7].pos_out*REDUCTION_RATIO_3508);
       
      
      if(RC_UPPER_RIGHT_SW_MID)
      {
        SetMotorValue(&hcan1, Moto_Chassis_Pid_Spd[4].pos_out, Moto_Chassis_Pid_Spd[5].pos_out,  Moto_Chassis_Pid_Spd[6].pos_out,  Moto_Chassis_Pid_Spd[7].pos_out); 
      }
    }  
}
void CatchingReset(void)//��λ���������ĸ�λ��ûд
{
  int32_t change_angle3;
  int32_t catch_reset_sign = 0;
  
  if(KEY_Z)
  {
    rotate_angle0 = 0;
    rotate_angle1 = (-rotate_angle0);//��λ
    rotational_delay = 0;
    catch_reset_sign = 1;
  }  
  else if((rotational_delay>6)&&(rotational_delay<10)&&(catch_reset_sign==1))//�Ȱ���ת��λ�������½�
  {
    if(SinSign==1)
    {
        SinRampInit(&Sin_Test);
        SinSign=0;
    }
    Sin_Test.sin_ramp_switch = 1;
    lift_angle = 0;//�½����ϵ�λ��
    
    catch_reset_sign = 0;//��Ҫʵ�ʵ��ԣ�����ȷ���Ƿ�д����
    Even_Take_Sign = 0;
  } 

  change_angle3 = SinRampCalc(&Sin_Test, lift_angle, 3, 3);
    
  pid_calc(&Moto_Chassis_Pid_Pos[4], Chassis_Motor[4].total_angle, change_angle3);
  pid_calc(&Moto_Chassis_Pid_Spd[4], Chassis_Motor[4].speed_rpm, Moto_Chassis_Pid_Pos[4].pos_out*REDUCTION_RATIO_3508);
       
  pid_calc(&Moto_Chassis_Pid_Pos[5], Chassis_Motor[5].total_angle, change_angle3);
  pid_calc(&Moto_Chassis_Pid_Spd[5], Chassis_Motor[5].speed_rpm, Moto_Chassis_Pid_Pos[5].pos_out*REDUCTION_RATIO_3508);
   
  pid_calc(&Moto_Chassis_Pid_Pos[6], Chassis_Motor[6].total_angle, rotate_angle0);
  pid_calc(&Moto_Chassis_Pid_Spd[6], Chassis_Motor[6].speed_rpm, Moto_Chassis_Pid_Pos[6].pos_out*REDUCTION_RATIO_3508);
      
  pid_calc(&Moto_Chassis_Pid_Pos[7], Chassis_Motor[7].total_angle, rotate_angle1);
  pid_calc(&Moto_Chassis_Pid_Spd[7], Chassis_Motor[7].speed_rpm, Moto_Chassis_Pid_Pos[7].pos_out*REDUCTION_RATIO_3508);
    
  SetMotorValue(&hcan1, Moto_Chassis_Pid_Spd[4].pos_out, Moto_Chassis_Pid_Spd[5].pos_out,  Moto_Chassis_Pid_Spd[6].pos_out,  Moto_Chassis_Pid_Spd[7].pos_out);  
}

