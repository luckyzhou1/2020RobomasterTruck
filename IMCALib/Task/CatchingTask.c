#include "CatchingTask.h"
#include "ramp.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "ChassisControl.h"
#include "ControlTask.h"
#include "gpio.h"

SinRampState Sin_Test;
int32_t lift_angle;//抬升角度
int32_t rotate_angle0;//3508翻转的角度
int32_t rotate_angle1;
int32_t set_speed; 
int delay_100ms; //延时100ms
int sinsign=1;//斜坡函数是否已经初始化的标志位
int catching_sign=1;//判断是否进行夹取的标志卫

int flip_box_delay;//弹箱子延时
int flip_box_sign;//弹箱子标志位
int Even_Take_Sign=0;//用于判断是哪一步分解动作

int rotational_delay;
int Rotational_Sign;
/************************J-Scope波形显示定义*************************/
float Jscope_set_angle;
float Jscope_get_angle;
float Jscope_real_speed;

/******************************END***********************************/
void CatchingControlOne1(void)//夹取动作1的分解动作1
{
  int32_t change_angle1;
  
  if(KEY_C)
	{
		if(sinsign==1)
			{
				SinRampInit(&Sin_Test);
				sinsign=0;
			}
		 Sin_Test.sin_ramp_switch = 1;
		 lift_angle = -313000;
	
	}
 
  if(KEY_R)
	{
		if(sinsign==1)
			{
				SinRampInit(&Sin_Test);
				sinsign=0;
			}
		 Sin_Test.sin_ramp_switch = 1;
		 lift_angle = 313000;//上升2
	
	}
 
  if(KEY_Q)
	{		
      catch_count = 2;
      Even_Take_Sign=1;//初始化
	}
		 
	 if((catch_count==2)&&(Even_Take_Sign==1)&&(catching_sign==1))
	 {
			rotate_angle0 = 1111;//夹取时旋转的角度，还没调试
     rotate_angle1 = (-rotate_angle0);
			catch_count++;
			rotational_delay = 0;//给一个延时给3508旋转的时间
     catching_sign = 0;
	 }
	 else if((catch_count==3)&&(rotational_delay>5)&&(rotational_delay<10))
	 {
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);	//夹紧	
      catch_count++;
      rotational_delay = 0;//给一个延时以夹紧
	 }
	 else if((catch_count==4)&&(rotational_delay ==1))
	 { 
		 if(sinsign==1)
			{
				SinRampInit(&Sin_Test);
				sinsign=0;
			}
		 Sin_Test.sin_ramp_switch = 1;
		 lift_angle = 156400;//上升1
	 }
	 else if((catch_count==5)&&(delay_100ms==1 ))
	 {
      rotate_angle0 = 0;//夹取时旋转的角度，还没调试
      rotate_angle1 = (-rotate_angle0);
      catch_count++;
      rotational_delay = 0;

	 }
	 else if((rotational_delay<10)&&(rotational_delay>5)&&(catch_count==6))
	 {
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);//放松夹具
      catch_count++;
	 }
	 else if(catch_count==7)
	{
		  if(sinsign==1)
			{
				SinRampInit(&Sin_Test);
				sinsign=0;
			}
      Sin_Test.sin_ramp_switch = 1;
      lift_angle = -156400;//下降1
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

void CatchingControlOne2(void)//夹取动作1的分解动作2
{
  int32_t change_angle2;
    
  if(Even_Take_Sign==2)
  {
    {
        catch_count = 2;
    }
       
      if((catch_count==2)&&(Even_Take_Sign==2))
     {
        rotate_angle0 = 1111;//夹取时旋转的角度，还没调试
       rotate_angle1 = (-rotate_angle0);
      //	catch_count++;
        rotational_delay = 0;//给一个延时给3508旋转的时间
     }
    else if((	rotational_delay >2)&&(rotational_delay<5)&&(catch_count==2))
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);//弹箱子
        flip_box_delay=0;
        flip_box_sign=1;
    }
    if((1< flip_box_delay)&&(flip_box_delay<4)&&(flip_box_sign==1))
    {
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);//收回弹箱子
      
        flip_box_sign=0;
        catch_count++;
    }
     else if((catch_count==3)&&(rotational_delay>5)&&(rotational_delay<10))
     {
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);	//夹紧	
       catch_count++;
       rotational_delay = 0;//给一个延时以夹紧
     }
     else if((catch_count==4)&&(rotational_delay ==1))
     { 
       if(sinsign==1)
        {
          SinRampInit(&Sin_Test);
          sinsign=0;
        }
       Sin_Test.sin_ramp_switch = 1;
       lift_angle = 156400;//上升1
     }
     else if((catch_count==5)&&(delay_100ms==1 ))
     {
        rotate_angle0 = 0;//夹取时旋转的角度，还没调试
        rotate_angle1 = (-rotate_angle0);
        catch_count++;
        rotational_delay = 0;
     }
     else if((rotational_delay<10)&&(rotational_delay>5)&&(catch_count==6))
     {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);//放松夹具
        catch_count++;
       
     }
     else if(catch_count==7)
     { 
        if(sinsign==1)
        {
          SinRampInit(&Sin_Test);
          sinsign=0;
        }
        Sin_Test.sin_ramp_switch = 1;
        lift_angle = -156400;//下降1
        Even_Take_Sign=3;
        catching_sign = 0;        
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
void CatchingReset(void)//复位动作，阀的复位还没写
{
  int32_t change_angle3;
  int32_t catch_reset_sign = 0;
  
  if(KEY_Z)
  {
    rotate_angle0 = 0;
    rotate_angle1 = (-rotate_angle0);//复位
    rotational_delay = 0;
    catch_reset_sign = 1;
  }  
  else if((rotational_delay>6)&&(rotational_delay<10)&&(catch_reset_sign==1))//先把旋转复位，才能下降
  {
    if(sinsign==1)
    {
        SinRampInit(&Sin_Test);
        sinsign=0;
    }
    Sin_Test.sin_ramp_switch = 1;
    lift_angle = 0;//下降到上电位置
    
    catch_reset_sign = 0;//还要实际调试，才能确定是否写在这
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

