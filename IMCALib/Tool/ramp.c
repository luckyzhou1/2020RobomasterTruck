#include "ramp.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "ChassisControl.h"
#include "CatchingTask.h"
#include "ControlTask.h"

/*一阶低通滤波结构体*/
first_order_filter_type_t chassis_cmd_slow_set_vx;
first_order_filter_type_t chassis_cmd_slow_set_vy;


float Sin_up_table[200+2] = {0,6.1684e-05,0.00024672,0.00055506,0.00098664,0.0015413,0.002219,0.0030195,0.0039426,0.0049882,0.0061558,0.0074453,0.0088564,0.010389,0.012042,0.013815,0.015708,0.017721,0.019853,0.022103,0.024472,0.026957,0.02956,0.032278,0.035112,0.03806,0.041123,0.044298,0.047586,0.050986,0.054497,0.058117,0.061847,0.065684,0.069629,0.07368,0.077836,0.082096,0.08646,0.090925,0.095492,0.10016,0.10492,0.10978,0.11474,0.1198,0.12494,0.13018,0.13552,0.14094,0.14645,0.15204,0.15773,0.16349,0.16934,0.17528,0.18129,0.18738,0.19355,0.19979,0.20611,0.2125,0.21896,0.22549,0.23209,0.23875,0.24548,0.25227,0.25912,0.26604,0.273,0.28003,0.28711,0.29424,0.30143,0.30866,0.31594,0.32326,0.33063,0.33804,0.34549,0.35298,0.3605,0.36806,0.37566,0.38328,0.39093,0.39861,0.40631,0.41404,0.42178,0.42955,0.43733,0.44513,0.45295,0.46077,0.4686,0.47645,0.48429,0.49215,0.5,0.50785,0.51571,0.52355,0.5314,0.53923,0.54705,0.55487,0.56267,0.57045,0.57822,0.58596,0.59369,0.60139,0.60907,0.61672,0.62434,0.63194,0.6395,0.64702,0.65451,0.66196,0.66937,0.67674,0.68406,0.69134,0.69857,0.70576,0.71289,0.71997,0.727,0.73396,0.74088,0.74773,0.75452,0.76125,0.76791,0.77451,0.78104,0.7875,0.79389,0.80021,0.80645,0.81262,0.81871,0.82472,0.83066,0.83651,0.84227,0.84796,0.85355,0.85906,0.86448,0.86982,0.87506,0.8802,0.88526,0.89022,0.89508,0.89984,0.90451,0.90907,0.91354,0.9179,0.92216,0.92632,0.93037,0.93432,0.93815,0.94188,0.9455,0.94901,0.95241,0.9557,0.95888,0.96194,0.96489,0.96772,0.97044,0.97304,0.97553,0.9779,0.98015,0.98228,0.98429,0.98618,0.98796,0.98961,0.99114,0.99255,0.99384,0.99501,0.99606,0.99698,0.99778,0.99846,0.99901,0.99944,0.99975,0.99994,1,1};

float Sin_down_table[200+2] = {1,0.99994,0.99975,0.99944,0.99901,0.99846,0.99778,0.99698,0.99606,0.99501,0.99384,0.99255,0.99114,0.98961,0.98796,0.98618,0.98429,0.98228,0.98015,0.9779,0.97553,0.97304,0.97044,0.96772,0.96489,0.96194,0.95888,0.9557,0.95241,0.94901,0.9455,0.94188,0.93815,0.93432,0.93037,0.92632,0.92216,0.9179,0.91354,0.90907,0.90451,0.89984,0.89508,0.89022,0.88526,0.8802,0.87506,0.86982,0.86448,0.85906,0.85355,0.84796,0.84227,0.83651,0.83066,0.82472,0.81871,0.81262,0.80645,0.80021,0.79389,0.7875,0.78104,0.77451,0.76791,0.76125,0.75452,0.74773,0.74088,0.73396,0.727,0.71997,0.71289,0.70576,0.69857,0.69134,0.68406,0.67674,0.66937,0.66196,0.65451,0.64702,0.6395,0.63194,0.62434,0.61672,0.60907,0.60139,0.59369,0.58596,0.57822,0.57045,0.56267,0.55487,0.54705,0.53923,0.5314,0.52355,0.51571,0.50785,0.5,0.49215,0.48429,0.47645,0.4686,0.46077,0.45295,0.44513,0.43733,0.42955,0.42178,0.41404,0.40631,0.39861,0.39093,0.38328,0.37566,0.36806,0.3605,0.35298,0.34549,0.33804,0.33063,0.32326,0.31594,0.30866,0.30143,0.29424,0.28711,0.28003,0.273,0.26604,0.25912,0.25227,0.24548,0.23875,0.23209,0.22549,0.21896,0.2125,0.20611,0.19979,0.19355,0.18738,0.18129,0.17528,0.16934,0.16349,0.15773,0.15204,0.14645,0.14094,0.13552,0.13018,0.12494,0.1198,0.11474,0.10978,0.10492,0.10016,0.095492,0.090925,0.08646,0.082096,0.077836,0.07368,0.069629,0.065684,0.061847,0.058117,0.054497,0.050986,0.047586,0.044298,0.041123,0.03806,0.035112,0.032278,0.02956,0.026957,0.024472,0.022103,0.019853,0.017721,0.015708,0.013815,0.012042,0.010389,0.0088564,0.0074453,0.0061558,0.0049882,0.0039426,0.0030195,0.002219,0.0015413,0.00098664,0.00055506,0.00024672,6.1684e-05,0,0};



/*************************************************************************************************************
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
*************************************************************************************************************/
void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/*************************************************************************************************************
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
*************************************************************************************************************/
void FirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}


//底盘斜坡初始化
void ChassisRampInit(void)
{
    const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //用一阶滤波代替斜坡函数生成
    FirstOrderFilterInit(&chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    FirstOrderFilterInit(&chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    
}



int32_t SinRampCalc(SinRampState *sin_state, int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down)//不断地进入这个函数
{
        
    int32_t variation_out;
    const int Sin_point = 200;
    static int Sin_up_count, Sin_down_count, Sin_up_x, Sin_down_x, last_out;
    
    
    if(variation > 0)//正转
    {
        if((sin_state->compare_value_up < variation)&&(sin_state->sin_ramp_switch == 1))//电机需要SIN函数转时一直进入
        {
            
            variation_out = variation*(((Sin_up_table[Sin_up_count+1]-Sin_up_table[Sin_up_count])*(Sin_up_x))/Sin_control_time_up + Sin_up_table[Sin_up_count]) + last_out;//每次输出一点
            /*每次进入都改变下面的一些参数*/
            Sin_up_x++;
            if(Sin_up_x >Sin_control_time_up)
            {
                 Sin_up_x = 1;
                 Sin_up_count++;//计数加一
            }
            if(Sin_up_count>Sin_point)
            {
                Sin_up_count = Sin_point;//不断计数，直到相等
            }
            
            if(variation_out == variation + last_out)//如果电机达到设定值
            {
               Sin_up_count = 0;
               Sin_up_x = 0;//归零          
               sin_state->compare_value_up = variation;
               
               sinsign=1;
               rotational_delay = 0;
               catch_count++;
               delay_100ms = 0; 
              
               last_out += variation;//每次的变化量累加
            }
            
        }
        else
        {
            variation_out = last_out;//当没有值输入时，保持上次的位置
        }
        
    }
    else if (variation < 0)//反转
    {
        if((sin_state->compare_value_dowm > variation)&&(sin_state->sin_ramp_switch == 1))
        {
            variation_out = (-variation)*(((Sin_down_table[Sin_down_count+1]-Sin_down_table[Sin_down_count])*(Sin_down_x))/Sin_control_time_down + Sin_down_table[Sin_down_count]) + (last_out + variation);  
            Sin_down_x++;
            if(Sin_down_x >Sin_control_time_down)
            {
                 Sin_down_x = 1;
                 Sin_down_count++;
            }
            if(Sin_down_count>Sin_point)
            {
                Sin_down_count = Sin_point;
            }
            if(variation_out == last_out + variation)
            {
                Sin_down_count = 0;
                Sin_down_x = 0;
                
                sin_state->compare_value_dowm = variation;
						  	sinsign=1;
							  rotational_delay = 0;
							  catch_count++;
                delay_100ms = 0; 
              
                last_out += variation;
            }
            
        }
        else
        {
            variation_out = last_out;
        }
            
    }
    else
    {
        variation_out = 0;
    }
		
    return variation_out;
    
}


void SinRampInit(SinRampState *sin_state)
{
    
    sin_state->sin_ramp_switch = 0;
    sin_state->compare_value_up = 0;
    sin_state->compare_value_dowm = 0;
    
}

/**************************************斜坡测试**************************************/

//SinRampState Sin_Test;
//int32_t lift_angle;
//int32_t set_speed; 

//void SinTest(void)
//{
////    int32_t change_angle;
//    
//    if(RC_UPPER_LEFT_SW_MID)
//    {
////        SinRampInit(&Sin_Test);
//        lift_angle = 0;
//        
////        set_speed = 0;
//        
//    }
//    else if(RC_UPPER_LEFT_SW_UP)
//    {
////        Sin_Test.sin_ramp_switch = 1;
//        lift_angle = 6400;
//        
////        set_speed = 100;
//    }
//    else if(RC_UPPER_LEFT_SW_DOWN)
//    {
////        Sin_Test.sin_ramp_switch = 1;
//        lift_angle = -3200;
//        
////        set_speed = -150;
//    }
//    
////    change_angle = SinRampCalc(&Sin_Test, lift_angle, 3, 3);
//    
//    pid_calc(&Moto_Chassis_Pid_Pos[4], Chassis_Motor[4].total_angle, lift_angle);
//    pid_calc(&Moto_Chassis_Pid_Spd[4], Chassis_Motor[4].speed_rpm, Moto_Chassis_Pid_Pos[4].pos_out*REDUCTION_RATIO_3508);
//    
//    SetMotorValue(&hcan1, Moto_Chassis_Pid_Spd[4].pos_out, 0, 0, 0); 
//    
//}


/**************************************END*******************************************/


