#include "PrintfInfo.h"
#include "CanBus_Task.h"
#include "RC_Task.h"
#include "stdio.h"
#include "IOI2C.h"


uint8_t Printf_Info;

void PrintfInfo(void)
{
    
    #if defined (REMOTE_MEASURE)    /*打印遥控器各通道信息*/
        printf("\n\r The RC ch1.ch2.ch3.ch4 is:%d  %d  %d  %d\n\r",remote_control.ch1,remote_control.ch2,remote_control.ch3,remote_control.ch4);
        printf("\n\r The RC switch_left and switch_right is:%d  %d\n\r",remote_control.switch_left,remote_control.switch_right);
    
    #elif defined (M3508_MOTOR_MEASURE)  /*打印底盘3508电机的信息*/
        printf("\n\r The motor angle and speed_rpm is:%d  %d\n\r", Chassis_Motor[0].angle, Chassis_Motor[0].speed_rpm);  //打印电机的转子机械角度和转子转度
        printf("\n\r The motor current is:%f\n\r", Chassis_Motor[0].real_current);  //打印电机转矩电流
        printf("\n\r The motor hall is:%d\n\r", Chassis_Motor[0].hall);  //打印电机温度
        
    #elif defined (M6020_MOTOR_MEASURE)  /*打印云台6020电机的信息*/
        printf("\n\r The motor angle and total_angle is:%d  %d\n\r",Gimbal_Motor[0].angle, Gimbal_Motor[0].total_angle);  //打印电机的转子机械角度和转子转度
//        printf("\n\r Motor speed_rpm: %d \n\r", Gimbal_Motor[0].speed_rpm); //转过的总角度
//        printf("\n\r The motor current is:%f\n\r",Gimbal_Motor[0].real_current);  //打印电机转矩电流
    
    #elif defined (IMU_MEASURE)
        printf("0x50:   Angle:%.3f %.3f %.3f \r\n",ImuData.Angle[0],ImuData.Angle[1],ImuData.Angle[2]);
    
    #else
        printf("\n\r The offset_angle is:%d\n\r",moto_chassis[0].offset_angle );
        
        printf("\n\r The total_angle is:%d\n\r",moto_chassis[0].total_angle );
    #endif
		
	
	/*注：电机转子机械角度值范围：0 ~ 8192
	      转子转速值的单位为：rpm
	      电机温度的单位为：摄氏度
	*/

}

