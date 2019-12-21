#ifndef __PRINTFINFO
#define __PRINTFINFO

#include "mytype.h"


/*想串口显示那款电机的信息就取消那个电机的宏定义的注释*/
//#define M6020_MOTOR_MEASURE  //云台6020
#define M3508_MOTOR_MEASURE  //底盘3508
//#define REMOTE_MEASURE
#define IMU_MEASURE  //陀螺仪信息

extern uint8_t Printf_Info;

void PrintfInfo(void);


#endif
