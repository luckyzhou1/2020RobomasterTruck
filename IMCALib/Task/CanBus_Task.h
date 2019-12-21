#ifndef __CANBUS_TASK
#define __CANBUS_TASK


#include "mytype.h"
#include "can.h"
#include "pid.h"


#define FILTER_BUF_LEN		5


/*定义CAN发送或是接收的ID*/
typedef enum
{

	CAN_3508Moto1_ID = 0x201,
	CAN_3508Moto2_ID = 0x202,
	CAN_3508Moto3_ID = 0x203,
	CAN_3508Moto4_ID = 0x204,
    CAN_YAW_Motor_ID = 0x205,
    CAN_PITCH_Motor_ID = 0x206,
	
}CAN_Message_ID;


/*接收到的云台电机的参数结构体*/
typedef struct{
	int16_t	 	speed_rpm;
    float  	    real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;		//abs angle range:[0,8191]
	uint16_t 	last_angle;	  //abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	u8			buf_idx;
	u16			angle_buf[FILTER_BUF_LEN];
	u16			fited_angle;
	u32			msg_cnt;
}moto_measure_t;


extern moto_measure_t  Chassis_Motor[];  //底盘电机参数结构体
extern moto_measure_t  Gimbal_Motor[];  //云台电机参数结构体


/*CAN1过滤器的配置和CAN的开启*/
void CANFilterInit(void);
/*获得电机的机械角度*/
void GetMotorOffset(moto_measure_t *ptr, uint8_t can_rx_data[]);
/*接收3508电机通过CAN发过来的信息*/
void GetMotorMeasure(moto_measure_t *ptr, uint8_t can_rx_data[]);
/*发送电机的信息到CAN总线上，此函数用于一路CAN的前4个电机的控制*/
void SetChassisMotorCurrent(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4);
/*发送电机的信息到CAN总线上，此函数用于一路CAN的后4个电机的控制*/
void SetGimbalMotorValue(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4);



#endif

