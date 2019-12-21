#ifndef __CANBUS_TASK
#define __CANBUS_TASK


#include "mytype.h"
#include "can.h"
#include "pid.h"


#define FILTER_BUF_LEN		5


/*����CAN���ͻ��ǽ��յ�ID*/
typedef enum
{

	CAN_3508Moto1_ID = 0x201,
	CAN_3508Moto2_ID = 0x202,
	CAN_3508Moto3_ID = 0x203,
	CAN_3508Moto4_ID = 0x204,
    CAN_YAW_Motor_ID = 0x205,
    CAN_PITCH_Motor_ID = 0x206,
	
}CAN_Message_ID;


/*���յ�����̨����Ĳ����ṹ��*/
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


extern moto_measure_t  Chassis_Motor[];  //���̵�������ṹ��
extern moto_measure_t  Gimbal_Motor[];  //��̨��������ṹ��


/*CAN1�����������ú�CAN�Ŀ���*/
void CANFilterInit(void);
/*��õ���Ļ�е�Ƕ�*/
void GetMotorOffset(moto_measure_t *ptr, uint8_t can_rx_data[]);
/*����3508���ͨ��CAN����������Ϣ*/
void GetMotorMeasure(moto_measure_t *ptr, uint8_t can_rx_data[]);
/*���͵������Ϣ��CAN�����ϣ��˺�������һ·CAN��ǰ4������Ŀ���*/
void SetChassisMotorCurrent(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4);
/*���͵������Ϣ��CAN�����ϣ��˺�������һ·CAN�ĺ�4������Ŀ���*/
void SetGimbalMotorValue(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4);



#endif

