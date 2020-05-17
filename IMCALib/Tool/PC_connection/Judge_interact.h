#ifndef _Judge_INTERACT_H
#define _Judge_INTERACT_H

#include "stm32f4xx_hal.h"


/*--------����ϵͳЭ��--------*/
//���ڹٷ�����ϵͳ��һЩע��
//����ϵͳ���͵�ÿһ֡���ݲ���������֡ͷ���ȹ̶���֡βCRC16����У��ʱ��Ҫͳ�Ƴ�ÿ֡�����ݳ��ȣ�
//Ҫע�����CRC8У��Ϊһ�ֽڣ�CRC16Ϊ���ֽڡ�CRC8��ͷУ�飬CRC16������������У��
//-------------header----------------//-cmd_id-//--data------//---tail-----//
//--SOF-----datalenth---seq---CRC8--//--------//------------//---CRC16----//
//--0XA5-----2byte---1byte----1byte//--2byte-//---nbyte-----//---2byte----//
/*--ע��--*/
//�Ӿ�����������16λ����ƫ�����ݸߵͰ�λ��ת��Ҫ����һ��ת����

#define    SE_BUFFER_SIZE 20  //����ϵͳ�������ݻ����ֽ�
#define    JUD_RC_BUFFER_SIZE 200 //����ϵͳ�������ݻ����ֽ�
//��ʼ�ֽڣ� Ϊ0XA5
#define    JUDGE_FRMAE_HEADER 0XA5

//Э��֡����
#define    JUDGE_LEN_HAEDER 5 //֡ͷ��
#define    JUDGE_LEN_CMDID 2 //�����볤�� 0xXXXX
#define    JUDGE_LEN_TAIL  2 //֡β����
//��֡ƫ����
typedef enum 
{
	FRAME_HEADER         = 0,//ͷƫ��λ��
	CMD_ID               = 5,//������ƫ��λ��
	DATA                 = 7,//���ݶ�ƫ��λ��
	
}JudgeFrameOffset;
//֡ͷƫ��λ��
typedef enum
{
	SOF        =0, //��ʼλ
	DATA_LENGTH=1,//֡���ݳ��ȣ�������ϵͳ���η��͵����ݳ��ȣ�
	SEQ        =3,//�����
	CRC8       =4,//CRC8У��
}FrameHeaderOffset;	
/***************������ID(2019����ϵͳ)********************/

/* 

	ID: 0x0001  Byte:  3    ����״̬����       			����Ƶ�� 1Hz      
	ID: 0x0002  Byte:  1    �����������         		������������      
	ID: 0x0003  Byte:  2    ���������˴������   		1Hz����  
	ID: 0x0101  Byte:  4    �����¼�����   				 �¼��ı����
	ID: 0x0102  Byte:  3    ���ز���վ������ʶ����    	�����ı���� 
	ID: 0X0103  Byte:  2    ���ز���վԤԼ�ӵ�����      �����ӷ��ͣ�10Hz 
	ID: 0X0201  Byte: 15    ������״̬����        		10Hz
	ID: 0X0202  Byte: 14    ʵʱ������������   			  50Hz       
	ID: 0x0203  Byte: 16    ������λ������           	10Hz
	ID: 0x0204  Byte:  1    ��������������           	����״̬�ı����
	ID: 0x0205  Byte:  3    ���л���������״̬����      10Hz
	ID: 0x0206  Byte:  1    �˺�״̬����           		�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           		�ӵ��������
	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz
	
*/
//������ID,�����жϽ��յ���ʲô����
typedef enum
{ 
	ID_game_state       			    = 0x0001,//����״̬����
	ID_game_result 	   				    = 0x0002,//�����������
	ID_game_robot_survivors       = 0x0003,//���������˴������
	ID_event_data  					      = 0x0101 ,//�����¼����� 
	ID_supply_projectile_action   = 0x0102,//���ز���վ������ʶ����
	ID_supply_projectile_booking 	= 0x0103,//���ز���վԤԼ�ӵ�����
	ID_game_robot_state    			  = 0x0201,//������״̬����
	ID_power_heat_data    			  = 0x0202,//ʵʱ������������
	ID_game_robot_pos        		  = 0x0203,//������λ������
	ID_buff_musk					        = 0x0204,//��������������
	ID_aerial_robot_energy			  = 0x0205,//���л���������״̬����
	ID_robot_hurt					        = 0x0206,//�˺�״̬����
	ID_shoot_data					        = 0x0207,//ʵʱ�������

} CmdID;

//���������ݶγ���
typedef enum
{	
	LEN_game_state       				=  3,	//0x0001
	LEN_game_result       				=  1,	//0x0002
	LEN_game_robot_survivors       		=  2,	//0x0003
	LEN_event_data  					=  4,	//0x0101
	LEN_supply_projectile_action        =  3,	//0x0102
	LEN_supply_projectile_booking		=  2,	//0x0103
	LEN_game_robot_state    			= 15,	//0x0201
	LEN_power_heat_data   				= 14,	//0x0202
	LEN_game_robot_pos        			= 16,	//0x0203
	LEN_buff_musk        				=  1,	//0x0204
	LEN_aerial_robot_energy        		=  3,	//0x0205
	LEN_robot_hurt        				=  1,	//0x0206
	LEN_shoot_data       				=  6,	//0x0207	
}JudgeDataLength;


//����ϵͳ��־λ����(����У��͵���)
typedef struct
{
	//֡ͷ
	uint8_t sof;//�׵�ַ
	uint16_t data_lenth;//����֡��data����
	uint8_t seq; //֡���
	uint8_t crc8;//CRC8У��λ
	//��������
	uint16_t cmdid;//������
	//��һ֡����
	uint8_t  next_frame_header;//��һ֡����ͷ
	//����β
	uint16_t crc16;//CRC16У��λ
}stoJudgeRecvData_t;

/***************����ϵͳ��������(2019����ϵͳ)********************/

/* ID: 0x0001  Byte:  3    ����״̬���� */
typedef __packed struct 
{ 
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
} ext_game_state_t; 


/* ID: 0x0002  Byte:  1    ����������� */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003  Byte:  2    ���������˴������ */
typedef __packed struct 
{ 
	uint16_t robot_legion;
} ext_game_robot_survivors_t; 


/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  3    ���ز���վ������ʶ���� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
} ext_supply_projectile_action_t; 


/* ID: 0X0103  Byte:  2    ���ز���վԤԼ�ӵ����� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;    
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 


/* ID: 0X0201  Byte: 15    ������״̬���� */
typedef __packed struct 
{ 
	uint8_t robot_id;                           //������ID��������У�鷢��
	uint8_t robot_level;                       //1һ����2������3����
	uint16_t remain_HP;                       //������ʣ��Ѫ��
	uint16_t max_HP;                         //��������Ѫ��
	uint16_t shooter_heat0_cooling_rate;     //������ 17mm �ӵ�������ȴ�ٶ� ��λ /s
	uint16_t shooter_heat0_cooling_limit;   // ������ 17mm �ӵ���������
	uint16_t shooter_heat1_cooling_rate;    
	uint16_t shooter_heat1_cooling_limit;   
	uint8_t mains_power_gimbal_output : 1;  //����ϵͳ��Դģ�������־λ 1Ϊ�����
	uint8_t mains_power_chassis_output : 1;  
	uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_state_t; 


/* ID: 0X0202  Byte: 14    ʵʱ������������ */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   //���������ѹ
	uint16_t chassis_current;//�����������    
	float chassis_power;   //˲ʱ���� 
	uint16_t chassis_power_buffer;//60������������
	uint16_t shooter_heat0;//17mmǹ������
	uint16_t shooter_heat1;//42mmǹ������  
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct  //10HZ
{   
	float x;              //��λm
	float y;              //��λm
	float z;              //��λm
	float yaw;            //ǹ��λ�� ��λ��
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_musk_t; 


/* ID: 0x0205  Byte:  3    ���л���������״̬���� */
typedef __packed struct 
{ 
	uint8_t energy_point;
	uint8_t attack_time; 
} aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct 
{ 
	uint8_t armor_id : 4;  //bit0-3 �����˺�װ��ID
	uint8_t hurt_type : 4; //bit4-7 �˺����� 0x0װ���˺� 0X1ģ������˺� 0x2��ǹ��������Ѫ 0x3 �����̹��ʿ�Ѫ
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    ʵʱ������� */
typedef __packed struct 
{ 
	uint8_t bullet_type;  //����ӵ�����1��17mm���� 2��42mm���� 
	uint8_t bullet_freq;  //�ӵ���Ƶ HZ 
	float bullet_speed;   //�ӵ����� m/s
} ext_shoot_data_t; 

/*������Ϣ�ṹ�壨δ�ã�*/
typedef struct
{
	uint8_t bullet_type;
	uint8_t fire_frequence;
	uint16_t tiny_shooter_heat;
	uint16_t thick_shooter_heat;
	float   bullet_speed;	
}sto_fire_info_t;



extern uint8_t Jud_Rx_Buffer[JUD_RC_BUFFER_SIZE];//���ջ�������
extern uint8_t Judge_Buffer[JUD_RC_BUFFER_SIZE];//�Ӿ���������
/*--------�������ݶ�ȡ--------*/
void JudgeRead(uint8_t *RecvUsartData);//����ϵͳ����
void HAL_UART6_Receive_IT_IDLE(void);//����6�����жϿ���
/*--------�������ݵ���--------*/
float RoboChassisHeat(void);//���ʶ�ȡ
uint16_t RoboHP(void);//������Ѫ����ȡ
float FireSpeed(void);//����
uint16_t FireHeat17(void);//17mm����
uint8_t FireFeq(void);//��Ƶ
uint16_t FireNum(void);//ͳ�Ʒ�����
uint16_t FireCoolingLimit(void);//ǹ��������ȴ����
uint16_t FireCoolingRate(void);//ǹ������ÿ����ȴֵ

/*���ڿ����жϽ��ղ���ϵͳ���ݣ�������Ӧ�Ĵ����ж���ִ��*/
void UART_Receive_IT_IDLE_Judge(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

/*--------������ӡ--------*/
void ShowJudgeMeassge(void);//JSCOOP������ӡ
#endif
