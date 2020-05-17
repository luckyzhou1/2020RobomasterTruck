#include "Driver_Judge.h"
#include "Judge_interact.h"
#include "Vision_interact.h"
#include "string.h"
#include "usart.h"
#include "stdio.h"
#include "user_lib.h"



/*--------����ϵͳ��ȡV1.0--------*/
/*-----------------------------------------------------------------------
//��ǰ����ϵͳ�����˵��̹��ʺ���Ƶ��ǹ�������ȵĶ�ȡ������ϵͳ������ʱ��û������
//����3DMAͨѶ
//��������
//����ϵͳ��һ�����ݷ����У��������֡�����ݡ����ڵ�ǰ������û����������
//��һ�����ݷ��͵�ʱ�����յ�������λ��Ϊ0XA5��֡,ʹУ��ʧ�ܡ�����һ���̶ȵĵ���
//����Ӱ��ǰһ֡�ѹ�У��Ķ�ȡ����׼�ȡ�
*--------------------------------------------------------------------*/

/*Jscope���Բ���*/
float  Jscope_Chassis_Heat;//���̹���
float  Jscope_Fire_Speed;  //��������
uint8_t  Jscope_Fire_Feq;  //��Ƶ
int16_t  Jscope_Fire_Heat; //17mmǹ������
int16_t  Jscope_Fire_Num;  //������
uint16_t USART5_RX_Lenth = 0;  //����״̬���	


/*****************����ϵͳ���ݶ���**********************/
ext_game_state_t       				    Game_State;					      //0x0001
ext_game_result_t            		  Game_Result;				      //0x0002
ext_game_robot_survivors_t        Game_Robot_Survivors;     //0x0003
ext_event_data_t        			    Event_Data;					      //0x0101
ext_supply_projectile_action_t		Supply_Projectile_Action;	//0x0102
ext_supply_projectile_booking_t		Supply_Projectile_Booking;//0x0103
ext_game_robot_state_t			  	  Game_Robot_Stat;				  //0x0201
ext_power_heat_data_t		  		    Power_Heat_Data;				  //0x0202
ext_game_robot_pos_t				      Game_Robot_Pos;				    //0x0203
ext_buff_musk_t						        Buff_Musk;					      //0x0204
aerial_robot_energy_t			        Aerial_Robot_Energy;		  //0x0205
ext_robot_hurt_t					        Robot_Hurt;					      //0x0206
ext_shoot_data_t					        Shoot_Data;					      //0x0207

stoJudgeRecvData_t  Judge_Recv_Data;//����ϵͳУ���־λ���սṹ��

uint8_t Jud_Tx_Buffer[SE_BUFFER_SIZE] ={0};//���ͻ�������
uint8_t Jud_Rx_Buffer[JUD_RC_BUFFER_SIZE] ={0};//���ջ�������

/*-----------------------------------------------------------------------
\@brief ����ϵͳ���ݶ�ȡ
\@param ��
\@return ��
\@reference 
*--------------------------------------------------------------------*/
void JudgeRead(uint8_t *RecvUsartData)
{
	uint8_t Judge_length =0;//֡���ݳ���

	
	//У���־λ��ֵ(���ڴ��ڴ�ӡ����)
	
	Judge_Recv_Data.crc8 =(RecvUsartData[4]);
	Judge_Recv_Data.data_lenth=(RecvUsartData[2]<<8|RecvUsartData[1]);
	Judge_Recv_Data.cmdid=(RecvUsartData[6]<<8|RecvUsartData[5]);

	//�ж�֡ͷ���ֽ��Ƿ�Ϊ0XA5
	if(RecvUsartData[0] == JUDGE_FRMAE_HEADER)
	{
		//CRC8֡ͷУ��
		if(Verify_CRC8_Check_Sum(RecvUsartData, JUDGE_LEN_HAEDER) ==TRUE)
		{
			//����һ֡���ݵĳ��ȣ�����CRC16У��
			Judge_length = RecvUsartData[DATA_LENGTH] + JUDGE_LEN_HAEDER + JUDGE_LEN_CMDID + JUDGE_LEN_TAIL;
			
			Judge_Recv_Data.crc16 =(RecvUsartData[Judge_length-1]<<8|RecvUsartData[Judge_length-2]);
			//CRC16֡βУ��
			if(Verify_CRC16_Check_Sum(RecvUsartData, Judge_length) ==TRUE)
		 {
				switch(Judge_Recv_Data.cmdid)
				{
					case ID_game_state:        			//0x0001
						memcpy(&Game_State, (RecvUsartData + DATA), LEN_game_state);
					break;
					
					case ID_game_result:          		//0x0002
						memcpy(&Game_Result, (RecvUsartData + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_survivors:       //0x0003
						memcpy(&Game_Robot_Survivors, (RecvUsartData + DATA), LEN_game_robot_survivors);
					break;
					
					case ID_event_data:    				//0x0101
						memcpy(&Event_Data, (RecvUsartData + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:   //0x0102
						memcpy(&Supply_Projectile_Action, (RecvUsartData + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_supply_projectile_booking:  //0x0103
						memcpy(&Supply_Projectile_Booking, (RecvUsartData + DATA), LEN_supply_projectile_booking);
					break;
					
					case ID_game_robot_state:      		//0x0201
						memcpy(&Game_Robot_Stat, (RecvUsartData + DATA), LEN_game_robot_state);;
					break;
					
					case ID_power_heat_data:      		//0x0202
						memcpy(&Power_Heat_Data, (RecvUsartData + DATA), LEN_power_heat_data);

					break;
					
					case ID_game_robot_pos:      		//0x0203
						memcpy(&Game_Robot_Pos, (RecvUsartData + DATA), LEN_game_robot_pos);
					  
					break;
					
					case ID_buff_musk:      			//0x0204
						memcpy(&Buff_Musk, (RecvUsartData + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205
						memcpy(&Aerial_Robot_Energy, (RecvUsartData + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      			//0x0206
						memcpy(&Robot_Hurt, (RecvUsartData + DATA), LEN_robot_hurt);
					break;
					
					case ID_shoot_data:      			//0x0207
						memcpy(&Shoot_Data, (RecvUsartData + DATA), LEN_shoot_data);
            FireNum();//��¼������
					break;
				}

				Judge_Recv_Data.next_frame_header =*(RecvUsartData+JUDGE_LEN_HAEDER+JUDGE_LEN_CMDID+JUDGE_LEN_TAIL+Judge_Recv_Data.data_lenth);
         /*****************��ӡ��ǰУ��֡���ࣨ������,������ǵ�ע�Ͳ�Ȼ��Ӱ��ϵͳ����ʱ��**********************/
//				printf("cmdid=%x\r\n",Judge_Recv_Data.cmdid);
//				printf("data_lenth=%d\r\n",Judge_Recv_Data.data_lenth);
//				printf("crc8=%x\r\n",Judge_Recv_Data.crc8);
//				printf("tail_crc16=%x\r\n",Judge_Recv_Data.crc16);
//				printf("next_frame_header=%x\r\n",Judge_Recv_Data.next_frame_header);
				/************************************************************/
				//����ϵͳ��һ�����ݷ��ͻ������֡���ݣ�����ǰ֡���ݵĺ�һλΪ֡ͷ��־λ�������һ�ν���
				if(*(RecvUsartData+JUDGE_LEN_HAEDER+JUDGE_LEN_CMDID+JUDGE_LEN_TAIL+Judge_Recv_Data.data_lenth)==0XA5)
				{
					JudgeRead(RecvUsartData+JUDGE_LEN_HAEDER+JUDGE_LEN_CMDID+JUDGE_LEN_TAIL+Judge_Recv_Data.data_lenth);
				}

			}
	    else
			{
			 printf("Vreify_crc16_error!\r\n");
			}	
    }
		else
		{
			printf("Vreify_crc8_error!\r\n");
		}	
  } 
	else
	{
		printf("Vreify_header_error!\r\n");
	}	
	
}	



/*-----------------------------------------------------------------------
\@brief ����6DMA��ʼ��
\@param ��
\@return ��
\@reference 
*--------------------------------------------------------------------*/
void HAL_UART6_Receive_IT_IDLE(void)
{
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);//���������ж�
    HAL_UART_Receive_DMA(&huart6,Jud_Rx_Buffer,RC_BUFFER_SIZE);//����DMA����
}

/**********************����ϵͳ��ȡ����**************************************/
//����������� ��λW �� 50HZ
float RoboChassisHeat(void)
{
	return Power_Heat_Data.chassis_power;
}
//ǹ������ÿ����ȴֵ 10HZ
uint16_t FireCoolingRate(void)
{
	return Game_Robot_Stat.shooter_heat0_cooling_limit;
}
//ǹ��������ȴ���� 10HZ
uint16_t FireCoolingLimit(void)
{
	return Game_Robot_Stat.shooter_heat0_cooling_limit;
}
//������ʣ��Ѫ�� 10HZ
uint16_t RoboHP(void)
{
	return Game_Robot_Stat.remain_HP;
}
//���������� 
float FireSpeed(void)
{
	return Shoot_Data.bullet_speed;
}
//17mmǹ������
uint16_t FireHeat17(void)
{
	return Power_Heat_Data.shooter_heat0;
}

//��Ƶ
uint8_t FireFeq(void)
{
	return Shoot_Data.bullet_freq;
}
/*-----------------------------------------------------------------------
\@brief ����������
\@param ��
\@return uint16_t ������
\@reference 
*--------------------------------------------------------------------*/
//ͳ�Ʒ�����
float Shoot_Speed[2]={0};
uint16_t ShootNum=0;
uint16_t FireNum(void)
{
	
	
	Shoot_Speed[NOW_t]=Shoot_Data.bullet_speed;
	if(Shoot_Speed[NOW_t]!=Shoot_Speed[LAST_t])//�����ٶȲ���Ⱦ��൱�ڷ�����һ�ŵ���
	{
		ShootNum++;
		Shoot_Speed[LAST_t]=Shoot_Speed[NOW_t];
	}
	return ShootNum;
}


void UART_Receive_IT_IDLE_Judge(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
    uint32_t buff_length;//�������ݳ���
    
    if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE))!= RESET)//��⵽DMA��·����
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
		//printf("ODLE\r\n");
		HAL_UART_DMAStop(huart);//�ر�DMA����
		__HAL_UART_CLEAR_IDLEFLAG(huart);//ͨ����ȡSR,DR�Ĵ���ȥ����,ʹ���˳���Usart3_Clean_IDLE_Flag���жϡ�
		buff_length=JUD_RC_BUFFER_SIZE-(__HAL_DMA_GET_COUNTER(hdma));//��ȡDMAʣ�ഫ����
		USART5_RX_Lenth=buff_length;//��ȡ���ݳ���
		JudgeRead(Jud_Rx_Buffer);//����ϵͳ����
		memset(Jud_Rx_Buffer,0,100);
		HAL_UART_Receive_DMA(huart,Jud_Rx_Buffer,JUD_RC_BUFFER_SIZE);//����DMA����

	}
}


/*-----------------------------------------------------------------------
\@brief JSCOOP������ӡ
\@param ��
\@return 
\@reference  ������ӡ����ϵͳ��ȡ�Ĺ��ʣ���Ƶ�Ȳ���
*--------------------------------------------------------------------*/
void ShowJudgeMeassge(void)
{
         Jscope_Chassis_Heat=RoboChassisHeat();
         Jscope_Fire_Speed=FireSpeed();
         Jscope_Fire_Feq=FireFeq();
         Jscope_Fire_Heat=FireHeat17();
	       Jscope_Fire_Num=FireNum();
}

