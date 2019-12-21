#ifndef __RC_TASK
#define __RC_TASK

#include "stm32f4xx_hal.h"

#define RC_CONTROL_ENABLE   /*ң��������ʹ�ܺ궨��*/
#define RC_Frame_Lentgh		18


/*ң�������Ϸ������Ϸ�����������״̬*/
#define  RC_UPPER_LEFT_SW_UP     (remote_control.switch_left == Switch_Up)
#define  RC_UPPER_LEFT_SW_MID    (remote_control.switch_left == Switch_Middle)     
#define  RC_UPPER_LEFT_SW_DOWN   (remote_control.switch_left == Switch_Down)
#define  RC_UPPER_RIGHT_SW_UP    (remote_control.switch_right == Switch_Up)
#define  RC_UPPER_RIGHT_SW_MID   (remote_control.switch_right == Switch_Middle)
#define  RC_UPPER_RIGHT_SW_DOWN  (remote_control.switch_right == Switch_Down)


//#define  RC_CH1_OFFSET   (remote_control.ch1)
//#define  RC_CH2_OFFSET   (remote_control.ch2)
//#define  RC_CH3_OFFSET   (remote_control.ch3)
//#define  RC_CH4_OFFSET   (remote_control.ch4)

typedef struct {
	int16_t ch1;	//each ch value from -364 -- +364
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	
	uint8_t switch_left;	//3 value
	uint8_t switch_right;
	
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
	
		uint8_t press_left;
		uint8_t press_right;
	}mouse;
	
	struct {
		uint16_t key_code;
/**********************************************************************************
   * ����ͨ��:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/

	}keyBoard;
	

}RC_Type;


enum{
	Switch_Up = 1,
	Switch_Middle = 3,
	Switch_Down = 2,
};

enum{
	Key_W,
	Key_S,
	//...
};


extern RC_Type remote_control;
extern uint8_t UART_Buffer[100];


/*ң�ؽ������ݽ���*/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff);
/*���ô���û��DMA�жϵĽ��պ���*/
HAL_StatusTypeDef UART_Receive_DMA_NoIT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
/*�������ڿ����ж�*/
HAL_StatusTypeDef HAL_UART_Receive_IT_IDLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
/*���ڿ����жϣ���USART2�ж��е���*/
void HAL_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart);
/*ң������������*/
int16_t RcDeadlineLimit(int16_t input, uint8_t dealine);

#endif

