#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "pid.h"
//���ڲ����ڱ�ʱ��ȥ�������ע��
//#define SKYGUARD_CHASSIS       //�ڱ����̿���

//3508�ڴ������ָ��������յ�����µ����ת��Ϊ430rpm���ң���������ٶȾ�����Ҫ���������ֵ
#define GUARD_CHASSIS_MAXSPEED_RPM    400    //�ڱ����̵�����ת�٣���λ��rpm
#define CHASSIS_MAXSPEED_RPM          400    //�������̵�����ת�٣���λ��rpm
#define CHASSIS_VX_MAXSPEED_RPM       350    //��������ƽ��ʱ�����̵��������ٶȣ���λ��rpm
#define REDUCTION_RATIO_3508          19.2f   //3508������ٱ�


/*���̵�����*/
enum{
    
    FRON_RIGH_201 = 0, //ǰ��
    FRON_LEFT_202 = 1, //ǰ��
    REAR_LEFT_203 = 2, //����
    REAR_RIGH_204 = 3, //����
    
};


/*���̽ṹ��*/
typedef struct{
    
    int32_t  fr_motor_rpm_201; //ǰ�ҵ��
    int32_t  fl_motor_rpm_202; //ǰ����
    int32_t  rl_motor_rpm_203; //������
    int32_t  rr_motor_rpm_204; //���ҵ��
    

    float vx; //����ƽ��
    float vy; //ǰ��
    float vw; //��ת
    
    //��������ϵ�е��ٶ�
    float car_vx; //��λ��m/s
    float car_vy;
    float car_vw; //rad/s
    
    //���Ӷ�Ӧ��������ϵ���ٶ�
    float wheel_rad_201; //���ӵ�ת�٣���λ��rad/s
    float wheel_rad_202;
    float wheel_rad_203;
    float wheel_rad_204;
    
}chassis_t;


extern uint8_t Chassis_Ctrl;
extern pid_t  Moto_Chassis_Pid_Pos[8];  //λ�û�PID�ṹ��
extern pid_t  Moto_Chassis_Pid_Spd[8];  //�ٶȻ�PID�ṹ��
extern chassis_t  Chassis;


void ChassisTask(void);
void ChassisDataUpdate(void);
void ChassisPidCalc(void);
void ParamInit(void);
void ChassisDataCanSend(void);

/****************************************������****************************************/
void ChassisSpeedTest(void);



/*****************************************END******************************************/



#endif
