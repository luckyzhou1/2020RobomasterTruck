#include "ShootControl.h"
#include "CanBus_Task.h"
#include "RC_Task.h"


shoot_t Shoot;




//摩擦轮速度控制
void FrictiongearSpeedControl(void)
{
    if(RC_UPPER_LEFT_SW_UP)
    {
        Shoot.set_frictiongear_speed = 0;
    }
    else if(RC_UPPER_LEFT_SW_MID)
    {
        Shoot.set_frictiongear_speed = 300;
    }
    else if(RC_UPPER_LEFT_SW_DOWN)
    {
        Shoot.set_frictiongear_speed = 350;
    }
}


//拨弹控制
void PluckControl(void)
{
    static uint8_t flag;
    
    if(RC_UPPER_RIGHT_SW_MID&&(flag == 0))
    {
        Shoot.set_trigger_angle += 0;
        flag = 1;
    }
    else if(RC_UPPER_RIGHT_SW_UP&&(flag == 1))
    {
        Shoot.set_trigger_angle += 1365;
        flag = 0;
    }
    else if(RC_UPPER_RIGHT_SW_DOWN&&(flag == 1))
    {
        Shoot.set_trigger_angle += 8192;
        flag = 0;
    }
}


