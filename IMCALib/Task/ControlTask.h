#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "mytype.h"


void ParamInit(void);
void AllTask(void);

/*速度环PID参数重置*/
void PidResetSpeed(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd);
/*位置环PID参数重置*/
void PidResetPosition(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd);

void LEDTest(int flag);

#endif
