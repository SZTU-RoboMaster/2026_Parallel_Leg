#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "PID.h"
#include "filter.h"
#include "DJI_Motor.h"
#include "key_board.h"
#include "robot_def.h"

void Gimbal_Mode_Set(void);

void Gimbal_Ctrl_Info_Set(void);

void Gimbal_task(void const*pvParameters);

#endif
