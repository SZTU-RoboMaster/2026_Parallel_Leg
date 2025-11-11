#ifndef LAUNCHER_H
#define LAUNCHER_H

#include "DJI_Motor.h"
#include "user_lib.h"
#include "PID.h"

void Launcher_Init(void);

void Launcher_Mode_Set(void);

void Launcher_Disable(void);

void Launcher_Control(void);

#endif