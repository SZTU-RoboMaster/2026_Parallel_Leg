#include "wheel.h"
// #include "ramp.h"
#include "robot_def.h"

Lk9025 wheel[2];
// ramp_function_source_t wheel_ramp[2];

/** ��챵����ʼ�� **/
void wheel_init(void)
{
    lk9025_init(&wheel[L], WHEEL_L_SEND);
    lk9025_init(&wheel[R], WHEEL_R_SEND);
    // ramp_init(&wheel_ramp[L],0.00000001f,LK9025_MAX_SPEED ,-LK9025_MAX_SPEED,0.1f);
    // ramp_init(&wheel_ramp[R],0.00000001f,LK9025_MAX_SPEED,-LK9025_MAX_SPEED,0.1f);
}

/** ������챵��ָ�� **/
Lk9025* get_wheel_motors(void){
    return wheel;
}
