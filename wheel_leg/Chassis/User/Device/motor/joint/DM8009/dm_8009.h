#ifndef DM_8009_H
#define DM_8009_H

#include "stdint-gcc.h"
#include "can_device.h"

/********************   电机参数限制   *****************************/

#define DM8009_P_MIN -12.5f
#define DM8009_P_MAX 12.5f
#define DM8009_V_MIN -45.0f
#define DM8009_V_MAX 45.0f
#define DM8009_KP_MIN 0.0f
#define DM8009_KP_MAX 500.0f
#define DM8009_KD_MIN 0.0f
#define DM8009_KD_MAX 5.0f
#define DM8009_T_MIN -50.0f
#define DM8009_T_MAX 50.0f


typedef struct{
    uint32_t id;
    /** 绝对位置 **/
    float pos_r;

    /** 关节电机角速度 **/
    float angular_vel;

    /** 关节电机反馈力矩 **/
    float torque;
} Dm8009;

/** 初始化电机ID **/
void dm8009_init(Dm8009 *motor, uint32_t device_id);

/** 使能电机 **/
void set_dm8009_enable(Dm8009* motor);

/** 失能电机 **/
void set_dm8009_disable(Dm8009* motor);

/** 位置速度模式模式 **/
void set_dm8009_pos_speed(Dm8009* motor,
                          float pos_rad,
                          float speed_rps);

/** 单电机MIT模式 **/
void set_dm8009_MIT(Dm8009* motor,
                    float pos,
                    float speed,
                    float kp,
                    float kd,
                    float torque);

/** 关节电机反馈解析 **/
void dm8009_info_update(Dm8009* motor, uint8_t data[]);


#endif
