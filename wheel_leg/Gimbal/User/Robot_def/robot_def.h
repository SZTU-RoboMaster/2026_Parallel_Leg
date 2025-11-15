#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "DJI_Motor.h"
#include "pid.h"
#include "user_lib.h"
#include "filter.h"
#include "../App/Vision_Task/RoboMaster_Protocol.h"

/* 云台任务初始化时间 */
#define GIMBAL_TASK_INIT_TIME 800

/* 云台任务运行周期 */
#define GIMBAL_PERIOD 1

/**********************************************************************************
 *                                      云台                                       *
 **********************************************************************************/

/** 宏定义 **/

/* Pitch动态限位 */
#define MAX_ABS_ANGLE 25
#define MIN_ABS_ANGLE (-25)

/* 云台回中编码值 */
#define PITCH_OFFSET_ECD 0
#define YAW_OFFSET_ECD 3488

/** PID参数 **/

// Pitch
#define GIMBAL_PITCH_ANGLE_PID_KP          30.0f // 30.0f 60.0f
#define GIMBAL_PITCH_ANGLE_PID_KI           0.0f
#define GIMBAL_PITCH_ANGLE_PID_KD           0.0f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT         0.0f
#define GIMBAL_PITCH_ANGLE_MAX_OUT          9000.f

#define GIMBAL_PITCH_SPEED_PID_KP           55.0f // 120.0f 200.0f
#define GIMBAL_PITCH_SPEED_PID_KI           0.0f
#define GIMBAL_PITCH_SPEED_PID_KD           0.0f
#define GIMBAL_PITCH_SPEED_MAX_IOUT         0.0f
#define GIMBAL_PITCH_SPEED_MAX_OUT          15000.f

// Yaw
#define GIMBAL_YAW_ANGLE_PID_KP             20.0f // 30 35 35 35 35
#define GIMBAL_YAW_ANGLE_PID_KI             0.0f
#define GIMBAL_YAW_ANGLE_PID_KD             20.0f // 0 100 120 150 170
#define GIMBAL_YAW_ANGLE_MAX_IOUT           0.0f
#define GIMBAL_YAW_ANGLE_MAX_OUT            10000.f

#define GIMBAL_YAW_SPEED_PID_KP             300.0f // 500
#define GIMBAL_YAW_SPEED_PID_KI             0.0f
#define GIMBAL_YAW_SPEED_PID_KD             0.0f
#define GIMBAL_YAW_SPEED_MAX_IOUT           0.0f
#define GIMBAL_YAW_SPEED_MAX_OUT            15000.0f

/** 云台模式 **/
typedef enum {
    GIMBAL_DISABLE,   // 失能
    GIMBAL_ENABLE,    // 使能
    GIMBAL_AUTO,    //云台自瞄模式
    GIMBAL_FIRE,    //云台打弹模式
}Gimbal_Mode;

/** 云台电机结构体 **/
typedef struct {
    DJI_Motor_t motor_measure;    // 电机的真实信息

    float gyro; // °/s

    Pid speed_pid;   //速度环 PID
    Pid angle_pid;   //角度环 PID

    int16_t target_current; // 期望电流值

    fp32 absolute_angle_get; // ° IMU角度值
    fp32 relative_angle_get; // ° 与电机上电复位位置的角度差值

    fp32 absolute_angle_set; // °

}Motor_Gimbal_t;


/** 云台结构体 **/
typedef struct {

    /** 云台电机 **/
    Motor_Gimbal_t yaw;
    Motor_Gimbal_t pitch;

    /** 云台控制模式 **/
    Gimbal_Mode gimbal_ctrl_mode;
    Gimbal_Mode gimbal_last_ctrl_mode;

    /** 滤波器 **/

    // 鼠标输入滤波
    first_order_filter_type_t mouse_in_y;
    first_order_filter_type_t mouse_in_x;

    // 对视觉传回的yaw和pitch滤波
    first_order_filter_type_t auto_pitch;
    first_order_filter_type_t auto_yaw[2];

    // 角速度滤波
    first_order_filter_type_t pitch_gyro_filter;
    first_order_filter_type_t yaw_gyro_filter;

}gimbal_t;

extern gimbal_t gimbal;

/**********************************************************************************
 *                                    发射机构                                      *
 **********************************************************************************/

/** 堵转检测参数 **/
#define BLOCK_TRI_MINECD 5000     // 单发任务完成阈值 ecd
#define BLOCK_TRI_MAXTIME 1000    // 一次单发任务、反转任务最大执行时间 1s
#define BLOCK_TRI_MAXSPEED 100    // 堵转转速阈值 rpm

#define CONTINUE_BLOCK_TRI_MAXTIME 100 // 连发堵转阈值 ms

// 摩擦轮转速
#define FIRE_SPEED  4700

// 拨盘转速
#define TRIGGER_SPEED -2000 // 正转：从弹仓后往正方向看，拨盘顺时针转动

// 2006编码器端转一圈编码值加8192，减速比为1:36，则输出端转一圈（2π）的总编码器值为36×8192
#define DEGREE_45_TO_ENCODER  (36 * 8192) / 8 // (pi/4)

/** PID参数 **/
// 拨盘
#define TRIGGER_ANGLE_PID_KP        0.5f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        0.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT  0.0f
#define TRIGGER_ANGLE_PID_MAX_OUT   5000.0f

#define TRIGGER_SPEED_PID_KP        10.0f
#define TRIGGER_SPEED_PID_KI        0.0f
#define TRIGGER_SPEED_PID_KD        0.0f
#define TRIGGER_SPEED_PID_MAX_IOUT  0.0f
#define TRIGGER_SPEED_PID_MAX_OUT   10000.0f

// 摩擦轮
#define FIR_WHEEL_SPEED_PID_KP        50.0f
#define FIR_WHEEL_SPEED_PID_KI        0.0f
#define FIR_WHEEL_SPEED_PID_KD        0.0f
#define FIR_WHEEL_SPEED_PID_MAX_IOUT  0.0f
#define FIR_WHEEL_SPEED_PID_MAX_OUT   10000.0f


/** 摩擦轮状态 **/
typedef enum{
    Fire_OFF=0,             //关闭
    Fire_ON=1,              //开启
}Fir_Wheel_Mode;

/** 拨盘状态 **/
typedef enum{
    TRIGGER_CLOSE=0,              // 失能

    TRIGGER_SINGLE,               // 单发   （即视觉发一帧火控数据，拨盘转动一个弹丸角度）

    TRIGGER_CONTINUE,             // 连发

    TRIGGER_INVERSE,             // 反转

}Trigger_Mode;

typedef enum {

    SHOOT_OVER_STATE = 0,        // 发射完毕

    SHOOT_ING_STATE,             // 发射中

    SHOOT_BLOCK_STATE,           // 堵转

    SHOOT_FAIL_STATE             // 拨盘坏了，需要重启拨盘

}Shoot_State;

/** 堵转检测结构体 **/
typedef struct {

    /******** 1 单发模式 堵转检测 ********/
    float total_ecd_error; // 拨盘期望总编码器值与实际总编码器值之差
    uint16_t single_shoot_time; // 一次单发任务的时间 ms


    /******** 2 连发模式 堵转检测 ********/
    uint16_t block_continue_time; // 堵转持续时间

    /** 用于判定堵转是否被解决 **/
    uint16_t inverse_time;


}Block_Check;

/** 发射机构电机结构体 **/
typedef struct {
    DJI_Motor_t motor_measure;    // 电机的真实信息

    float target_speed;           // 摩擦轮期望转速
    int32_t target_total_ecd;     // 拨盘期望总编码值 用于堵转检测

    Pid angle_pid;                // 角度环 pid
    Pid speed_pid;                // 速度环 pid

    int16_t target_current;       // 期望电流值

}Motor_Launcher_t;

/** 发射机构结构体 **/
typedef struct {

    /** 发射机构电机 **/
    Motor_Launcher_t fire_l;
    Motor_Launcher_t fire_r;
    Motor_Launcher_t trigger;

    /** 摩擦轮模式 **/
    Fir_Wheel_Mode fir_wheel_last_mode;
    Fir_Wheel_Mode fir_wheel_mode;

    /** 拨盘模式 **/
    Trigger_Mode trigger_last_mode;
    Trigger_Mode trigger_mode;

    /** 射击状态 **/
    Shoot_State shoot_state;

    /** 堵转检测 **/
    Block_Check block_check;

    /** 滤波器 **/
    first_order_filter_type_t filter_fire;
    first_order_filter_type_t filter_trigger;

}launcher_t;

extern launcher_t launcher;


/**********************************************************************************
 *                                    Vision                                      *
 **********************************************************************************/

extern robot_ctrl_info_t robot_ctrl;

#endif
