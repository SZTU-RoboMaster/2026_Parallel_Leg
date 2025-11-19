#include "launcher.h"
#include "gimbal_task.h"
#include "key_board.h"
#include "RoboMaster_Protocol.h"
#include "robot_def.h"

/*********************************************************************************************************
*                                              内部变量                                                   *
*********************************************************************************************************/
static uint8_t rc_last_sw_L;            // 拨杆上一时刻的状态值记录

/*************************************************************************************************
 *                                        Function                                               *
 *************************************************************************************************/

/** 发射机构PID初始化 **/
static void launcher_pid_init(void)
{
    /** 拨盘 **/
    // 位置环
    pid_init(&launcher.trigger.angle_pid,
             TRIGGER_ANGLE_PID_MAX_OUT,
             TRIGGER_ANGLE_PID_MAX_IOUT,
             TRIGGER_ANGLE_PID_KP,
             TRIGGER_ANGLE_PID_KI,
             TRIGGER_ANGLE_PID_KD);

    // 速度环
    pid_init(&launcher.trigger.speed_pid,
             TRIGGER_SPEED_PID_MAX_OUT,
             TRIGGER_SPEED_PID_MAX_IOUT,
             TRIGGER_SPEED_PID_KP,
             TRIGGER_SPEED_PID_KI,
             TRIGGER_SPEED_PID_KD);

    /** 摩擦轮 **/
    // 速度环
    pid_init(&launcher.fire_l.speed_pid,
             FIR_WHEEL_SPEED_PID_MAX_OUT,
             FIR_WHEEL_SPEED_PID_MAX_IOUT,
             FIR_WHEEL_SPEED_PID_KP,
             FIR_WHEEL_SPEED_PID_KI,
             FIR_WHEEL_SPEED_PID_KD);

    pid_init(&launcher.fire_r.speed_pid,
             FIR_WHEEL_SPEED_PID_MAX_OUT,
             FIR_WHEEL_SPEED_PID_MAX_IOUT,
             FIR_WHEEL_SPEED_PID_KP,
             FIR_WHEEL_SPEED_PID_KI,
             FIR_WHEEL_SPEED_PID_KD);
}

/** 发射机构初始化 **/
void Launcher_Init(void) {

    /** 发射机构电流置零 **/
    launcher.fire_l.target_current = 0;
    launcher.fire_r.target_current = 0;
    launcher.trigger.target_current = 0;

    /** 初始化发射机构模式 **/
    // 摩擦轮
    launcher.fir_wheel_mode  = Fire_OFF;
    // 拨盘
    launcher.trigger_mode = TRIGGER_CLOSE;

    /** 发射机构PID初始化 **/
    launcher_pid_init();

    /** 滤波器 **/
    first_order_filter_init(&launcher.filter_fire,0.05f, 0.5f);
    first_order_filter_init(&launcher.filter_trigger,1,1);
}

/** 摩擦轮模式设置 **/
static void fir_wheel_mode_set(void)
{
    if ((!switch_is_up(rc_last_sw_L)) && switch_is_up(rc_ctrl.rc.s[RC_s_L]))
    {// 当 「上一时刻左边拨盘不在上」 且 「该时刻模式在上」 时进入发射机构模式切换  说人话就是只有往上拨才能开关摩擦轮

        // 只有当云台不为失能模式时才能控制发射机构
        if(gimbal.gimbal_ctrl_mode != GIMBAL_DISABLE)
        {
            // 通过三元运算符对当前摩擦轮模式状态进行反转
            launcher.fir_wheel_mode = (launcher.fir_wheel_mode == Fire_ON) ? Fire_OFF : Fire_ON;
        }

    }

}

/** 发射机构模式设置 **/
void Launcher_Mode_Set(void) {

    /** 摩擦轮模式判断 **/
    fir_wheel_mode_set();

    /** 更新上一次的左拨杆值 **/
    rc_last_sw_L = rc_ctrl.rc.s[RC_s_L];

}


/** 拨盘控制 **/


uint16_t block_time = 0;
bool block_flag = false;
bool block_to_dead = false;

int32_t single_target_ecd = 0;
int32_t continuous_target_ecd = 0;

static void trigger_control(void)
{
    /****  堵转  ****/
    // 单发堵转
    if(ABS(launcher.trigger.single_speed_pid.out) > BLOCK_CURRENT)
    {
        block_time ++;

        if(block_time == BLOCK_THRESHOLD_TIME)
        {
            block_flag = true;

            single_target_ecd = continuous_target_ecd = launcher.trigger.motor_measure.total_ecd;

            single_target_ecd += DEGREE_45_TO_ENCODER;
        }

        if(block_time >= BLOCK_DISABLE_TIME)
        {
            block_flag = false;
            block_to_dead = true;

            single_target_ecd = continuous_target_ecd = launcher.trigger.motor_measure.total_ecd;
        }
    }
    // 连发堵转
    else if(ABS(launcher.trigger.continuous_speed_pid.out) > BLOCK_CURRENT)
    {
        block_time ++;

        if(block_time == BLOCK_THRESHOLD_TIME)
        {
            block_flag = true;

            single_target_ecd = continuous_target_ecd = launcher.trigger.motor_measure.total_ecd;

            continuous_target_ecd += 2 * DEGREE_45_TO_ENCODER;
        }

        if(block_time >= BLOCK_DISABLE_TIME)
        {
            block_flag = false;
            block_to_dead = true;

            single_target_ecd = continuous_target_ecd = launcher.trigger.motor_measure.total_ecd;
        }
    }

    /****  发弹  ****/
    else
    {
        // 单发
        if(get_channel_info(rc_ctrl.rc.ch[2]))
        {
            launcher.trigger_mode = TRIGGER_SINGLE;

            single_target_ecd = launcher.trigger.motor_measure.total_ecd;

            // 清空堵转时间
            block_time = 0;
            block_flag = false;
            block_to_dead = false;

            // 单发和连发之间切换
            continuous_target_ecd = single_target_ecd;
        }

        else if(rc_ctrl.rc.ch[2] > 600)
        {

        }
    }

}

/** 摩擦轮控制 **/
static void fir_wheel_control(void)
{
    if(launcher.fir_wheel_mode == Fire_ON)
    {
        launcher.fire_l.target_speed = -FIRE_SPEED;
        launcher.fire_r.target_speed =  FIRE_SPEED;
    }
    else
    {
        launcher.fire_l.target_speed = 0;
        launcher.fire_r.target_speed = 0;

        /** 失能发射机构（因为某种意义上来说，摩擦轮是主导，如果摩擦轮关闭，拨盘也应该被关闭） **/
        Launcher_Disable();
    }

    /** 摩擦轮电流计算 **/
    launcher.fire_l.target_current = pid_calc(&launcher.fire_l.speed_pid,
                                              launcher.fire_l.motor_measure.speed_rpm,
                                              launcher.fire_l.target_speed);

    launcher.fire_r.target_current = pid_calc(&launcher.fire_r.speed_pid,
                                              launcher.fire_r.motor_measure.speed_rpm,
                                              launcher.fire_r.target_speed);

}




/*************************************************************************************************
 *                                           Task                                                *
 *************************************************************************************************/

/** 发射机构失能 **/
void Launcher_Disable(void) {

    /** 左右摩擦轮和拨盘电流置零 **/
    launcher.fire_r.target_current = 0;
    launcher.fire_l.target_current = 0;
    launcher.trigger.target_current = 0;

    /** 关闭摩擦轮 **/
    launcher.fir_wheel_mode = Fire_OFF;

    /** 关闭拨盘 **/
    launcher.trigger_mode = TRIGGER_CLOSE;

    /** 令拨盘电机期望总编码器值总等于反馈的总编码器值，便于下次发射 **/
    single_target_ecd = continuous_target_ecd = (int32_t)launcher.trigger.motor_measure.total_ecd;


}

/** 发射机构控制 **/
void Launcher_Control(void)
{
    /** 拨盘控制 **/
    trigger_control();

    /** 摩擦轮控制 **/
    fir_wheel_control();
}