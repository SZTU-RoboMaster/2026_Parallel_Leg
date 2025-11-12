#include "launcher.h"
#include "gimbal_task.h"
#include "key_board.h"
#include "protocol_balance.h"
#include "robot_def.h"

/*********************************************************************************************************
*                                              内部变量                                                   *
*********************************************************************************************************/
extern robot_ctrl_info_t robot_ctrl;    // 上位机数据
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
    launcher.fir_wheel_mode = launcher.fir_wheel_last_mode = Fire_OFF;
    // 拨盘
    launcher.trigger_mode = launcher.trigger_last_mode = TRIGGER_CLOSE;

    /** 发射机构PID初始化 **/
    launcher_pid_init();

    /** 初始化堵转检测 **/
    launcher.block_check.shoot_state = SHOOT_OVER_STATE;
    launcher.block_check.continue_time = 0;

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
            launcher.fir_wheel_last_mode = launcher.fir_wheel_mode;
            // 通过三元运算符对当前摩擦轮模式状态进行反转
            launcher.fir_wheel_mode = (launcher.fir_wheel_mode == Fire_ON) ? Fire_OFF : Fire_ON;
        }

    }

}

/** 拨盘模式设置（单发、连发） **/
static void trigger_mode_set(void) {

    // 只有在摩擦轮开启的情况下才能控制拨盘
    if(launcher.fir_wheel_mode == Fire_ON)
    {
        if (switch_is_down(rc_last_sw_L) || (KeyBoard.Mouse_l.status == KEY_PRESS))
        {
            // 1.回拨模式（处理堵转）   处理堵转的优先级要高于单发和连发
            if(launcher.block_check.shoot_state == SHOOT_BLOCK_STATE)
            {
                launcher.trigger_last_mode = launcher.trigger_mode;
                launcher.trigger_mode = TRIGGER_BACK;
            }

            // 2.「视觉-单发」模式
            else if ((gimbal.gimbal_ctrl_mode == GIMBAL_AUTO) && (robot_ctrl.fire_command == 1)) // 自瞄模式
            {
                launcher.trigger_last_mode = launcher.trigger_mode;
                launcher.trigger_mode = TRIGGER_SINGLE;
            }

            // 3.「连发」模式（优先级最低）
            else
            {
                launcher.trigger_last_mode = launcher.trigger_mode;
                launcher.trigger_mode = TRIGGER_CONTINUE;
            }

        }

    }
    else
    {
        launcher.trigger_mode = TRIGGER_CLOSE;
    }

}

/** 发射机构模式设置 **/
void Launcher_Mode_Set(void) {

    /** 摩擦轮模式判断 **/
    fir_wheel_mode_set();

    /** 拨盘模式判断 ***/
    trigger_mode_set();

    /** 更新上一次的左拨杆值 **/
    rc_last_sw_L = rc_ctrl.rc.s[RC_s_L];

}

/** 堵转检测 **/
static void block_check(void)
{
    // 记录拨盘期望总编码值和实际总编码值的误差
    launcher.block_check.total_ecd_error = ABS(launcher.trigger.target_total_ecd - launcher.trigger.motor_measure.total_ecd);

    /** 1 单发堵转判定 **/
    if(launcher.trigger_mode == TRIGGER_SINGLE)
    {
        if(launcher.block_check.total_ecd_error > BLOCK_TRI_MINECD) // 表示单发任务尚未完成
        {
            launcher.block_check.single_time ++;

            if(launcher.block_check.single_time > BLOCK_TRI_MAXTIME)// 单发任务超时，且仍未完成单发任务，判定为单发堵转
            {
                // 更新射击状态
                launcher.block_check.shoot_state = SHOOT_BLOCK_STATE;

                launcher.block_check.single_time = 0;
            }

            // 若未超过单发任务最大时间，则继续执行单发任务

        }
        else if(launcher.block_check.total_ecd_error < BLOCK_TRI_MINECD) // 单发任务执行完毕
        {
            // 更新射击状态
            launcher.block_check.shoot_state = SHOOT_OVER_STATE;

            launcher.block_check.single_time  = 0;
        }
    }

    /** 2 连发堵转判定 **/
    else if(launcher.trigger_mode == TRIGGER_CONTINUE)
    {
        // 记录「反馈转速小于堵转速度阈值」的时间
        if(ABS(launcher.trigger.motor_measure.speed_rpm) < BLOCK_TRI_MAXSPEED)
        {
            launcher.block_check.continue_time ++;
        }
        else
        {
            launcher.block_check.continue_time = 0;
        }

        // 堵转判定
        if(launcher.block_check.continue_time > BLOCK_TRI_MAXTIME)
        {
            // 更新射击状态
            launcher.block_check.shoot_state = SHOOT_BLOCK_STATE;
        }

        // 若未超过最大时间，则继续执行连发任务

    }

}

/** 判断堵转是否被成功解决 **/
static void block_handle_judge(void)
{


}

/** 拨盘控制 **/
static void trigger_control(void)
{
    /*********************************  位置环  ************************************/

    // 回拨模式
    if(launcher.trigger_mode == TRIGGER_BACK)
    {
        // 此时射击状态仍为SHOOT_BLOCK_STATE

        int32_t target_ecd = launcher.trigger.motor_measure.total_ecd + 2 * DEGREE_45_TO_ENCODER;// 感觉有点意思

        // 反转
        launcher.trigger.target_speed = pid_calc(&launcher.trigger.angle_pid,
                                                 launcher.trigger.motor_measure.total_ecd,
                                                 target_ecd);

        // 检查是否到达目标位置
        if (ABS(launcher.trigger.motor_measure.total_ecd - target_ecd) < BLOCK_TRI_MINECD)
        {
            launcher.block_check.shoot_state = SHOOT_OVER_STATE; // 回退完成
        }
    }

    // 单发模式
    else if(launcher.trigger_mode == TRIGGER_SINGLE)
    {
        // 更新射击状态
        launcher.block_check.shoot_state = SHOOT_ING;

        launcher.trigger.target_total_ecd -= DEGREE_45_TO_ENCODER;

        launcher.trigger.target_speed = pid_calc(&launcher.trigger.angle_pid,
                                                 launcher.trigger.motor_measure.total_ecd,
                                                 launcher.trigger.target_total_ecd);
    }

    // 连发模式
    else if(launcher.trigger_mode == TRIGGER_CONTINUE)
    {
        // 更新射击状态
        launcher.block_check.shoot_state = SHOOT_ING;

        // 连发过程中保持拨盘期望总编码值等于反馈总编码值
        launcher.trigger.target_total_ecd = launcher.trigger.motor_measure.total_ecd;

        launcher.trigger.target_speed = TRIGGER_SPEED;
    }


    /*********************************  速度环  ************************************/

    if(launcher.trigger_mode != TRIGGER_CLOSE)
    {
        launcher.trigger.target_current = pid_calc(&launcher.trigger.speed_pid,
                                                   launcher.trigger.motor_measure.speed_rpm,
                                                   launcher.trigger.target_speed);
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
    launcher.fir_wheel_mode = launcher.fir_wheel_last_mode = Fire_OFF;

    /** 关闭拨盘 **/
    launcher.trigger_mode = launcher.trigger_last_mode = TRIGGER_CLOSE;

    /** 重置射击状态 **/
    launcher.block_check.shoot_state = SHOOT_OVER_STATE;

    /** 令拨盘电机期望总编码器值总等于反馈的总编码器值，便于下次发射 **/
    launcher.trigger.target_total_ecd = (int32_t)launcher.trigger.motor_measure.total_ecd;


}

/** 发射机构控制 **/
void Launcher_Control(void)
{
    /** 拨盘控制 **/
    trigger_control();

    /** 摩擦轮控制 **/
    fir_wheel_control();

    /** 堵转检测 **/
    block_check();
}