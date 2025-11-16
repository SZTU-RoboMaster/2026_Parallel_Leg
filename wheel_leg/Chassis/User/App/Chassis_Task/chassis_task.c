#include <math.h>
#include "chassis_task.h"
#include "robot_def.h"
#include "user_lib.h"
#include "joint.h"
#include "wheel.h"
#include "remote.h"
#include "vmc.h"
#include "error.h"
#include "ins_task.h"
#include "vx_kalman_filter.h"
#include "lqr.h"
#include "bsp_delay.h"
#include "bsp_dwt.h"
#include "vofa.h"

/** 底盘pid初始化 **/
static void chassis_pid_init() {

    /** Wheel **/

    // 转向PID
    pid_init(&chassis.chassis_turn_pos_pid,
             CHASSIS_TURN_POS_PID_OUT_LIMIT,
             CHASSIS_TURN_POS_PID_IOUT_LIMIT,
             CHASSIS_TURN_POS_PID_P,
             CHASSIS_TURN_POS_PID_I,
             CHASSIS_TURN_POS_PID_D);

    pid_init(&chassis.chassis_turn_speed_pid,
             CHASSIS_TURN_SPEED_PID_OUT_LIMIT,
             CHASSIS_TURN_SPEED_PID_IOUT_LIMIT,
             CHASSIS_TURN_SPEED_PID_P,
             CHASSIS_TURN_SPEED_PID_I,
             CHASSIS_TURN_SPEED_PID_D);

    /** Joint **/

    // 防劈叉PID
    pid_init(&chassis.chassis_leg_coordination_pid,
             CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_P,
             CHASSIS_LEG_COORDINATION_PID_I,
             CHASSIS_LEG_COORDINATION_PID_D);

    // 腿长位置环PID
    pid_init(&chassis.leg_L.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_P,
             CHASSIS_LEG_L0_POS_PID_I,
             CHASSIS_LEG_L0_POS_PID_D);

    pid_init(&chassis.leg_R.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_P,
             CHASSIS_LEG_L0_POS_PID_I,
             CHASSIS_LEG_L0_POS_PID_D);

    // 腿长速度环PID
    pid_init(&chassis.leg_L.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_P,
             CHASSIS_LEG_L0_SPEED_PID_I,
             CHASSIS_LEG_L0_SPEED_PID_D);

    pid_init(&chassis.leg_R.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_P,
             CHASSIS_LEG_L0_SPEED_PID_I,
             CHASSIS_LEG_L0_SPEED_PID_D);

    // 离地后的腿长PID 暂时没用到
    pid_init(&chassis.leg_L.offground_leg_pid,
             CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT,
             CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT,
             CHASSIS_OFFGROUND_LO_PID_P,
             CHASSIS_OFFGROUND_L0_PID_I,
             CHASSIS_OFFGROUND_L0_PID_D);

    pid_init(&chassis.leg_R.offground_leg_pid,
             CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT,
             CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT,
             CHASSIS_OFFGROUND_LO_PID_P,
             CHASSIS_OFFGROUND_L0_PID_I,
             CHASSIS_OFFGROUND_L0_PID_D);

    // Roll补偿PID
    pid_init(&chassis.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);
}

/** 底盘初始化 **/
void chassis_init(void)
{
    /** 初始化底盘模式 **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** 关节电机初始化 **/
    joint_init();

    /** 轮毂电机初始化 **/
    wheel_init();

    /** 底盘pid初始化 **/
    chassis_pid_init();

    /** 滤波器初始化 **/
    // 低通滤波
    low_pass_filter_init(&chassis.leg_L.theta_dot_lpf, 0.75f);
    low_pass_filter_init(&chassis.leg_R.theta_dot_lpf, 0.75f);

    // 卡尔曼滤波
    xvEstimateKF_Init(&vaEstimateKF);

}

/** 底盘自起状态判断 **/
static void chassis_recover_state_check(void) {

    if(chassis.chassis_recover_finish == false)
    {
        // 机体姿态检测
        if (ABS(chassis.imu_reference.pitch_rad) <= NOT_BALANCE_RAD) {
            chassis.chassis_body_state = CHASSIS_BODY_NORMAL;
        } else {
            chassis.chassis_body_state = CHASSIS_BODY_UNNORMAL;
        }

        // 判断倒地时的腿倾角姿态
        if((ABS(chassis.leg_L.state_variable_feedback.theta) <= LEG_NORMAL_RAD)
           && (ABS(chassis.leg_R.state_variable_feedback.theta) <= LEG_NORMAL_RAD))
        {
            chassis.chassis_fall_leg_state = CHASSIS_FALL_LEG_NORMAL;
        }
        else
        {
            chassis.chassis_fall_leg_state = CHASSIS_FALL_LEG_UNNORMAL;
        }

        // 倒地自起判断
        // 倒地时，只有当腿倾角处于正常范围才允许自起
        if(chassis.chassis_fall_leg_state == CHASSIS_FALL_LEG_NORMAL)
        {
            chassis.chassis_recover_state = CHASSIS_COULD_RECOVER;
        }
        else
        {
            chassis.chassis_recover_state = CHASSIS_COULD_NOT_RECOVER;
        }

    }
}


/** 底盘倒地自救 **/
static void chassis_selfhelp(void)
{
    chassis_recover_state_check();

    if(chassis.chassis_recover_finish == false)
    {
        chassis.leg_L.joint_B_torque = 0.0f;
        chassis.leg_L.joint_F_torque = 0.0f;
        chassis.leg_R.joint_B_torque = 0.0f;
        chassis.leg_R.joint_F_torque = 0.0f;

        Kd = 0;
        vel = 0;

        if(chassis.chassis_recover_state == CHASSIS_COULD_RECOVER)
        {
            Kd = 0.0f;
            vel = 0.0f;
        }
        else
        {
            chassis.leg_L.wheel_torque = 0;
            chassis.leg_R.wheel_torque = 0;

            // 腿复位
            if(chassis.imu_reference.pitch_rad < 0.0f)
            {
                Kd = 3.0f;
                vel = -2.0f;
            }
            else
            {
                Kd = 3.0f;
                vel = 2.0f;
            }

        }
    }

    if(ABS(chassis.imu_reference.pitch_rad) <= RECOVER_RAD)
    {
        chassis.chassis_recover_finish = true;
    }
    else if(ABS(chassis.imu_reference.pitch_rad) >= NOT_BALANCE_RAD)
    {
        chassis.chassis_recover_finish = false;
    }
}

/** 获取底盘传感器数据 **/
static void get_IMU_info(void) {

    /** Yaw **/
    chassis.imu_reference.yaw_rad = -INS.Yaw * DEGREE_TO_RAD;

    chassis.imu_reference.yaw_total_rad = -INS.YawTotalAngle * DEGREE_TO_RAD;

    /** Pitch **/
    chassis.imu_reference.pitch_rad = -INS.Roll * DEGREE_TO_RAD;

    /** Roll **/
    chassis.imu_reference.roll_rad = INS.Pitch * DEGREE_TO_RAD;

    /** 更新各轴加速度和角速度 **/
    chassis.imu_reference.pitch_gyro = -INS.Gyro[Y];
    chassis.imu_reference.yaw_gyro = -INS.Gyro[Z];
    chassis.imu_reference.roll_gyro = INS.Gyro[X];

    chassis.imu_reference.ax = INS.Accel[X];
    chassis.imu_reference.ay = INS.Accel[Y];
    chassis.imu_reference.az = INS.Accel[Z];

    /** 机体竖直方向加速度 **/
    chassis.imu_reference.robot_az = INS.MotionAccel_n[Z];
}

/** 更新底盘变量 **/
static void chassis_variable_update(void) {

    get_IMU_info();

    // 5.phi
    chassis.leg_L.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;
    chassis.leg_R.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;

    // 6.phi_dot
    chassis.leg_L.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;
    chassis.leg_R.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;

    //theta_last
    chassis.leg_L.state_variable_feedback.theta_last = chassis.leg_L.state_variable_feedback.theta;
    chassis.leg_R.state_variable_feedback.theta_last = chassis.leg_R.state_variable_feedback.theta;

    //1.theta
    chassis.leg_L.state_variable_feedback.theta = cal_leg_theta(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0, chassis.leg_L.state_variable_feedback.phi);
    chassis.leg_R.state_variable_feedback.theta = cal_leg_theta(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0, chassis.leg_R.state_variable_feedback.phi);

    //2. theta_dot
    float theta_dot_raw_L;
    float theta_dot_raw_R;

    theta_dot_raw_L = (chassis.leg_L.state_variable_feedback.theta - chassis.leg_L.state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);

    chassis.leg_L.state_variable_feedback.theta_dot_last = chassis.leg_L.state_variable_feedback.theta_dot;
    chassis.leg_L.state_variable_feedback.theta_dot = update_low_pass_filter(&chassis.leg_L.theta_dot_lpf, theta_dot_raw_L);

    theta_dot_raw_R = (chassis.leg_R.state_variable_feedback.theta - chassis.leg_R.state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);

    chassis.leg_R.state_variable_feedback.theta_dot_last = chassis.leg_R.state_variable_feedback.theta_dot;
    chassis.leg_R.state_variable_feedback.theta_dot = update_low_pass_filter(&chassis.leg_R.theta_dot_lpf, theta_dot_raw_R);

    // 2.1 theta_ddot
    chassis.leg_L.state_variable_feedback.theta_ddot = (chassis.leg_L.state_variable_feedback.theta_dot - chassis.leg_L.state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);
    chassis.leg_R.state_variable_feedback.theta_ddot = (chassis.leg_R.state_variable_feedback.theta_dot - chassis.leg_R.state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);


    //4.x_dot
    chassis.leg_L.state_variable_feedback.x_dot = vel_acc[0];
    chassis.leg_R.state_variable_feedback.x_dot = vel_acc[0];

    //3.x

    if(chassis.chassis_ctrl_info.v_m_per_s != 0.0f)
    {
        chassis.leg_L.state_variable_feedback.x = 0.0f;
        chassis.leg_R.state_variable_feedback.x = 0.0f;
    }
    else
    {
        chassis.leg_L.state_variable_feedback.x = chassis.leg_L.state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * chassis.leg_L.state_variable_feedback.x_dot;
        chassis.leg_R.state_variable_feedback.x = chassis.leg_R.state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * chassis.leg_R.state_variable_feedback.x_dot;

    }

    // 4.1 x_ddot
    chassis.leg_L.state_variable_feedback.x_dot_last = chassis.leg_L.state_variable_feedback.x_dot;
    chassis.leg_L.state_variable_feedback.x_ddot = (chassis.leg_L.state_variable_feedback.x_dot - chassis.leg_L.state_variable_feedback.x_dot_last) / (CHASSIS_PERIOD * 0.001f);

    chassis.leg_R.state_variable_feedback.x_dot_last = chassis.leg_R.state_variable_feedback.x_dot;
    chassis.leg_R.state_variable_feedback.x_ddot = (chassis.leg_R.state_variable_feedback.x_dot - chassis.leg_R.state_variable_feedback.x_dot_last) / (CHASSIS_PERIOD * 0.001f);
}

/** 计算驱动轮力矩 **/

static void wheel_calc(void)
{
    /******************************* Wheel *************************************/

    /** 根据腿长和三次拟合系数拟合出反馈增益K **/
    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);

    float target_yaw_speed = pid_calc(&chassis.chassis_turn_pos_pid,
                                      gimbal_unpack_data.yaw_relative_angle * DEGREE_TO_RAD,0.0f);

    if(chassis.chassis_ctrl_mode == CHASSIS_SPIN)
    {
        target_yaw_speed = SPIN_SPEED;
    }

    chassis.wheel_turn_torque = pid_calc(&chassis.chassis_turn_speed_pid,
                                         chassis.imu_reference.yaw_gyro,
                                         target_yaw_speed);



    chassis.leg_L.wheel_torque =  wheel_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
                                  + wheel_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                                  + wheel_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
                                  + wheel_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                  + wheel_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - PHI_BALANCE)
                                  + wheel_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_R.wheel_torque =  wheel_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
                                  + wheel_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                                  + wheel_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
                                  + wheel_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                  + wheel_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - PHI_BALANCE)
                                  + wheel_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_L.wheel_torque -= chassis.wheel_turn_torque;
    chassis.leg_R.wheel_torque += chassis.wheel_turn_torque;
    chassis.leg_R.wheel_torque *= -1;

    VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);

}

/** 计算关节力矩 **/

static void joint_calc(void)
{
/******************************* Joint *************************************/

    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

    /** Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp **/

    /****** 防劈叉pid ******/
    chassis.steer_compensatory_torque =  CHASSIS_LEG_COORDINATION_PID_P * (0.0f - chassis.phi0_error)
                                         + CHASSIS_LEG_COORDINATION_PID_D * (0.0f - chassis.d_phi0_error); // 注意微分项正负

    // USART_Vofa_Justfloat_Transmit(chassis.phi0_error,0,0);
    //Left
    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
                                                                         + joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                                                                         + joint_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
                                                                         + joint_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                                                         + joint_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - PHI_BALANCE)
                                                                         + joint_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);


    //Right
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
                                                                         + joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                                                                         + joint_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
                                                                         + joint_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                                                         + joint_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - PHI_BALANCE)
                                                                         + joint_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);
    //
    // chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0;
    // chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0;

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;

    /** End End End End End End End End End End End End End End End End End End End End End End End End **/


    /** F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F **/

    /****** Leg pid ******/
    float L_L0_dot_set = pid_calc(&chassis.leg_L.leg_pos_pid,
                                  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                                  chassis.chassis_ctrl_info.height_m);

    float R_L0_dot_set = pid_calc(&chassis.leg_R.leg_pos_pid,
                                  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                                  chassis.chassis_ctrl_info.height_m);

    pid_calc(&chassis.leg_L.leg_speed_pid,
             chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
             L_L0_dot_set);

    pid_calc(&chassis.leg_R.leg_speed_pid,
             chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
             R_L0_dot_set);

    /****** Roll pid ******/

    chassis.roll_f =  CHASSIS_ROLL_PID_P * (0.0f - chassis.roll_error)
                                             + CHASSIS_ROLL_PID_D * (0.0f - chassis.d_roll_error); // 注意微分项正负

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_L.state_variable_feedback.theta)
                                                                         + chassis.leg_L.leg_speed_pid.out;

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_R.state_variable_feedback.theta)
                                                                         + chassis.leg_R.leg_speed_pid.out;

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += chassis.roll_f;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= chassis.roll_f;

    /** End End End End End End End End End End End End End End End End End End End End End End End End **/

    // 计算关节电机力矩
    vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);
    vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

    chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B
    //
    // USART_Vofa_Justfloat_Transmit(chassis.leg_L.joint_F_torque - chassis.leg_R.joint_F_torque
    //     , chassis.leg_L.joint_B_torque -  chassis.leg_R.joint_B_torque  ,0);

    // 输出限幅
    VAL_LIMIT(chassis.leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);

}

/** 控制器计算 **/
static void controller_calc(void)
{
    /** 更新五连杆参数 **/
    vmc_calc();
    /** 速度融合 **/
    speed_calc();
    /** 更新底盘变量 **/
    chassis_variable_update();
    /** 计算驱动轮力矩 **/
    wheel_calc();
    /** 计算关节力矩 **/
    joint_calc();
}


/*******************************************************************************
 *                                  Task                                       *
 *******************************************************************************/

/** 底盘失能任务 **/
static void chassis_disable_task() {

    chassis.leg_L.wheel_torque = 0;
    chassis.leg_R.wheel_torque = 0;

    chassis.leg_L.joint_F_torque = 0;
    chassis.leg_L.joint_B_torque = 0;
    chassis.leg_R.joint_F_torque = 0;
    chassis.leg_R.joint_B_torque = 0;

    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    chassis.leg_L.state_variable_feedback.x = 0.0f;
    chassis.leg_R.state_variable_feedback.x = 0.0f;

    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    chassis.chassis_ctrl_info.height_m = MIN_L0;

    // 底盘状态
    chassis.chassis_body_state = CHASSIS_BODY_UNNORMAL;

    chassis.chassis_fall_leg_state = CHASSIS_FALL_LEG_UNNORMAL;

    chassis.chassis_recover_state = CHASSIS_COULD_NOT_RECOVER;

    /** 初始化标志位 **/

    // 底盘初始化标志位
    chassis.init_flag = false;

    chassis.chassis_recover_finish = false;

}

/** 底盘初始化任务 **/
static void chassis_init_task()
{
    joint_enable();

    chassis.init_flag = true;
}

/** 底盘使能任务 **/
static void chassis_enable_task(void)
{
    /** 控制器计算 **/
    controller_calc();

    /** 倒地自救 **/
    chassis_selfhelp();
}
/** 跳跃任务 **/
/**
 * @brief 跳跃任务函数
 * 实现机器人的跳跃功能，包含收腿蓄力、蹬腿起跳、腾空收腿、伸腿降落、落地缓冲五个阶段
 * 并在空中阶段实现姿态控制
 */
static void chassis_jump_task()
{
    // 静态变量用于记录跳跃状态
    static JumpState jump_state = NOT_READY;
    static bool air_control_enabled = false; // 空中控制使能标志


    switch(jump_state)
    {
        case NOT_READY:
            // 收腿蓄力阶段：将腿部收缩至最短腿长
            chassis.chassis_ctrl_info.height_m = MIN_L0;

            // 检查是否达到最短腿长
            if(fabsf(chassis.chassis_ctrl_info.height_m - MIN_L0) < 0.1f &&
               fabsf(chassis.chassis_ctrl_info.height_m - MIN_L0) < 0.1f)
            {
                jump_state = READY;
            }
            break;

        case READY:
            // 蹬腿起跳阶段：设置腿长为目标最大值
            chassis.chassis_ctrl_info.height_m = 0.32f; // 使用最大腿长

            // 检查腿是否开始伸展
            if(chassis.chassis_ctrl_info.height_m > (MIN_L0 + 0.05f) ||
               chassis.chassis_ctrl_info.height_m > (MIN_L0 + 0.05f))
            {
                jump_state = STRETCHING;
            }
            break;

        case STRETCHING:
            // 腾空收腿阶段：检测到腿伸展至最长后收腿
            if(chassis.chassis_ctrl_info.height_m > 0.29f ||
               chassis.chassis_ctrl_info.height_m > 0.29f)
            {
                chassis.chassis_ctrl_info.height_m = MIN_L0;
                jump_state = SHRINKING;
                air_control_enabled = true; // 启用空中控制
            }
            break;

        case SHRINKING:
            // 伸腿降落阶段：收腿完成后设置为标准腿长
            // 在空中保持腿部姿态垂直于地面
            if(air_control_enabled) {
                // 使用Roll PID控制保持腿部姿态
                chassis.roll_error = chassis.imu_reference.roll_rad - 0.0f; // 目标Roll角度为0
                chassis.d_roll_error = chassis.imu_reference.roll_gyro - 0.0f; // 目标Roll角速度为0

                // 计算Roll控制输出
                chassis.roll_f = CHASSIS_ROLL_PID_P * (0.0f - chassis.imu_reference.roll_rad)
                               + CHASSIS_ROLL_PID_D * (0.0f - chassis.imu_reference.roll_gyro);

                // 将Roll控制输出应用到腿部支撑力
                chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += chassis.roll_f;
                chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= chassis.roll_f;
            }

            if(fabsf(chassis.chassis_ctrl_info.height_m - MIN_L0) < 0.01f &&
               fabsf(chassis.chassis_ctrl_info.height_m - MIN_L0) < 0.01f)
            {
                chassis.chassis_ctrl_info.height_m = 0.22f; // 标准腿长
                jump_state = LANDING;
            }
            break;

        case LANDING:
            // 着陆前继续空中姿态控制
            if(air_control_enabled) {
                chassis.roll_error = chassis.imu_reference.roll_rad - 0.0f;
                chassis.d_roll_error = chassis.imu_reference.roll_gyro - 0.0f;

                chassis.roll_f = CHASSIS_ROLL_PID_P * (0.0f - chassis.imu_reference.roll_rad)
                               + CHASSIS_ROLL_PID_D * (0.0f - chassis.imu_reference.roll_gyro);

                chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += chassis.roll_f;
                chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= chassis.roll_f;
            }

            // 检测到着陆冲击时增加腿部支撑力
            if(chassis.imu_reference.robot_az > 5.0f)
            {
                // 增加缓冲转矩
                float landing_boost = 1.5f;
                chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point *= landing_boost;
                chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point *= landing_boost;
                air_control_enabled = false; // 着陆后关闭空中控制

                // 当向下加速度减小到一定程度时，认为缓冲完成
                if(fabsf(chassis.imu_reference.robot_az) < 0.5f)
                {
                    jump_state = NOT_READY;
                    chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
                }
            }
            break;

        default:
            jump_state = NOT_READY;
            air_control_enabled = false;
            break;
    }
}



/** 发送力矩任务 **/
static void send_torque_task(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque,
                             float wheel_L_torque, float wheel_R_torque,
                             float vel, float Kd)
{
    set_dm8009_MIT(&joint[LF],0.0f, vel, 0, Kd,joint_LF_torque);
    set_dm8009_MIT(&joint[LB],0.0f, vel, 0, Kd,joint_LB_torque);
    DWT_Delay(0.0002f);
    set_dm8009_MIT(&joint[RF],0.0f, -vel, 0, Kd,joint_RF_torque);
    set_dm8009_MIT(&joint[RB],0.0f, -vel, 0, Kd,joint_RB_torque);

    lk9025_multi_torque_set(wheel_L_torque, wheel_R_torque);

}

void chassis_task(void)
{
    /** 获取遥控器信息(模式 + 数据) **/
    remote_cmd();

    switch (chassis.chassis_ctrl_mode)
    {
        case CHASSIS_DISABLE:
        {
            chassis_disable_task();
            break;
        }


        case CHASSIS_INIT:
        {
            chassis_init_task();
            break;
        }


        case CHASSIS_ENABLE:
        case CHASSIS_SPIN:

        {
            chassis_enable_task();
            break;
        }
        case CHASSIS_JUMP: {
            chassis_enable_task();
            chassis_jump_task();
            break;
        }

        default:
        {
            break;
        }
    }
//testing
        send_torque_task(-chassis.leg_L.joint_F_torque,
                     -chassis.leg_L.joint_B_torque,
                     chassis.leg_R.joint_F_torque,
                     chassis.leg_R.joint_B_torque,
                     -chassis.leg_L.wheel_torque,
                     -chassis.leg_R.wheel_torque,
                     vel,
                     Kd);

    // send_torque_task(-chassis.leg_L.joint_F_torque,
    //          -chassis.leg_L.joint_B_torque,
    //          chassis.leg_R.joint_F_torque,
    //          chassis.leg_R.joint_B_torque,
    //          0,0,
    //          vel,
    //          Kd);


    // send_torque_task(0,
    //                  0,
    //                  0,
    //                  0,
    //                  0,
    //                  0,
    //                  0,
    //                  0);


}