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
static int calc_count = 0;//Test
/** ����pid��ʼ�� **/
static void chassis_pid_init() {

    /** Wheel **/

    // ת��PID
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

    // ������PID
    pid_init(&chassis.chassis_leg_coordination_pid,
             CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_P,
             CHASSIS_LEG_COORDINATION_PID_I,
             CHASSIS_LEG_COORDINATION_PID_D);


    pid_init(&chassis.leg_L.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_M,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_M,
             CHASSIS_LEG_L0_POS_PID_P_M,
             CHASSIS_LEG_L0_POS_PID_I_M,
             CHASSIS_LEG_L0_POS_PID_D_M);
    pid_init(&chassis.leg_R.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_M,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_M,
             CHASSIS_LEG_L0_POS_PID_P_M,
             CHASSIS_LEG_L0_POS_PID_I_M,
             CHASSIS_LEG_L0_POS_PID_D_M);

    pid_init(&chassis.leg_L.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_M,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_M,
             CHASSIS_LEG_L0_SPEED_PID_P_M,
             CHASSIS_LEG_L0_SPEED_PID_I_M,
             CHASSIS_LEG_L0_SPEED_PID_D_M);
    pid_init(&chassis.leg_R.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_M,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_M,
             CHASSIS_LEG_L0_SPEED_PID_P_M,
             CHASSIS_LEG_L0_SPEED_PID_I_M,
             CHASSIS_LEG_L0_SPEED_PID_D_M);


        // ��غ���ȳ�PID ��ʱû�õ�
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

    // Roll����PID
    pid_init(&chassis.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);
}

/** ���̳�ʼ�� **/
void chassis_init(void)
{
    /** ��ʼ������ģʽ **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** �ؽڵ����ʼ�� **/
    joint_init();

    /** ��챵����ʼ�� **/
    wheel_init();

    /** ����pid��ʼ�� **/
    chassis_pid_init();

    /** �˲�����ʼ�� **/
    // ��ͨ�˲�
    low_pass_filter_init(&chassis.leg_L.theta_dot_lpf, 0.75f);
    low_pass_filter_init(&chassis.leg_R.theta_dot_lpf, 0.75f);

    // �������˲�
    xvEstimateKF_Init(&vaEstimateKF);

    // ��ʼ����Ծ״̬
    chassis.jump_state = NOT_READY;
}
/** ��ؼ���ж� **/
static void chassis_off_ground_check(void) {
    calculate_f();
    // ��ȡ��ǰ֧����
    float left_support = chassis.leg_L.Fn;
    float right_support = chassis.leg_R.Fn;
    float total_support = left_support + right_support;
//    USART_Vofa_Justfloat_Transmit(total_support,chassis.chassis_on_ground_state,0);
    float expected_weight = -6.5f ; //����²��� //40
    // ������ȳ�~���ȳ� total -50~10
    if (total_support < expected_weight)
    {
        chassis.chassis_on_ground_state = CHASSIS_Off_Ground;
    }
    else
        chassis.chassis_on_ground_state = CHASSIS_On_Ground;
}

/** ��������״̬�ж� **/
static void chassis_recover_state_check(void) {

    if(chassis.chassis_recover_finish == false)
    {
        // ������̬���
        if (ABS(chassis.imu_reference.pitch_rad) <= NOT_BALANCE_RAD) {
            chassis.chassis_body_state = CHASSIS_BODY_NORMAL;
        } else {
            chassis.chassis_body_state = CHASSIS_BODY_UNNORMAL;
        }

        // �жϵ���ʱ���������̬
        if((ABS(chassis.leg_L.state_variable_feedback.theta) <= LEG_NORMAL_RAD)
           && (ABS(chassis.leg_R.state_variable_feedback.theta) <= LEG_NORMAL_RAD))
        {
            chassis.chassis_fall_leg_state = CHASSIS_FALL_LEG_NORMAL;
        }
        else
        {
            chassis.chassis_fall_leg_state = CHASSIS_FALL_LEG_UNNORMAL;
        }

        // ���������ж�
        // ����ʱ��ֻ�е�����Ǵ���������Χ����������
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

/** ��ؼ�� **/
static void chassis_off_ground(void)
{
    chassis_off_ground_check();

    if((chassis.chassis_on_ground_state == CHASSIS_Off_Ground) && (chassis.chassis_ctrl_mode != CHASSIS_JUMP))
    {
        joint_K_L[0] = 0;
        joint_K_L[1] = 0;
        joint_K_R[0] = 0;
        joint_K_R[1] = 0;
    }
}


/** ���̵����Ծ� **/

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

            // �ȸ�λ
            if(chassis.imu_reference.pitch_rad < 0.0f)
            {
                Kd = 8.0f;
                vel = -5.0f;
            }
            else
            {
                Kd = 8.0f;
                vel = 5.0f;
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
///test
// static void chassis_selfhelp(void)
//{
//    static uint32_t selfhelp_timer = 0;
//    static SelfHelpState selfhelp_state = SELFHELP_IDLE;
//
//    chassis_recover_state_check();
//
//    if(chassis.chassis_recover_finish == false) {
//        // 如果状态机为空闲，根据倾斜方向初始化
//        if(selfhelp_state == SELFHELP_IDLE) {
//            selfhelp_state = SELFHELP_PREPARE;
//            selfhelp_timer = xTaskGetTickCount();
//        }
//
//        uint32_t current_time = xTaskGetTickCount();
//        uint32_t elapsed_time = current_time - selfhelp_timer;
//
//        switch(selfhelp_state) {
//            case SELFHELP_PREPARE:
//                // 准备阶段：收缩腿部
//                chassis.chassis_ctrl_info.height_m = 0.15f;  // 适当收缩
//
//                // 直接设置关节力矩进行收缩
//                if(chassis.imu_reference.pitch_rad < 0.0f) {
//                    // 向前倾斜
//                    chassis.leg_L.joint_F_torque = -30.0f;
//                    chassis.leg_L.joint_B_torque = -30.0f;
//                    chassis.leg_R.joint_F_torque = -30.0f;
//                    chassis.leg_R.joint_B_torque = -30.0f;
//                } else {
//                    // 向后倾斜
//                    chassis.leg_L.joint_F_torque = 30.0f;
//                    chassis.leg_L.joint_B_torque = 30.0f;
//                    chassis.leg_R.joint_F_torque = 30.0f;
//                    chassis.leg_R.joint_B_torque = 30.0f;
//                }
//
//                // 准备1秒后进入抬腿阶段
//                if(elapsed_time > 1000) {
//                    selfhelp_state = SELFHELP_LIFT_LEG;
//                    selfhelp_timer = current_time;
//                }
//                break;
//
//            case SELFHELP_LIFT_LEG:
//                // 抬腿阶段：强力抬腿
//                if(chassis.imu_reference.pitch_rad < 0.0f) {
//                    // 向前倾斜：抬前腿
//                    chassis.leg_L.joint_F_torque = 80.0f;   // 大力抬前腿
//                    chassis.leg_L.joint_B_torque = -40.0f;  // 后腿推地
//                    chassis.leg_R.joint_F_torque = 80.0f;
//                    chassis.leg_R.joint_B_torque = -40.0f;
//                    chassis.leg_L.wheel_torque = 25.0f;     // 轮子向后
//                    chassis.leg_R.wheel_torque = 25.0f;
//                } else {
//                    // 向后倾斜：抬后腿
//                    chassis.leg_L.joint_F_torque = -40.0f;  // 前腿推地
//                    chassis.leg_L.joint_B_torque = 80.0f;   // 大力抬后腿
//                    chassis.leg_R.joint_F_torque = -40.0f;
//                    chassis.leg_R.joint_B_torque = 80.0f;
//                    chassis.leg_L.wheel_torque = -25.0f;    // 轮子向前
//                    chassis.leg_R.wheel_torque = -25.0f;
//                }
//
//                // 抬腿0.5秒后进入推地阶段
//                if(elapsed_time > 500) {
//                    selfhelp_state = SELFHELP_PUSH_GROUND;
//                    selfhelp_timer = current_time;
//                }
//                break;
//
//            case SELFHELP_PUSH_GROUND:
//                // 推地阶段：快速伸展
//                chassis.chassis_ctrl_info.height_m = 0.3f;  // 伸展
//
//                // 施加很大的伸展力矩
//                if(chassis.imu_reference.pitch_rad < 0.0f) {
//                    chassis.leg_L.joint_F_torque = -60.0f;  // 前腿向下推
//                    chassis.leg_L.joint_B_torque = 100.0f;  // 后腿强力推
//                    chassis.leg_R.joint_F_torque = -60.0f;
//                    chassis.leg_R.joint_B_torque = 100.0f;
//                    chassis.leg_L.wheel_torque = 30.0f;
//                    chassis.leg_R.wheel_torque = 30.0f;
//                } else {
//                    chassis.leg_L.joint_F_torque = 100.0f;  // 前腿强力推
//                    chassis.leg_L.joint_B_torque = -60.0f;  // 后腿向下推
//                    chassis.leg_R.joint_F_torque = 100.0f;
//                    chassis.leg_R.joint_B_torque = -60.0f;
//                    chassis.leg_L.wheel_torque = -30.0f;
//                    chassis.leg_R.wheel_torque = -30.0f;
//                }
//
//                // 推地0.3秒后检查
//                if(elapsed_time > 300) {
//                    selfhelp_state = SELFHELP_CHECK;
//                    selfhelp_timer = current_time;
//                }
//                break;
//
//            case SELFHELP_CHECK:
//                // 检查阶段：短暂保持
//                if(elapsed_time > 200) {
//                    // 检查倾角是否恢复
//                    if(ABS(chassis.imu_reference.pitch_rad) <= RECOVER_RAD) {
//                        // 恢复成功
//                        selfhelp_state = SELFHELP_IDLE;
//                        chassis.chassis_recover_finish = true;
//
//                        // 恢复常规控制
//                        chassis.leg_L.joint_F_torque = 0;
//                        chassis.leg_L.joint_B_torque = 0;
//                        chassis.leg_R.joint_F_torque = 0;
//                        chassis.leg_R.joint_B_torque = 0;
//                        chassis.leg_L.wheel_torque = 0;
//                        chassis.leg_R.wheel_torque = 0;
//                    } else {
//                        // 恢复失败，重新尝试
//                        selfhelp_state = SELFHELP_PREPARE;
//                        selfhelp_timer = current_time;
//                    }
//                }
//                break;
//        }
//    } else {
//        // 自救完成，重置状态机
//        selfhelp_state = SELFHELP_IDLE;
//    }
//}
/** ��ȡ���̴��������� **/
static void get_IMU_info(void) {

    /** Yaw **/
    chassis.imu_reference.yaw_rad = -INS.Yaw * DEGREE_TO_RAD;

    chassis.imu_reference.yaw_total_rad = -INS.YawTotalAngle * DEGREE_TO_RAD;

    /** Pitch **/
    chassis.imu_reference.pitch_rad = -INS.Roll * DEGREE_TO_RAD;

    /** Roll **/
    chassis.imu_reference.roll_rad = INS.Pitch * DEGREE_TO_RAD;

    /** ���¸�����ٶȺͽ��ٶ� **/
    chassis.imu_reference.pitch_gyro = -INS.Gyro[Y];
    chassis.imu_reference.yaw_gyro = -INS.Gyro[Z];
    chassis.imu_reference.roll_gyro = INS.Gyro[X];

    chassis.imu_reference.ax = INS.Accel[X];
    chassis.imu_reference.ay = INS.Accel[Y];
    chassis.imu_reference.az = INS.Accel[Z];

    /** ������ֱ������ٶ� **/
    chassis.imu_reference.robot_az = INS.MotionAccel_n[Z];
}

/** ���µ��̱��� **/
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

/** �������������� **/

static void wheel_calc(void)
{
    /******************************* Wheel *************************************/

    /** �����ȳ����������ϵ����ϳ���������K **/
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

/** ����ؽ����� **/
static void joint_calc(void)
{

//    if(chassis.chassis_recover_finish == false) {
//        // 只在自救状态机中直接设置力矩
//        return;
//    }

/******************************* Joint *************************************/

    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

    /** ��ؼ�� **/
    chassis_off_ground();

    /** Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp **/

    /****** ������pid ******/
    chassis.steer_compensatory_torque =  CHASSIS_LEG_COORDINATION_PID_P * (0.0f - chassis.phi0_error)
                                         + CHASSIS_LEG_COORDINATION_PID_D * (0.0f - chassis.d_phi0_error); // ע��΢��������

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

    if(chassis.chassis_ctrl_mode == CHASSIS_JUMP)
    {
        if(chassis.chassis_ctrl_info.height_m >= 0.3f)
        {
            pid_init(&chassis.leg_L.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_JH,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_JH,
                     CHASSIS_LEG_L0_POS_PID_P_JH,
                     CHASSIS_LEG_L0_POS_PID_I_JH,
                     CHASSIS_LEG_L0_POS_PID_D_JH);

            pid_init(&chassis.leg_R.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_JH,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_JH,
                     CHASSIS_LEG_L0_POS_PID_P_JH,
                     CHASSIS_LEG_L0_POS_PID_I_JH,
                     CHASSIS_LEG_L0_POS_PID_D_JH);

            pid_init(&chassis.leg_L.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_JH,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_JH,
                     CHASSIS_LEG_L0_SPEED_PID_P_JH,
                     CHASSIS_LEG_L0_SPEED_PID_I_JH,
                     CHASSIS_LEG_L0_SPEED_PID_D_JH);
            pid_init(&chassis.leg_R.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_JH,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_JH,
                     CHASSIS_LEG_L0_SPEED_PID_P_JH,
                     CHASSIS_LEG_L0_SPEED_PID_I_JH,
                     CHASSIS_LEG_L0_SPEED_PID_D_JH);
        }
        else if(chassis.chassis_ctrl_info.height_m >= 0.15f && chassis.chassis_ctrl_info.height_m < 0.30f)
        {
            pid_init(&chassis.leg_L.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_JM,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_JM,
                     CHASSIS_LEG_L0_POS_PID_P_JM,
                     CHASSIS_LEG_L0_POS_PID_I_JM,
                     CHASSIS_LEG_L0_POS_PID_D_JM);
            pid_init(&chassis.leg_R.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_JM,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_JM,
                     CHASSIS_LEG_L0_POS_PID_P_JM,
                     CHASSIS_LEG_L0_POS_PID_I_JM,
                     CHASSIS_LEG_L0_POS_PID_D_JM);

            pid_init(&chassis.leg_L.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_JM,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_JM,
                     CHASSIS_LEG_L0_SPEED_PID_P_JM,
                     CHASSIS_LEG_L0_SPEED_PID_I_JM,
                     CHASSIS_LEG_L0_SPEED_PID_D_JM);
            pid_init(&chassis.leg_R.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_JM,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_JM,
                     CHASSIS_LEG_L0_SPEED_PID_P_JM,
                     CHASSIS_LEG_L0_SPEED_PID_I_JM,
                     CHASSIS_LEG_L0_SPEED_PID_D_JM);
        }
        else if(chassis.chassis_ctrl_info.height_m < 0.15f)

        {
            pid_init(&chassis.leg_L.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_JL,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_JL,
                     CHASSIS_LEG_L0_POS_PID_P_JL,
                     CHASSIS_LEG_L0_POS_PID_I_JL,
                     CHASSIS_LEG_L0_POS_PID_D_JL);
            pid_init(&chassis.leg_R.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_JL,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_JL,
                     CHASSIS_LEG_L0_POS_PID_P_JL,
                     CHASSIS_LEG_L0_POS_PID_I_JL,
                     CHASSIS_LEG_L0_POS_PID_D_JL);

            pid_init(&chassis.leg_L.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_JL,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_JL,
                     CHASSIS_LEG_L0_SPEED_PID_P_JL,
                     CHASSIS_LEG_L0_SPEED_PID_I_JL,
                     CHASSIS_LEG_L0_SPEED_PID_D_JL);
            pid_init(&chassis.leg_R.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_JL,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_JL,
                     CHASSIS_LEG_L0_SPEED_PID_P_JL,
                     CHASSIS_LEG_L0_SPEED_PID_I_JL,
                     CHASSIS_LEG_L0_SPEED_PID_D_JL);
        }
    }
    else{
        if(chassis.chassis_ctrl_info.height_m >= 0.3f)
        {
            pid_init(&chassis.leg_L.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_H,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_H,
                     CHASSIS_LEG_L0_POS_PID_P_H,
                     CHASSIS_LEG_L0_POS_PID_I_H,
                     CHASSIS_LEG_L0_POS_PID_D_H);

            pid_init(&chassis.leg_R.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_H,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_H,
                     CHASSIS_LEG_L0_POS_PID_P_H,
                     CHASSIS_LEG_L0_POS_PID_I_H,
                     CHASSIS_LEG_L0_POS_PID_D_H);

            pid_init(&chassis.leg_L.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_H,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_H,
                     CHASSIS_LEG_L0_SPEED_PID_P_H,
                     CHASSIS_LEG_L0_SPEED_PID_I_H,
                     CHASSIS_LEG_L0_SPEED_PID_D_H);
            pid_init(&chassis.leg_R.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_H,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_H,
                     CHASSIS_LEG_L0_SPEED_PID_P_H,
                     CHASSIS_LEG_L0_SPEED_PID_I_H,
                     CHASSIS_LEG_L0_SPEED_PID_D_H);
        }
        else if(chassis.chassis_ctrl_info.height_m >= 0.15f && chassis.chassis_ctrl_info.height_m < 0.30f)
        {
            pid_init(&chassis.leg_L.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_M,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_M,
                     CHASSIS_LEG_L0_POS_PID_P_M,
                     CHASSIS_LEG_L0_POS_PID_I_M,
                     CHASSIS_LEG_L0_POS_PID_D_M);
            pid_init(&chassis.leg_R.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_M,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_M,
                     CHASSIS_LEG_L0_POS_PID_P_M,
                     CHASSIS_LEG_L0_POS_PID_I_M,
                     CHASSIS_LEG_L0_POS_PID_D_M);

            pid_init(&chassis.leg_L.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_M,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_M,
                     CHASSIS_LEG_L0_SPEED_PID_P_M,
                     CHASSIS_LEG_L0_SPEED_PID_I_M,
                     CHASSIS_LEG_L0_SPEED_PID_D_M);
            pid_init(&chassis.leg_R.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_M,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_M,
                     CHASSIS_LEG_L0_SPEED_PID_P_M,
                     CHASSIS_LEG_L0_SPEED_PID_I_M,
                     CHASSIS_LEG_L0_SPEED_PID_D_M);
        }
        else if(chassis.chassis_ctrl_info.height_m < 0.15f)

        {
            pid_init(&chassis.leg_L.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_L,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_L,
                     CHASSIS_LEG_L0_POS_PID_P_L,
                     CHASSIS_LEG_L0_POS_PID_I_L,
                     CHASSIS_LEG_L0_POS_PID_D_L);
            pid_init(&chassis.leg_R.leg_pos_pid,
                     CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_L,
                     CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_L,
                     CHASSIS_LEG_L0_POS_PID_P_L,
                     CHASSIS_LEG_L0_POS_PID_I_L,
                     CHASSIS_LEG_L0_POS_PID_D_L);

            pid_init(&chassis.leg_L.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_L,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_L,
                     CHASSIS_LEG_L0_SPEED_PID_P_L,
                     CHASSIS_LEG_L0_SPEED_PID_I_L,
                     CHASSIS_LEG_L0_SPEED_PID_D_L);
            pid_init(&chassis.leg_R.leg_speed_pid,
                     CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_L,
                     CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_L,
                     CHASSIS_LEG_L0_SPEED_PID_P_L,
                     CHASSIS_LEG_L0_SPEED_PID_I_L,
                     CHASSIS_LEG_L0_SPEED_PID_D_L);
        }
    }


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
                                             + CHASSIS_ROLL_PID_D * (0.0f - chassis.d_roll_error); // ע��΢��������

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_L.state_variable_feedback.theta)
                                                                         + chassis.leg_L.leg_speed_pid.out;

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_R.state_variable_feedback.theta)
                                                                         + chassis.leg_R.leg_speed_pid.out;

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += chassis.roll_f;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= chassis.roll_f;

     // 跳跃模式特殊处理
    if(chassis.chassis_ctrl_mode == CHASSIS_JUMP) {
        switch(chassis.jump_state) {
            case STRETCHING:
                // 在弹射阶段添加很大的跳跃力
                float extra_jump_force = 650.0f; // 很大的弹射力//650
//                chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += 0.75f * extra_jump_force;
                chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point +=  extra_jump_force;
                chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += extra_jump_force;
                break;

            case SHRINKING_IN_AIR:
                // 在空中收缩阶段添加反推力
                float retract_compensation = 180.0f; // 反推力，加速腿部收缩
                chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= retract_compensation;
                chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= retract_compensation;
                break;

            case RECOVERING:
                // 恢复阶段，增加恢复力
                float recover_force = 100.0f; // 恢复力
                chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += recover_force;
                chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += recover_force;
                break;

            default:
                // 其他状态不添加额外力
                break;
        }
    }

    /** End End End End End End End End End End End End End End End End End End End End End End End End **/

    // ����ؽڵ������
    vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);
    vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

    chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B
    //
    // USART_Vofa_Justfloat_Transmit(chassis.leg_L.joint_F_torque - chassis.leg_R.joint_F_torque
    //     , chassis.leg_L.joint_B_torque -  chassis.leg_R.joint_B_torque  ,0);

    // ����޷�
    VAL_LIMIT(chassis.leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);

}

/** ���������� **/
static void controller_calc(void)
{
    /** ���������˲��� **/
    vmc_calc();
    /** �ٶ��ں� **/
    speed_calc();
    /** ���µ��̱��� **/
    chassis_variable_update();
    /** �������������� **/
    wheel_calc();
    /** ����ؽ����� **/
    joint_calc();
}


/*******************************************************************************
 *                                  Task                                       *
 *******************************************************************************/

/** ����ʧ������ **/
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

    // ����״̬
    chassis.chassis_body_state = CHASSIS_BODY_UNNORMAL;

    chassis.chassis_fall_leg_state = CHASSIS_FALL_LEG_UNNORMAL;

    chassis.chassis_recover_state = CHASSIS_COULD_NOT_RECOVER;

    /** ��ʼ����־λ **/

    // ���̳�ʼ����־λ
    chassis.init_flag = false;

    chassis.chassis_recover_finish = false;

    chassis.jump_flag = false;

}

/** ���̳�ʼ������ **/
static void chassis_init_task()
{
    joint_enable();

    chassis.init_flag = true;
}

/** ����ʹ������ **/
static void chassis_enable_task(void)
{
    /** ���������� **/
    controller_calc();

    /** �����Ծ� **/
    chassis_selfhelp();
}
/** 跳跃任务 **/
static void chassis_jump_task()
{
    static uint32_t jump_timer = 0;

    switch(chassis.jump_state)
    {
        case NOT_READY:
            // 初始化跳跃状态
            // 先将腿长收到最短，以获得最长的加速距离
            chassis.chassis_ctrl_info.height_m = 0.09685f;
            jump_timer = 0;

            // 检查是否准备好跳跃（腿部收缩到最小）
            if((fabsf(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 - chassis.chassis_ctrl_info.height_m) < 0.05f &&
                fabsf(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 - chassis.chassis_ctrl_info.height_m) < 0.05f) )
                //||((xTaskGetTickCount() - jump_timer) > 1000)
            {
                chassis.jump_state = STRETCHING; // 直接进入弹射阶段
                jump_timer = xTaskGetTickCount(); // 记录开始时间
            }

            break;

        case STRETCHING:
            // 弹射阶段，施加弹射力并控制时间
            // 设置伸展目标高度（例如最大伸展高度）
            chassis.chassis_ctrl_info.height_m = 0.38685f;

            if((fabsf(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 - chassis.chassis_ctrl_info.height_m) < 0.03f &&
                fabsf(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 - chassis.chassis_ctrl_info.height_m) < 0.03f) ){
                // ||((xTaskGetTickCount() - jump_timer) > 500)
                // 弹射力结束后，设置目标为最小长度（空中收缩）
                chassis.chassis_ctrl_info.height_m = 0.09685f;
                chassis.jump_state = SHRINKING_IN_AIR;
                jump_timer = xTaskGetTickCount();
            }
            break;

        case SHRINKING_IN_AIR:
            // 空中收缩腿部阶段，施加反推力
            chassis.chassis_ctrl_info.height_m = 0.09685f;

            // 检查是否已收缩到最小长度
            if((fabsf(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 - chassis.chassis_ctrl_info.height_m) < 0.03f &&
                fabsf(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 - chassis.chassis_ctrl_info.height_m) < 0.03f) )
            // ||
                //               ((xTaskGetTickCount() - jump_timer) > 500)
                {
                chassis.jump_state = RECOVERING;
                jump_timer = xTaskGetTickCount();
            }
            break;

        case RECOVERING:
            // 恢复到跳跃前的高度
            chassis.chassis_ctrl_info.height_m = 0.25f; // 跳跃前的高度

            // 检查是否恢复到目标高度
            if((fabsf(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 - chassis.chassis_ctrl_info.height_m) < 0.03f &&
                fabsf(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 - chassis.chassis_ctrl_info.height_m) < 0.03f))
            // || ((xTaskGetTickCount() - jump_timer) > 1000)
            {
                // 完成跳跃
                chassis.jump_state = JUMP_OVER;
            }
            break;
        default:
            chassis.jump_state = NOT_READY;
            break;
    }
//    chassis.jump_flag = true;
}



/** ������������ **/
static void send_torque_task(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque,
                             float wheel_L_torque, float wheel_R_torque,
                             float vel, float Kd)
{
//    // 自救模式下使用不同的控制参数
//    if(chassis.chassis_recover_finish == false) {
//        // 自救模式下，直接使用计算出的关节力矩
//        // 而不是MIT控制
//        set_dm8009_MIT(&joint[LF], 0.0f, 0.0f, 0.0f, 0.0f, joint_LF_torque);
//        set_dm8009_MIT(&joint[LB], 0.0f, 0.0f, 0.0f, 0.0f, joint_LB_torque);
//        DWT_Delay(0.0002f);
//        set_dm8009_MIT(&joint[RF], 0.0f, 0.0f, 0.0f, 0.0f, joint_RF_torque);
//        set_dm8009_MIT(&joint[RB], 0.0f, 0.0f, 0.0f, 0.0f, joint_RB_torque);
//    } else {
        // 正常模式
        set_dm8009_MIT(&joint[LF], 0.0f, vel, 0, Kd, joint_LF_torque);
        set_dm8009_MIT(&joint[LB], 0.0f, vel, 0, Kd, joint_LB_torque);
        DWT_Delay(0.0002f);
        set_dm8009_MIT(&joint[RF], 0.0f, -vel, 0, Kd, joint_RF_torque);
        set_dm8009_MIT(&joint[RB], 0.0f, -vel, 0, Kd, joint_RB_torque);
//    }

    lk9025_multi_torque_set(wheel_L_torque, wheel_R_torque);
}

void chassis_task(void)
{
    /** ��ȡң������Ϣ(ģʽ + ����) **/
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

//    send_torque_task(-chassis.leg_L.joint_F_torque,
//                     -chassis.leg_L.joint_B_torque,
//                     chassis.leg_R.joint_F_torque,
//                     chassis.leg_R.joint_B_torque,
//                      100,
//                     100,
//                     vel,
//                     Kd);

//     send_torque_task(5,
//              -5,
//              5,
//              5,
//              0,0,
//              vel,
//              Kd);


    // send_torque_task(0,
    //                  0,
    //                  0,
    //                  0,
    //                  0,
    //                  0,
    //                  0,
    //                  0);

//     USART_Vofa_Justfloat_Transmit(-chassis.leg_L.wheel_torque,-chassis.leg_R.wheel_torque,0);
//    5jump 1off2on 0not
//    USART_Vofa_Justfloat_Transmit(-chassis.leg_L.wheel_torque ,-chassis.leg_R.wheel_torque,0);

    USART_Vofa_Justfloat_Transmit((float)((int)(joint[RB].torque) * 100 + (int)(joint[RF].torque)),//4 3     4 4    26 39    -4 0       1 30
                                  (float)((int)(joint[LB].torque) * 100 + (int)(joint[LF].torque)),//2 1    -5 4    0 26     -4/-2 4    0  1
                                  0); // 添加缺失的参数

}