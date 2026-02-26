#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "robot_def.h"
#include "vmc.h"
#include "user_lib.h"
#include "joint.h"
#include "moving_filter.h"
#include "vofa.h"

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

/*               正方向
 *    phi4                      phi4
 *
 *    phi1                      phi1
 */



/** 计算状态变量theta **/
float cal_leg_theta(float phi0, float phi)
{
    float theta = 0, alpha = 0;//alpha is the Angle at which the virtual joint motor is turned
    alpha = PI / 2 - phi0;

    if (alpha * phi < 0) {
        theta = ABS(alpha) - ABS(phi);
        if ((alpha > 0) && (phi < 0)) {
            theta *= -1;
        } else {

        }
    } else {
        theta = ABS(alpha) + ABS(phi);
        if ((alpha < 0) && (phi < 0)) {
        } else {
            theta *= -1;
        }
    }
    return theta;
}

/* =========================== 1. 正解算 =========================== */

void forward_kinematics()
{

    /* =========================== 中间变量 =========================== */

    float temp=0,y=0,x=0;

    /* =========================== 左腿 =========================== */

        /* =========================== 1.2.1 初始角度 =========================== */

        float LF_joint_pos = (get_joint_motors() + 0)->pos_r;
        float LB_joint_pos = (get_joint_motors() + 1)->pos_r;

        /* =========================== 1.2.2  实际角度  =========================== */

        chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1 = (5*PI)/4 + LB_joint_pos - 7.0f * DEGREE_TO_RAD;
        chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4 = -PI/4 + LF_joint_pos + 10.0f * DEGREE_TO_RAD;

        /* =========================== 1.3  B,D坐标 =========================== */

        chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_x = chassis_physical_config.l1 * cosf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1);
        chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_y = chassis_physical_config.l1 * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1);
        chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_x = chassis_physical_config.l5 + chassis_physical_config.l4 * cosf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4);
        chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_y = chassis_physical_config.l4 * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4);

        /* =========================== 1.4  lBD^2  =========================== */

        float L_BD_sq =  (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_x - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_x) * (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_x - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_x)
                     + (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_y - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_y) * (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_y - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_y);

        /* =========================== 1.5  phi2 =========================== */

        float L_A0 = 2.0f * chassis_physical_config.l2 * (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_x - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_x);
        float L_B0 = 2.0f * chassis_physical_config.l2 * (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_y - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_y);
        float L_C0 = chassis_physical_config.l2 * chassis_physical_config.l2 + L_BD_sq - chassis_physical_config.l3 * chassis_physical_config.l3;
        temp = L_A0 * L_A0 + L_B0 * L_B0 - L_C0 * L_C0;
        y = L_B0 + sqrtf(ABS(temp));
        x = L_A0 + L_C0;
        chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

        /* =========================== 1.5  phi3 =========================== */

        chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_x = chassis_physical_config.l1 * cosf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1) + chassis_physical_config.l2 * cosf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2);
        chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_y = chassis_physical_config.l1 * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1) + chassis_physical_config.l2 * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2);
        y = chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_y - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_y;
        x = chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_x - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_x;
        chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

        /* =========================== 1.6  L0 =========================== */

        chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_last = chassis.leg_L.vmc.forward_kinematics.fk_L0.L0;

        temp =  (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_x - chassis_physical_config.l5 * 0.5f) * (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_x - chassis_physical_config.l5 * 0.5f)
                + chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_y * chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_y;
        chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));

        chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot_last = chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot;
        chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot =
                (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 - chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_last)
                / (CHASSIS_PERIOD * 0.001f);
        chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_ddot =
                (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot - chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot_last)
                / (CHASSIS_PERIOD * 0.001f);

        /* =========================== 1.6  phi0 =========================== */

        chassis.leg_L.vmc.forward_kinematics.fk_phi.last_phi0 = chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0;

        y = chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_y;
        x = chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_x - chassis_physical_config.l5 * 0.5f;
        chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);

        chassis.leg_L.vmc.forward_kinematics.fk_phi.last_d_phi0 = chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0;
        chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0 =  (chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_L.vmc.forward_kinematics.fk_phi.last_phi0)
                                                       / (CHASSIS_PERIOD * 0.001f);
        chassis.leg_L.vmc.forward_kinematics.fk_phi.dd_phi0 =  (chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0 - chassis.leg_L.vmc.forward_kinematics.fk_phi.last_d_phi0)
                                                        / (CHASSIS_PERIOD * 0.001f);

        chassis.leg_L.vmc.forward_kinematics.d_alpha = 0.0f - chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0;


    /* =========================== 右腿 =========================== */

        /* =========================== 1.2.1 初始角度 =========================== */

        float RF_joint_pos = (get_joint_motors() + 2)->pos_r;
        float RB_joint_pos = (get_joint_motors() + 3)->pos_r;

        /* =========================== 1.2.2  实际角度 =========================== */

        chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1 = (5*PI)/4 - RB_joint_pos - 9.0f * DEGREE_TO_RAD;
        chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4 = -PI/4 - RF_joint_pos + 7.0f * DEGREE_TO_RAD;

        /* =========================== 1.3  B,D坐标 =========================== */

        chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_x = chassis_physical_config.l1 * cosf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1);
        chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_y = chassis_physical_config.l1 * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1);
        chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_x = chassis_physical_config.l5 + chassis_physical_config.l4 * cosf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4);
        chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_y = chassis_physical_config.l4 * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4);

        /* =========================== 1.4  lBD^2  =========================== */

        float R_BD_sq =  (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_x - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_x) * (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_x - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_x)
                         + (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_y - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_y) * (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_y - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_y);

        /* =========================== 1.5  phi2 =========================== */

        float R_A0 = 2.0f * chassis_physical_config.l2 * (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_x - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_x);
        float R_B0 = 2.0f * chassis_physical_config.l2 * (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_y - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_y);
        float R_C0 = chassis_physical_config.l2 * chassis_physical_config.l2 + R_BD_sq - chassis_physical_config.l3 * chassis_physical_config.l3;
        temp = R_A0 * R_A0 + R_B0 * R_B0 - R_C0 * R_C0;
        y = R_B0 + sqrtf(ABS(temp));
        x = R_A0 + R_C0;
        chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

        /* =========================== 1.5  phi3 =========================== */

        chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x = chassis_physical_config.l1 * cosf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1) + chassis_physical_config.l2 * cosf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2);
        chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y = chassis_physical_config.l1 * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1) + chassis_physical_config.l2 * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2);
        y = chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_y;
        x = chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_x;
        chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

        /* =========================== 1.6  L0 =========================== */

        chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_last = chassis.leg_R.vmc.forward_kinematics.fk_L0.L0;

        temp =  (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x - chassis_physical_config.l5 * 0.5f) * (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x - chassis_physical_config.l5 * 0.5f)
                + chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y * chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y;
        chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));

        chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot_last = chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot;
        chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot = (chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 - chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_last) / (CHASSIS_PERIOD * 0.001f);
        chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_ddot = (chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot - chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot_last) / (CHASSIS_PERIOD * 0.001f);

        /* =========================== 1.6  phi0 =========================== */

        chassis.leg_R.vmc.forward_kinematics.fk_phi.last_phi0 = chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0;

        y = chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y;
        x = chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x - chassis_physical_config.l5 * 0.5f;
        chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);

        chassis.leg_R.vmc.forward_kinematics.fk_phi.last_d_phi0 = chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0;
        chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0 =  (chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_R.vmc.forward_kinematics.fk_phi.last_phi0) / (CHASSIS_PERIOD * 0.001f);
        chassis.leg_R.vmc.forward_kinematics.fk_phi.dd_phi0 =  (chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0 - chassis.leg_R.vmc.forward_kinematics.fk_phi.last_d_phi0) / (CHASSIS_PERIOD * 0.001f);

        chassis.leg_R.vmc.forward_kinematics.d_alpha = 0.0f - chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0;

    /** 防劈叉 **/
    chassis.last_phi0_error = chassis.phi0_error;
    chassis.phi0_error = chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0;
    chassis.d_phi0_error = (chassis.phi0_error - chassis.last_phi0_error) / (CHASSIS_PERIOD * 0.001f);

    /** roll **/
    chassis.last_roll_error = chassis.roll_error;
    chassis.roll_error = chassis.imu_reference.roll_rad;
    chassis.d_roll_error = (chassis.roll_error - chassis.last_roll_error) / (CHASSIS_PERIOD * 0.001f);
}

/* =========================== 2. 正动力学变换 =========================== */

void vmc_forward_dynamics()
{
    /* =========================== 左腿 =========================== */

        /* =========================== 矩阵 =========================== */
        chassis.leg_L.vmc.forward_kinematics.J_F_to_T.E.x1_1 =
                chassis_physical_config.l1 * cosf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3) * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2)
                / (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2));

        chassis.leg_L.vmc.forward_kinematics.J_F_to_T.E.x1_2 =
                chassis_physical_config.l1 * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3) * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2)
                / sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2);

        chassis.leg_L.vmc.forward_kinematics.J_F_to_T.E.x2_1 =
                chassis_physical_config.l4 * cosf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2) * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4)
                / (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2));

        chassis.leg_L.vmc.forward_kinematics.J_F_to_T.E.x2_2 = chassis_physical_config.l4 * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2) * sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4)
                                                  / sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2);
        /* =========================== T1,T4 =========================== */
        Matrix_multiply(2, 2, chassis.leg_L.vmc.forward_kinematics.J_F_to_T.array,
                        2, 1, chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.array,
                        chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.array);

    /* =========================== 右腿 =========================== */

        /* =========================== 矩阵 =========================== */
        chassis.leg_R.vmc.forward_kinematics.J_F_to_T.E.x1_1 =
            chassis_physical_config.l1 * cosf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3) * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2)
            / (chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2));

        chassis.leg_R.vmc.forward_kinematics.J_F_to_T.E.x1_2 =
                chassis_physical_config.l1 * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3) * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2)
                / sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2);

        chassis.leg_R.vmc.forward_kinematics.J_F_to_T.E.x2_1 =
                chassis_physical_config.l4 * cosf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2) * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4)
                / (chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2));

        chassis.leg_R.vmc.forward_kinematics.J_F_to_T.E.x2_2 = chassis_physical_config.l4 * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2) * sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4)
                                                  / sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2);

        /* =========================== T1,T4 =========================== */
        Matrix_multiply(2, 2, chassis.leg_R.vmc.forward_kinematics.J_F_to_T.array,
                        2, 1, chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.array,
                        chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.array);
}


// 逆解算腿长变化速度、摆角变化速度
static void vmc_inverse_kinematics(VMC *vmc,
                                   float w1,
                                   float w4,
                                   const ChassisPhysicalConfig *chassis_physical_config) {
    if (vmc == NULL) {
        return;
    }
    vmc->inverse_kinematics.W_fdb.E.w1_fdb = w1;
    vmc->inverse_kinematics.W_fdb.E.w4_fdb = w4;

    vmc->inverse_kinematics.J_w_to_v.E.x1_1 =  -chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                                               / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);


    vmc->inverse_kinematics.J_w_to_v.E.x1_2 = -chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);


    vmc->inverse_kinematics.J_w_to_v.E.x2_1 = -chassis_physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                                              / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));


    vmc->inverse_kinematics.J_w_to_v.E.x2_2 = -chassis_physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.V_fdb.E.last_d_L0_fdb = vmc->inverse_kinematics.V_fdb.E.d_L0_fdb;

    Matrix_multiply(2, 2, vmc->inverse_kinematics.J_w_to_v.array,
                    2, 1, vmc->inverse_kinematics.W_fdb.array,
                    vmc->inverse_kinematics.V_fdb.array);


    vmc->inverse_kinematics.V_fdb.E.dd_L0_fdb = (vmc->inverse_kinematics.V_fdb.E.d_L0_fdb - vmc->inverse_kinematics.V_fdb.E.last_d_L0_fdb) / (CHASSIS_PERIOD * 0.001f);
}

// 逆解算出虚拟力矩和沿腿方向支持力
static void vmc_inverse_dynamics(VMC *vmc,
                                 float T1, // phi1
                                 float T4, // phi4
                                 const ChassisPhysicalConfig *chassis_physical_config) {
    if (vmc == NULL) {
        return;
    }
    vmc->inverse_kinematics.T1_T4_fdb.E.T1_fdb = T1;
    vmc->inverse_kinematics.T1_T4_fdb.E.T4_fdb = T4;

    vmc->inverse_kinematics.J_T_to_F.E.x1_1 =
            vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
            / (chassis_physical_config->l1
               * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->inverse_kinematics.J_T_to_F.E.x1_2 =
            vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
            / (chassis_physical_config->l4
               * sinf(vmc->forward_kinematics.fk_phi.phi4 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.J_T_to_F.E.x2_1 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
                                              / (chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi1));

    vmc->inverse_kinematics.J_T_to_F.E.x2_2 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
                                              / (chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4));


    Matrix_multiply(2, 2, vmc->inverse_kinematics.J_T_to_F.array,
                    2, 1, vmc->inverse_kinematics.T1_T4_fdb.array,
                    vmc->inverse_kinematics.Fxy_fdb.array);
}

// 计算竖直方向支持力
static void fn_cal(Leg *leg, float body_az, const ChassisPhysicalConfig *chassis_physical_config) {

    if (leg == NULL) {
        return;
    }

    // 用逆解算的数据计算
    float P = leg->vmc.inverse_kinematics.Fxy_fdb.E.Fy_fdb * cosf(leg->state_variable_feedback.theta)
              + leg->vmc.inverse_kinematics.Fxy_fdb.E.Tp_fdb * sinf(leg->state_variable_feedback.theta) / leg->vmc.forward_kinematics.fk_L0.L0;

    float wheel_az = body_az - leg->vmc.inverse_kinematics.V_fdb.E.dd_L0_fdb * cosf(leg->state_variable_feedback.theta)
                     + 2.0f * leg->vmc.inverse_kinematics.V_fdb.E.d_L0_fdb * leg->state_variable_feedback.theta_dot
                       * sinf(leg->state_variable_feedback.theta)
                     + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_ddot
                       * sinf(leg->state_variable_feedback.theta)
                     + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_dot
                       * leg->state_variable_feedback.theta_dot * cosf(leg->state_variable_feedback.theta);


    leg->Fn = P + chassis_physical_config->wheel_weight * (GRAVITY + wheel_az);

}
/** 逆解算计算腿部支撑力 **/
void calculate_f(void)
{
    // 计算左腿支撑力
    vmc_inverse_kinematics(&chassis.leg_L.vmc,joint[LF].angular_vel,joint[LB].angular_vel,&chassis_physical_config);
    vmc_inverse_dynamics(&chassis.leg_L.vmc,
                         chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1,
                         chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4,
                         &chassis_physical_config);

    fn_cal(&chassis.leg_L, chassis.imu_reference.robot_az, &chassis_physical_config);

    // 计算右腿支撑力
    vmc_inverse_kinematics(&chassis.leg_R.vmc,joint[RF].angular_vel,joint[RB].angular_vel,&chassis_physical_config);
    vmc_inverse_dynamics(&chassis.leg_R.vmc,
                         chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1,
                         chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4,
                         &chassis_physical_config);

    fn_cal(&chassis.leg_R, chassis.imu_reference.robot_az, &chassis_physical_config);
}
/*******************************************************************************
 *                                     VMC                                     *
 *******************************************************************************/
