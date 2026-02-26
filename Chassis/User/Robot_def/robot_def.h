#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdbool.h>

#include "pid.h"
#include "moving_filter.h"
#include "can_device.h"
#include "user_lib.h"
#include "board_communication_task.h"
#include "low_pass_filter.h"


/*******************************************************************************
 *                                    ????                                     *
 *******************************************************************************/

/** ???? **/

// ????????????
#define CHASSIS_PERIOD 5 // ms ???????: 200Hz

// ????
#define PHI_BALANCE 0.0f * DEGREE_TO_RAD // ??????0.0??

// ??????
#define SPIN_SPEED 5.0f

// ???????
#define NOT_BALANCE_RAD 13.0f * DEGREE_TO_RAD//13*PI/180
#define RECOVER_RAD 5.0f * DEGREE_TO_RAD //5*PI/180

// ????
#define LEG_NORMAL_RAD 50.0f * DEGREE_TO_RAD//30

// //LK9025??????
// #define LK9025_MAX_SPEED 490

/** ?????°§?? **/
// x : 2-???? ; 0-????
// y : 3-???? ; 1-????

#define CHASSIS_VX_CHANNEL 1
#define CHASSIS_LEG_CHANNEL 2

/** ??????? **/
#define MIN_L0 0.15f

#define MAX_CHASSIS_VX_SPEED 2.1f
#define MAX_WHEEL_TORQUE 10.f
#define MIN_WHEEL_TORQUE (-10.f)
#define MAX_JOINT_TORQUE 40.f//40 //10
#define MIN_JOINT_TORQUE (-40.f)//-40

/** ????????? **/
#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define MAX_CHASSIS_YAW_INCREMENT 0.01f
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)


/****** PID???? ******/

/** Wheel **/

// ???PID
// #define CHASSIS_TURN_POS_PID_P 5.0f
// #define CHASSIS_TURN_POS_PID_I 0.0f
// #define CHASSIS_TURN_POS_PID_D 0.0f
// #define CHASSIS_TURN_POS_PID_IOUT_LIMIT 0.0f
// #define CHASSIS_TURN_POS_PID_OUT_LIMIT 3.0f
//
// #define CHASSIS_TURN_SPEED_PID_P 10.0f
// #define CHASSIS_TURN_SPEED_PID_I 0.0f
// #define CHASSIS_TURN_SPEED_PID_D 0.0f
// #define CHASSIS_TURN_SPEED_PID_IOUT_LIMIT 0.0f
// #define CHASSIS_TURN_SPEED_PID_OUT_LIMIT 4.0f

#define CHASSIS_TURN_POS_PID_P 10.0f
#define CHASSIS_TURN_POS_PID_I 0.0f
#define CHASSIS_TURN_POS_PID_D 23.0f
#define CHASSIS_TURN_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_TURN_POS_PID_OUT_LIMIT 20.0f//5.5

#define CHASSIS_TURN_SPEED_PID_P 5.0f
#define CHASSIS_TURN_SPEED_PID_I 0.0f
#define CHASSIS_TURN_SPEED_PID_D 0.0f
#define CHASSIS_TURN_SPEED_PID_IOUT_LIMIT 0.0f
#define CHASSIS_TURN_SPEED_PID_OUT_LIMIT 5.0f


/** Joint **/

// ??????PID
// #define CHASSIS_LEG_COORDINATION_PID_P 50.0f // 20.0f 30.0f
// #define CHASSIS_LEG_COORDINATION_PID_I 0.0f
// #define CHASSIS_LEG_COORDINATION_PID_D 5.0f // 1.0f 5.0f
// #define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 0.0f
// #define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 10.0f

#define CHASSIS_LEG_COORDINATION_PID_P 10.0f//12
#define CHASSIS_LEG_COORDINATION_PID_I 0.0f
#define CHASSIS_LEG_COORDINATION_PID_D 5.0f
#define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 10.0f

// // ???¶À???PID
// #define CHASSIS_LEG_L0_POS_PID_P 15.0f
// #define CHASSIS_LEG_L0_POS_PID_I 0.0f
// #define CHASSIS_LEG_L0_POS_PID_D 15.0f
// #define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT 0.0f
// #define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT 2.0f
//
// // ???????PID
// #define CHASSIS_LEG_L0_SPEED_PID_P 30.0f // 50.0f
// #define CHASSIS_LEG_L0_SPEED_PID_I 0.0f
// #define CHASSIS_LEG_L0_SPEED_PID_D 0.0f
// #define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT 0.0f
// #define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT 60.0f

// ???¶À???PID
#define CHASSIS_LEG_L0_POS_PID_P_H 30.0f//10
#define CHASSIS_LEG_L0_POS_PID_I_H 0.0f
#define CHASSIS_LEG_L0_POS_PID_D_H 15.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_H 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_H 2.0f

#define CHASSIS_LEG_L0_POS_PID_P_M 20.0f//10
#define CHASSIS_LEG_L0_POS_PID_I_M 0.0f
#define CHASSIS_LEG_L0_POS_PID_D_M 15.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_M 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_M 2.0f

#define CHASSIS_LEG_L0_POS_PID_P_L 20.0f//10
#define CHASSIS_LEG_L0_POS_PID_I_L 0.0f
#define CHASSIS_LEG_L0_POS_PID_D_L 15.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_L 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_L 2.0f

#define CHASSIS_LEG_L0_POS_PID_P_JH 28.0f//10
#define CHASSIS_LEG_L0_POS_PID_I_JH 0.0f
#define CHASSIS_LEG_L0_POS_PID_D_JH 15.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_JH 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_JH 2.0f

#define CHASSIS_LEG_L0_POS_PID_P_JM 20.0f//10
#define CHASSIS_LEG_L0_POS_PID_I_JM 0.0f
#define CHASSIS_LEG_L0_POS_PID_D_JM 15.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_JM 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_JM 2.0f

#define CHASSIS_LEG_L0_POS_PID_P_JL 20.0f//10
#define CHASSIS_LEG_L0_POS_PID_I_JL 0.0f
# define CHASSIS_LEG_L0_POS_PID_D_JL 15.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT_JL 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT_JL 2.0f


// ???????PID
#define CHASSIS_LEG_L0_SPEED_PID_P_H 10.0f//25
#define CHASSIS_LEG_L0_SPEED_PID_I_H 5.0f
#define CHASSIS_LEG_L0_SPEED_PID_D_H 50.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_H 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_H 60.0f

#define CHASSIS_LEG_L0_SPEED_PID_P_M 20.0f//25
#define CHASSIS_LEG_L0_SPEED_PID_I_M 5.0f
#define CHASSIS_LEG_L0_SPEED_PID_D_M 5.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_M 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_M 60.0f

#define CHASSIS_LEG_L0_SPEED_PID_P_L 15.0f//25
#define CHASSIS_LEG_L0_SPEED_PID_I_L 5.0f
#define CHASSIS_LEG_L0_SPEED_PID_D_L 5.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_L 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_L 60.0f

#define CHASSIS_LEG_L0_SPEED_PID_P_JH 15.0f//25 //15
#define CHASSIS_LEG_L0_SPEED_PID_I_JH 5.0f
#define CHASSIS_LEG_L0_SPEED_PID_D_JH 5.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_JH 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_JH 60.0f

#define CHASSIS_LEG_L0_SPEED_PID_P_JM 40.0f//25 //15
#define CHASSIS_LEG_L0_SPEED_PID_I_JM 5.0f
#define CHASSIS_LEG_L0_SPEED_PID_D_JM 5.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_JM 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_JM 60.0f

#define CHASSIS_LEG_L0_SPEED_PID_P_JL 35.0f
#define CHASSIS_LEG_L0_SPEED_PID_I_JL 5.0f
# define CHASSIS_LEG_L0_SPEED_PID_D_JL 5.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT_JL 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT_JL 60.0f


// ????????PID ???????
#define CHASSIS_OFFGROUND_LO_PID_P 0.0f
#define CHASSIS_OFFGROUND_L0_PID_I 0.0f
#define CHASSIS_OFFGROUND_L0_PID_D 0.0f
#define CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT 0.0f
#define CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT 0.0f

// Roll????PID pid_calc
// #define CHASSIS_ROLL_PID_P 500.0f
// #define CHASSIS_ROLL_PID_I 0.0f
// #define CHASSIS_ROLL_PID_D 0.0f
// #define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
// #define CHASSIS_ROLL_PID_OUT_LIMIT 50.0f

// #define CHASSIS_ROLL_PID_P 1500.0f
// #define CHASSIS_ROLL_PID_I 0.0f
// #define CHASSIS_ROLL_PID_D 0.0f
// #define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
// #define CHASSIS_ROLL_PID_OUT_LIMIT 50.0f

// Roll????PID ???+
#define CHASSIS_ROLL_PID_P 1300.0f//1500
#define CHASSIS_ROLL_PID_I 0.0f
#define CHASSIS_ROLL_PID_D 20.0f
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
#define CHASSIS_ROLL_PID_OUT_LIMIT 50.0f

/** ??????????????? **/
typedef struct{
    float wheel_radius; // ???????
    float body_weight; // ????????(?????????????)
    float wheel_weight; // ??????????(??????)
    float machine_limit_angle; // ??????????(??????)

    float l1, l2, l3, l4, l5; // ?????????
} ChassisPhysicalConfig;

/** ?????????? **/
typedef enum{
    CHASSIS_DISABLE = 1, // ?????
    CHASSIS_INIT, // ???????
    CHASSIS_ENABLE, // ?????
    CHASSIS_SPIN, // ß≥????
    CHASSIS_JUMP, // ?????
} ChassisCtrlMode;

/** ?????????? -- ??????????&????? **/
typedef enum{
    CHASSIS_BODY_UNNORMAL=0,
    CHASSIS_BODY_NORMAL,
} ChassisBodyState;

typedef enum{
    CHASSIS_FALL_LEG_UNNORMAL,
    CHASSIS_FALL_LEG_NORMAL,
} ChassisFallLegState;

typedef enum{
    CHASSIS_COULD_NOT_RECOVER,
    CHASSIS_COULD_RECOVER,
} ChassisRecoverState;

typedef enum{
    CHASSIS_Off_Ground=1,
    CHASSIS_On_Ground
} ChassisOffGroundState;

/** ?????? **/
typedef enum {
    SELFHELP_IDLE = 0,      // ????
    SELFHELP_PREPARE,       // ?????¶≤????????
    SELFHELP_LIFT_LEG,      // ?????
    SELFHELP_PUSH_GROUND,   // ?????
    SELFHELP_CHECK          // ?????
} SelfHelpState;

typedef struct{
    float v_m_per_s; // ???????
    float yaw_rad;
    float roll_rad;
    float height_m ; // ??????? testing
    float spin_speed;

} ChassisCtrlInfo;


/** ????? **/
typedef enum {
    NOT_READY = 0,      // ¶ƒ???
    STRETCHING,         // ?????¶≤????????????
    SHRINKING_IN_AIR,   // ??????¶≤????????????
    PREPARE_LANDING,    // ?????¶≤??????????
    RECOVERING,          // ?????¶≤?????????????
    JUMP_OVER
} JumpState;

/** ?????????? **/
typedef struct{
    // ?????
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float yaw_total_rad;


    //????????
    float pitch_gyro;
    float yaw_gyro;
    float roll_gyro;

    //????????
    float ax;
    float ay;
    float az;

    // ????????????????
    float robot_az;

} IMUReference;


/** ?????????? **/
typedef struct{
    float theta; // ??????1
    float theta_dot; // ??????2
    float theta_last;
    float theta_dot_last;
    float theta_ddot;

    float x; // ??????3
    float x_dot; // ??????4
    float x_dot_last;
    float x_ddot;

    float phi; // ??????5
    float phi_dot; // ??????6
} StateVariable;

/** VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC **/

/** ??????????  FK == Forward Kinematics(??????) **/
typedef struct{// ???
    float L0;
    float L0_last;
    float L0_dot;
    float L0_dot_last;
    float L0_ddot;
} FKL0;

typedef struct{// ???????ß÷???
    float phi1;
    float phi2;
    float phi3;
    float phi4;

    float phi0; // ????
    float last_phi0;
    float d_phi0;// ???Å£???
    float last_d_phi0;
    float dd_phi0;
} FKPhi;

typedef struct{// ???????ß÷??????(Coordinates)
    float a_x, a_y;
    float b_x, b_y;
    float c_x, c_y;
    float d_x, d_y;
    float e_x, e_y;
} FKPointCoordinates;

typedef struct{
    FKL0 fk_L0;
    FKPhi fk_phi;
    FKPointCoordinates fk_point_coordinates;
    float d_alpha; // ?

/** ???????????(Forward Dynamics)???? ?????(F Tp) ?? ????????(T1 T4) **/
    union { // ?????????????????: union
        float array[2][2];
        struct {
            float Tp_set_point;
            float Fy_set_point;
        } E;
    } Fxy_set_point;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_F_to_T;

    union {
        float array[2][1];
        struct {
            float T1_set_point;
            float T4_set_point;
        } E;
    } T1_T4_set_point;

} ForwardKinematics;


/** ?—ƒ???????(Inverse Dynamics): ?? ????????(T1 T4) ?? ?????(T Tp) **/
typedef struct {
    union {
        float array[2][1];
        struct {
            float T1_fdb;
            float T4_fdb;
        } E;
    } T1_T4_fdb;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_T_to_F;

    union {
        float array[2][1];
        struct {
            float Tp_fdb;
            float Fy_fdb;
        } E;
    } Fxy_fdb;

    /** ??????????(Inverse Dynamics): ?? ????????(w1 w4) ?? ??????(d_L0 d_phi0) **/
    union {
        float array[2][1];
        struct {
            float w1_fdb;// ???????????????????
            float w4_fdb;
        } E;
    } W_fdb;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_w_to_v;

    union {
        float array[2][1];
        struct {
            float d_L0_fdb; // ????Å£???
            float d_phi0_fdb; // ???(phi0)?Å£???

            float last_d_L0_fdb;
            float dd_L0_fdb;
        } E;
    } V_fdb;

}InverseKinematics;

/** ???VMC???? **/
typedef struct{
    ForwardKinematics forward_kinematics;
    InverseKinematics inverse_kinematics;
} VMC;
/*****************************************************************************/



/** ??????? **/
typedef struct{

    ChassisCtrlInfo chassis_ctrl_info;

    /** ?????? **/
    StateVariable state_variable_feedback;  // ??????????
    StateVariable state_variable_set_point; // ??????????
    StateVariable state_variable_error;     // ??? = ???? - ????
    StateVariable state_variable_wheel_out; // ?????????????lqr???????????????
    StateVariable state_variable_joint_out; // ?????????????lqr??????????????

    /** ???VMC **/
    VMC vmc;

    /** ???????PID **/
    Pid leg_pos_pid; // ???¶À???
    Pid leg_speed_pid; // ???????

    /** ????????PID **/
    Pid offground_leg_pid; // ????????pid  ????????????ó®???????

    float wheel_torque; // ???????
    float joint_F_torque; // ???????
    float joint_B_torque;

    /** ???????????? **/
    float Fn;

    /** ????? **/

    // theta_dot?????
    LowPassFilter theta_dot_lpf;


} Leg;

/** ??????? **/
typedef struct{

    /** ?????? **/
    IMUReference imu_reference;

    /** ???????? **/
    ChassisCtrlMode chassis_ctrl_mode;
    ChassisCtrlMode chassis_last_ctrl_mode;
    ChassisCtrlInfo chassis_ctrl_info;

    /** ??? **/
    Leg leg_L;
    Leg leg_R;


    /****** PID ******/
    /** Wheel **/

    // ???PID
    Pid chassis_turn_pos_pid;
    Pid chassis_turn_speed_pid;

    float wheel_turn_torque;          // ???????

    /** Joint **/

    // ??????PID
    float phi0_error;
    float last_phi0_error;
    float d_phi0_error;
    Pid chassis_leg_coordination_pid;
    float steer_compensatory_torque;  // ??????????

    // Roll????PID
    float roll_error;
    float last_roll_error;
    float d_roll_error;
    Pid chassis_roll_pid;
    float roll_f;


    // ??????
    ChassisBodyState chassis_body_state;

    ChassisFallLegState chassis_fall_leg_state;

    ChassisRecoverState chassis_recover_state;

    ChassisOffGroundState chassis_on_ground_state;

    // ???
    JumpState jump_state;

    // flag
    bool init_flag;            // ?????????????¶À
    bool chassis_recover_finish;
    bool jump_flag;

} Chassis;


extern float Kd;
extern float vel;

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;


/*******************************************************************************
 *                                  ??????                                    *
 *******************************************************************************/
extern Gimbal_Unpack_Data gimbal_unpack_data;

#endif