#include "gimbal_task.h"
#include "launcher.h"
#include "../Vision_Task/RoboMaster_Protocol.h"
#include "ins_task.h"
#include "packet.h"
#include "cmsis_os.h"
#include "board_communication_task.h"


vision_t vision_data;                   // 给视觉传信息
extern robot_ctrl_info_t robot_ctrl;    // 获取视觉信息
//todo: 图传的
extern uint8_t control_flag;        // 通过状态判断是什么链路

/** 云台PID初始化 **/
static void Gimbal_Pid_Init(void)
{
    // Pitch
    pid_init(&gimbal.pitch.speed_pid,
             GIMBAL_PITCH_SPEED_MAX_OUT,
             GIMBAL_PITCH_SPEED_MAX_IOUT,
             GIMBAL_PITCH_SPEED_PID_KP,
             GIMBAL_PITCH_SPEED_PID_KI,
             GIMBAL_PITCH_SPEED_PID_KD);

    pid_init(&gimbal.pitch.angle_pid,
             GIMBAL_PITCH_ANGLE_MAX_OUT,
             GIMBAL_PITCH_ANGLE_MAX_IOUT,
             GIMBAL_PITCH_ANGLE_PID_KP,
             GIMBAL_PITCH_ANGLE_PID_KI,
             GIMBAL_PITCH_ANGLE_PID_KD);

    // Yaw
    pid_init(&gimbal.yaw.speed_pid,
             GIMBAL_YAW_SPEED_MAX_OUT,
             GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP,
             GIMBAL_YAW_SPEED_PID_KI,
             GIMBAL_YAW_SPEED_PID_KD);

    pid_init(&gimbal.yaw.angle_pid,
             GIMBAL_YAW_ANGLE_MAX_OUT,
             GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP,
             GIMBAL_YAW_ANGLE_PID_KI,
             GIMBAL_YAW_ANGLE_PID_KD);

}

/** 云台初始化 **/
static void Gimbal_Init(void) {

    gimbal.pitch.target_current = 0;
    gimbal.yaw.target_current = 0;

    /** 初始化云台模式 **/
    gimbal.gimbal_ctrl_mode = gimbal.gimbal_last_ctrl_mode = GIMBAL_DISABLE;

    /** 云台PID初始化 **/
    Gimbal_Pid_Init();

    /** 确定Yaw、Pitch上电复位的电机编码值 **/
    gimbal.yaw.motor_measure.offset_ecd = YAW_OFFSET_ECD;
    gimbal.pitch.motor_measure.offset_ecd = PITCH_OFFSET_ECD;

    /** 低通滤波初始化 **/
    // 鼠标输入滤波
    first_order_filter_init(&gimbal.mouse_in_x, 1, 40);
    first_order_filter_init(&gimbal.mouse_in_y, 1, 10);

    // 角速度滤波
    first_order_filter_init(&gimbal.pitch_gyro_filter, 1, 20);
    first_order_filter_init(&gimbal.yaw_gyro_filter, 5, 30);

    // 对视觉传回的yaw和pitch滤波
    first_order_filter_init(&gimbal.auto_pitch, 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[0], 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[1], 1, 15);

    /** 发射机构初始化 **/
    Launcher_Init();

}


/** 更新云台角度 **/
static void Gimbal_Angle_Update(void)
{
    /** Pitch **/
    // IMU角度
    gimbal.pitch.absolute_angle_get = INS_angle[2] * MOTOR_RAD_TO_ANGLE;
    // 与上电复位位置的角度差
    gimbal.pitch.relative_angle_get = Motor_Ecd_To_Angle_Change(gimbal.pitch.motor_measure.ecd,
                                                                gimbal.pitch.motor_measure.offset_ecd);
    // 角速度
    gimbal.pitch.gyro = -INS_gyro[0] * MOTOR_RAD_TO_ANGLE;

    /** Yaw **/
    // IMU角度
    gimbal.yaw.absolute_angle_get = INS_angle[0] * MOTOR_RAD_TO_ANGLE;
    // 与上电复位位置的角度差
    gimbal.yaw.relative_angle_get = Motor_Ecd_To_Angle_Change(gimbal.yaw.motor_measure.ecd,
                                                             gimbal.yaw.motor_measure.offset_ecd);
    // 角速度
    gimbal.yaw.gyro = INS_gyro[2] * MOTOR_RAD_TO_ANGLE;

}

// 因为这个任务是在1kHz的云台当中运行的，我希望它以500Hz运行
static void Gimbal_Send_Chassis_Data(void) {

    static int count = 1;

    if(count % 2 == 1) // 奇数发，偶数不发，实现500Hz
    {
        Send_Chassis_Data(rc_ctrl.rc.ch[CHASSIS_VX_CHANNEL],
                          rc_ctrl.rc.ch[CHASSIS_LEG_CHANNEL],
                          rc_ctrl.rc.s[RC_s_R],
                          gimbal.yaw.relative_angle_get);
    }

    count ++;

}

/** 云台控制器计算 **/
static void Gimbal_Control(void)
{
    float yaw_gyro_set;
    float pitch_gyro_set;

    /** Yaw **/
    yaw_gyro_set = pid_loop_calc(&gimbal.yaw.angle_pid,
                            gimbal.yaw.absolute_angle_get,
                            gimbal.yaw.absolute_angle_set,
                            180,
                            -180);

    // 反馈角速度滤波
    first_order_filter_cali(&gimbal.yaw_gyro_filter, gimbal.yaw.gyro);

    gimbal.yaw.target_current = (int16_t)pid_calc(&gimbal.yaw.speed_pid,
                                                  gimbal.yaw_gyro_filter.out,
                                                  yaw_gyro_set);

    /** Pitch **/
    pitch_gyro_set = -pid_calc(&gimbal.pitch.angle_pid,
                        gimbal.pitch.absolute_angle_get,
                        gimbal.pitch.absolute_angle_set);

    // 反馈角速度滤波
    first_order_filter_cali(&gimbal.pitch_gyro_filter,gimbal.pitch.gyro);

    gimbal.pitch.target_current = -(int16_t)pid_calc(&gimbal.pitch.speed_pid,
                                                    gimbal.pitch_gyro_filter.out,
                                                     pitch_gyro_set);
}

/*************************************************************************************************
 *                                          Vision                                               *
 *************************************************************************************************/

// 单片机向小电脑发送数据
static void Robot_Send_Vision_Data(void)
{
    // 107:蓝 7:红
    if (Referee.GameRobotStat.robot_id < 10)
    {
        vision_data.id = 107;
    }
    else
    {
        vision_data.id = 7;
    }


    /* 给视觉发开自瞄 */
    if (gimbal.gimbal_ctrl_mode == GIMBAL_AUTO)
    {
        vision_data.mode = 0x21;
    }
    else
    {
        vision_data.mode = 0;
    }

    vision_data.pitch = gimbal.pitch.absolute_angle_get;
    vision_data.yaw   = gimbal.yaw.absolute_angle_get;

    // 发送四元数，用于视觉建立坐标系
    for (int i = 0; i < 4; ++i)
    {
        vision_data.quaternion[i] = INS_quat[i];
    }

    // 当前射速
    vision_data.shoot_speed = Referee.ShootData.bullet_speed;

    rm_queue_data(VISION_ID, &vision_data, sizeof(vision_t));
}

// 接收视觉传输的数据
static void Gimbal_Auto_Handle(void)
{
    if(gimbal.gimbal_ctrl_mode == GIMBAL_AUTO)
    {
        // 对视觉传来的角度进行滤波
        first_order_filter_cali(&gimbal.auto_pitch, robot_ctrl.pitch);
        first_order_filter_cali(&gimbal.auto_yaw[0], sinf(robot_ctrl.yaw / 180.0f * PI)); //yaw数据分解成x
        first_order_filter_cali(&gimbal.auto_yaw[1], cosf(robot_ctrl.yaw / 180.0f * PI)); //yaw数据分解成y

        gimbal.yaw.absolute_angle_set = atan2f(gimbal.auto_yaw[0].out, gimbal.auto_yaw[1].out) * 180.0f / PI; // 合成

        gimbal.pitch.absolute_angle_set = gimbal.auto_pitch.out;
    }
}

/*************************************************************************************************
 *                                           Task                                                *
 *************************************************************************************************/

static void Gimbal_Disable_Task(void)
{
    /** 云台电流置零 **/
    gimbal.pitch.target_current = 0;
    gimbal.yaw.target_current = 0;

    /** 初始化云台模式 **/
    gimbal.gimbal_ctrl_mode = gimbal.gimbal_last_ctrl_mode = GIMBAL_DISABLE;

    /** 避免PID误差过大，电机疯转 **/
    gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_get;
    gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;

    /** 发射机构失能子任务 **/
    Launcher_Disable();

}

static void Gimbal_Enable_Task(void)
{
    /** Pitch、Yaw控制 **/
    Gimbal_Control();

    /** 发射机构控制 **/
    Launcher_Control();
}

void Gimbal_task(void const*pvParameters) {
    
    /* 任务初始化时间 */
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    /** 云台初始化 **/
    Gimbal_Init();

    while(1) {

        // 根据遥控器设置云台、发射机构的模式、控制信息
        Gimbal_Remote_Cmd();

        // 更新云台角度
        Gimbal_Angle_Update();

        // 板间通信
        Gimbal_Send_Chassis_Data();

        // 单片机向视觉发送数据
        Robot_Send_Vision_Data();

        switch(gimbal.gimbal_ctrl_mode)
        {
            case GIMBAL_DISABLE:
            {
                Gimbal_Disable_Task();

                break;
            }

            case GIMBAL_ENABLE:
            {
                Gimbal_Enable_Task();
                break;
            }

            case GIMBAL_AUTO:
            {
                Gimbal_Auto_Handle();
                Gimbal_Enable_Task();
                break;
            }

            default:
            {
                break;
            }
        }

        // 发射机构
        DJI_Send_Motor_Mapping(CAN_1,
                               CAN_DJI_MOTOR_0x200_ID,
                               launcher.fire_l.target_current,    //201 左摩擦轮
                               launcher.fire_r.target_current,    //202 右摩擦轮
                               launcher.trigger.target_current,    //203 拨盘
                               0     // 204 无
        );

        // 云台
        DJI_Send_Motor_Mapping(CAN_1,
                               CAN_DJI_MOTOR_0x1FF_ID,
                               gimbal.yaw.target_current,     //205 无
                               gimbal.pitch.target_current,     //206 pitch
                               0,     //207 无
                               0      //208 无
        );

//        // 发射机构
//        DJI_Send_Motor_Mapping(CAN_1,
//                               CAN_DJI_MOTOR_0x200_ID,
//                               0,    //201 左摩擦轮
//                               0,    //202 右摩擦轮
//                               0,    //203 拨盘
//                               0     // 204 无
//        );
//
//        // 云台
//        DJI_Send_Motor_Mapping(CAN_1,
//                               CAN_DJI_MOTOR_0x1FF_ID,
//                               0,     //205 无
//                               0,     //206 pitch
//                               0,     //207 无
//                               0      //208 无
//        );





        vTaskDelay(GIMBAL_PERIOD);
    }
}
