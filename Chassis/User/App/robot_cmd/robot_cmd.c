/**
  ****************************(C) COPYRIGHT 2022 YUELU*********************************
  * @file      robot_cmd.c\h
  * @brief     robot_cmd是整个机器人的核心应用，其负责接受遥控器/上位机发来的指令，并将指
  *            令转化为实际的运动控制目标，发送给其他三个应用。后者会根据robot_cmd发来的命
  *            令，设定电机和其他执行单元的参考值等。(相当于集成了旧代码gimbal_behaviour和
  *             chassis_behaviour部分，且具有防掉线功能）
  *
  * @todo      1.完善自定义控制器模式
  *            2.完善遥控器模式下的录制模式选择还有校准部分设计
  *            3.编写自定义控制器掉线的相关应对措施
               4.底盘message功能需要修改
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0                     YUELU              1. 完成
  *  V2.0.0                      qhp               1.适配对应机器人
  *  V2.1.0                     nan_97             1.完善自定义控制器模式，适配工程
  *
  @verbatim
  ===================================================================================

  ===================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 YUELU*******************************
  */

#include "stm32f427xx.h"

#include "arm_math.h"
#include "core_cm4.h"

// app
#include "calibrate.h"
#include "robot_cmd.h"
#include "robot_def.h"

// #include "vision.h"

// module
#include "airpump.h"
#include "buzzer.h"
#include "dji_motor.h"
#include "message_center.h"
#include "mpu6500.h"
#include "remote_control.h"
#include "user_lib.h"

// #include "general_def.h"
// #include "unitree.h"

// bsp
#include "bsp_dwt.h"
// #include "bsp_log.h"
#include "bsp_gpio.h"
#include "bsp_usb.h"
#include "user_lib.h"

// 私有宏,自动将编码器转换成角度值
// #define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
// #define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) //
// pitch水平时电机的角度,0-360

// 这两个宏待定
#define YAW_ALIGN_ANGLE     0
#define PTICH_HORIZON_ANGLE 0
#define PITCH_MAX           15
#define PITCH_MIN           -8

RC_ctrl_t *rc_data;                                       // 遥控器数据,初始化时返回
INS_t *ins_data;                                          // IMU数据,初始化时返回
cali_data_t *calibrate_data;
AirPumpInstance *ee_airpump, left_airpump, right_airpump; // 气泵实例注册

static Publisher_t *chassis_cmd_pub;                      // 底盘控制消息发布者
static Publisher_t *chassis_feed_pub;
static Subscriber_t *chassis_feed_sub;                    // 底盘反馈信息订阅者
Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
Chassis_Upload_Data_s
    chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Publisher_t *robotarm_cmd_pub;       // 云台控制消息发布者
static Subscriber_t *robotarm_feed_sub;     // 云台反馈信息订阅者
Robotarm_Ctrl_Cmd_s robotarm_cmd_send;      // 传递给云台的控制信息
Robotarm_Upload_Data_s robotarm_fetch_data; // 从云台获取的反馈信息
USB_Rx_t *Vision_message;                   // 从虚拟串口读取的数据

float abslute_vx, abslute_vy;

// static Subscriber_t *vision_feed_sub;         // 视觉反馈信息订阅者
// Vision_Upload_Data_s vision_fetch_data; // 从视觉获取的反馈信息

Robot_Status_e robot_state; // 机器人整体工作状态

// static float PitchLimit(float origin) {
//     if (robotarm_cmd_send.gimbal_mode != GIMBAL_ZERO_FORCE) {
//         if (robotarm_cmd_send.imu_data->Pitch <= PITCH_MAX &&
//             robotarm_cmd_send.imu_data->Pitch >= PITCH_MIN) {
//             return origin;
//         }
//         else if (origin * (robotarm_cmd_send.imu_data->Pitch - PITCH_MAX) > 0 ||
//                  origin * (robotarm_cmd_send.imu_data->Pitch - PITCH_MIN) > 0) {
//             return origin;
//         }
//     }
//     return 0;
// }

/**
 * @brief  计算偏移角度
 *
 * @todo  后续需要针对臂进行优化，考虑是否改成陀螺仪反馈数据进行计算
 *
 */

static void CalcOffsetAngle() {
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = robotarm_fetch_data.yaw_relative_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                       // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = (angle - YAW_ALIGN_ANGLE);
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - PI)
        chassis_cmd_send.offset_angle = (angle - YAW_ALIGN_ANGLE);
    else
        chassis_cmd_send.offset_angle = (angle - YAW_ALIGN_ANGLE + 2 * PI);
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 * @todo  关节校准设置需要写入，还有录制模式，以及云台参数。这一部分要根据录制模式进行优化，
 *        同时要注意代码的简洁性，不能直接移植。底盘系数需要更改，300太大了
 */
uint8_t debug_mode1;
uint8_t debug_mode2;
static void RemoteControlSet(void) {
    static uint8_t last_mode         = GIMBAL_ZERO_FORCE;
    static uint8_t last_chassis_mode = CHASSIS_ZERO_FORCE;
    static float yaw_offset          = 0;
    last_mode                        = robotarm_cmd_send.gimbal_mode;
    last_chassis_mode                = chassis_cmd_send.chassis_mode;
    uint8_t current_mode;
    debug_mode2 = current_mode;
    if (robot_state != ROBOT_AUTO) {
        robot_state = ROBOT_READY;
    }

    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        robotarm_cmd_send.gimbal_mode = GIMBAL_FOLLOW_GIMBAL_YAW;

        if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
            robot_state = ROBOT_AUTO;
        }
        else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
            chassis_cmd_send.chassis_mode = CHASSIS_SPIN;
        }
        else {
            robot_state                   = ROBOT_READY;
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        }
    }
    else if (switch_is_mid(
                 rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        robotarm_cmd_send.gimbal_mode = GIMBAL_NO_FOLLOW;
    }

    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],无力
    {
        robot_state                   = ROBOT_STOP;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        robotarm_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    }

    if (current_mode != GIMBAL_ZERO_FORCE && last_mode == GIMBAL_ZERO_FORCE) {
        current_mode = GIMBAL_CALI;
    }

    // robotarm_cmd_send.robotarm_mode = current_mode;

    // 遥控器转底盘参数
    if (chassis_cmd_send.chassis_mode == CHASSIS_SPIN) {
        if (last_chassis_mode != CHASSIS_SPIN)
            yaw_offset = ins_data->Yaw;

        abslute_vx = 6.0f * (float)rc_data->rc.rocker_r_; // _水平方向
        abslute_vy = 6.0f * (float)rc_data->rc.rocker_r1; // 1数值方向

        float sin_theta     = arm_sin_f32(RadFormat((yaw_offset - ins_data->Yaw) * DEGREE_2_RAD));
        float cos_theta     = arm_cos_f32(RadFormat((yaw_offset - ins_data->Yaw) * DEGREE_2_RAD));
        chassis_cmd_send.vx = abslute_vx * cos_theta - abslute_vy * sin_theta;
        chassis_cmd_send.vy = abslute_vx * sin_theta + abslute_vy * cos_theta;
        chassis_cmd_send.wz = 0.05f * (float)rc_data->rc.rocker_l_;
    }
    else {
        chassis_cmd_send.vx = 20.0f * (float)rc_data->rc.rocker_r_; // _水平方向
        chassis_cmd_send.vy = 20.0f * (float)rc_data->rc.rocker_r1; // 1数值方向
        chassis_cmd_send.wz = 5.0f * (float)rc_data->rc.rocker_l_;
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置（目前只保留底盘控制）
 *
 * @todo  需要和自定义控制器配合，上赛场时候再考虑吧
 */
// static void MouseKeySet(void)
// {
//     chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].w * 15000 -
//     rc_data[TEMP].key[KEY_PRESS].s
//     * 15000; // 系数待测 chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].d * 15000 -
//     rc_data[TEMP].key[KEY_PRESS].a * 15000;

// 	rc_data[TEMP].mouse.x *= -1;
// 	rc_data[TEMP].mouse.y *= -1;

// }

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *         这一部分适配需要再进行考虑，考虑加入自定义控制器掉线的回复模式
 */
static void EmergencyHandler(void) {
    // 遥控器掉线无力
    if (rc_data->rc_state == 0) {
        robot_state = ROBOT_STOP;
    }
    if (robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        //        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        // 补充气泵相关模式
        // 气泵关闭
        // LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
        //      LOGINFO("[CMD] reinstate, robot ready");
    }
}

/**
 * @brief  机器人重要模块初始化
 *
 * @todo  添加对应的外设，和必要文件
 */
void RobotCmdInit(void) {
    ins_data = GetInsData();

    rc_data = RemoteControlInit(&huart1);

    robotarm_cmd_pub  = PubRegister("robotarm_cmd", sizeof(Robotarm_Ctrl_Cmd_s));
    robotarm_feed_sub = SubRegister("robotarm_feed", sizeof(Robotarm_Upload_Data_s));
    chassis_cmd_pub   = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub  = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    chassis_feed_pub           = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    robotarm_cmd_send.imu_data = ins_data;
    Vision_message             = USB_RxBufferRegister();
    // robotarm_cmd_send.customdate = customdate;

    //	gimbal_cmd_send.pitch = 0;		作用不明，保留
    robot_state = ROBOT_READY;
}

/**
 * @brief  机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 *
 * @todo  待修改控制模式选择
 */
void RobotCMDTask(void) {
    // 从其他应用获取回传数据
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
    SubGetMessage(robotarm_feed_sub, &robotarm_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    // 待修改，可能这里放校准？或者其他
    CalcOffsetAngle();

    RemoteControlSet();
    CalibrateTask(rc_data);
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
    chassis_fetch_data.pitch     = ins_data->Pitch;
    chassis_fetch_data.yaw       = ins_data->Yaw;
    chassis_fetch_data.roll      = ins_data->Roll;
    chassis_fetch_data.yaw_total = ins_data->YawTotalAngle;

    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
    PubPushMessage(chassis_feed_pub, (void *)&chassis_fetch_data);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(robotarm_cmd_pub, (void *)&robotarm_cmd_send);
}
