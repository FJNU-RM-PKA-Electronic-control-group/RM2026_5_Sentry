// app
#include "calibrate.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "vision.h"

// module
#include "INS_task.h"
#include "dji_motor.h"
#include "fliter.h"
#include "message_center.h"
#include "remote_control.h"
#include "user_lib.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "core_cm4.h"

// 私有宏,自动将编码器转换成角度值
// #define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
// #define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) //
// pitch水平时电机的角度,0-360
#define USE_VISION_FIRE_CONTROL

#define YAW_ALIGN_ANGLE     0
#define PTICH_HORIZON_ANGLE 0

RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
attitude_t *ins_data;
cali_data_t *calibrate_data;

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
Chassis_Ctrl_Cmd_s chassis_cmd_send;   // 发送给底盘应用的信息,包括控制信息和UI绘制相关
Chassis_Upload_Data_s
    chassis_fetch_data;             // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Publisher_t *gimbal_cmd_pub; // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;       // 云台反馈信息订阅者
Gimbal_Ctrl_Cmd_s gimbal_cmd_send;          // 传递给云台的控制信息
Gimbal_Upload_Data_s gimbal_fetch_data;     // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;          // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;        // 发射反馈信息订阅者
Shoot_Ctrl_Cmd_s shoot_cmd_send;            // 传递给发射的控制信息
Shoot_Upload_Data_s shoot_fetch_data;       // 从发射获取的反馈信息

static Subscriber_t *vision_feed_sub;       // 视觉反馈信息订阅者
static Subscriber_t *guidence_feed_sub;
Vision_Upload_Data_s vision_fetch_data;     // 从视觉获取的反馈信息
Guidence_Upload_Data_s guidence_fetch_data; // 导航

static Robot_Status_e robot_state;          // 机器人整体工作状态
LPFliterInstance yaw_offset_fliter;         // 底盘防翻滤波

static void CalcOffsetAngle() {
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw1_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                                // 如果大于180度
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

static float PitchLimit(float add) {
    if (gimbal_cmd_send.gimbal_mode != GIMBAL_ZERO_FORCE) {
        if (gimbal_fetch_data.pitch_ecd_angle <= (float)PITCH_MAX_ECD &&
            gimbal_fetch_data.pitch_ecd_angle >= (float)PITCH_MIN_ECD) {
            return add;
        }
        else if (add * (gimbal_fetch_data.pitch_ecd_angle - (float)PITCH_MAX_ECD) < 0 ||
                 add * (gimbal_fetch_data.pitch_ecd_angle - (float)PITCH_MIN_ECD) < 0) {
            return add;
        }
    }
    return 0;
}

void VisionControlSet(void) {
    // if (vision_fetch_data.fire_advice == SHOOT_STOP)
    //     shoot_cmd_send.shoot_mode = SHOOT_STOP;
    chassis_cmd_send.vx = guidence_fetch_data.vx;
    chassis_cmd_send.vy = guidence_fetch_data.vy;
    chassis_cmd_send.wz = guidence_fetch_data.wz;
    if (vision_fetch_data.vision_state == NO_TARGET) {
        // gimbal_cmd_send.gimbal_mode = GIMBAL_SCAN_MODE;
        return;
    }
    else {
        // gimbal_cmd_send.gimbal_mode = GIMBAL_VISION_MODE;
        gimbal_cmd_send.pitch = vision_fetch_data.aim_pitch;
        gimbal_cmd_send.yaw   = vision_fetch_data.aim_yaw;
    }
    // #ifdef USE_VISION_FIRE_CONTROL

    // #endif
}

static void RemoteControlSet(void) {
    static gimbal_mode_e last_gimbal_mode = GIMBAL_ZERO_FORCE;
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(
                 rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_FREE_MODE;
    }
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],无力
    {
        robot_state = ROBOT_STOP;
    }
    else {
        robot_state = ROBOT_READY;
    }

    if (switch_is_down(rc_data[TEMP].rc.switch_left) &&
        switch_is_down(rc_data[TEMP].rc.switch_right)) // 俩侧开关拨下,检测校准条件
    {
        CalibrateTask(rc_data);
    }

    // 云台参数,确定云台控制数据
    gimbal_cmd_send.pitch += PitchLimit(0.00001f * (float)rc_data[TEMP].rc.rocker_l1);
    gimbal_cmd_send.yaw =
        theta_format(gimbal_cmd_send.yaw - 0.001f * (float)rc_data[TEMP].rc.rocker_l_);
    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 20.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
    chassis_cmd_send.vy = 20.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1数值方向
    if (last_gimbal_mode != gimbal_cmd_send.gimbal_mode) {
        switch (gimbal_cmd_send.gimbal_mode) {
        case GIMBAL_FREE_MODE:
            gimbal_cmd_send.yaw   = ins_data->Yaw * RAD_2_DEGREE;
            gimbal_cmd_send.pitch = ins_data->Pitch;

            break;
        case GIMBAL_GYRO_MODE:
            gimbal_cmd_send.yaw   = ins_data->Yaw * RAD_2_DEGREE;
            gimbal_cmd_send.pitch = ins_data->Pitch;

        default:
            break;
        }
    }
    last_gimbal_mode = gimbal_cmd_send.gimbal_mode;

    // 发射参数
    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (rc_data[TEMP].rc.dial < -100 || rc_data[TEMP].rc.dial > 100) // 向上(下)超过100,打开摩擦轮
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
    }
    else {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    }
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < -500) {
        shoot_cmd_send.bullet_speed = TEST;
        shoot_cmd_send.load_mode    = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_mode   = SHOOT_ON;
    }
    else if (rc_data[TEMP].rc.dial > 500) {
        shoot_cmd_send.load_mode  = LOAD_REVERSE;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    else {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[下],视觉模式
    {
        VisionControlSet();
    }
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 2;
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet(void) {
    chassis_cmd_send.vy =
        rc_data[TEMP].key[KEY_PRESS].w * 15000 - rc_data[TEMP].key[KEY_PRESS].s * 15000; // 系数待测
    chassis_cmd_send.vx =
        rc_data[TEMP].key[KEY_PRESS].d * 15000 - rc_data[TEMP].key[KEY_PRESS].a * 15000;

    rc_data[TEMP].mouse.x *= -1;
    rc_data[TEMP].mouse.y *= -1;
    gimbal_cmd_send.yaw =
        RadFormat(gimbal_cmd_send.yaw + (float)rc_data[TEMP].mouse.x / 10000); // 系数待测
    gimbal_cmd_send.pitch += PitchLimit((float)rc_data[TEMP].mouse.y / 10000);

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = TEST;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = TEST;
        break;
    default:
        shoot_cmd_send.bullet_speed = TEST;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 3) // V键切换云台模式
    {
    case 0:
        robot_state = ROBOT_STOP;
        break;
    case 1:
        robot_state                   = ROBOT_READY;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_FREE_MODE;
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        shoot_cmd_send.shoot_mode     = SHOOT_ON;
        break;
    default:
        robot_state                   = ROBOT_READY;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        shoot_cmd_send.shoot_mode     = SHOOT_ON;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].ctrl) // ctrl小陀螺
    {
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;

    default:

        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:
        //        chassis_cmd_send.chassis_mode = CHASSIS_MAXSPEED;
        break;

    default:

        break;
    }
    switch (rc_data[TEMP].mouse.press_l) {
    case 1:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    }
    switch (rc_data[TEMP].mouse.press_r) {
    case 1:
        VisionControlSet();
        break;
    default:
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 4) // B键reset
    {
    case 3:
        __disable_irq();
        NVIC_SystemReset();
        break;
    default:
        break;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler(void) {
    // 遥控器掉线无力
    if (rc_data->rc_state == 0) {
        robot_state = ROBOT_STOP;
    }
    if (robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode     = SHOOT_OFF;
        shoot_cmd_send.friction_mode  = FRICTION_OFF;
        shoot_cmd_send.load_mode      = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
        robot_state               = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

void RobotCmdInit(void) {
    rc_data        = RemoteControlInit(&huart3);
    ins_data       = GetInsData();
    calibrate_data = CalibrateInit();

    gimbal_cmd_pub    = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub   = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub     = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub    = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    chassis_cmd_pub   = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub  = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    vision_feed_sub   = SubRegister("vision_feed", sizeof(Vision_Upload_Data_s));
    guidence_feed_sub = SubRegister("guidence_feed", sizeof(Guidence_Upload_Data_s));

    Low_Pass_Fliter_Init_Config_s offset_lp_fliter = {
        .RC = 0.0f,
    };

    LowPassFliterInit(&yaw_offset_fliter, &offset_lp_fliter);

    //	gimbal_cmd_send.pitch = 0;		作用不明，保留
    robot_state = ROBOT_READY;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void) {
    // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(vision_feed_sub, &vision_fetch_data);
    SubGetMessage(guidence_feed_sub, &guidence_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    if (switch_is_up(rc_data[TEMP].rc.switch_left) ||
        rc_data->rc_state == 2) // 左侧开关状态为[上],待添加
    {
        MouseKeySet();
    }
    else {
        RemoteControlSet();
    }
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

                        // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionTask(RadToDeg(ins_data->Roll), -RadToDeg(ins_data->Pitch), RadToDeg(ins_data->Yaw));

    // 推送消息,双板通信,视觉通信等
    chassis_cmd_send.offset_angle = theta_format(FliterCalcutale(
        (FliterInstance *)&yaw_offset_fliter, gimbal_fetch_data.yaw1_motor_single_round_angle));

    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
