/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */

#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

#include "custom_drv.h"
#include "mpu6500.h"
#include "stdint.h"

// #define VISION_USE_VCP  // 使用虚拟串口发送�?��?�数�?
// #define VISION_USE_UART  // 使用串口发送�?��?�数�?

/* 机器人重要参数定�?,注意根据不同机器人进行修�?,�?点数需要以.0或f结尾,无�?�号�?u结尾
 */

// 机器人底盘修改的参数,单位为mm(�?�?)
#define WHEEL_BASE  284 // �����־�(ǰ�����˷���)
#define TRACK_WIDTH 284 // �����־�(����ƽ�Ʒ���)
#define CENTER_GIMBAL_OFFSET_X                                                                     \
    0 // 云台旋转�?心距底盘几何�?心的距�??,前后方向,云台位于正中心时默�?��?�为0
#define CENTER_GIMBAL_OFFSET_Y                                                                     \
    0 // 云台旋转�?心距底盘几何�?心的距�??,左右方向,云台位于正中心时默�?��?�为0
#define RADIUS_WHEEL 5 // �?子半�?
#define REDUCTION_RATIO_WHEEL                                                                      \
    19.0f // 电机减速比,因为编码器量测的�?�?子的速度而不�?输出轴的速度故需进�?�转�?

// ��ȥȷ��
#define GYRO2GIMBAL_DIR_YAW                                                                        \
    1           // 陀螺仪数据相较于云台的yaw的方�?,1为相�?,-1为相�?
#define GYRO2GIMBAL_DIR_PITCH                                                                      \
    1           // 陀螺仪数据相较于云台的pitch的方�?,1为相�?,-1为相�?
#define GYRO2GIMBAL_DIR_ROLL                                                                       \
    1           // 陀螺仪数据相较于云台的roll的方�?,1为相�?,-1为相�?

#pragma pack(1) // 压缩结构�?,取消字节对齐,下面的数�?都可能�??传输

/* -------------------------基本控制模式和数�?类型定义-------------------------*/
/**
 * @brief
 * 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数�?的一部分
 *
 */
// 机器人状�?
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
    ROBOT_AUTO,
} Robot_Status_e;

// 应用状�?
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief
 * 后续考虑�?改为云台跟随底盘,而不�?让底盘去追云�?,云台的惯量比底盘�?.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输�?
    CHASSIS_NO_FOLLOW,         // 不跟随，允�?�全向平�?
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加�?�度�?控制
    CHASSIS_SPIN,

} chassis_mode_e;
typedef enum
{
    GIMBAL_ZERO_FORCE = 0,    // 电流零输�?
    GIMBAL_NO_FOLLOW,         // 不跟随，允�?�全向平�?
    GIMBAL_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加�?�度�?控制
    GIMBAL_FULL_AUTO,

} gimbal_mode_e;

// ��̨ģʽ����
typedef enum
{
    ROBOTARM_ZERO_FORCE = 0, // 电流零输�?
    ROBOTARM_TEACH,          // 示教模式
    ROBOTARM_CUSTOM,         // �?定义控制器模�?
    ROBOTARM_CALI,
} robotarm_mode_e;

// 臂上电机名字
typedef enum
{
    YAW,
    JOINT1,
    JOINT2,
    JOINT3,
    EE1,
    EE2,
} armmotor_name_e;

// 关节校准状态机
typedef enum
{
    JOINT_CALI_INT,
    JOINT_CALI_ING,
    JOINT_CALI_END,
    JOINT_READY,

} joint_cali_state_e;

typedef enum
{
    ENEMY_RAD  = 0,
    ENEMY_BLUE = 1,

} Enemy_Color_e;

typedef enum
{
    OFFLINE = 0,
    ONLINE  = 1,
} online_state_e;

typedef enum
{

    yaw_unitree = 0,
    joint1,
    joint2,
    joint3,
    ee_pitch,
    ee_roll,

} Custom_name_e;

// 校准变量
typedef struct {
    uint8_t cali_status;
    joint_cali_state_e joint_cali_state;
    float cali_yaw;

    float cali_unitree_angle;
    float cali_joint1_angle;
    float cali_joint2_angle;
    float cali_4310_angle;
    float cali_ee1_angle;
    float cali_ee2_angle;

} cali_data_t;

typedef struct {
    float custom_angle[6];      // 解算后的角度
    float custom_init_angle[5]; // 初�?��?�度
    uint8_t custom_ch[6];

} Custom_ctrl;

/* ----------------CMD应用发布的控制数�?,应当由gimbal/chassis/shoot订阅----------------
 */
/**
 * @brief 对于双板情况,遥控器和pc在云�?,裁判系统在底�?
 *
 */
// cmd发布的底盘控制数�?,由chassis订阅
typedef struct {
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // �?移方向速度
    float wz;           // 旋转速度
    float yaw;
    float offset_angle; // 底盘和归�?位置的夹�?
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...
    robotarm_mode_e robotarm_mode; // 机�?�臂状�?

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数�?,由gimbal订阅
typedef struct {
    INS_t *imu_data;
    float pitch;
    float yaw;

    float yaw_total_angle_current;
    float yaw_gyro_current;
    float pitch_angle_current;
    gimbal_mode_e gimbal_mode;

} Robotarm_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数�?----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根�?需要获�?.
 *
 */

typedef struct {

    // 后续增加底盘的真实速度
    //     float real_vx;
    //     float real_vy;
    //     float real_wz;

    INS_t *imu_data;
    Enemy_Color_e enemy_color; // 0 for blue, 1 for red

} Chassis_Upload_Data_s;

typedef struct {
    // 关节电机数据

    float yaw_relative_angle;
    float *yaw_total_angle;

} Robotarm_Upload_Data_s;

// 视�?�数�?
//  typedef struct
//  {
//      float aim_pitch;
//  	float aim_yaw;

// } Vision_Upload_Data_s;

#pragma pack() // 开�?字节对齐,结束前面�?#pragma pack(1)

#endif         /* __ROBOT_DEF_H */
