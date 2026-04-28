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

#include "mpu6500.h"
#include "stdbool.h"
#include "stdint.h"

// #define VISION_USE_VCP  // 使用虚拟串口发送�?��?�数�?
// #define VISION_USE_UART  // 使用串口发送�?��?�数�?

/* 机器人重要参数定�?,注意根据不同机器人进行修�?,�?点数需要以.0或f结尾,无�?�号�?u结尾
 */

// 下列参数为需要修改的参数,单位为mm(毫米)
#define WHEEL_BASE  360.0f // 轮距基准(前后轮轴距离)
#define TRACK_WIDTH 360.0f // 轮距基准(左右平行轮距离)
#define CENTER_GIMBAL_OFFSET_X                                                                     \
    0                      // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认该值为0
#define CENTER_GIMBAL_OFFSET_Y                                                                     \
    0                      // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认该值为0
#define RADIUS_WHEEL 58    // 轮子半径
#define REDUCTION_RATIO_WHEEL                                                                      \
    19.0f                  // 电机减速比,因为编码器量测的是电机的速度而不是输出轴的速度故需进行转换

// 陀螺仪方向
#define GYRO2GIMBAL_DIR_YAW   1 // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL  1 // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

#define SQRT2 1.4142135623730950488016887242097
#define SIN45 SQRT2 / 2

#define FLAG_DEFAULT        0 // 默认状态
#define RFID_SUPPLY_ZONE    (1 << 0) // 第一位:基地补给区RFID
#define RFID_SUPPLY_OUTPOST (1 << 1) // 第二位:前哨站RFID
#define STATUS_COMBATING    (1 << 2) // 第三位:脱战状态 脱战:1 非脱战:0
#define TIME_GT_2MIN        (1 << 3) // 第四位:剩余时间是否大于2min
#define ENERMY_RED          (1 << 4) // 第五位:敌人颜色 红:1 蓝:0

#pragma pack(1)                        // 压缩结构体,取消字节对齐,下面的数据都可能被传输

/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief
 * 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态

typedef enum
{
    GAME_NOT_STARTED         = 0, // 0：未开始比赛
    GAME_PREPARATION         = 1, // 1：准备阶段
    GAME_15S_SELF_CHECK      = 2, // 2：十五秒裁判系统自检阶段
    GAME_5S_COUNTDOWN        = 3, // 3：五秒倒计时
    GAME_IN_PROGRESS         = 4, // 4：比赛中
    GAME_CALCULATING_RESULTS = 5  // 5：比赛结算中
} Game_Progress_e;

// typedef enum
// {
//     RFID_NONE        = 0, // 0：什么都没有 (未检测到或不在上述区域)
//     RFID_SUPPLY_ZONE = 1, // 1：补给区
//     RFID_CENTER_GAIN = 2  // 2：中心增益点 (仅 RMUL 适用)
// } RFID_Status_e;

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
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 无力模式
    CHASSIS_NO_FOLLOW,         // 分离模式
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式
    CHASSIS_SPIN,              // 小陀螺模式
    CHASSIS_MAXSPEED           // 满速，我还没写，哈哈哈

} chassis_mode_e;
typedef enum
{
    GIMBAL_ZERO_FORCE = 0,    // 无力模式
    GIMBAL_NO_FOLLOW,         // 分离模式
    GIMBAL_FOLLOW_GIMBAL_YAW, // 跟随模式
    GIMBAL_FULL_AUTO,         // 全自动运行(烧饼烧烤中)
    GIMBAL_CALI,              // 校准中....

} gimbal_mode_e;

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

// 校准变量
typedef struct {
    uint8_t cali_status;
    float max_pitch;
    float min_pitch;
    float zero_pitch;
    float zero_yaw;

} cali_data_t;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅----------------
 */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef __packed struct {
    // 控制部分
    float vx;           // 前进方向速度, mm/s
    float vy;           // 横移方向速度, mm/s
    float wz;           // 旋转速度, rad/s
    float offset_angle; // 底盘和正方向位置的夹角
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

typedef __packed struct {
    uint16_t power_limit;
    uint16_t buffer_energy;

    // UI部分
    //  ...

} Referee_Ctrl_Cmd_s;

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
    // RFID_Status_e rfid_status;     // RFID状态
    Game_Progress_e game_progress; // 比赛进程状态
    uint16_t current_HP;           // 当前血量
    uint16_t ally_base_HP;
    uint16_t ally_outpost_HP;
    uint8_t status_flag; // 状态标志位,可以用来表示一些特殊状态,比如是否打击到敌人,是否进入补给区等
    uint16_t shooter_heat;

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
