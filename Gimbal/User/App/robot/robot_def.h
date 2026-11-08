#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

#include "ins_task.h"
// #include "master_process.h"
#include "stdbool.h"
#include "stdint.h"

/* 板卡配置与控制,记录时注意不要忽视应用场景;修改的配置需要重新编译,只能选择一个配置!
 */
// #define ONE_BOARD // 一体化单板
//  #define CHASSIS_BOARD // 底盘板
//  #define GIMBAL_BOARD  // 云台板

#define VISION_USE_VCP // 使用虚拟串口方式通信接收
// #define VISION_USE_UART // 使用串口方式通信接收

/* 云台相关需要根据实际调试,注意不同机器人需修改,除非要求一致.0或f结尾,无法使用u结尾
 */
// 云台电机
#define YAW1_ZEROPOINT_ECD                                                                         \
    -0.0218114853 // 云台大电机与云台指针同向时刻的电机编码器数值,如更换云台须仔细测量并调整
#define YAW2_ZEROPOINT_ECD                                                                         \
    292.146362  // 小yaw与大yaw电机指针同向时刻的电机编码器数值,如更换云台须仔细测量并调整
#define YAW_ECD_GREATER_THAN_4096 0        // ALIGN_ECD值是否大于4096,为1,否为0;用于纠正云台偏转角度

#define PITCH_HORIZON_ECD      0.373961449 // 云台处于水平位置时的编码值,如更换云台须仔细测量并调整
#define PITCH_MAX_ECD          -1.1F
#define PITCH_MIN_ECD          -1.9f
#define PITCH_MAX_ANGLE        -15674.30f // 云台竖直方向最大角度 (注意反向调整,调整时写入实际的角度)
#define PITCH_MIN_ANGLE        -15746.30f // 云台竖直方向最小角度 (注意反向调整,调整时写入实际的角度)
// 弹药相关
#define ONE_BULLET_DELTA_ANGLE 60     // 转动一发弹丸前进转过的角度,可由机械图纸查询
#define REDUCTION_RATIO_LOADER 100.0f // 2006电机的减速比,英文需要修改为3508的19.0f
#define NUM_PER_CIRCLE         6      // 转盘一圈装弹数量
// 底盘参数的修改的参数,单位为mm(毫米)
#define WHEEL_BASE             469 // 轴距(前后轮的距离)
#define TRACK_WIDTH            365 // 轮距(左右平行距离)
#define CENTER_GIMBAL_OFFSET_X                                                                     \
    0                   // 云台旋转中心到底盘几何中心的距离,前进方,云台位置垂直于地面时默认设为0
#define CENTER_GIMBAL_OFFSET_Y                                                                     \
    0                   // 云台旋转中心到底盘几何中心的距离,左右方向,云台位置垂直于地面时默认设为0
#define RADIUS_WHEEL 60 // 轮胎半径
#define REDUCTION_RATIO_WHEEL                                                                      \
    19.0f               // 轮胎减速比,用于将电机转速转到轮胎转速,电机转速除以减速比得轮胎转速
// YAW 2或3

#define GYRO2GIMBAL_DIR_YAW   1   // 陀螺仪方向与云台的yaw的方向,1为同向,-1为反向
#define GYRO2GIMBAL_DIR_PITCH 1   // 陀螺仪方向与云台的pitch的方向,1为同向,-1为反向
#define GYRO2GIMBAL_DIR_ROLL  1   // 陀螺仪方向与云台的roll的方向,1为同向,-1为反向

#define FLAG_DEFAULT     0        // 默认状态
#define RFID_SUPPLY_ZONE (1 << 0) // 第一位:基地补给区RFID
#define RFID_FORTRESS    (1 << 1) // 第二位:堡垒区RFID
#define STATUS_COMBATING (1 << 2) // 第三位:脱战状态 脱战:0 非脱战:1
#define ENERMY_RED       (1 << 3) // 第四位:敌人颜色 红:1 蓝:0
#define GET_HURT         (1 << 4) // 第五位:是否受到伤害

// 云台是否需要经过某定位处和归零,只能选一个配置生效,其他的自动关闭
// #if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
//    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
//    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
// #error Conflict board definition! You can only define one board type.
// #endif

#pragma pack(1) // 压缩结构体,取消字节对齐,便于数据传输与存储
/* -------------------------遥控器模式与命令与数据-------------------------*/
/**
 * @brief
 * 这些枚举类型和结构体用于CMD命令、数据和对应应用的数据的一个映射
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
    ROBOT_AUTO
} Robot_Status_e;

typedef enum
{
    GAME_NOT_STARTED         = 0, // 0未开始阶段
    GAME_PREPARATION         = 1, // 1准备阶段
    GAME_15S_SELF_CHECK      = 2, // 2十五秒系统自检阶段
    GAME_5S_COUNTDOWN        = 3, // 3五秒倒计时
    GAME_IN_PROGRESS         = 4, // 4进行中
    GAME_CALCULATING_RESULTS = 5  // 5计算结果阶段
} Game_Progress_e;

// typedef enum
// {
//     RFID_NONE        = 0, // 0什么都没有 (未检测到特定区域)
//     RFID_SUPPLY_ZONE = 1, // 1补给站
//     RFID_CENTER_GAIN = 2  // 2中心增益区 (仅 RMUL 适用)
// } RFID_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式定义
/**
 * @brief
 * 建议更改为跟随平台控制,或者去追踪平台,平台的功能要比较小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 无力模式
    CHASSIS_NO_FOLLOW,         // 不跟随模式
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式
    CHASSIS_ROTATE,            // 小陀螺模式
    // CHASSIS_MAXSPEED,
} chassis_mode_e;

// 云台模式定义
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 云台无力输出
    GIMBAL_FREE_MODE, // 云台自由运动模式,控制旋转速度(此时底盘应为NO_FOLLOW)输入值为陀螺仪total_angle;陀螺仪可以作为全局的IMU参考?
    GIMBAL_GYRO_MODE, // 云台跟随陀螺仪模式,输入值为陀螺仪pitch,total_yaw_angle,此时底盘可以为小陀螺和跟随模式
    GIMBAL_SCAN_MODE,
    GIMBAL_VISION_MODE,

} gimbal_mode_e;

// 射击模式定义
typedef enum
{
    SHOOT_OFF  = 0,
    SHOOT_ON   = 1,
    SHOOT_STOP = 2,
} shoot_mode_e;

typedef enum
{
    SPEEDMODE_ABSOLUTE,
    SPEEDMODE_NORMAL,
    // CHASSIS_MAXSPEED,
} speed_mode_e;

typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止装弹
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_BURSTFIRE, // 连射
} loader_mode_e;

typedef enum
{
    BIG_AMU_10   = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16   = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30,
    TEST         = 0,
} Bullet_Speed_e;

typedef enum
{
    ENEMY_RED  = 0, // 敌方红
    ENEMY_BLUE = 1, // 敌方蓝
} Enemy_Color_e;

typedef enum
{
    NO_TARGET   = 0,
    TARGET_LOCK = 1,
} Vision_State_e;

typedef enum
{
    OFFLINE = 0,
    ONLINE  = 1,
} online_state_e;

typedef enum
{
    STATE_NULL      = 0,
    STATE_ATTACKING = 1,
    STATE_DEFENDING = 2,
    STATE_MOVING    = 3,

} Sentry_State_e;

// 电源信息,从裁判系统获取,是否需要检查?
typedef struct { // 底盘功率
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用方向的命令数据,应该在gimbal/chassis/shoot模块---------------- */
/**
 * @brief 遥控双路拨杆,遥控摇杆pc命令台,控制系统在的地点
 *
 */
// cmd命令的底盘控制命令,给chassis模块
typedef __packed struct {
    // 运动参数
    float vx;           // 前后方向速度
    float vy;           // 左右方向速度
    float wz;           // 旋转速度
    float offset_angle; // 陀螺和底盘位置的角度
    uint8_t chassis_mode : 4;
    uint8_t sentry_state : 4;

} Chassis_Ctrl_Cmd_s;

// cmd命令的云台控制命令,给gimbal模块
typedef struct { // 云台角度控制
    float yaw;
    float pitch;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd命令的射击射控命令,给shoot模块
typedef struct {
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    uint16_t heat_limit;
    float shoot_rate; // 射击频率射频率,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis数据的反馈回传----------------*/
/**
 * @brief 从cmd来,通常对应的也是可以重新获得需要数据.
 *
 */

// typedef struct {
// #if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 是单独时需要接收并回传(可能不需要)
//     // attitude_t chassis_imu_data;
// #endif
//     // 底层左边的速度的实际速度
//     // float real_vx;
//     // float real_vy;
//     // float real_wz;

//     float pitch;
//     float yaw;
//     float roll;
//     float yaw_total;
//     Enemy_Color_e enemy_color; // 0 for blue, 1 for red

// } Chassis_Upload_Data_s;

typedef struct {

    // 后续增加底盘的真实速度
    //     float real_vx;
    //     float real_vy;
    //     float real_wz;
    // RFID_Status_e rfid_status;     // RFID状态
    // RFID_Status_e rfid_status;     // RFID状态
    Game_Progress_e game_progress; // 比赛进程状态
    uint16_t current_HP;           // 当前血量
    uint16_t bullet_rest;
    uint16_t ally_outpost_HP;
    uint8_t status_flag; // 状态标志位,可以用来表示一些特殊状态,比如是否打击到敌人,是否进入补给区等
    uint16_t shooter_heat;
    uint16_t remain_time;

} Chassis_Upload_Data_s;

typedef struct {
    attitude_t gimbal_imu_data;
    float yaw1_motor_single_round_angle;
    float yaw2_motor_single_round_angle;
    float pitch_ecd_angle;

} Gimbal_Upload_Data_s;

typedef struct {
    float aim_pitch;
    float aim_yaw;
    shoot_mode_e fire_advice;
    Vision_State_e vision_state;
    // Sentry_State_e sentry_state;
} Vision_Upload_Data_s;

typedef struct {
    float vx;
    float vy;
    float wz;
    Sentry_State_e state;
} Guidence_Upload_Data_s;

typedef struct {
    uint8_t no_error;
    // code to go here
    // ...
} Shoot_Upload_Data_s;

#pragma pack() // 恢复字节对齐,取消前面的#pragma pack(1)

#endif
