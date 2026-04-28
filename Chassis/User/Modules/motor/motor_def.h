
/**
  ****************************(C) COPYRIGHT 2022 YUELU****************************
  * @file      motor_def.h
  *
  * @brief     ����?�õ����ݽṹ����
  *
  * @todo
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0                     neozng             1.���?
  *  V2.0.0                      qhp               1.�����Ӧ������?
  *  V2.1.0                     nan_97             1.���������?
  *
  @verbatim
  ================================================================================

  ================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 YUELU****************************
  */
#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "bsp_can.h"
#include "controller.h"
#include "stdint.h"

#ifndef PI
#define PI 3.1415926535f
#endif
#define PI2 (PI * 2.0f)                     // 2 pi

#define RAD_2_DEGREE 57.2957795f            // 180/pi
#define DEGREE_2_RAD 0.01745329252f         // pi/180
#define RADPS_2_RPM  9.549296586f           // 30/pi
#define RPM_2_RADPS  0.104719755f           // pi/30

#define RPM_2_ANGLE_PER_SEC    6.0f         // ��360��/60sec
#define RPM_2_RAD_PER_SEC      0.104719755f // ��2pi/60sec
// #define RPM_TO_ANGLE_SPEED     0.00290888208665721596153948461415f // AHH,WHAT'S THIS
#define REDUCETION_RATIO_3508  19.203208556f
#define REDUCETION_RATIO_2006  36
#define REDUCETION_RATIO_8009  9
#define REDUCETION_RATIO_4310  10
#define UNITREE_TO_OUTPUTSPEED 0.00387731f   // (6.2831/256/6.33)
#define UNITREE_TO_OUTPUTANGLE 0.0000302915f // (6.2831/32768/6.33)

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/**
 * @brief �ջ�����,������?����ջ�?,��ʹ�û�����
 *        ������Ҫ�ٶȻ��͵�����: CURRENT_LOOP|SPEED_LOOP
 */
typedef enum
{
    OPEN_LOOP    = 0,
    CURRENT_LOOP = 1,
    SPEED_LOOP   = 2,
    ANGLE_LOOP   = 4,

    // only for checking
    SPEED_AND_CURRENT_LOOP = 3,
    ANGLE_AND_SPEED_LOOP   = 6,
    ALL_THREE_LOOP         = 7,
} Closeloop_Type_e;

/**
 * @brief ǰ������
 *
 * @todo
 * ���Ӷ�Ӧ��ǰ���ļ���������Ҫ�滮���ȵ��ֱ��������?
 */
typedef enum
{
    FEEDFORWARD_NONE              = 0,
    CURRENT_FEEDFORWARD           = 1,
    SPEED_FEEDFORWARD             = 2,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/* ������Դ�趨,����ΪOTHER_FEED����Ҫָ��������Դָ��,���Motor_Controller_s*/
typedef enum
{
    MOTOR_FEED = 0,
    OTHER_FEED,

} Feedback_Source_e;

/*������Դ������?*/
// û��
typedef enum
{
    MOTOR_OUTPUT = 0, // ���������ص����?
    OTHER_OUTPUT,     // ����������Դ�����?
} Output_Source_e;

/* ��������?��־ */
typedef enum
{
    MOTOR_DIRECTION_NORMAL  = 0,
    MOTOR_DIRECTION_REVERSE = 1
} Motor_Reverse_Flag_e;

/* ������������־ */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL  = 0,
    FEEDBACK_DIRECTION_REVERSE = 1

} Feedback_Reverse_Flag_e;

/* �������״�? */
// �о�ֻ��dm���õ����ٿ�����
typedef enum
{
    MOTOR_STOP    = 0,
    MOTOR_ENALBED = 1,
    MOTOR_CALI    = 2,
} Motor_Working_Type_e;

typedef enum
{
    MOTOR_PID_MUTILOOP   = 0,
    MOTOR_PID_SINGLELOOP = 1,

} Motor_PID_Mode_e;

/* Motor control settings, including closed-loop type, rotation direction, and feedback source
 */
typedef struct {
    Closeloop_Type_e outer_loop_type;              // 外部闭环类型，未指定时默认为最外层闭环
    Closeloop_Type_e close_loop_type;              // 使用的闭环类型（内部）
    Motor_Reverse_Flag_e motor_reverse_flag;       // 是否反转
    Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
    Feedback_Source_e angle_feedback_source;       // 角度反馈源
    Feedback_Source_e speed_feedback_source;       // 速度反馈源
    Feedfoward_Type_e feedforward_flag;            // 前馈标志
    Output_Source_e output_flag;                   // 输出源标志
    Motor_PID_Mode_e pid_mode; // PID运行模式标志位，用来确定pid运算用单圈闭环还是多圈闭环

} Motor_Control_Setting_s;

/* 电机控制器，包含反馈源指针和三个控制环的参考值 */
// 电机控制器前馈指针
typedef struct {
    float *other_angle_feedback_ptr; // 其他角度反�?�源指针
    float *other_speed_feedback_ptr; // 其他速度反�?�源指针
    float *speed_feedforward_ptr;    // 速度前�?�指�?
    float *current_feedforward_ptr;  // 电流前�?�指�?
    float *other_output_ptr;         // 其他输出源指�?
    float zero_point_offset;         // 零点偏移设定值

    PIDInstance current_PID;         // 电流PID实例
    PIDInstance speed_PID;           // 速度PID实例
    PIDInstance angle_PID;           // 角度PID实例

    float pid_ref;                   // PID参考值，通常用于各控制环
    float final_out;                 // PID计算后，发送给电机前的值，需要进行功率限制
} Motor_Controller_s;

/* 电机类型枚举 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    M3508,   // 电机*4+joint1+joint2
    M2006,   // �?�?抓取机构*2+云台，反馈需取减速后
    GM6020,
    DMJ4310, // joint3
    DMJ8009,
    DM10010L,
    Unitree8010, // ��yaw
    MG996,

} Motor_Type_e;

/**
 * @brief 电机控制器初始化结构体，用于配置PID参数以及相关反馈指针
 *        某些场合需要特殊控制，所以不使用默认的PID配置
 *        需要指定反馈源指针和闭环类型，还需要Motor_Control_Setting_s中的相关输出源标志
 *
 * @note
 * 目前只有部分场合需要判断是否需要数据融合控制，其他情况下默认即可，具体使用时请根据实际需求指定
 */
typedef struct {
    float *other_angle_feedback_ptr; // 角度反馈数据指针,注意电机使用total_angle
    float *other_speed_feedback_ptr; // 速度反馈数据指针,单位为angle per sec

    float *speed_feedforward_ptr;    // 速度前馈数据指针
    float *current_feedforward_ptr;  // 电流前馈数据指针

    float angle_zero_point_offset;   // 角度环电机零点偏移量

    PID_Init_Config_s current_PID;
    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;
} Motor_Controller_Init_s;

/**
 * @brief 用于初始化化CAN电机的结构体，包�?所有通道
 *
 * @note  需要指定电机类型、pid参数和相关配�?数据、反馈指针、输出方向等
 *        还包括can通信参数，�?�can通道和帧id�?
 *
 */
typedef struct {
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    Motor_Type_e motor_type;
    CAN_Init_Config_s can_init_config;

} Motor_Init_Config_s;

#endif // !MOTOR_DEF_H
