#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

#include "ins_task.h"
// #include "master_process.h"
#include "stdint.h"

/* ���������Ͷ���,��¼ʱע�ⲻҪŪ����Ӧ����;�޸Ķ������Ҫ���±���,ֻ�ܴ���һ������!
 */
// #define ONE_BOARD // �����������
//  #define CHASSIS_BOARD //���̰�
//  #define GIMBAL_BOARD  //��̨��

#define VISION_USE_VCP // ʹ�����⴮�ڷ����Ӿ�����
// #define VISION_USE_UART // ʹ�ô��ڷ����Ӿ�����

/* ��������Ҫ��������,ע����ݲ�ͬ�����˽����޸�,��������Ҫ��.0��f��β,�޷�����u��β
 */
// ��̨����
#define YAW1_ZEROPOINT_ECD                                                                         \
    3.96606445 // ��̨�͵��̶���ָ����ͬ����ʱ�ĵ��������ֵ,������̨�л�е�Ķ���Ҫ�޸�
#define YAW2_ZEROPOINT_ECD                                                                         \
    281.731384 // Сyaw�ʹ�yaw����ָ����ͬ����ʱ�ĵ��������ֵ,������̨�л�е�Ķ���Ҫ�޸�
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECDֵ�Ƿ����4096,��Ϊ1,��Ϊ0;���ڼ�����̨ƫת�Ƕ�

#define PITCH_HORIZON_ECD                                                                          \
    0.373961449 // ��̨����ˮƽλ��ʱ������ֵ,������̨�л�е�Ķ���Ҫ�޸�
#define PITCH_MAX_ECD          0.7F
#define PITCH_MIN_ECD          0.0f
#define PITCH_MAX_ANGLE        -15674.30f // ��̨��ֱ�������Ƕ� (ע�ⷴ������������ǣ�����д�����ǵĽǶ�)
#define PITCH_MIN_ANGLE        -15746.30f // ��̨��ֱ������С�Ƕ� (ע�ⷴ������������ǣ�����д�����ǵĽǶ�)
// �������
#define ONE_BULLET_DELTA_ANGLE 60 // ����һ�����貦��ת���ľ���,�ɻ�е���ͼֽ����
#define REDUCTION_RATIO_LOADER                                                                     \
    100.0f // 2006���̵���ļ��ٱ�,Ӣ����Ҫ�޸�Ϊ3508��19.0f
#define NUM_PER_CIRCLE         6 // ����һȦ��װ����
// �����˵����޸ĵĲ���,��λΪmm(����)
#define WHEEL_BASE             469   // �������(ǰ�����˷���)
#define TRACK_WIDTH            365   // �����־�(����ƽ�Ʒ���)
#define CENTER_GIMBAL_OFFSET_X 0     // ��̨��ת���ľ���̼������ĵľ���,ǰ����,��̨λ��������ʱĬ����Ϊ0
#define CENTER_GIMBAL_OFFSET_Y 0     // ��̨��ת���ľ���̼������ĵľ���,���ҷ���,��̨λ��������ʱĬ����Ϊ0
#define RADIUS_WHEEL           60    // ���Ӱ뾶
#define REDUCTION_RATIO_WHEEL  19.0f // ������ٱ�,��Ϊ�������������ת�ӵ��ٶȶ������������ٶȹ������ת��
// YAW 2��3

#define GYRO2GIMBAL_DIR_YAW   1 // �����������������̨��yaw�ķ���,1Ϊ��ͬ,-1Ϊ�෴
#define GYRO2GIMBAL_DIR_PITCH 1 // �����������������̨��pitch�ķ���,1Ϊ��ͬ,-1Ϊ�෴
#define GYRO2GIMBAL_DIR_ROLL  1 // �����������������̨��roll�ķ���,1Ϊ��ͬ,-1Ϊ�෴

// ����Ƿ�������ذ嶨���ͻ,ֻ����һ�������嶨�����,���������Զ�����
// #if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
//    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
//    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
// #error Conflict board definition! You can only define one board type.
// #endif

#pragma pack(1) // ѹ���ṹ��,ȡ���ֽڶ���,��������ݶ����ܱ�����
/* -------------------------��������ģʽ���������Ͷ���-------------------------*/
/**
 * @brief
 * ��Щö�����ͺͽṹ�����ΪCMD�������ݺ͸�Ӧ�õķ������ݵ�һ����
 *
 */
// ������״̬
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
    ROBOT_AUTO
} Robot_Status_e;

typedef enum
{
    GAME_NOT_STARTED         = 0, // 0��δ��ʼ����
    GAME_PREPARATION         = 1, // 1��׼���׶�
    GAME_15S_SELF_CHECK      = 2, // 2��ʮ�������ϵͳ�Լ�׶�
    GAME_5S_COUNTDOWN        = 3, // 3�����뵹��ʱ
    GAME_IN_PROGRESS         = 4, // 4��������
    GAME_CALCULATING_RESULTS = 5  // 5������������
} Game_Progress_e;

typedef enum
{
    RFID_NONE = 0, // 0��ʲô��û�� (δ��⵽������������)
    RFID_SUPPLY_ZONE = 1, // 1��������
    RFID_CENTER_GAIN = 2  // 2����������� (�� RMUL ����)
} RFID_Status_e;

// Ӧ��״̬
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// ����ģʽ����
/**
 * @brief
 * ���������޸�Ϊ��̨�������,�������õ���ȥ׷��̨,��̨�Ĺ����ȵ���С.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // ����ģʽ
    CHASSIS_NO_FOLLOW,         // ����ģʽ
    CHASSIS_FOLLOW_GIMBAL_YAW, // ����ģʽ
    CHASSIS_ROTATE,            // С����ģʽ
    // CHASSIS_MAXSPEED,
} chassis_mode_e;

// ��̨ģʽ����
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // ����������
    GIMBAL_FREE_MODE, // ��̨�����˶�ģʽ,������̷���(���̴�ʱӦΪNO_FOLLOW)����ֵΪ���total_angle;�ƺ����Ը�Ϊȫ����IMU����?
    GIMBAL_GYRO_MODE, // ��̨�����Ƿ���ģʽ,����ֵΪ������pitch,total_yaw_angle,���̿���ΪС���ݺ͸���ģʽ
    GIMBAL_SCAN_MODE,
    GIMBAL_VISION_MODE,

} gimbal_mode_e;

// ����ģʽ����
typedef enum
{
    SHOOT_OFF  = 0,
    SHOOT_ON   = 1,
    SHOOT_STOP = 2,
} shoot_mode_e;

typedef enum
{
    FRICTION_OFF = 0, // Ħ���ֹر�
    FRICTION_ON,      // Ħ���ֿ���
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // ���ոǴ�
    LID_CLOSE,    // ���ոǹر�
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // ֹͣ����
    LOAD_REVERSE,   // ��ת
    LOAD_1_BULLET,  // ����
    LOAD_BURSTFIRE, // ����
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
    ENEMY_RED  = 0, // �췽
    ENEMY_BLUE = 1, // ����
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
    STATE_NOPE      = 0,
    STATE_ATTACKING = 2,
    STATE_DEFENDING = 3,
    STATE_MOVING    = 1,

} Sentry_State_e;

// ��������,�Ӳ���ϵͳ��ȡ,�Ƿ��б�Ҫ����?
typedef struct { // ���ʿ���
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMDӦ�÷����Ŀ�������,Ӧ����gimbal/chassis/shoot����---------------- */
/**
 * @brief ����˫�����,ң������pc����̨,����ϵͳ�ڵ���
 *
 */
// cmd�����ĵ��̿�������,��chassis����
typedef struct {
    // ���Ʋ���
    float vx;           // ǰ�������ٶ�
    float vy;           // ���Ʒ����ٶ�
    float wz;           // ��ת�ٶ�
    float offset_angle; // ���̺�������λ�õļн�
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
} Chassis_Ctrl_Cmd_s;

// cmd��������̨��������,��gimbal����
typedef struct { // ��̨�Ƕȿ���
    float yaw;
    float pitch;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd�����ķ����������,��shoot����
typedef struct {
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // ����ö��
    uint8_t rest_heat;
    uint16_t heat_limit;
    float shoot_rate; // �����������Ƶ,unit per s,��/��
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis�����ķ�������----------------*/
/**
 * @brief ��cmd����,����Ӧ��Ҳ���Ը�����Ҫ��ȡ.
 *
 */

// typedef struct {
// #if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // �ǵ����ʱ����̻���imu���ݻش�(���б�Ҫ)
//     // attitude_t chassis_imu_data;
// #endif
//     // �������ӵ��̵���ʵ�ٶ�
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

    // �������ӵ��̵���ʵ�ٶ�
    //     float real_vx;
    //     float real_vy;
    //     float real_wz;
    RFID_Status_e rfid_status;     // RFID״̬
    Game_Progress_e game_progress; // ��������״̬
    Enemy_Color_e enemy_color;     // 0 for blue, 1 for red
    uint16_t current_HP;
    uint16_t shooter_heat;

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

#pragma pack() // �����ֽڶ���,����ǰ���#pragma pack(1)

#endif
