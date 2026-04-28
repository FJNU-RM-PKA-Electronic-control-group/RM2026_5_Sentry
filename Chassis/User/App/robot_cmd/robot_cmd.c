/**
    ****************************(C) COPYRIGHT 2022 YUELU*********************************
    * @file      robot_cmd.c\h
    * @brief     robot_cmd模块提供上层的函数应用，为上层的遥控器/位置命令提供数据指令
    *
    *            转换为实际的运动控制目标，同时传递给对应的应用。同步获取robot_cmd模块的反馈信息
    *
    *            获定义模块内的执行元件的参考值等。(相当于整合了gimbal_behaviour和
    *             chassis_behaviour的工作，我们进行了功能整合)
    *
    * @todo      1.添加自动巡航模式
    *            2.完善遥控器模式下的标定模式选择与校准参数调整
    *            3.重写自动控制的反馈与执行的反应应用
                             4.优化message通信需要修改
    *
    * @note
    * @history
    *  Version    Date            Author          Modification
    *  V1.0.0                     YUELU              1. 建立
    *  V2.0.0                      qhp               1.完善应用代码
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
#include "referee_task.h"
#include "rm_referee.h"
#include "user_lib.h"

// ˽�к�,�Զ���������ת���ɽǶ�ֵ
// #define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // ����ʱ�ĽǶ�,0-360
// #define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) //
// pitchˮƽʱ����ĽǶ�,0-360

// �����������
#define YAW_ALIGN_ANGLE     0
#define PTICH_HORIZON_ANGLE 0
#define PITCH_MAX           15
#define PITCH_MIN           -8

// 底盘控制命令单位约定: vx/vy 为 mm/s, wz 为 rad/s
#define CHASSIS_SPIN_MAX_VXY_MMPS 1200.0f
#define CHASSIS_MAX_VXY_MMPS      3000.0f
#define CHASSIS_SPIN_MAX_WZ_RADPS 2.5f
#define CHASSIS_MAX_WZ_RADPS      4.0f

RC_ctrl_t *rc_data;                                       // 遥控器数据,初始化时赋值
INS_t *ins_data;                                          // IMU数据,初始化时赋值
cali_data_t *calibrate_data;
AirPumpInstance *ee_airpump, left_airpump, right_airpump; // 气泵实例注册
referee_info_t *referee_info;

static Publisher_t *chassis_cmd_pub;   // 底盘控制信息发布器
static Publisher_t *chassis_feed_pub;
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅器
static Publisher_t *referee_info_pub;
Chassis_Ctrl_Cmd_s chassis_cmd_send;   // 发送给底盘应用的控制信息,储存底层信息和UI信息
Chassis_Upload_Data_s
    chassis_fetch_data; // 从底盘应用获取的反馈信息,信息,底盘电流,冷却液温度,摩擦轮转速等
Referee_Ctrl_Cmd_s referee_chassis_send;

static Publisher_t *robotarm_cmd_pub;       // 机臂控制信息发布器
static Subscriber_t *robotarm_feed_sub;     // 机臂反馈信息订阅器
Robotarm_Ctrl_Cmd_s robotarm_cmd_send;      // 发送给机臂的控制信息
Robotarm_Upload_Data_s robotarm_fetch_data; // 从机臂获取的反馈信息
USB_Rx_t *Vision_message;                   // 视觉处理器读取的消息

float abslute_vx, abslute_vy;

// static Subscriber_t *vision_feed_sub;         // 从视觉获取信息订阅器
// Vision_Upload_Data_s vision_fetch_data; // 从视觉获取的反馈信息

Robot_Status_e robot_state; // 机器人工作状态

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
 * @brief  ����ƫ�ƽǶ�
 *
 * @todo  ������Ҫ��Ա۽����Ż��������Ƿ�ĳ������Ƿ������ݽ��м���
 *
 */

static void CalcOffsetAngle() {
    // ����angle��߿ɶ���,��Ȼ̫���˲��ÿ�,��Ȼ�������ᶯ�������
    static float angle;
    angle = robotarm_fetch_data.yaw_relative_angle; // ����̨��ȡ�ĵ�ǰyaw�����Ȧ�Ƕ�
#if YAW_ECD_GREATER_THAN_4096                       // �������180��
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // С��180��
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = (angle - YAW_ALIGN_ANGLE);
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - PI)
        chassis_cmd_send.offset_angle = (angle - YAW_ALIGN_ANGLE);
    else
        chassis_cmd_send.offset_angle = (angle - YAW_ALIGN_ANGLE + 2 * PI);
#endif
}

/**
 * @brief ��������Ϊң����(����ʱ)��ģʽ�Ϳ���������
 * @todo
 * �ؽ�У׼������Ҫд�룬����¼��ģʽ���Լ���̨��������һ����Ҫ����¼��ģʽ�����Ż���
 *        ͬʱҪע�����ļ���ԣ�����ֱ����ֲ������ϵ����Ҫ���ģ�300̫����
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

    if (switch_is_up(
            rc_data[TEMP]
                .rc.switch_right)) // �Ҳ࿪��״̬[��],���̸�����̨
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
    else if (
        switch_is_mid(
            rc_data[TEMP]
                .rc
                .switch_right)) // �Ҳ࿪��״̬[��],���̺���̨����,���̱��ֲ�ת��
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        robotarm_cmd_send.gimbal_mode = GIMBAL_NO_FOLLOW;
    }

    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // �Ҳ࿪��״̬[��],����
    {
        robot_state                   = ROBOT_STOP;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        robotarm_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    }

    if (current_mode != GIMBAL_ZERO_FORCE && last_mode == GIMBAL_ZERO_FORCE) {
        current_mode = GIMBAL_CALI;
    }

    // robotarm_cmd_send.robotarm_mode = current_mode;

    // ң����ת���̲���
    if (chassis_cmd_send.chassis_mode == CHASSIS_SPIN) {
        if (last_chassis_mode != CHASSIS_SPIN)
            yaw_offset = ins_data->Yaw;

        abslute_vx = CHASSIS_SPIN_MAX_VXY_MMPS * (float)rc_data->rc.rocker_r_;
        abslute_vy = CHASSIS_SPIN_MAX_VXY_MMPS * (float)rc_data->rc.rocker_r1;

        float sin_theta     = arm_sin_f32(RadFormat((yaw_offset - ins_data->Yaw) * DEGREE_2_RAD));
        float cos_theta     = arm_cos_f32(RadFormat((yaw_offset - ins_data->Yaw) * DEGREE_2_RAD));
        chassis_cmd_send.vx = abslute_vx * cos_theta - abslute_vy * sin_theta;
        chassis_cmd_send.vy = abslute_vx * sin_theta + abslute_vy * cos_theta;
        chassis_cmd_send.wz = CHASSIS_SPIN_MAX_WZ_RADPS * (float)rc_data->rc.rocker_l_;
    }
    else {
        chassis_cmd_send.vx = CHASSIS_MAX_VXY_MMPS * (float)rc_data->rc.rocker_r_;
        chassis_cmd_send.vy = CHASSIS_MAX_VXY_MMPS * (float)rc_data->rc.rocker_r1;
        chassis_cmd_send.wz = CHASSIS_MAX_WZ_RADPS * (float)rc_data->rc.rocker_l_;
    }
}

/**
 * @brief  紧急停止,包括遥控器掉线或信号丢失/需要模式切换/双通道失效等
 *         停止赋值'300'可修改成惯性阻尼值,但作为应急控制.
 *
 * @todo   可以修改为遥控器紧急停止(关闭遥控器停止),通过遥控器模式和daemon实现
 *         这一功能需要进一步检查,目的是完善自动化和手动联动的响应模式
 */
static void EmergencyHandler(void) {
    // 遥控器数据检查
    if (rc_data->rc_state == 0) {
        robot_state = ROBOT_STOP;
    }
    if (robot_state == ROBOT_STOP) // ����������ҪӦ�ú�ģ�����ߵ��ж�
    {
        //        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        // �����������ģʽ
        // ���ùر�
        // LOGERROR("[CMD] emergency stop!");
    }
    // ң�����Ҳ࿪��Ϊ[��],�ָ���������
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
        //      LOGINFO("[CMD] reinstate, robot ready");
    }
}

/**
 * @brief  ��������Ҫģ���ʼ��
 *
 * @todo  ���Ӷ�Ӧ�����裬�ͱ�Ҫ�ļ�
 */
void RobotCmdInit(void) {
    ins_data = GetInsData();

    rc_data = RemoteControlInit(&huart1);

    robotarm_cmd_pub  = PubRegister("robotarm_cmd", sizeof(Robotarm_Ctrl_Cmd_s));
    robotarm_feed_sub = SubRegister("robotarm_feed", sizeof(Robotarm_Upload_Data_s));
    chassis_cmd_pub   = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub  = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    referee_info_pub  = PubRegister("referee_info", sizeof(Referee_Ctrl_Cmd_s));

    chassis_feed_pub           = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    robotarm_cmd_send.imu_data = ins_data;
    Vision_message             = USB_RxBufferRegister();
    referee_info               = RefereeInit(&huart8);
    // robotarm_cmd_send.customdate = customdate;

    //	gimbal_cmd_send.pitch = 0;		���ò���������
    robot_state = ROBOT_READY;
}
/**
 * @brief �������ݸ���̨��RMUL��
 *
 */
void RefereeDataTransmit(void) {
    referee_chassis_send.power_limit   = referee_info->GameRobotState.chassis_power_limit;
    referee_chassis_send.buffer_energy = referee_info->PowerHeatData.buffer_energy;
    chassis_fetch_data.current_HP      = referee_info->GameRobotState.current_HP;
    chassis_fetch_data.game_progress   = (Game_Progress_e)referee_info->GameState.game_progress;
    chassis_fetch_data.shooter_heat    = referee_info->PowerHeatData.shooter_17mm_barrel_heat;
    chassis_fetch_data.ally_base_HP    = referee_info->GameRobotHP.ally_base_HP;
    chassis_fetch_data.ally_outpost_HP = referee_info->GameRobotHP.ally_outpost_HP;
    uint8_t communicate_flags          = FLAG_DEFAULT;
    if ((referee_info->RFIDStatus.rfid_status >> 18) & 0x01) {
        communicate_flags |= RFID_SUPPLY_OUTPOST;
    }
    if (((referee_info->RFIDStatus.rfid_status >> 20) & 0x01) &&
        ((referee_info->RFIDStatus.rfid_status >> 19) & 0x01)) {
        communicate_flags |= RFID_SUPPLY_ZONE;
    }
    if (referee_info->GameState.stage_remain_time >= 120) {
        communicate_flags |= TIME_GT_2MIN;
    }
    if ((referee_info->SentryInfo.sentry_info_2) & 0x01) {
        communicate_flags |= STATUS_COMBATING;
    }
    chassis_fetch_data.status_flag = communicate_flags;

    // 以下是联盟赛协议
    //  if ((referee_info->RFIDStatus.rfid_status >> 19) & 0x01) {
    //      chassis_fetch_data.rfid_status = RFID_SUPPLY_ZONE;
    //  }
    //  else if ((referee_info->RFIDStatus.rfid_status >> 23) & 0x01) {
    //      chassis_fetch_data.rfid_status = RFID_CENTER_GAIN;
    //  }
    //  else {
    //      chassis_fetch_data.rfid_status = RFID_NONE;
    //  }
}

/**
 * @brief  �����˺��Ŀ�������,200HzƵ������(��������Ӿ�����Ƶ��)
 *
 * @todo  ���޸Ŀ���ģʽѡ��
 */
void RobotCMDTask(void) {
    // ������Ӧ�û�ȡ�ش�����
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
    SubGetMessage(robotarm_feed_sub, &robotarm_fetch_data);

    // ����gimbal�ķ���ֵ������̨�͵���������ļн�,����Ҫ����,ͨ��static˽�б������
    // ���޸ģ����������У׼����������
    CalcOffsetAngle();

    RemoteControlSet();
    CalibrateTask(rc_data);
    EmergencyHandler(); // ����ģ�����ߺ�ң������ͣ�Ƚ������
    RefereeDataTransmit();
    // chassis_fetch_data

    // ����Ӧ������Ŀ���������remotecontrolsetmode��mousekeysetmode���������
    PubPushMessage(chassis_feed_pub, (void *)&chassis_fetch_data);
    // PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(robotarm_cmd_pub, (void *)&robotarm_cmd_send);
    PubPushMessage(referee_info_pub, (void *)&referee_chassis_send);
}
