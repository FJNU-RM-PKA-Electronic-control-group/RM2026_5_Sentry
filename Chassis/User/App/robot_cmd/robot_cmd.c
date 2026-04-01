/**
  ****************************(C) COPYRIGHT 2022 YUELU*********************************
  * @file      robot_cmd.c\h
  * @brief
  robot_cmd�����������˵ĺ���Ӧ�ã��为�����ң����/��λ��������ָ�����ָ
  *
  ��ת��Ϊʵ�ʵ��˶�����Ŀ�꣬���͸���������Ӧ�á����߻����robot_cmd��������
  *
  ��趨���������ִ�е�Ԫ�Ĳο�ֵ�ȡ�(�൱�ڼ����˾ɴ���gimbal_behaviour��
  *             chassis_behaviour���֣��Ҿ��з����߹��ܣ�
  *
  * @todo      1.�����Զ��������ģʽ
  *            2.����ң����ģʽ�µ�¼��ģʽѡ����У׼�������
  *            3.��д�Զ�����������ߵ����Ӧ�Դ�ʩ
               4.����message������Ҫ�޸�
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0                     YUELU              1. ���
  *  V2.0.0                      qhp               1.�����Ӧ������
  *  V2.1.0                     nan_97             1.�����Զ��������ģʽ�����乤��
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

RC_ctrl_t *rc_data;                                       // ң��������,��ʼ��ʱ����
INS_t *ins_data;                                          // IMU����,��ʼ��ʱ����
cali_data_t *calibrate_data;
AirPumpInstance *ee_airpump, left_airpump, right_airpump; // ����ʵ��ע��
referee_info_t *referee_info;

static Publisher_t *chassis_cmd_pub;   // ���̿�����Ϣ������
static Publisher_t *chassis_feed_pub;
static Subscriber_t *chassis_feed_sub; // ���̷�����Ϣ������
static Publisher_t *referee_info_pub;
Chassis_Ctrl_Cmd_s
    chassis_cmd_send; // ���͸�����Ӧ�õ���Ϣ,����������Ϣ��UI�������
Chassis_Upload_Data_s chassis_fetch_data; // �ӵ���Ӧ�ý��յķ�����Ϣ��Ϣ,���̹���ǹ������������˶�״̬��
Referee_Ctrl_Cmd_s referee_chassis_send;

static Publisher_t *robotarm_cmd_pub;       // ��̨������Ϣ������
static Subscriber_t *robotarm_feed_sub;     // ��̨������Ϣ������
Robotarm_Ctrl_Cmd_s robotarm_cmd_send;      // ���ݸ���̨�Ŀ�����Ϣ
Robotarm_Upload_Data_s robotarm_fetch_data; // ����̨��ȡ�ķ�����Ϣ
USB_Rx_t *Vision_message;                   // �����⴮�ڶ�ȡ������

float abslute_vx, abslute_vy;

// static Subscriber_t *vision_feed_sub;         // �Ӿ�������Ϣ������
// Vision_Upload_Data_s vision_fetch_data; // ���Ӿ���ȡ�ķ�����Ϣ

Robot_Status_e robot_state; // ���������幤��״̬

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

        abslute_vx = 6.0f * (float)rc_data->rc.rocker_r_; // _ˮƽ����
        abslute_vy = 6.0f * (float)rc_data->rc.rocker_r1; // 1��ֵ����

        float sin_theta     = arm_sin_f32(RadFormat((yaw_offset - ins_data->Yaw) * DEGREE_2_RAD));
        float cos_theta     = arm_cos_f32(RadFormat((yaw_offset - ins_data->Yaw) * DEGREE_2_RAD));
        chassis_cmd_send.vx = abslute_vx * cos_theta - abslute_vy * sin_theta;
        chassis_cmd_send.vy = abslute_vx * sin_theta + abslute_vy * cos_theta;
        chassis_cmd_send.wz = 0.05f * (float)rc_data->rc.rocker_l_;
    }
    else {
        chassis_cmd_send.vx = 20.0f * (float)rc_data->rc.rocker_r_; // _ˮƽ����
        chassis_cmd_send.vy = 20.0f * (float)rc_data->rc.rocker_r1; // 1��ֵ����
        chassis_cmd_send.wz = 5.0f * (float)rc_data->rc.rocker_l_;
    }
}

/**
 * @brief ����Ϊ����ʱģʽ�Ϳ��������ã�Ŀǰֻ�������̿��ƣ�
 *
 * @todo  ��Ҫ���Զ����������ϣ�������ʱ���ٿ��ǰ�
 */
// static void MouseKeySet(void)
// {
//     chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].w * 15000 -
//     rc_data[TEMP].key[KEY_PRESS].s
//     * 15000; // ϵ������ chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].d * 15000 -
//     rc_data[TEMP].key[KEY_PRESS].a * 15000;

// 	rc_data[TEMP].mouse.x *= -1;
// 	rc_data[TEMP].mouse.y *= -1;

// }

/**
 * @brief  ����ֹͣ,����ң�������ϲದ�ִ���/��Ҫģ������/˫��ͨ��ʧЧ��
 *         ֹͣ����ֵ'300'���޸ĳɺ��ʵ�ֵ,���Ϊ���ؿ���.
 *
 * @todo
 * �����޸�Ϊң������������ֹͣ(�ر�ң������ͣ),ͨ����ң����ģ������daemonʵ��
 *         ��һ����������Ҫ�ٽ��п��ǣ����Ǽ����Զ�����������ߵĻظ�ģʽ
 */
static void EmergencyHandler(void) {
    // ң������������
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
    chassis_fetch_data.game_progress   = referee_info->GameState.game_progress;
    chassis_fetch_data.shooter_heat    = referee_info->PowerHeatData.shooter_17mm_barrel_heat;
    if ((referee_info->RFIDStatus.rfid_status >> 19) & 0x01) {
        chassis_fetch_data.rfid_status = RFID_SUPPLY_ZONE;
    }
    else if ((referee_info->RFIDStatus.rfid_status >> 23) & 0x01) {
        chassis_fetch_data.rfid_status = RFID_CENTER_GAIN;
    }
    else {
        chassis_fetch_data.rfid_status = RFID_NONE;
    }
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
