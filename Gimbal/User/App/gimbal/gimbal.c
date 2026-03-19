#include "INS_task.h"
#include "LK9025.h"
#include "dji_motor.h"
#include "dm_imu.h"
#include "dm_motor.h"
#include "feed_forward.h"
#include "fliter.h"
#include "gimbal.h"
#include "message_center.h"
#include "rmd_motor.h"
#include "robot_def.h"
#include "user_lib.h"

// #include "bmi088.h"
#define YAW1 0
#define YAW2 1

attitude_t *IMU_data_rad;
imu_t *DM_IMU_data;
float yaw1_imu_data;
// static attitude_t *IMU_data_rad; // 云台IMU数据
DMMotorInstance *pitch_motor;
LKMotorInstance *yaw1_motor;
DJIMotorInstance *yaw2_motor;
FeedForwardInstance *pitch_ff;
FeedForwardInstance *yaw_ff;

FeedForwardInstance DEBUG_pitch_ff;
DMMotorInstance DEBUG_pitch_motor;
LKMotorInstance DEBUG_yaw1_motor;
DJIMotorInstance DEBUG_yaw2_motor;
float DEBUG_dm_pitch, DEBUG_dm_yaw, DEBUG_dm_roll;
float target_yaw[2];
float DEBUG_yaw1_angle, DEBUG_yaw1_target;
float DEBUG_yaw2_angle, DEBUG_pitch_rad, DEBUG_IMU_yaw, DEBUG_Pitch_target;

static Publisher_t *gimbal_pub;            // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;           // cmd控制消息订阅者
Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

// static BMI088Instance *bmi088; // 云台IMU

void GimbalInit(void) {

    IMU_data_rad = GetInsData(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    DM_IMU_data  = GetIMUData();
    // YAW
    FeedForward_Init_Config_s yaw_ff_config = {
        .K1      = -0,
        .K2      = 0,
        .RC      = 0.05,
        .max_out = 1000,
    };
    yaw_ff                          = FeedForwardInit(&yaw_ff_config);
    Motor_Init_Config_s yaw2_config = {
        .can_init_config =
            {
                .can_handle = &hcan1,
                .rx_id      = 1,
                .tx_id      = 1,
            },
        .controller_param_init_config =
            {
                .angle_PID = {.Kp = 23.0f,
                              .Ki = 25.0f,
                              .Kd = 0.0f,
                              .Improve =
                                  (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                      PID_Derivative_On_Measurement |
                                                      PID_DerivativeFilter),
                              .CoefA             = 0.01,
                              .CoefB             = 0.1,
                              .MaxOut            = 150.0f,
                              .IntegralLimit     = 33.0f,
                              .Derivative_LPF_RC = 0.1f},
                .speed_PID =
                    {
                        .Kp = 150.0f, //
                        .Ki = 10.0f,  //
                        .Kd = 150.0f,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_DerivativeFilter),
                        .IntegralLimit     = 3000,
                        .MaxOut            = 15000,
                        .Derivative_LPF_RC = 0.05f,
                    },
                .other_angle_feedback_ptr = &IMU_data_rad->Yaw,
                .other_speed_feedback_ptr = &IMU_data_rad->Gyro[2],
                .angle_zero_point_offset  = YAW2_ZEROPOINT_ECD,
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = OTHER_FEED,
                .speed_feedback_source = OTHER_FEED,
                //.feedforward_flag      = SPEED_FEEDFORWARD,
                .outer_loop_type       = ANGLE_LOOP,
                .close_loop_type       = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
                .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
                .pid_mode              = MOTOR_PID_SINGLELOOP,
                .angle_rad_format_flag = FORMAT_ENABLE,
            },
        .motor_type = GM6020,
    };
    yaw2_motor = DJIMotorInit(&yaw2_config);

    Motor_Init_Config_s yaw1_config = {
        .can_init_config =
            {
                .can_handle = &hcan2,
                .tx_id      = 2,
                .rx_id      = 2,
            },
        .controller_param_init_config =
            {
                .angle_PID =
                    {
                        .Kp       = 7.0f, // 8
                        .Ki       = 0.0f,
                        .Kd       = 0.35f,
                        .DeadBand = 0.0,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_DerivativeFilter | PID_ChangingIntegrationRate),
                        .CoefA             = 0.01,
                        .CoefB             = 0.1,
                        .IntegralLimit     = 50,
                        .MaxOut            = 300.0F,
                        .Derivative_LPF_RC = 0.01f,
                    },
                .speed_PID =
                    {
                        .Kp = 35.0f, // 50
                        .Ki = 0.0f,  // 200
                        .Kd = 0.15f,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_DerivativeFilter),
                        .IntegralLimit = 1000,
                        .MaxOut        = 4000,
                    },
                .yaw2_motor = (void *)yaw2_motor,
                // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
                .other_angle_feedback_ptr = &DM_IMU_data->yaw,
                .other_speed_feedback_ptr = &DM_IMU_data->gyro[2],
                .speed_feedforward_ptr    = &yaw_ff->final_out,
                .angle_zero_point_offset  = YAW1_ZEROPOINT_ECD,
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = OTHER_FEED,
                .speed_feedback_source = OTHER_FEED,
                //.feedforward_flag      = SPEED_FEEDFORWARD,
                .outer_loop_type    = ANGLE_LOOP,
                .close_loop_type    = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                .pid_mode           = MOTOR_PID_SINGLELOOP,
                //.angle_rad_format_flag = FORMAT_ENABLE,
            },
        .motor_type = LK9025,
    };
    yaw1_motor = LKMotorInit(&yaw1_config);
    // PITCH
    FeedForward_Init_Config_s pitch_ff_config = {
        .K1      = 30,
        .K2      = 0,
        .RC      = 0.05,
        .max_out = 400,
    };
    pitch_ff                         = FeedForwardInit(&pitch_ff_config);
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {.can_handle = &hcan1, .tx_id = 0x0A, .rx_id = 0x09},
        .controller_param_init_config =
            {
                .angle_PID =
                    {
                        .Kp = 13.5f,  // 20
                        .Ki = 1.0,    // 17
                        .Kd = 0.005f, // 2
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_ChangingIntegrationRate),
                        .CoefA         = 0.01,
                        .CoefB         = 0.1,
                        .IntegralLimit = 50,
                        .MaxOut        = 400,
                    },
                .speed_PID =
                    {
                        .Kp = 1.3f,  // 150
                        .Ki = 10.0f, // 30
                        .Kd = 0.007, // 3
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement | PID_OutputFilter),
                        .Output_LPF_RC = 0.008,
                        .IntegralLimit = 0.15,
                        .MaxOut        = 3,
                    },
                .other_angle_feedback_ptr = &IMU_data_rad->Pitch,
                // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
                .other_speed_feedback_ptr = (&IMU_data_rad->Gyro[1]),
                .speed_feedforward_ptr    = &pitch_ff->final_out,
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = OTHER_FEED,
                .speed_feedback_source = OTHER_FEED,
                .feedforward_flag      = SPEED_FEEDFORWARD,
                .outer_loop_type       = ANGLE_LOOP,
                .close_loop_type       = (Closeloop_Type_e)(SPEED_LOOP | ANGLE_LOOP),
                .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
                .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                // .angle_rad_format_flag = FORMAT_ENABLE,
            },
        .motor_type = DMJ4310,
    };
    pitch_motor = DMMotorInit(&pitch_config);

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动

    FeedForwardSetRef(pitch_ff, &pitch_motor->motor_controller.pid_ref);
    // FeedForwardSetRef(yaw_ff, &yaw_motor->motor_controller.pid_ref);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

static void YawSetTarget(float recv_yaw, gimbal_mode_e mode) {
    // static bool_t target_setted = 0;
    static Yaw1_mode_e yaw1_mode = YAW1_NOMOVE;
    static Yaw2_mode_e yaw2_mode = YAW2_NOSCAN;
    // float current_yaw           = IMU_data_rad->Yaw * RAD_2_DEGREE;
    static gimbal_mode_e last_mode = GIMBAL_ZERO_FORCE;
    float yaw2_ecd_err             = theta_format(DJIMotorGetAngle(yaw2_motor));
    // float yaw_err               = -NearbyErrFormat(recv_yaw - current_yaw, 180);

    if (last_mode != mode) {
        target_yaw[YAW1] = DM_IMU_data->yaw;
        target_yaw[YAW2] = IMU_data_rad->Yaw * RAD_2_DEGREE;
    }
    if (mode != GIMBAL_SCAN_MODE) {
        yaw2_mode = YAW2_NOSCAN;
        switch (yaw1_mode) {
        case YAW1_NOMOVE:
            if (yaw2_ecd_err > 50) {
                // target_yaw[YAW1] = DM_IMU_data->yaw += 5.0;
                yaw1_mode = YAW1_TURNLEFT;
            }
            else if (yaw2_ecd_err < -50) {
                // target_yaw[YAW1] = DM_IMU_data->yaw -= 5.0;
                yaw1_mode = YAW1_TURNRIGHT;
            }
            break;
        case YAW1_TURNLEFT:
            target_yaw[YAW1] = DM_IMU_data->yaw + float_constrain(yaw2_ecd_err - 48.5f, 0.0f, 5.0f);
            if (yaw2_ecd_err < 48)
                yaw1_mode = YAW1_NOMOVE;
            break;
        case YAW1_TURNRIGHT:
            target_yaw[YAW1] =
                DM_IMU_data->yaw + float_constrain(yaw2_ecd_err + 48.5f, -5.0f, 0.0f);
            if (yaw2_ecd_err > -43)
                yaw1_mode = YAW1_NOMOVE;
            break;
        default:
            break;
        }
        target_yaw[YAW2] = recv_yaw;
    }
    else {
        target_yaw[YAW1] += 0.2f; // 大Yaw云台慢速扫描
        switch (yaw2_mode) {
        case YAW2_TURNLEFT:
            target_yaw[YAW2] += 0.55f;
            if (yaw2_ecd_err > 50) {
                yaw2_mode        = YAW2_TURNRIGHT;
                target_yaw[YAW2] = IMU_data_rad->Yaw * RAD_2_DEGREE;
            }
            break;
        case YAW2_TURNRIGHT:
            target_yaw[YAW2] -= 0.05f;
            if (yaw2_ecd_err < -45) {
                yaw2_mode        = YAW2_TURNLEFT;
                target_yaw[YAW2] = IMU_data_rad->Yaw * RAD_2_DEGREE;
            }
            break;
        case YAW2_NOSCAN:
            yaw2_mode = YAW2_TURNLEFT;
            break;
        default:
            break;
        }
    }

    target_yaw[YAW1] = theta_format(target_yaw[YAW1]);
    target_yaw[YAW2] = theta_format(target_yaw[YAW2]);
    last_mode        = mode;

    // if (target_setted == 0) {
    //     if (yaw_err > (50 - yaw2_ecd_err)) {
    //         target_yaw[YAW1] =
    //         target_setted    = 1;
    //     }
    //     else if (yaw_err < (-50 + yaw2_ecd_err)) {
    //         target_yaw[YAW1] =
    //         target_setted    = 1;
    //     }
    // }
    // else {
    //     if (yaw_err < (-50 + yaw2_ecd_err) || yaw_err > (-50 + yaw2_ecd_err)) {
    //         target_setted = 0;
    //     }
    // }
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask(void) {
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    DEBUG_pitch_motor = *pitch_motor;
    DEBUG_yaw1_motor  = *yaw1_motor;
    DEBUG_yaw2_motor  = *yaw2_motor;
    DEBUG_pitch_ff    = *pitch_ff;

    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    // yaw1_imu_data = AngleFormatfp(IMU_data_rad->Yaw + DJIMotorGetAngle(yaw2_motor));

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode) {
    // 停止
    case GIMBAL_ZERO_FORCE:
        YawSetTarget(gimbal_cmd_recv.yaw, GIMBAL_ZERO_FORCE);

        LKMotorStop(yaw1_motor);
        DJIMotorStop(yaw2_motor);
        DMMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        YawSetTarget(gimbal_cmd_recv.yaw, GIMBAL_GYRO_MODE);

        LKMotorEnable(yaw1_motor);
        DJIMotorEnable(yaw2_motor);
        DMMotorEnable(pitch_motor);
        LKMotorChangeFeed(yaw1_motor, ANGLE_LOOP, OTHER_FEED);
        LKMotorChangeFeed(yaw1_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);

        // DMMotorSetRef(yaw_motor, 0); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // LKMotorSetRef(yaw1_motor, 0);
        DJIMotorSetRef(yaw2_motor, target_yaw[YAW2]);
        LKMotorSetRef(yaw1_motor, target_yaw[YAW1]);
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        // DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); //
        // yaw和pitch会在robot_cmd中处理好多圈和单圈 RMDMotorSetRef(pitch_motor,
        // gimbal_cmd_recv.pitch);

        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE:
        YawSetTarget(gimbal_cmd_recv.yaw, GIMBAL_FREE_MODE);

        LKMotorEnable(yaw1_motor);
        DJIMotorEnable(yaw2_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw2_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, ANGLE_LOOP, OTHER_FEED);
        LKMotorChangeFeed(yaw1_motor, ANGLE_LOOP, OTHER_FEED);
        LKMotorChangeFeed(yaw1_motor, SPEED_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw2_motor, target_yaw[YAW2]);
        LKMotorSetRef(yaw1_motor, target_yaw[YAW1]);
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        // yaw和pitch会在robot_cmd中处理好多圈和单圈 RMDMotorSetRef(pitch_motor,
        // gimbal_cmd_recv.pitch);
        break;
    case GIMBAL_SCAN_MODE:
        YawSetTarget(gimbal_cmd_recv.yaw, GIMBAL_SCAN_MODE);

        LKMotorEnable(yaw1_motor);
        DJIMotorEnable(yaw2_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw2_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, ANGLE_LOOP, OTHER_FEED);
        LKMotorChangeFeed(yaw1_motor, ANGLE_LOOP, OTHER_FEED);
        LKMotorChangeFeed(yaw1_motor, SPEED_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);

        DJIMotorSetRef(yaw2_motor, target_yaw[YAW2]);
        LKMotorSetRef(yaw1_motor, target_yaw[YAW1]);
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;

    case GIMBAL_VISION_MODE:
        YawSetTarget(gimbal_cmd_recv.yaw, GIMBAL_VISION_MODE);

        LKMotorEnable(yaw1_motor);
        DJIMotorEnable(yaw2_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw2_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, ANGLE_LOOP, OTHER_FEED);
        LKMotorChangeFeed(yaw1_motor, ANGLE_LOOP, OTHER_FEED);
        LKMotorChangeFeed(yaw1_motor, SPEED_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);

        DJIMotorSetRef(yaw2_motor, target_yaw[YAW2]);
        LKMotorSetRef(yaw1_motor, target_yaw[YAW1]);
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;

    default:
        break;
    }
    // DJIMotorStop(yaw2_motor);
    // LKMotorStop(yaw1_motor);
    // DEBUG_dm_pitch    = DM_IMU_data->pitch;
    // DEBUG_dm_yaw      = DM_IMU_data->yaw;
    // DEBUG_dm_roll     = DM_IMU_data->roll;
    // DEBUG_yaw1_target = AngleFormatfp(NearbyErrFormat(LKMotorGetAngle(yaw1_motor), 180) / 2);
    // DEBUG_yaw1_angle  = LKMotorGetAngle(yaw1_motor);
    // DEBUG_yaw2_angle  = theta_format(DJIMotorGetAngle(yaw2_motor));

    // LKMotorSetRef(yaw_motor, 10.0F);

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    // gimbal_feedback_data.gimbal_imu_data              = *IMU_data_rad;
    gimbal_feedback_data.yaw2_motor_single_round_angle = DJIMotorGetAngle(yaw2_motor);
    gimbal_feedback_data.yaw1_motor_single_round_angle = LKMotorGetAngle(yaw1_motor);
    gimbal_feedback_data.pitch_ecd_angle               = pitch_motor->measure.position;
    DEBUG_pitch_rad                                    = -(IMU_data_rad->Pitch);
    DEBUG_IMU_yaw                                      = IMU_data_rad->Yaw * RAD_2_DEGREE;
    DEBUG_Pitch_target                                 = gimbal_cmd_recv.pitch;

    // gimbal_feedback_data.pitch_ecd_angle              = pitch_motor->measure.total_angle;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
