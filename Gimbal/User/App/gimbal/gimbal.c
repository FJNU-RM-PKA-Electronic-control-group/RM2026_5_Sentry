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
DMMotorInstance *yaw1_motor;
DJIMotorInstance *yaw2_motor;
FeedForwardInstance *pitch_ff;
FeedForwardInstance *yaw_current_ff;
FeedForwardInstance *yaw_speed_ff;

FeedForwardInstance DEBUG_pitch_ff;
DMMotorInstance DEBUG_pitch_motor;
DMMotorInstance DEBUG_yaw1_motor;
DJIMotorInstance DEBUG_yaw2_motor;
float DEBUG_dm_pitch, DEBUG_dm_yaw, DEBUG_dm_roll;
float target_yaw[2], target_pitch;
float DEBUG_yaw1_angle, DEBUG_yaw1_target;
float DEBUG_yaw2_angle, DEBUG_pitch_rad, DEBUG_IMU_yaw, DEBUG_Pitch_target;

static Publisher_t *gimbal_pub;            // 云台应用消息发布�?(云台反�?�给cmd)
static Subscriber_t *gimbal_sub;           // cmd控制消息订阅�?
Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信�?
Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信�?

// static BMI088Instance *bmi088; // 云台IMU

void GimbalInit(void) {

    IMU_data_rad = GetInsData(); // IMU先初始化,获取姿态数�?指针赋给yaw电机的其他数�?来源
    DM_IMU_data  = GetIMUData();
    // YAW
    FeedForward_Init_Config_s yaw_speed_ff_config = {
        .format_value = PI,
        .K1           = 0.0139062498f,  // 0.01
        .K2           = 0.0f,
        .RC           = 7.00000019e-05, // 4.99999987e-05
        .max_out      = 5.0f,
    };
    yaw_speed_ff                                    = FeedForwardInit(&yaw_speed_ff_config);
    FeedForward_Init_Config_s yaw_current_ff_config = {
        .K1      = -7500.0f,        // 0
        .K2      = 0.0f,
        .RC      = 2.14999964e-07f, // 4.99999987e-05
        .max_out = 3000.0f,
    };
    yaw_current_ff                  = FeedForwardInit(&yaw_current_ff_config);
    Motor_Init_Config_s yaw2_config = {
        .can_init_config =
            {
                .can_handle = &hcan1,
                .rx_id      = 1,
                .tx_id      = 1,
            },
        .controller_param_init_config =
            {
                .angle_PID = {.Kp = 0.7f,
                              .Ki = 0.0f,
                              .Kd = 0.003f,
                              .Improve =
                                  (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                      PID_Derivative_On_Measurement |
                                                      PID_DerivativeFilter),
                              .CoefA             = 0.01f,
                              .CoefB             = 0.1f,
                              .MaxOut            = 10.0f,
                              .IntegralLimit     = 33.0f,
                              .Derivative_LPF_RC = 0.1f},
                .speed_PID =
                    {
                        .Kp = 10000.0f, //
                        .Ki = 25000.0f, //
                        .Kd = 0.0f,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_DerivativeFilter),
                        .IntegralLimit     = 15000.0f,
                        .MaxOut            = 22000.0f,
                        .Derivative_LPF_RC = 0.05f,
                    },
                .other_angle_feedback_ptr = &IMU_data_rad->Yaw,
                .other_speed_feedback_ptr = &IMU_data_rad->Gyro[2],
                .speed_feedforward_ptr    = &yaw_speed_ff->final_out,
                .current_feedforward_ptr  = &yaw_current_ff->final_out,
                .angle_zero_point_offset  = YAW2_ZEROPOINT_ECD,
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = OTHER_FEED,
                .speed_feedback_source = OTHER_FEED,
                .feedforward_flag   = (Feedfoward_Type_e)(SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD),
                .outer_loop_type    = ANGLE_LOOP,
                .close_loop_type    = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                .pid_mode           = MOTOR_PID_SINGLELOOP,
                .angle_rad_format_flag = FORMAT_ENABLE,
            },
        .motor_type = GM6020,
    };
    yaw2_motor = DJIMotorInit(&yaw2_config);

    Motor_Init_Config_s yaw1_config = {
        .can_init_config =
            {
                .can_handle = &hcan2,
                .tx_id      = 0x0A,
                .rx_id      = 0x09,
            },
        .controller_param_init_config =
            {
                .angle_PID =
                    {
                        .Kp       = 10.0f, // 8
                        .Ki       = 0.0f,
                        .Kd       = 0.7f,
                        .DeadBand = 0.0f,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_DerivativeFilter | PID_ChangingIntegrationRate),
                        .CoefA             = 0.01f,
                        .CoefB             = 0.1f,
                        .IntegralLimit     = 50.0f,
                        .MaxOut            = 75.0F,
                        .Derivative_LPF_RC = 0.01f,
                    },
                .speed_PID =
                    {
                        .Kp = 0.1f, // 50
                        .Ki = 0.0f, // 200
                        .Kd = 0.001f,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_DerivativeFilter),
                        .IntegralLimit = 1.0f,
                        .MaxOut        = 9.0f,
                        .Output_LPF_RC = 0.1f,
                    },
                .yaw2_motor = (void *)yaw2_motor,
                // 还需要�?�加角速度额�?�反馈指�?,注意方向,ins_task.md�?有c板的bodyframe坐标系�?�明
                .other_angle_feedback_ptr = &DM_IMU_data->yaw,
                .other_speed_feedback_ptr = &DM_IMU_data->gyro[2],
                .angle_zero_point_offset  = YAW1_ZEROPOINT_ECD,
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = OTHER_FEED,
                .speed_feedback_source = MOTOR_FEED,
                //.feedforward_flag      = SPEED_FEEDFORWARD,
                .outer_loop_type    = ANGLE_LOOP,
                .close_loop_type    = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                .pid_mode           = MOTOR_PID_SINGLELOOP,
                //.angle_rad_format_flag = FORMAT_ENABLE,
            },
        .motor_type = DMJ4310,
    };
    yaw1_motor = DMMotorInit(&yaw1_config);
    // PITCH
    FeedForward_Init_Config_s pitch_ff_config = {
        .K1      = -0.775f,
        .K2      = 0.0f,
        .RC      = 7.00000019e-05f,
        .max_out = 1.0f,
    };
    pitch_ff                         = FeedForwardInit(&pitch_ff_config);
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {.can_handle = &hcan1, .tx_id = 0x0A, .rx_id = 0x09},
        .controller_param_init_config =
            {
                .angle_PID =
                    {
                        .Kp = 30.0f,  // 20
                        .Ki = 20.0f,  // 17
                        .Kd = 0.001f, // 2
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_ChangingIntegrationRate),
                        .CoefA             = 0.01f,
                        .CoefB             = 0.1f,
                        .IntegralLimit     = 4.0f,
                        .MaxOut            = 5.0f,
                        .Derivative_LPF_RC = 0.01f,
                    },
                .speed_PID =
                    {
                        .Kp = 1.4f,  // 150
                        .Ki = 10.0f, // 30
                        .Kd = 0.01f, // 3
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement | PID_OutputFilter),
                        .Output_LPF_RC = 0.008f,
                        .IntegralLimit = 1.0f,
                        .MaxOut        = 18.0f,
                    },
                .other_angle_feedback_ptr = &IMU_data_rad->Pitch,
                // 还需要�?�加角速度额�?�反馈指�?,注意方向,ins_task.md�?有c板的bodyframe坐标系�?�明
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
                .pid_mode              = MOTOR_PID_MUTILOOP,
                // .angle_rad_format_flag = FORMAT_ENABLE,
            },
        .motor_type = DMJ4310,
    };
    pitch_motor = DMMotorInit(&pitch_config);

    // 电机对total_angle�?�?,上电时为�?,会保持静�?,收到遥控器数�?再动

    FeedForwardSetRef(pitch_ff, &pitch_motor->motor_controller.pid_ref);
    FeedForwardSetRef(yaw_speed_ff, &yaw2_motor->motor_controller.pid_ref);
    FeedForwardSetRef(yaw_current_ff, &yaw2_motor->motor_controller.speed_PID.Ref);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

static void YawSetTarget(float recv_yaw, gimbal_mode_e mode) {
    // static bool_t target_setted = 0;
    static Yaw1_mode_e yaw1_mode   = YAW1_NOMOVE;
    static Yaw2_mode_e yaw2_mode   = YAW2_NOSCAN;
    static pitch_mode_e pitch_mode = PITCH_NOMOVE;
    // float current_yaw           = IMU_data_rad->Yaw * RAD_2_DEGREE;
    static gimbal_mode_e last_mode = GIMBAL_ZERO_FORCE;
    float yaw2_ecd_err             = theta_format(DJIMotorGetAngle(yaw2_motor));
    // float yaw_err               = -NearbyErrFormat(recv_yaw - current_yaw, 180);

    if (last_mode != mode) {
        target_yaw[YAW1] = DM_IMU_data->yaw;
        target_yaw[YAW2] = IMU_data_rad->Yaw * RAD_2_DEGREE;
    }
    if (mode != GIMBAL_SCAN_MODE) {
        pitch_mode = PITCH_NOMOVE;
        yaw2_mode  = YAW2_NOSCAN;
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
            target_yaw[YAW1] = DM_IMU_data->yaw + float_constrain(yaw2_ecd_err - 48.5f, 0.0f, 2.0f);
            if (yaw2_ecd_err < 48)
                yaw1_mode = YAW1_NOMOVE;
            break;
        case YAW1_TURNRIGHT:
            target_yaw[YAW1] =
                DM_IMU_data->yaw + float_constrain(yaw2_ecd_err + 48.5f, -2.0f, 0.0f);
            if (yaw2_ecd_err > -48)
                yaw1_mode = YAW1_NOMOVE;
            break;
        default:
            break;
        }
        target_yaw[YAW2] = recv_yaw;
        target_pitch     = gimbal_cmd_recv.pitch;
    }
    else {
        target_yaw[YAW1] += 0.25f; // �?Yaw云台慢速扫�?
        switch (yaw2_mode) {
        case YAW2_TURNLEFT:
            target_yaw[YAW2] += 0.50f;
            if (yaw2_ecd_err > 50) {
                yaw2_mode        = YAW2_TURNRIGHT;
                target_yaw[YAW2] = IMU_data_rad->Yaw * RAD_2_DEGREE;
            }
            break;
        case YAW2_TURNRIGHT:
            target_yaw[YAW2] -= 0.10f;
            if (yaw2_ecd_err < -50) {

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
    switch (pitch_mode) {
    case PITCH_UP:
        target_pitch += 0.005f;
        if (pitch_motor->measure.position > PITCH_MAX_ECD) {
            pitch_mode = PITCH_DOWN;
            // target_pitch = IMU_data_rad->Pitch;
        }

        break;
    case PITCH_DOWN:
        target_pitch -= 0.005f;
        if (pitch_motor->measure.position < PITCH_MIN_ECD) {
            pitch_mode = PITCH_UP;
            // target_pitch = IMU_data_rad->Pitch;
        }
        /* code */
        break;
    case PITCH_NOMOVE:
        pitch_mode = PITCH_UP;
        break;
    default:
        break;
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

/* 机器人云台控制核心任�?,后续考虑�?保留IMU控制,不再需要电机的反�?? */
void GimbalTask(void) {
    // 获取云台控制数据
    // 后续增加�?收到数据的�?�理
    DEBUG_pitch_motor = *pitch_motor;
    DEBUG_yaw1_motor  = *yaw1_motor;
    DEBUG_yaw2_motor  = *yaw2_motor;
    DEBUG_pitch_ff    = *pitch_ff;

    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    // yaw1_imu_data = AngleFormatfp(IMU_data_rad->Yaw + DJIMotorGetAngle(yaw2_motor));

    // @todo:现在已不再需要电机反�?,实际上可以�?�终使用IMU的姿态数�?来作为云台的反�??,yaw电机的offset�?�?用来跟随底盘
    // 根据控制模式进�?�电机反馈切换和过渡,视�?�模式在robot_cmd模块就已经�?�置�?,gimbal�?看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode) {
    // 停�??
    case GIMBAL_ZERO_FORCE:
        YawSetTarget(gimbal_cmd_recv.yaw, GIMBAL_ZERO_FORCE);

        DMMotorStop(yaw1_motor);
        DJIMotorStop(yaw2_motor);
        DMMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反�?,底盘根据yaw电机的offset跟随云台或�?��?�模式采�?
    case GIMBAL_GYRO_MODE: // 后续�?保留此模�?
        YawSetTarget(gimbal_cmd_recv.yaw, GIMBAL_GYRO_MODE);

        DMMotorEnable(yaw1_motor);
        DJIMotorEnable(yaw2_motor);
        DMMotorEnable(pitch_motor);
        DMMotorChangeFeed(yaw1_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(yaw1_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);

        // DMMotorSetRef(yaw_motor, 0); // yaw和pitch会在robot_cmd�?处理好�?�圈和单�?
        // DMMotorSetRef(yaw1_motor, 0);
        DJIMotorSetRef(yaw2_motor, target_yaw[YAW2]);
        DMMotorSetRef(yaw1_motor, target_yaw[YAW1]);
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        // DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); //
        // yaw和pitch会在robot_cmd�?处理好�?�圈和单�? RMDMotorSetRef(pitch_motor,
        // gimbal_cmd_recv.pitch);

        break;
    // 云台�?由模�?,使用编码器反�?,底盘和云台分�?,仅云台旋�?,一�?用于调整云台姿�?(英雄吊射�?)/能量机关
    case GIMBAL_FREE_MODE:
        YawSetTarget(gimbal_cmd_recv.yaw, GIMBAL_FREE_MODE);

        DMMotorEnable(yaw1_motor);
        DJIMotorEnable(yaw2_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw2_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(yaw1_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(yaw1_motor, SPEED_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw2_motor, target_yaw[YAW2]);
        DMMotorSetRef(yaw1_motor, target_yaw[YAW1]);
        DMMotorSetRef(pitch_motor, target_pitch);
        // yaw和pitch会在robot_cmd�?处理好�?�圈和单�? RMDMotorSetRef(pitch_motor,
        // gimbal_cmd_recv.pitch);
        break;
    case GIMBAL_SCAN_MODE:
        YawSetTarget(gimbal_cmd_recv.yaw, GIMBAL_SCAN_MODE);

        DMMotorEnable(yaw1_motor);
        DJIMotorEnable(yaw2_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw2_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(yaw1_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(yaw1_motor, SPEED_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);

        DJIMotorSetRef(yaw2_motor, target_yaw[YAW2]);
        DMMotorSetRef(yaw1_motor, target_yaw[YAW1]);
        DMMotorSetRef(pitch_motor, target_pitch);
        break;

    case GIMBAL_VISION_MODE:
        YawSetTarget(gimbal_cmd_recv.yaw, GIMBAL_VISION_MODE);

        DMMotorEnable(yaw1_motor);
        DJIMotorEnable(yaw2_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw2_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(yaw1_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(yaw1_motor, SPEED_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);

        DJIMotorSetRef(yaw2_motor, target_yaw[YAW2]);
        DMMotorSetRef(yaw1_motor, target_yaw[YAW1]);
        DMMotorSetRef(pitch_motor, target_pitch);
        break;

    default:
        break;
    }
    // DJIMotorStop(yaw2_motor);
    // DMMotorStop(yaw1_motor);
    // DEBUG_dm_pitch    = DM_IMU_data->pitch;
    // DEBUG_dm_yaw      = DM_IMU_data->yaw;
    // DEBUG_dm_roll     = DM_IMU_data->roll;
    // DEBUG_yaw1_target = AngleFormatfp(NearbyErrFormat(DMMotorGetAngle(yaw1_motor), 180) / 2);
    // DEBUG_yaw1_angle  = DMMotorGetAngle(yaw1_motor);
    // DEBUG_yaw2_angle  = theta_format(DJIMotorGetAngle(yaw2_motor));

    // DMMotorSetRef(yaw_motor, 10.0F);

    // 在合适的地方添加pitch重力补偿前�?�力�?
    // 根据IMU姿�?/pitch电机角度反�?��?�算出当前配重下的重力矩
    // ...

    // 设置反�?�数�?,主�?�是imu和yaw的ecd
    // gimbal_feedback_data.gimbal_imu_data              = *IMU_data_rad;
    gimbal_feedback_data.yaw2_motor_single_round_angle = DJIMotorGetAngle(yaw2_motor);
    gimbal_feedback_data.yaw1_motor_single_round_angle = DMMotorGetAngle(yaw1_motor) * RAD_2_DEGREE;
    gimbal_feedback_data.pitch_ecd_angle               = pitch_motor->measure.position;
    DEBUG_pitch_rad                                    = -(IMU_data_rad->Pitch);
    DEBUG_IMU_yaw                                      = IMU_data_rad->Yaw * RAD_2_DEGREE;
    DEBUG_Pitch_target                                 = gimbal_cmd_recv.pitch;

    // gimbal_feedback_data.pitch_ecd_angle              = pitch_motor->measure.total_angle;

    // 推送消�?
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
