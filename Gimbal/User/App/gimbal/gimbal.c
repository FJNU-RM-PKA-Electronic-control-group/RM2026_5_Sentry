#include "INS_task.h"
#include "LK9025.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "feed_forward.h"
#include "fliter.h"
#include "gimbal.h"
#include "message_center.h"
#include "rmd_motor.h"
#include "robot_def.h"
#include "user_lib.h"

// #include "bmi088.h"

attitude_t *IMU_data_rad;

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

static Publisher_t *gimbal_pub;            // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;           // cmd控制消息订阅者
Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

// static BMI088Instance *bmi088; // 云台IMU
void GimbalInit(void) {

    IMU_data_rad = GetInsData(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
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
                .angle_PID =
                    {
                        .Kp = 35.0f,
                        .Ki = 1.50f,
                        .Kd = 0.005f,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_DerivativeFilter),
                        .CoefA         = 0.01,
                        .CoefB         = 0.1,
                        .MaxOut        = 100.0f,
                        .IntegralLimit = 33.0f,
                    },
                .speed_PID =
                    {
                        .Kp = 80.0f, // 50
                        .Ki = 5.0f,  // 200
                        .Kd = 0.03f,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_DerivativeFilter),
                        .IntegralLimit = 3000,
                        .MaxOut        = 15000,
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
                        .Kp       = 4.0f, // 8
                        .Ki       = 1.50f,
                        .Kd       = 0.005f,
                        .DeadBand = 0.0,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_DerivativeFilter | PID_ChangingIntegrationRate),
                        .CoefA         = 0.01,
                        .CoefB         = 0.1,
                        .IntegralLimit = 33,
                        .MaxOut        = 50,
                    },
                .speed_PID =
                    {
                        .Kp = 40.0f, // 50
                        .Ki = 5.0f,  // 200
                        .Kd = 0.03f,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_DerivativeFilter),
                        .IntegralLimit = 1000,
                        .MaxOut        = 5000,
                    },
                .yaw2_motor = (void *)yaw2_motor,
                // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
                .other_speed_feedback_ptr = &IMU_data_rad->Gyro[2],
                .speed_feedforward_ptr    = &yaw_ff->final_out,
                .angle_zero_point_offset  = YAW1_ZEROPOINT_ECD,
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
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
    // // PITCH
    // FeedForward_Init_Config_s pitch_ff_config = {
    //     .K1      = 300,
    //     .K2      = 0,
    //     .RC      = 0.05,
    //     .max_out = 1000,
    // };
    // pitch_ff                         = FeedForwardInit(&pitch_ff_config);
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {.can_handle = &hcan1, .tx_id = 0x01, .rx_id = 0x00},
        .controller_param_init_config =
            {
                .angle_PID =
                    {
                        .Kp = 39.0f, // 20
                        .Ki = 0.0,   // 17
                        .Kd = 2.0f,  // 2
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement |
                                                PID_ChangingIntegrationRate),
                        .CoefA         = 0.01,
                        .CoefB         = 0.1,
                        .IntegralLimit = 10,
                        .MaxOut        = 5,
                    },
                .speed_PID =
                    {
                        .Kp = 180.0f, // 150
                        .Ki = 0.0f,   // 30
                        .Kd = 3,      // 3
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement | PID_OutputFilter),
                        .Output_LPF_RC = 0.008,
                        .IntegralLimit = 30,
                        .MaxOut        = 500,
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
                .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
                .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            },
        .motor_type = DMJ4310,
    };
    pitch_motor = DMMotorInit(&pitch_config);
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动

    // FeedForwardSetRef(pitch_ff, &pitch_motor->motor_controller.pid_ref);
    // FeedForwardSetRef(yaw_ff, &yaw_motor->motor_controller.pid_ref);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
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
        LKMotorStop(yaw1_motor);
        DJIMotorStop(yaw2_motor);
        DMMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        LKMotorEnable(yaw1_motor);
        DJIMotorEnable(yaw2_motor);
        DMMotorEnable(pitch_motor);
        // RMDMotorEnable(pitch_motor);
        LKMotorChangeFeed(yaw1_motor, ANGLE_LOOP, MOTOR_FEED);
        LKMotorChangeFeed(yaw1_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw2_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, ANGLE_LOOP, OTHER_FEED);

        // RMDMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        // RMDMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        // //        DMMotorSetRef(yaw_motor, 0); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // LKMotorSetRef(yaw1_motor, 0);
        DJIMotorSetRef(yaw2_motor, gimbal_cmd_recv.yaw);
        LKMotorSetRef(yaw1_motor, 0);
        // DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); //
        // yaw和pitch会在robot_cmd中处理好多圈和单圈 RMDMotorSetRef(pitch_motor,
        // gimbal_cmd_recv.pitch);

        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)

        LKMotorEnable(yaw1_motor);
        DJIMotorEnable(yaw2_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw2_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw2_motor, ANGLE_LOOP, OTHER_FEED);
        LKMotorChangeFeed(yaw1_motor, ANGLE_LOOP, MOTOR_FEED);
        LKMotorChangeFeed(yaw1_motor, SPEED_LOOP, MOTOR_FEED);
        // RMDMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        // RMDMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw2_motor, gimbal_cmd_recv.yaw);
        LKMotorSetRef(yaw1_motor, 0);
        // yaw和pitch会在robot_cmd中处理好多圈和单圈 RMDMotorSetRef(pitch_motor,
        // gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }
    DMMotorStop(pitch_motor);
    // LKMotorSetRef(yaw_motor, 10.0F);

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    // gimbal_feedback_data.gimbal_imu_data              = *IMU_data_rad;
    gimbal_feedback_data.yaw2_motor_single_round_angle = DJIMotorGetAngle(yaw2_motor);
    gimbal_feedback_data.yaw1_motor_single_round_angle = LKMotorGetAngle(yaw1_motor);

    // gimbal_feedback_data.pitch_ecd_angle              = pitch_motor->measure.total_angle;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
