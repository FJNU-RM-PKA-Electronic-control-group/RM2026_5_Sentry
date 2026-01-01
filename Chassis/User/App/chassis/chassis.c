/**
  ****************************(C) COPYRIGHT 2022 YUELU****************************
  * @file      chassis.c\h
  * @brief
  搴曠洏搴旂敤,璐熻矗鎺ユ敹robot_cmd鐨勬帶鍒跺懡浠ゅ苟鏍规嵁鍛戒护杩涜?岃繍鍔ㄥ?﹁В绠?,寰楀埌杈撳嚭娉ㄦ剰
  *
  搴曠洏閲囧彇鍙虫墜绯?,瀵逛簬骞抽潰瑙嗗浘,搴曠洏绾靛悜杩愬姩鐨勬?ｅ墠鏂逛负x姝ｆ柟鍚?;妯?鍚戣繍鍔ㄧ殑鍙充晶涓?
  *            y姝ｆ柟鍚戙€?
  *            鍏蜂綋鎺у埗娴佺▼瑙乧hassis.md
  *
  * @todo      1.琛ュ厖chassis_task鐨刾id杩愮畻閮ㄥ垎
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0                     YUELU              1. 瀹屾垚
  *  V2.0.0                      qhp               1.閫傞厤瀵瑰簲鏈哄櫒浜?
  *  V2.1.0                     nan_97             1.閫傞厤宸ョ▼鏈哄櫒浜?
  *
  @verbatim
  ================================================================================

  ================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 YUELU****************************
  */
#include "stm32f427xx.h"

#include "arm_math.h"
#include "bsp_dwt.h"
#include "chassis.h"
#include "controller.h"
#include "dji_motor.h"
#include "fliter.h"
#include "message_center.h"
#include "referee_UI.h"
#include "referee_task.h"
#include "robot_def.h"
#include "time.h"
#include "user_lib.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE  (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)    // 半轮距
#define PERIMETER_WHEEL  (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define Square(x)        (x) * (x)

#define VECTOR 0

#define LF                    0
#define RF                    1
#define LB                    3
#define RB                    2
#define WHEEL_COUNT           4
#define VECTORMOTOR_OFFSET_LF 227.107758
#define VECTORMOTOR_OFFSET_RF 17.1824951
#define VECTORMOTOR_OFFSET_LB 72.6410828
#define VECTORMOTOR_OFFSET_RB 44.0768356

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令 // !ONE_BOARD
Chassis_Ctrl_Cmd_s chassis_cmd_recv;                // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

// static referee_info_t* referee_data; // 用于获取裁判系统的数据
// static Referee_Interactive_info_t ui_data; //
// UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

// static SuperCapInstance *cap;                                       // 超级电容
DJIMotorInstance *speed_motor[4], *vector_motor[4]; // left right forward back
PIDInstance follow_pid;
DJIMotorInstance DEBUG_CHASSIS;

// 用于正解算的变量
PIDInstance Force_PID_vx, Force_PID_vy, Force_PID_wz;
LPFliterInstance vx_fliter, vy_fliter, wz_fliter;
float chassis_vx_FK, chassis_vy_FK, chassis_wz_FK;
static float chassis_vx_feedback[4], chassis_vy_feedback[4];
float vx_output, vy_output, wz_output;
float force_wz, force_vx, force_vy;
float force_ff[4];
/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
float chassis_vx, chassis_vy; // 将云台系的速度投影到底盘
float vt[4], anglet[4];       // 底盘速度解算后的临时输出,待进行限幅
float vector_feedback[4];
float debug_anglet[4];
void ChassisInit(void) {
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_speed_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config =
            {
                .speed_PID =
                    {
                        .Kp            = 4,    // 4.5
                        .Ki            = 0,    // 0
                        .Kd            = 0.01, // 0
                        .IntegralLimit = 3000,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement),
                        .MaxOut = 12000,
                    },
                .current_PID =
                    {
                        .Kp            = 0.5, // 0.4
                        .Ki            = 0,   // 0
                        .Kd            = 0,
                        .IntegralLimit = 3000,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement),
                        .MaxOut = 15000,
                    },
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type       = SPEED_LOOP,
                .close_loop_type       = (Closeloop_Type_e)(SPEED_LOOP | CURRENT_LOOP),
                .pid_mode              = MOTOR_PID_MUTILOOP,
                .feedforward_flag      = CURRENT_FEEDFORWARD,
            },
        .motor_type = M3508,
    };
    Motor_Init_Config_s chassis_vector_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config =
            {
                .angle_PID =
                    {
                        .Kp            = 50,  // 4.5
                        .Ki            = 0,   // 0
                        .Kd            = 0.1, // 0
                        .IntegralLimit = 3000,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement),
                        .MaxOut = 12000,
                    },
                .speed_PID =
                    {
                        .Kp            = 25, // 0.4
                        .Ki            = 0,  // 0
                        .Kd            = 0.3,
                        .IntegralLimit = 3000,
                        .Improve =
                            (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                                PID_Derivative_On_Measurement),
                        .MaxOut = 15000,
                    },
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type       = ANGLE_LOOP,
                .close_loop_type       = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
                .pid_mode              = MOTOR_PID_SINGLELOOP,
            },
        .motor_type = GM6020,
    };
    //  @todo:
    //  当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_speed_motor_config.can_init_config.tx_id = 1;
    // chassis_speed_motor_config.can_init_config.rx_id = 1;
    chassis_speed_motor_config.controller_param_init_config.current_feedforward_ptr = &force_ff[LF];
    chassis_speed_motor_config.controller_setting_init_config.motor_reverse_flag =
        MOTOR_DIRECTION_NORMAL;
    speed_motor[LF] = DJIMotorInit(&chassis_speed_motor_config);

    chassis_speed_motor_config.can_init_config.tx_id = 2;
    // chassis_speed_motor_config.can_init_config.rx_id = 2;
    chassis_speed_motor_config.controller_param_init_config.current_feedforward_ptr = &force_ff[RF];
    chassis_speed_motor_config.controller_setting_init_config.motor_reverse_flag =
        MOTOR_DIRECTION_NORMAL;
    speed_motor[RF] = DJIMotorInit(&chassis_speed_motor_config);

    chassis_speed_motor_config.can_init_config.tx_id = 4;
    // chassis_speed_motor_config.can_init_config.rx_id = 4;
    chassis_speed_motor_config.controller_param_init_config.current_feedforward_ptr = &force_ff[LB];
    chassis_speed_motor_config.controller_setting_init_config.motor_reverse_flag =
        MOTOR_DIRECTION_NORMAL;
    speed_motor[LB] = DJIMotorInit(&chassis_speed_motor_config);

    chassis_speed_motor_config.can_init_config.tx_id = 3;
    // chassis_speed_motor_config.can_init_config.rx_id = 3;
    chassis_speed_motor_config.controller_param_init_config.current_feedforward_ptr = &force_ff[RB];
    chassis_speed_motor_config.controller_setting_init_config.motor_reverse_flag =
        MOTOR_DIRECTION_NORMAL;
    speed_motor[RB] = DJIMotorInit(&chassis_speed_motor_config);

    // 舵向电机
    chassis_vector_motor_config.can_init_config.tx_id = 1;
    // chassis_vector_motor_config.can_init_config.rx_id = 1;
    chassis_vector_motor_config.controller_setting_init_config.motor_reverse_flag =
        MOTOR_DIRECTION_NORMAL;
    chassis_vector_motor_config.controller_param_init_config.other_angle_feedback_ptr =
        &vector_feedback[LF];
    chassis_vector_motor_config.controller_param_init_config.angle_zero_point_offset =
        VECTORMOTOR_OFFSET_LF;
    vector_motor[LF] = DJIMotorInit(&chassis_vector_motor_config);

    chassis_vector_motor_config.can_init_config.tx_id = 2;
    // chassis_vector_motor_config.can_init_config.rx_id = 2;
    chassis_vector_motor_config.controller_setting_init_config.motor_reverse_flag =
        MOTOR_DIRECTION_NORMAL;
    chassis_vector_motor_config.controller_param_init_config.other_angle_feedback_ptr =
        &vector_feedback[RF];
    chassis_vector_motor_config.controller_param_init_config.angle_zero_point_offset =
        VECTORMOTOR_OFFSET_RF;
    vector_motor[RF] = DJIMotorInit(&chassis_vector_motor_config);

    chassis_vector_motor_config.can_init_config.tx_id = 4;
    // chassis_vector_motor_config.can_init_config.rx_id = 4;
    chassis_vector_motor_config.controller_param_init_config.other_angle_feedback_ptr =
        &vector_feedback[LB];
    chassis_vector_motor_config.controller_setting_init_config.motor_reverse_flag =
        MOTOR_DIRECTION_NORMAL;
    chassis_vector_motor_config.controller_param_init_config.angle_zero_point_offset =
        VECTORMOTOR_OFFSET_LB;
    vector_motor[LB] = DJIMotorInit(&chassis_vector_motor_config);

    chassis_vector_motor_config.can_init_config.tx_id = 3;
    // chassis_vector_motor_config.can_init_config.rx_id = 3;
    chassis_vector_motor_config.controller_param_init_config.other_angle_feedback_ptr =
        &vector_feedback[RB];
    chassis_vector_motor_config.controller_setting_init_config.motor_reverse_flag =
        MOTOR_DIRECTION_NORMAL;
    chassis_vector_motor_config.controller_param_init_config.angle_zero_point_offset =
        VECTORMOTOR_OFFSET_RB;
    vector_motor[RB] = DJIMotorInit(&chassis_vector_motor_config);

    //    referee_data = UITaskInit(&huart6,&ui_data); // 裁判系统初始化,会同时初始化UI

    //    SuperCap_Init_Config_s cap_conf = {
    //        .can_config = {
    //            .can_handle = &hcan2,
    //            .tx_id = 0x302, // 超级电容默认接收id
    //            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    //        }};
    //    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    PID_Init_Config_s pid_config = {
        .Kp            = 0.9, // 0.4
        .Ki            = 0,   // 0
        .Kd            = 0.0f,
        .IntegralLimit = 3000,
        .Improve       = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                       PID_Derivative_On_Measurement),
        .DeadBand      = 7.0,
        .MaxOut        = 20,
    };
    PIDInit(&follow_pid, &pid_config);

    PID_Init_Config_s force_pid_config = {
        .Kp            = 1, // 0.4
        .Ki            = 0, // 0
        .Kd            = 0.05,
        .IntegralLimit = 3000,
        .DeadBand      = 0,
        .Improve       = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit |
                                       PID_Derivative_On_Measurement),
        .MaxOut        = 150,
    };
    PIDInit(&Force_PID_vx, &force_pid_config);
    PIDInit(&Force_PID_vy, &force_pid_config);
    PIDInit(&Force_PID_wz, &force_pid_config);

    Low_Pass_Fliter_Init_Config_s lp_fliter_config;
    lp_fliter_config.RC = 0.05;
    LowPassFliterInit(&vx_fliter, &lp_fliter_config);
    LowPassFliterInit(&vy_fliter, &lp_fliter_config);
    LowPassFliterInit(&wz_fliter, &lp_fliter_config);

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

#define LF_CENTER                                                                                  \
    ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) *      \
     DEGREE_2_RAD)
#define RF_CENTER                                                                                  \
    ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) *      \
     DEGREE_2_RAD)
#define LB_CENTER                                                                                  \
    ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) *      \
     DEGREE_2_RAD)
#define RB_CENTER                                                                                  \
    ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) *      \
     DEGREE_2_RAD)
/**
 * @brief 舵轮运动学正解
 *
 * @todo 看看测试后要不要加个低通滤波
 */
static void SteeringWheelFK(void) {
    // 获取计算所需反馈值
    for (size_t i = 0; i < 4; i++) {
        float speed = speed_motor[i]->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL; // aps转mmps
        float angle = RadFormat(DJIMotorGetAngle(vector_motor[i]) * DEGREE_2_RAD);
        float wheel_sin_theta  = arm_sin_f32(angle);
        float wheel_cos_theta  = arm_cos_f32(angle);
        debug_anglet[i]        = angle;
        chassis_vx_feedback[i] = speed * wheel_cos_theta;
        chassis_vy_feedback[i] = speed * wheel_sin_theta;
    }
    // 为低通滤波做点小准备，若测试后发现不需要可删除
    chassis_vx_feedback[RF] *= -1;
    chassis_vx_feedback[RB] *= -1;
    chassis_vy_feedback[RF] *= -1;
    chassis_vy_feedback[RB] *= -1;

    // clang-format off
    // 解算部分，应该视情况进行加权平均
    //注意，因为我希望底盘初始化时3508屁股朝内部，所以RF，RB两个轮向电机速度反馈是负的
    vx_output = (chassis_vx_feedback[LF] + chassis_vx_feedback[RF] + 
                chassis_vx_feedback[LB] + chassis_vx_feedback[RB]) / 40;

    vy_output = (chassis_vy_feedback[LF] + chassis_vy_feedback[RF] + 
                chassis_vy_feedback[LB] + chassis_vy_feedback[RB]) / 40;

    wz_output = (chassis_vx_feedback[LF] + chassis_vx_feedback[LB] - 
                chassis_vx_feedback[RB] - chassis_vx_feedback[RF] + 
                chassis_vy_feedback[LF] + chassis_vy_feedback[RB] -
                chassis_vy_feedback[RF] - chassis_vy_feedback[LB]) / 8;
    // clang-format on

    chassis_vx_FK = FliterCalcutale((FliterInstance *)&vx_fliter, vx_output);
    chassis_vy_FK = FliterCalcutale((FliterInstance *)&vy_fliter, vy_output);
    chassis_wz_FK = FliterCalcutale((FliterInstance *)&wz_fliter, wz_output);
}
/**
 * @brief 舵轮动力学逆解
 *
 */
static void SteeringWheelIK(void) {
    // 舵向电机角度解算
    float wz_projected = chassis_cmd_recv.wz * SIN45;
    // force_wz *= -SIN45;

    anglet[LF] = atan2f(chassis_vx + (wz_projected * HALF_WHEEL_BASE),
                        chassis_vy + (wz_projected * HALF_TRACK_WIDTH));
    anglet[RF] = atan2f(chassis_vx + (wz_projected * HALF_WHEEL_BASE),
                        chassis_vy - (wz_projected * HALF_TRACK_WIDTH));
    anglet[LB] = atan2f(chassis_vx - (wz_projected * HALF_WHEEL_BASE),
                        chassis_vy + (wz_projected * HALF_TRACK_WIDTH));
    anglet[RB] = atan2f(chassis_vx - (wz_projected * HALF_WHEEL_BASE),
                        chassis_vy - (wz_projected * HALF_TRACK_WIDTH));

    // 轮向电机速度解算，就是求个模长
    arm_sqrt_f32(Square(chassis_vx + (wz_projected * HALF_TRACK_WIDTH)) +
                     Square(chassis_vy + (wz_projected * HALF_WHEEL_BASE)),
                 &vt[LF]);
    arm_sqrt_f32(Square(chassis_vx + (wz_projected * HALF_TRACK_WIDTH)) +
                     Square(chassis_vy - (wz_projected * HALF_WHEEL_BASE)),
                 &vt[RF]);
    arm_sqrt_f32(Square(chassis_vx - (wz_projected * HALF_TRACK_WIDTH)) +
                     Square(chassis_vy + (wz_projected * HALF_WHEEL_BASE)),
                 &vt[LB]);
    arm_sqrt_f32(Square(chassis_vx - (wz_projected * HALF_TRACK_WIDTH)) +
                     Square(chassis_vy - (wz_projected * HALF_WHEEL_BASE)),
                 &vt[RB]);
    vt[RF] = -vt[RF];
    vt[RB] = -vt[RB];
    // if (!((chassis_vx == 0) && (chassis_vy == 0) && chassis_cmd_recv.wz == 0)) {
    //     force_vx *= 0.25;
    //     force_vy *= 0.25;
    //     force_wz *= 0.25;
    //     arm_sqrt_f32(Square(force_vx + (force_wz * HALF_TRACK_WIDTH)) +
    //                      Square(force_vy + (force_wz * HALF_WHEEL_BASE)),
    //                  &force_ff[LF]);
    //     arm_sqrt_f32(Square(force_vx + (force_wz * HALF_TRACK_WIDTH)) +
    //                      Square(force_vy - (force_wz * HALF_WHEEL_BASE)),
    //                  &force_ff[RF]);
    //     arm_sqrt_f32(Square(force_vx - (force_wz * HALF_TRACK_WIDTH)) +
    //                      Square(force_vy + (force_wz * HALF_WHEEL_BASE)),
    //                  &force_ff[LB]);
    //     arm_sqrt_f32(Square(force_vx - (force_wz * HALF_TRACK_WIDTH)) +
    //                      Square(force_vy - (force_wz * HALF_WHEEL_BASE)),
    //                  &force_ff[RB]);
    //     force_ff[RF] = -force_ff[RF];
    //     force_ff[RB] = -force_ff[RB];
    // }
    // else {
    for (size_t i = 0; i < 4; i++) {
        force_ff[i] = 0;
    }
    // }

    // 将输出值化为电机需要的单位，同时做一个舵向电机就近转向优化
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        float angle_single_round = DJIMotorGetAngle(vector_motor[i]);
        // vt[i] /= RADIUS_WHEEL; // 将线速度化为角速度
        anglet[i] /= DEGREE_2_RAD;
        anglet[i] = AngleFormatfp(anglet[i]);
        //  计算舵向电机的角度差
        float delta_angle = anglet[i] - angle_single_round;
        // 轮向电机换向，以及防打滑优化
        // 新写的待测试
        vt[i] *= arm_cos_f32(NearbyErrFormat((delta_angle), 180.0F) * DEGREE_2_RAD);

        // 舵向电机就近转向
        anglet[i] = AngleFormatfp(angle_single_round + NearbyErrFormat(delta_angle, 90.0f));
    }
}

/**
 * @brief 现在只有赋值功能，后续添加功率限制，但是估计不会在这
 *
 */
static void LimitChassisOutput(void) {
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(speed_motor[LF], vt[LF]);
    DJIMotorSetRef(speed_motor[RF], vt[RF]);
    DJIMotorSetRef(speed_motor[LB], vt[LB]);
    DJIMotorSetRef(speed_motor[RB], vt[RB]);
    if (!((chassis_vx == 0) && (chassis_vy == 0) && chassis_cmd_recv.wz == 0)) {
        DJIMotorSetRef(vector_motor[LF], anglet[LF]);
        DJIMotorSetRef(vector_motor[RF], anglet[RF]);
        DJIMotorSetRef(vector_motor[LB], anglet[LB]);
        DJIMotorSetRef(vector_motor[RB], anglet[RB]);
    }
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed(void) {
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
}

/* 机器人底盘控制核心任务 */
void ChassisTask(void) {
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    DEBUG_CHASSIS = *speed_motor[LF];

    // VectorFeedbackCheck();

    if (chassis_cmd_recv.chassis_mode ==
        CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(speed_motor[LF]);
        DJIMotorStop(speed_motor[RF]);
        DJIMotorStop(speed_motor[LB]);
        DJIMotorStop(speed_motor[RB]);

        DJIMotorStop(vector_motor[LF]);
        DJIMotorStop(vector_motor[RF]);
        DJIMotorStop(vector_motor[LB]);
        DJIMotorStop(vector_motor[RB]);
    }
    else { // 正常工作
        DJIMotorEnable(speed_motor[LF]);
        DJIMotorEnable(speed_motor[RF]);
        DJIMotorEnable(speed_motor[LB]);
        DJIMotorEnable(speed_motor[RB]);

        DJIMotorEnable(vector_motor[LF]);
        DJIMotorEnable(vector_motor[RF]);
        DJIMotorEnable(vector_motor[LB]);
        DJIMotorEnable(vector_motor[RB]);
    }
    SteeringWheelFK(); // 运动学正解算，获取底盘当前速度状态
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode) {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        chassis_cmd_recv.wz = PIDCalculate(&follow_pid, 0, -chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_SPIN: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 20;
        break;
    default:
        break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta  = arm_cos_f32(theta_format(chassis_cmd_recv.offset_angle + 45.0f) * DEGREE_2_RAD);
    sin_theta  = arm_sin_f32(theta_format(chassis_cmd_recv.offset_angle + 45.0f) * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    force_wz   = PIDCalculate(&Force_PID_wz, chassis_wz_FK, chassis_cmd_recv.wz);
    force_vx   = PIDCalculate(&Force_PID_vx, chassis_vx_FK, chassis_cmd_recv.vx);
    force_vy   = PIDCalculate(&Force_PID_vy, chassis_vy_FK, chassis_cmd_recv.vy);

    // 根据控制模式进行运动学解算,计算底盘输出
    SteeringWheelIK(); // 动力学逆解算，计算目标所需控制电流

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    // EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed =
    // referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
    // PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}
