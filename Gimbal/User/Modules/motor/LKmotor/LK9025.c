#include "INS_task.h"
#include "LK9025.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "daemon.h"
#include "dji_motor.h"
#include "fliter.h"
#include "robot_def.h"
#include "stdlib.h"
#include "user_lib.h"

static uint8_t idx;
static LKMotorInstance *lkmotor_instance[LK_MOTOR_MX_CNT] = {NULL};
static attitude_t *__LK_IMU_data;
//  后续考虑兼容单电机和多电机指令.

/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void LKMotorDecode(CANInstance *_instance) {
    LKMotorInstance *motor =
        (LKMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    LKMotor_Measure_t *measure = &motor->measure;
    uint8_t *rx_buff           = _instance->rx_buff;

    DaemonReload(motor->daemon); // 喂狗
    measure->feed_dt = DWT_GetDeltaT(&measure->feed_dwt_cnt);

    measure->last_ecd = measure->ecd;
    measure->ecd      = (uint16_t)((rx_buff[7] << 8) | rx_buff[6]);

    measure->angle_single_round = ECD_ANGLE_COEF_LK * measure->ecd;

    measure->speed_rads =
        (1 - SPEED_SMOOTH_COEF) * measure->speed_rads +
        DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(rx_buff[5] << 8 | rx_buff[4]));

    measure->real_current = (1 - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rx_buff[3] << 8 | rx_buff[2]));

    measure->temperature = rx_buff[1];

    if (measure->ecd - measure->last_ecd > 32768)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -32768)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void LKMotorLostCallback(void *motor_ptr) {
    LKMotorInstance *motor = (LKMotorInstance *)motor_ptr;
    LOGWARNING("[LKMotor] motor lost, id: %d", motor->motor_can_ins->tx_id);
}

LKMotorInstance *LKMotorInit(Motor_Init_Config_s *config) {
    LKMotorInstance *motor;
    motor = (LKMotorInstance *)malloc(sizeof(LKMotorInstance));
    memset(motor, 0, sizeof(LKMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;
    motor->current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->zero_point_offset        = config->controller_param_init_config.angle_zero_point_offset;
    motor->yaw2_ins_ptr             = config->controller_param_init_config.yaw2_motor;

    config->can_init_config.id                  = motor;
    config->can_init_config.can_module_callback = LKMotorDecode;
    config->can_init_config.rx_id               = 0x140 + config->can_init_config.tx_id;
    config->can_init_config.tx_id               = config->can_init_config.tx_id + 0x140;

    motor->motor_can_ins = CANRegister(&config->can_init_config);

    Low_Pass_Fliter_Init_Config_s lp_fliter = {
        .RC = 0.5f,
    };

    LowPassFliterInit(&motor->angle_output_lpfliter, &lp_fliter);

    LKMotorStop(motor);
    if (!idx) {
        __LK_IMU_data = GetInsData();
    }
    DWT_GetDeltaT(&motor->measure.feed_dwt_cnt);
    lkmotor_instance[idx++] = motor;

    Daemon_Init_Config_s daemon_config = {
        .callback     = LKMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 10, // 50ms
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}
float debug_measure[3];
/* 第一个电机的can instance用于发送数据,向其tx_buff填充数据 */
void LKMotorControl() {
    float pid_measure, pid_ref, pid_err;
    int16_t set;
    LKMotorInstance *motor;
    LKMotor_Measure_t *measure;
    Motor_Control_Setting_s *setting;

    for (size_t i = 0; i < idx; ++i) {
        motor   = lkmotor_instance[i];
        measure = &motor->measure;
        setting = &motor->motor_settings;
        pid_ref = motor->pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP) {
            switch (setting->pid_mode) {
            case MOTOR_PID_MUTILOOP:
                if (setting->angle_feedback_source == OTHER_FEED)
                    pid_measure = *motor->other_angle_feedback_ptr;
                else if (setting->angle_feedback_source ==
                         MOTOR_AND_IMU_FEED) // 大yaw特供，纯堆屎山
                    pid_measure = (__LK_IMU_data->YawTotalAngle -
                                   DJIMotorGetAngle((DJIMotorInstance *)motor->yaw2_ins_ptr));

                else
                    pid_measure = measure->total_angle;

                pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
                break;
            case MOTOR_PID_SINGLELOOP:
                if (setting->angle_feedback_source == OTHER_FEED)
                    pid_measure = *motor->other_angle_feedback_ptr;
                else if (setting->angle_feedback_source ==
                         MOTOR_AND_IMU_FEED) { // 大yaw特供，纯堆屎山
                    pid_measure = AngleFormatfp(-FliterCalcutale(
                        (FliterInstance *)&motor->angle_output_lpfliter,
                        NearbyErrFormat(DJIMotorGetAngle(motor->yaw2_ins_ptr), 180)));
                }
                else
                    pid_measure =
                        AngleFormatfp(measure->angle_single_round - motor->zero_point_offset);
                debug_measure[0] = pid_measure;
                debug_measure[1] =
                    NearbyErrFormat(DJIMotorGetAngle((DJIMotorInstance *)motor->yaw2_ins_ptr), 180);
                debug_measure[2] = __LK_IMU_data->Yaw;
                pid_err          = -NearbyErrFormat((pid_ref - pid_measure), 180.0f);
                pid_ref          = PIDCalculate(&motor->angle_PID, pid_err, 0.0f);
                break;
            default:
                break;
            }
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->speed_feedforward_ptr;
        }

        if ((setting->close_loop_type & SPEED_LOOP) &&
            setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_speed_feedback_ptr;
            else
                pid_measure = measure->speed_rads;
            pid_ref = PIDCalculate(&motor->speed_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
                pid_ref += *motor->current_feedforward_ptr;
        }

        if (setting->close_loop_type & CURRENT_LOOP) {
            pid_ref = PIDCalculate(&motor->current_PID, measure->real_current, pid_ref);
        }

        set = (int16_t)pid_ref;

        // 编辑发送缓冲区，0xA1为转矩电流控制命令
        motor->motor_can_ins->tx_buff[0] = 0xA1;
        memcpy(motor->motor_can_ins->tx_buff + 4, &set, sizeof(uint16_t));

        // 若该电机处于停止状态,直接将发送buff置零
        if (motor->stop_flag == MOTOR_STOP) {
            memset(motor->motor_can_ins->tx_buff + 4, 0, sizeof(uint16_t));
        }
    }

    if (idx) // 如果有电机注册了
        CANTransmit(motor->motor_can_ins, 0.2);
}

void LKMotorStop(LKMotorInstance *motor) { motor->stop_flag = MOTOR_STOP; }

void LKMotorEnable(LKMotorInstance *motor) { motor->stop_flag = MOTOR_ENALBED; }

void LKMotorSetRef(LKMotorInstance *motor, float ref) { motor->pid_ref = ref; }

void LKMotorChangeFeed(LKMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type) {
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR(
            "[lk_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

float LKMotorGetAngle(LKMotorInstance *motor) {
    if (motor->motor_settings.pid_mode == MOTOR_PID_MUTILOOP)
        return motor->measure.total_angle;
    else if (motor->motor_settings.pid_mode == MOTOR_PID_SINGLELOOP)
        return AngleFormatfp(motor->measure.angle_single_round - motor->zero_point_offset);
}

float LKMotorGetRad(LKMotorInstance *motor) {
    float angle = LKMotorGetAngle(motor);
    float rad   = DegToRad(angle);
    return RadFormat(rad);
}

uint8_t LKMotorIsOnline(LKMotorInstance *motor) { return DaemonIsOnline(motor->daemon); }
