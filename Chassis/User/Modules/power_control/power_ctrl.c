#include "math.h"
#include "motor_def.h"
#include "power_ctrl.h"
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"

#define CHASSISPOWER_COMPENSATION         0.02
#define CHASSISPOWER_ERR_THRESHOLD        500.0
#define CHASSISPOWER_RESERVED_TOTAL_POWER 54.0
#define CHASSISPOWER_RESERVED_POWER       8.0

float DEBUG_total_power;
PowerControlInstance *local_ins;

// #define CMD_RAW_MAX         16384.0f
// #define WHEEL_CURRENT_A_MAX 20.0f

// static __inline float WheelCmdRawToCurrentA(float cmd_raw) {
//     return cmd_raw / CMD_RAW_MAX * WHEEL_CURRENT_A_MAX;
// }

// static __inline float WheelCurrentAToCmdRaw(float current_a) {
//     return current_a / WHEEL_CURRENT_A_MAX * CMD_RAW_MAX;
// }

// #define STEER_CURRENT_A_MAX 3.0f // 舵向电机电流范围 ±3A

// static __inline float SteerCurrentRawToAmp(float current_raw) {
//     return current_raw / CMD_RAW_MAX * STEER_CURRENT_A_MAX;
// }

// static __inline float SteerCurrentAmpToRaw(float current_a) {
//     return current_a / STEER_CURRENT_A_MAX * CMD_RAW_MAX;
// }·

static __inline void MotorPowerCalculate(motor_power_t *motor, Motor_Power_Predict_Mode_e mode) {
    motor->real_current =
        fabsf(*motor->power_input_data.I_cmd) / motor->arg.real_current_conversion;
    float w_aps = fabsf(*(motor->power_input_data.w_aps));
    float w_rpm = fabsf(w_aps / RPM_2_ANGLE_PER_SEC);
    float current;
    float *output_ptr;
    switch (mode) {
    case MOTORPOWER_PREDICT_DEFAULT:
        current    = fabsf(motor->real_current);
        output_ptr = &motor->predict_power;
        break;

    case MOTORPOWER_PREDOCT_LIMITED:
        current    = fabsf(motor->I_out) / motor->arg.real_current_conversion;
        output_ptr = &motor->managed_power;
        break;
    }
    // clang-format off
        float power =  (
            motor->arg.k0 +
            motor->arg.k1 * current +
            motor->arg.k2 * w_rpm +
            motor->arg.k3 * current * w_rpm +
            motor->arg.k4 * current * current +
            motor->arg.k5 * w_rpm * w_rpm);//*power_sign;
    // clang-format on
    *output_ptr = power;
    // switch (mode) {
    // case MOTORPOWER_PREDICT_DEFAULT:
    //     motor->predict_power = power;
    //     break;

    // case MOTORPOWER_PREDOCT_LIMITED:
    //     motor->managed_power = power;
    //     break;
    // }
}

static __inline void PowerAllocateByErr(motor_power_t *motor, float total_power_limit) {
    float power_limit_compensated = total_power_limit * (1 - CHASSISPOWER_COMPENSATION);
    float motor_err_abs[4];
    for (size_t i = 0; i < 4; i++) {
        motor_err_abs[i] = fabsf(*motor[i].power_input_data.pid_err);
    }
    // 功率过小，不分配
    if (power_limit_compensated <= 1e-9) {
        for (size_t i = 0; i < 4; i++) {
            motor[i].power_limit = 0;
        }
        return;
    }
    const float total_err =
        motor_err_abs[0] + motor_err_abs[1] + motor_err_abs[2] + motor_err_abs[3];
    // err不大，直接均分
    if (total_err <= CHASSISPOWER_ERR_THRESHOLD) {
        float power_managed = power_limit_compensated / 4;
        for (size_t i = 0; i < 4; i++) {
            motor[i].power_limit = power_managed;
        }
        return;
    }
    // 按err值加权分配功率，且预留一部分功率以防止分配过少导致无法克服摩擦等问题
    if (power_limit_compensated < CHASSISPOWER_RESERVED_TOTAL_POWER) {
        for (size_t i = 0; i < 4; i++) {
            const float ratio    = motor_err_abs[i] / total_err;
            motor[i].power_limit = ratio * power_limit_compensated;
        }
    }
    else {
        for (size_t i = 0; i < 4; i++) {
            const float ratio = motor_err_abs[i] / total_err;
            motor[i].power_limit =
                ratio * (power_limit_compensated - 4 * CHASSISPOWER_RESERVED_POWER) +
                CHASSISPOWER_RESERVED_POWER;
        }
    }
}

static void MotorPowerLimiter(motor_power_t *motor) {
    if (motor->power_input_data.I_cmd == NULL)
        return;
    power_arg_t arg   = motor->arg;
    float power_limit = motor->power_limit;
    float current_req = motor->real_current;
    float speed_fdb   = fabsf(*motor->power_input_data.w_aps / RPM_2_ANGLE_PER_SEC);
    float K           = 0.0f; // 电流衰减系数
    if (motor->predict_power <= motor->power_limit) {
        // 未超限
        power_limit  = motor->predict_power;
        K            = 1.0f;
        motor->I_out = (*motor->power_input_data.I_cmd);
        MotorPowerCalculate(motor, MOTORPOWER_PREDOCT_LIMITED);
        return;
    }

    // 计算功率方程系数
    float a     = arg.k4 * current_req * current_req;
    float b     = (arg.k1 + arg.k3 * speed_fdb) * current_req;
    float c     = arg.k0 + arg.k2 * speed_fdb + arg.k5 * speed_fdb * speed_fdb - power_limit;
    float delta = (b * b - 4 * a * c);
    if (fabsf(a) < 1e-9) {
        if (fabsf(b) <= 1e-9) {
            // a和b都很小，退化为常数方程,此时无论如何控制都无法影响功率
            if (c <= 1e-9) {
                K = 1.0f; // c也很小，说明功率本来就不大，虽然无法通过控制电流来调整功率，但也不需要调整
            }
            else {
                // a、b很小但c不小，无法满足功率限制，直接将输出电流设为0
                K = 0.0f;
            }
        }
        else {
            // 退化为一次方程
            K = -c / b;
            K = float_constrain(K, 0.0f, 1.0f); // 衰减系数限幅
        }
        motor->I_out = (*motor->power_input_data.I_cmd) * K;
        MotorPowerCalculate(motor, MOTORPOWER_PREDOCT_LIMITED);
        return;
    }
    if (delta < 0) {
        // 无实数解，无法满足功率限制，直接将输出电流设为0
        K = 0.0f;
    }
    else if (delta < 1e-9) {
        // 近似为delta = 0
        K = -b / (2 * a);
        K = float_constrain(K, 0.0f, 1.0f);
    }
    else {
        // delta大于零，使用求根公式
        float k1 = (-b + sqrtf(delta)) / (2 * a);
        float k2 = (-b - sqrtf(delta)) / (2 * a);
        if ((k1 >= 0.0 && k1 <= 1.0) && (k2 >= 0.0 && k2 <= 1.0)) {
            // 双根都合法，取较大者
            K = (k1 > k2) ? k1 : k2;
        }
        else if ((k1 > 0.0 && k1 < 1.0) && (k2 > 1.0 || k2 < 0.0)) {
            // 只有k1合法
            K = k1;
        }
        else if ((k1 < 0.0 || k1 > 1.0) && (k2 > 0.0 && k2 < 1.0)) {
            // 只有k2合法
            K = k2;
        }
        else {
            // 双根都不合法，无法满足功率限制，直接将输出电流设为0
            K = 0.0f;
        }
    }
    K            = float_constrain(K, 0.0f, 1.0f); // 衰减系数限幅
    motor->I_out = (*motor->power_input_data.I_cmd) * K;
    MotorPowerCalculate(motor, MOTORPOWER_PREDOCT_LIMITED);
    return;
}

static __inline void PowerSWAllocate(PowerControlInstance *ins) {
    float servo_rate = ins->steer_power_ratio;
    if (servo_rate > 1 || servo_rate < 0) {
        // 不正常，禁止分配
        return;
    }
    float steer_max_power = ins->max_power * servo_rate;

    if (ins->steer_power >= steer_max_power) {
        ins->steer_limited_power = steer_max_power;
    }
    else if (ins->steer_power < steer_max_power) {
        ins->steer_limited_power = ins->steer_power;
    }
    ins->wheel_limited_power = ins->max_power - ins->steer_limited_power;
}

static void ChassisPowerAllocate(PowerControlInstance *ins) {
    PowerSWAllocate(ins);
    PowerAllocateByErr(ins->motor[STEER], ins->steer_limited_power);
    PowerAllocateByErr(ins->motor[WHEEL], ins->wheel_limited_power);
}

static __inline void ChassisPowerCalculate(PowerControlInstance *ins) {
    ins->steer_power = 0;
    ins->wheel_power = 0;
    for (uint8_t i = 0; i < 4; i++) {
        MotorPowerCalculate(&ins->motor[STEER][i], MOTORPOWER_PREDICT_DEFAULT);
        MotorPowerCalculate(&ins->motor[WHEEL][i], MOTORPOWER_PREDICT_DEFAULT);
        ins->steer_power += fabsf(ins->motor[STEER][i].predict_power);
        ins->wheel_power += fabsf(ins->motor[WHEEL][i].predict_power);
    }
    ins->total_power = ins->steer_power + ins->wheel_power;
}

static void ChassisPowerLimiter(PowerControlInstance *ins) {
    for (size_t i = 0; i < 4; i++) {
        MotorPowerLimiter(&ins->motor[STEER][i]);
        MotorPowerLimiter(&ins->motor[WHEEL][i]);
    }
}

PowerControlInstance *ChassisPowerInit(power_control_init_t *config) {
    PowerControlInstance *ins = (PowerControlInstance *)malloc(sizeof(PowerControlInstance));
    memset(ins, 0, sizeof(PowerControlInstance));

    for (uint8_t i = 0; i < 4; i++) {

        ins->motor[STEER][i].power_input_data = config->motor_steer_data[i].power_input_data;
        ins->motor[STEER][i].arg              = config->motor_steer_data[i].arg;

        ins->motor[WHEEL][i].power_input_data = config->motor_wheel_data[i].power_input_data;
        ins->motor[WHEEL][i].arg              = config->motor_wheel_data[i].arg;
    }

    ins->steer_power_ratio = 0.5f; // 默认分配50%
    local_ins              = ins;
    local_ins->init_flag   = true;
    return ins;
}

void ChassisPowerRatioSet(PowerControlInstance *ins, float steer_ratio) {
    if (ins) {
        if (steer_ratio < 0.0f)
            steer_ratio = 0.0f;
        if (steer_ratio > 1.0f)
            steer_ratio = 1.0f;
        ins->steer_power_ratio = steer_ratio;
    }
}

void ChassisMaxPowerSet(PowerControlInstance *ins, float power) {
    if (ins)
        ins->max_power = power;
}

void ChassisPowerControl(void) {
    if (!local_ins->init_flag)
        return;
    ChassisPowerCalculate(local_ins);
    ChassisPowerAllocate(local_ins);
    ChassisPowerLimiter(local_ins);
    DEBUG_total_power = local_ins->total_power;
}
