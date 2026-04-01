#include "math.h"
#include "motor_def.h"
#include "power_ctrl.h"
#include "stdlib.h"
#include "string.h"

float DEBUG_total_power;

#define WHEEL_CMD_RAW_MAX   16384.0f
#define WHEEL_CURRENT_A_MAX 10.0f

static __inline float WheelCmdRawToCurrentA(float cmd_raw) {
    return cmd_raw / WHEEL_CMD_RAW_MAX * WHEEL_CURRENT_A_MAX;
}

static __inline float WheelCurrentAToCmdRaw(float current_a) {
    return current_a / WHEEL_CURRENT_A_MAX * WHEEL_CMD_RAW_MAX;
}

#define STEER_CURRENT_A_MAX 3.0f // 舵向电机电流范围 ±3A

static __inline float SteerCurrentRawToAmp(float current_raw) {
    return current_raw / 16384.0f * STEER_CURRENT_A_MAX;
}

static __inline float SteerCurrentAmpToRaw(float current_a) {
    return current_a / STEER_CURRENT_A_MAX * 16384.0f;
}

static __inline float PowerCalculate(motor_power_t *motor, float i_cmd) {
    float w_aps   = *(motor->power_input_data.w_aps);
    float w_radps = w_aps * DEGREE_2_RAD;
    // clang-format off
    return (motor->arg.K1 * i_cmd * i_cmd) + 
           (motor->arg.K2 * w_radps * w_radps) +
           (motor->arg.K_torque * i_cmd * w_radps) + 
            motor->arg.P_const;
    // clang-format on
}

PowerControlInstance *ChassisPowerInit(power_control_init_t *config) {
    PowerControlInstance *ins = (PowerControlInstance *)malloc(sizeof(PowerControlInstance));
    memset(ins, 0, sizeof(PowerControlInstance));

    for (uint8_t i = 0; i < 4; i++) {

        ins->motor[i][STEER].power_input_data = config->motor_steer_data[i].power_input_data;
        ins->motor[i][STEER].arg              = config->motor_steer_data[i].arg;

        ins->motor[i][WHEEL].power_input_data = config->motor_wheel_data[i].power_input_data;
        ins->motor[i][WHEEL].arg              = config->motor_wheel_data[i].arg;
    }

    ins->steer_power_ratio = 0.5f; // 默认分配50%

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

void ChassisPowerControl(PowerControlInstance *ins) {
    if (!ins)
        return;

    float steer_req_total = 0;
    float wheel_req_total = 0;

    // 1. 计算每个电机的需求功率
    for (uint8_t i = 0; i < 4; i++) {
        float steer_i_cmd_raw = *(ins->motor[i][STEER].power_input_data.I_cmd);
        float wheel_i_cmd_raw = *(ins->motor[i][WHEEL].power_input_data.I_cmd);

        ins->motor[i][STEER].power =
            PowerCalculate(&ins->motor[i][STEER], SteerCurrentRawToAmp(steer_i_cmd_raw));
        steer_req_total += ins->motor[i][STEER].power;

        ins->motor[i][WHEEL].power =
            PowerCalculate(&ins->motor[i][WHEEL], WheelCmdRawToCurrentA(wheel_i_cmd_raw));
        wheel_req_total += ins->motor[i][WHEEL].power;
    }

    ins->steer_power = steer_req_total;
    ins->wheel_power = wheel_req_total;
    ins->total_power = steer_req_total + wheel_req_total;

    float max_steer_power = ins->max_power * ins->steer_power_ratio;
    float max_wheel_power = ins->max_power * (1.0f - ins->steer_power_ratio);

    // 2. 功率分配与电流重算 (分为舵向和轮向两个组别)
    for (uint8_t type = STEER; type <= WHEEL; type++) {
        float req_total     = (type == STEER) ? steer_req_total : wheel_req_total;
        float max_allocated = (type == STEER) ? max_steer_power : max_wheel_power;

        if (req_total > max_allocated) {
            float power_scale = max_allocated / req_total;
            for (uint8_t i = 0; i < 4; i++) {
                ins->motor[i][type].scaled_give_power = ins->motor[i][type].power * power_scale;

                if (ins->motor[i][type].scaled_give_power < 0) {
                    ins->motor[i][type].I_out = *(ins->motor[i][type].power_input_data.I_cmd);
                    continue;
                }

                float w_aps    = *(ins->motor[i][type].power_input_data.w_aps);
                float w_radps  = w_aps * DEGREE_2_RAD;
                float k1       = ins->motor[i][type].arg.K1;
                float k2       = ins->motor[i][type].arg.K2;
                float k_torque = ins->motor[i][type].arg.K_torque;
                float p_const  = ins->motor[i][type].arg.P_const;

                // 求解一元二次方程: K1 * I^2 + K_torque * w * I + (K2 * w^2 + P_const -
                // scaled_give_power) = 0
                float a = k1;
                float b = k_torque * w_radps;
                float c = k2 * w_radps * w_radps + p_const - ins->motor[i][type].scaled_give_power;
                float delta = b * b - 4 * a * c;

                if (delta < 0) {
                    ins->motor[i][type].I_out = 0;
                }
                else if (delta > 0) {
                    float temp1     = (-b + sqrt(delta)) / (2 * a);
                    float temp2     = (-b - sqrt(delta)) / (2 * a);
                    float i_cmd_raw = *(ins->motor[i][type].power_input_data.I_cmd);
                    float i_cmd     = (type == WHEEL) ? WheelCmdRawToCurrentA(i_cmd_raw)
                                                      : SteerCurrentRawToAmp(i_cmd_raw);
                    if (fabs(temp1 - i_cmd) < fabs(temp2 - i_cmd))
                        ins->motor[i][type].I_out = (type == WHEEL) ? WheelCurrentAToCmdRaw(temp1)
                                                                    : SteerCurrentAmpToRaw(temp1);
                    else
                        ins->motor[i][type].I_out = (type == WHEEL) ? WheelCurrentAToCmdRaw(temp2)
                                                                    : SteerCurrentAmpToRaw(temp2);
                }
                else {
                    float i_out               = (-b) / (2 * a);
                    ins->motor[i][type].I_out = (type == WHEEL) ? WheelCurrentAToCmdRaw(i_out)
                                                                : SteerCurrentAmpToRaw(i_out);
                }
            }
        }
        else {
            // 功率未超限，直接输出电流指令
            for (uint8_t i = 0; i < 4; i++) {
                ins->motor[i][type].I_out = *(ins->motor[i][type].power_input_data.I_cmd);
            }
        }
    }
    DEBUG_total_power = ins->total_power;
}
