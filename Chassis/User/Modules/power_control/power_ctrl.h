#ifndef __POWER_CTRL_H
#define __POWER_CTRL_H

#include "stdint.h"
#define STEER 0
#define WHEEL 1

typedef enum
{
    MOTORPOWER_PREDICT_DEFAULT,
    MOTORPOWER_PREDOCT_LIMITED,

} Motor_Power_Predict_Mode_e;

typedef struct
{

    float *I_cmd;
    float *w_aps;
    float *pid_err;

} power_input_data_t;

typedef struct {

    float32_t k0, k1, k2, k3, k4, k5;
    float32_t real_current_conversion;

} power_arg_t;

typedef struct {
    power_input_data_t power_input_data;
    power_arg_t arg;
    float real_current;
    // float feedback_power;
    float managed_power;
    float predict_power;
    float power_limit;
    float I_out;

} motor_power_t;

typedef struct {

    power_input_data_t power_input_data;
    power_arg_t arg;

} motor_power_init_t;

typedef struct {

    motor_power_init_t motor_steer_data[4];
    motor_power_init_t motor_wheel_data[4];

} power_control_init_t;

typedef struct {
    motor_power_t motor[2][4];
    float total_power;
    float steer_power;
    float wheel_power;
    float steer_limited_power;
    float wheel_limited_power;
    float max_power;
    float steer_power_ratio; // 舵向电机总功率占比 (例如 0.6 表示舵向电机占总功率的60%)
} PowerControlInstance;

PowerControlInstance *ChassisPowerInit(power_control_init_t *config);
void ChassisPowerRatioSet(PowerControlInstance *ins, float steer_ratio);
void ChassisMaxPowerSet(PowerControlInstance *ins, float power);
void ChassisPowerControl(PowerControlInstance *ins);

#endif
