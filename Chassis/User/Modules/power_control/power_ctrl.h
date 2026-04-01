#ifndef __POWER_CTRL_H
#define __POWER_CTRL_H

#include "stdint.h"
#define STEER 0
#define WHEEL 1

typedef struct {

    float *I_cmd;
    float *w_aps;

} power_input_data_t;

typedef struct {

    float K1;
    float K2;
    float K_torque;
    float P_const;

} power_arg_t;

typedef struct {
    power_input_data_t power_input_data;
    power_arg_t arg;
    float power;
    float scaled_give_power;
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
    motor_power_t motor[4][2];
    float total_power;
    float steer_power;
    float wheel_power;
    float max_power;
    float steer_power_ratio; // 舵向电机总功率占比 (例如 0.6 表示舵向电机占总功率的60%)
} PowerControlInstance;

PowerControlInstance *ChassisPowerInit(power_control_init_t *config);
void ChassisPowerRatioSet(PowerControlInstance *ins, float steer_ratio);
void ChassisMaxPowerSet(PowerControlInstance *ins, float power);
void ChassisPowerControl(PowerControlInstance *ins);

#endif
