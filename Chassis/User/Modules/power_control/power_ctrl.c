#include "math.h"
#include "power_ctrl.h"
#include "stdlib.h"
#include "string.h"

#define STEER 0
#define WHEEL 1

static __inline float PowerCalculate(motor_power_t *motor) {
    float i_cmd = *(motor->power_input_data.I_cmd);
    float w_aps = *(motor->power_input_data.w_aps);
    float w_rpm = w_aps / 6.0f;
    // clang-format off
    return (motor->arg.K1 * i_cmd * i_cmd) + 
           (motor->arg.K2 * w_rpm * w_rpm) +
           (motor->arg.K_torque * i_cmd * w_rpm) + 
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

    return ins;
}