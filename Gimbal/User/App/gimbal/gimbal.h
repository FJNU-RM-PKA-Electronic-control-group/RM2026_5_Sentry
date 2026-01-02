#ifndef GIMBAL_H
#define GIMBAL_H

// 大yaw状态机枚举
typedef enum
{
    YAW1_NOMOVE,
    YAW1_TURNLEFT,
    YAW1_TURNRIGHT,

} Yaw1_mode_e;

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
void GimbalInit(void);

/**
 * @brief 云台任务
 *
 */
void GimbalTask(void);

#endif // GIMBAL_H
