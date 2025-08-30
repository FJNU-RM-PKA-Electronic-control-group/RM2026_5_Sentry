#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void RobotarmInit(void);

/**
 * @brief 云台任务
 * 
 */
void RobotarmTask(void);

#endif // GIMBAL_H
