#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"
#include "robot_def.h"

/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
referee_info_t *RefereeTaskInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief 在referee task之前调用,添加在freertos.c中
 *
 */
void MyUIInit(void);

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UITask(void);

#endif // REFEREE_H
