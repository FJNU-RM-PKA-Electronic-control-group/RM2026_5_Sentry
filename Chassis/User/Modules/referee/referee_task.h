#ifndef REFEREE_H
#define REFEREE_H

#include "referee_UI.h"
#include "rm_referee.h"
#include "robot_def.h"

/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
referee_info_t *RefereeTaskInit(UART_HandleTypeDef *referee_usart_handle);

void SentryCMDInit(void);

Sentry_State_e SentryGetState(void);

void SentryRevive(bool confirm);

void SentrybuyRevive(bool confirm);
void SentrySetState(Sentry_State_e state);

void SentryBuyBullet(uint8_t count);

void SentryRemoteSupply(void);

void SentryUpdateCheck(void);
void SentryRefreshCommand(Sentry_Cmd_t sentry_cmd);
/**
 * @brief 在referee task之前调用,添加在freertos.c中
 *
 */
void MyUIInit(void);

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void SentryCMDTask(void);

#endif // REFEREE_H
