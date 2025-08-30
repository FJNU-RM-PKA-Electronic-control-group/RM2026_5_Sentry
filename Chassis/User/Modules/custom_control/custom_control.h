#ifndef __CUSTOM_CONTROL_H
#define __CUSTOM_CONTROL_H

#include "main.h"
#include "robot_def.h"

// 用于自定义控制器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

//按键状态
#define KEY_0_UP ((uint16_t)1 << 0)
#define KEY_0_MID ((uint16_t)1 << 1)
#define KEY_0_DOWN ((uint16_t)1 << 2)
#define KEY_1_UP ((uint16_t)1 << 3)
#define KEY_1_MID ((uint16_t)1 << 4)
#define KEY_1_DOWN ((uint16_t)1 << 5)

//按键选择
#define Key_0_is_up(key) ((key & KEY_0_UP) ? 1 : 0)
#define Key_0_is_mid(key) ((key & KEY_0_MID) ? 1 : 0)
#define Key_0_is_down(key) ((key & KEY_0_DOWN) ? 1 : 0)
#define Key_1_is_up(key) ((key & KEY_1_UP) ? 1 : 0)
#define Key_1_is_mid(key) ((key & KEY_1_MID) ? 1 : 0)
#define Key_1_is_down(key) ((key & KEY_1_DOWN) ? 1 : 0)





custom_info_t *CustomControlInit(UART_HandleTypeDef *custom_usart_handle,Custom_ctrl *custom_ctrl);
void GetCustomAngle(custom_info_t *custom_info,Custom_ctrl *custom_ctrl);
void CustomReast(custom_info_t *custom_info,Custom_ctrl *custom_ctrl);
























#endif
