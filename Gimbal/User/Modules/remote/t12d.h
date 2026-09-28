/**
 * @file remote_control.h
 * @author DJI 2016
 * @author modified by neozng
 * @brief  遥控器模块定义头文件
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2016 DJI corp
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */
#ifndef T12D_CONTROL_H
#define T12D_CONTROL_H

#pragma anon_unions

#include "main.h"
#include "remote_control.h"
#include "robot_def.h"
#include "usart.h"
#include <stdint.h>

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
// #define LAST 1
// #define TEMP 0

//// 获取按键操作
// #define KEY_PRESS            0
// #define KEY_STATE            1
// #define KEY_PRESS_WITH_CTRL  1
// #define KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错

/* ----------------------- RC Switch Definition----------------------------- */
typedef enum
{
    T12DRC_SW_UP   = 326, // 开关向上时的值
    T12DRC_SW_MID  = 992, // 开关中间时的值
    T12DRC_SW_DOWN = 1659 // 开关向下时的值

} Sw_Mode;

/**
 * @brief T12D 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 * @return 1头校验+尾校验，校验成功 0头校验+尾校验，校验失败
 */
void RCDecodeT12D(const uint8_t *sbus_buf, RC_ctrl_t *rc_ptr);

#endif
