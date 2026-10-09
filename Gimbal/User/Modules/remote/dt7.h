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
#ifndef DT7_CONTROL_H
#define DT7_CONTROL_H

#pragma anon_unions

#include "main.h"
#include "remote_control.h"
#include "robot_def.h"
#include "usart.h"
#include <stdint.h>

/* ----------------------- RC Switch Definition----------------------------- */
// #define RC_SW_UP          ((uint16_t)1) // 开关向上时的值
// #define RC_SW_MID         ((uint16_t)3) // 开关中间时的值
// #define RC_SW_DOWN        ((uint16_t)2) // 开关向下时的值
// // 三个判断开关状态的宏
// #define switch_is_down(s) (s == RC_SW_DOWN)
// #define switch_is_mid(s)  (s == RC_SW_MID)
// #define switch_is_up(s)   (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
// 对应key[x][0~16],获取对应的键;例如通过key[KEY_PRESS][Key_W]获取W键是否按下,后续改为位域后删除

// typedef enum
// {
//     RC_SW_UP   = 1, // 326, // 开关向上时的值
//     RC_SW_MID  = 2, // 992, // 开关中间时的值
//     RC_SW_DOWN = 3, // 1659 // 开关向下时的值

// } Sw_Mode_e;

/* ----------------------- Data Struct ------------------------------------- */
// // 待测试的位域结构体,可以极大提升解析速度
// typedef union {
//     struct // 用于访问键盘状态
//     {
//         uint16_t w : 1;
//         uint16_t s : 1;
//         uint16_t a : 1;
//         uint16_t d : 1;
//         uint16_t shift : 1;
//         uint16_t ctrl : 1;
//         uint16_t q : 1;
//         uint16_t e : 1;
//         uint16_t r : 1;
//         uint16_t f : 1;
//         uint16_t g : 1;
//         uint16_t z : 1;
//         uint16_t x : 1;
//         uint16_t c : 1;
//         uint16_t v : 1;
//         uint16_t b : 1;
//     };
//     uint16_t keys; // 用于memcpy而不需要进行强制类型转换
// } Key_t;

// // @todo 当前结构体嵌套过深,需要进行优化
// typedef struct {
//     struct {
//         int16_t rocker_l_; // 左摇杆水平
//         int16_t rocker_l1; // 左摇杆竖直
//         int16_t rocker_r_; // 右摇杆水平
//         int16_t rocker_r1; // 右摇杆竖直

//         uint16_t vr_a;     // 旋钮(variable_resistor),参考说明书，从左到右ABCD
//         uint16_t vr_b;
//         uint16_t vr_c;
//         uint16_t vr_d;

//         Sw_Mode_e sw_a; // 开关(switch),参考说明书，从左到右ABCD
//         Sw_Mode_e sw_b;
//         Sw_Mode_e sw_c;
//         Sw_Mode_e sw_d;

//     } rc;
//     struct {
//         int16_t x;
//         int16_t y;
//         int16_t z;
//         uint8_t press_l;
//         uint8_t press_r;
//     } mouse;

//     Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

//     uint8_t key_count[3][16];
//     uint8_t rc_state;
// } RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief DT7 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
void RCDecodeDT7(const uint8_t *sbus_buf, RC_ctrl_t *rc_ptr);

#endif
