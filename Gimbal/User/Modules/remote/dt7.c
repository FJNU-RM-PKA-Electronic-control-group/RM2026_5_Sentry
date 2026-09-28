#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dt7.h"
#include "gimbal_referee.h"
#include "referee_task.h"
#include "remote_control.h"
#include "stdlib.h"
#include "string.h"

#define DT7_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小

// 检查接收值是否出错
#define DT7RC_CH_VALUE_MIN    ((uint16_t)364)
#define DT7RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define DT7RC_CH_VALUE_MAX    ((uint16_t)1684)
#define DT7RC_CH_VALUE_VALID  (DT7RC_CH_VALUE_MAX - DT7RC_CH_VALUE_OFFSET)
#define DT7RC_CH_VALUE_SPAN   (DT7RC_CH_VALUE_MAX - DT7RC_CH_VALUE_MIN)

// // 遥控器数据
// RC_ctrl_t rc_ctrl[2]; //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断

/**
 * @brief t12d遥控器数据格式化 回中型:[mid-valid, mid+valid] → [-RC_HALF, +RC_HALF](越界回中)
 *
 * @param raw   输入
 * @return int16_t -500
 */
__STATIC_INLINE int16_t DT7ScaleBipolar(uint16_t raw) {
    return RCScaleBipolar(raw, DT7RC_CH_VALUE_OFFSET, DT7RC_CH_VALUE_VALID);
}
/**
 * @brief t12d遥控器数据格式化行程型:[lo, lo+span] → [0, RC_SCALE](越界钳位)
 *
 * @param raw   输入
 * @return int16_t
 */
__STATIC_INLINE int16_t DT7ScaleUnipolar(uint16_t raw) {
    return RCScaleUnipolar(raw, DT7RC_CH_VALUE_MIN, DT7RC_CH_VALUE_SPAN);
}

/**
 * @brief DT7 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
void RCDecodeDT7(const uint8_t *sbus_buf, RC_ctrl_t *rc_ptr) {
    // 摇杆,直接解算时减去偏置
    rc_ptr->rc.rocker_r_ =
        DT7ScaleBipolar((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff);                 //!< Channel 0
    rc_ptr->rc.rocker_r1 =
        DT7ScaleBipolar(((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff);          //!< Channel 1
    rc_ptr->rc.rocker_l_ = DT7ScaleBipolar(
        ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff);    //!< Channel 2
    rc_ptr->rc.rocker_l1 =
        DT7ScaleBipolar(((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff);          //!< Channel 3
    rc_ptr->rc.vr_a = DT7ScaleBipolar((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF); // 左侧拨轮
    // 开关,0左1右
    rc_ptr->rc.sw_a = ((sbus_buf[5] >> 4) & 0x0003);      //!< Switch right
    rc_ptr->rc.sw_b = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ptr->mouse.x       = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ptr->mouse.y       = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ptr->mouse.press_l = sbus_buf[12];                       //!< Mouse Left Is Press ?
    rc_ptr->mouse.press_r = sbus_buf[13];                       //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&rc_ptr->key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ptr->key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ptr->key[KEY_PRESS_WITH_CTRL] = rc_ptr->key[KEY_PRESS];
    else
        memset(&rc_ptr->key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ptr->key[KEY_PRESS].shift) // shift键按下
        rc_ptr->key[KEY_PRESS_WITH_SHIFT] = rc_ptr->key[KEY_PRESS];
    else
        memset(&rc_ptr->key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    // uint16_t key_now   = rc_ptr->key[KEY_PRESS].keys,            // 当前按键是否按下
    //     key_last       = rc_ctrl[LAST].key[KEY_PRESS].keys,            // 上一次按键是否按下
    //     key_with_ctrl  = rc_ptr->key[KEY_PRESS_WITH_CTRL].keys,  // 当前ctrl组合键是否按下
    //     key_with_shift = rc_ptr->key[KEY_PRESS_WITH_SHIFT].keys, //  当前shift组合键是否按下
    //     key_last_with_ctrl =
    //         rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,               //
    //         上一次ctrl组合键是否按下
    //     key_last_with_shift =
    //         rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys;              //
    //         上一次shift组合键是否按下

    // for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++) {
    //     if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
    //         continue;
    //     //
    //     如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
    //     if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
    //         rc_ptr->key_count[KEY_PRESS][i]++;
    //     // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
    //     if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
    //         rc_ptr->key_count[KEY_PRESS_WITH_CTRL][i]++;
    //     // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
    //     if ((key_with_shift & j) && !(key_last_with_shift & j))
    //         rc_ptr->key_count[KEY_PRESS_WITH_SHIFT][i]++;
    // }
    rc_ptr->recv_type = RCTYPE_DT7;
    rc_ptr->rc_state  = 1;
}
