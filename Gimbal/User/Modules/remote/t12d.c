#include "T12D.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "gimbal_referee.h"
#include "referee_task.h"
#include "remote_control.h"
#include "stdlib.h"
#include "string.h"

// #define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小//dbus协议通道数据
#define T12D_CONTROL_FRAME_SIZE 25u // 遥控器接收的buffer大小//wbus协议通道数据

#define T12DRC_CH_VALUE_MIN    ((uint16_t)326)
#define T12DRC_CH_VALUE_OFFSET ((uint16_t)992)
#define T12DRC_CH_VALUE_MAX    ((uint16_t)1659)
#define T12DRC_CH_VALUE_VALID  (T12DRC_CH_VALUE_MAX - T12DRC_CH_VALUE_OFFSET)
#define T12DRC_CH_VALUE_SPAN   (T12DRC_CH_VALUE_MAX - T12DRC_CH_VALUE_MIN)
// 遥控器数据

/**
 * @brief t12d遥控器数据格式化 回中型:[mid-valid, mid+valid] → [-RC_HALF, +RC_HALF](越界回中)
 *
 * @param raw   输入
 * @return int16_t -500
 */
__STATIC_INLINE int16_t T12DScaleBipolar(uint16_t raw) {
    return RCScaleBipolar(raw, T12DRC_CH_VALUE_OFFSET, T12DRC_CH_VALUE_VALID);
}
/**
 * @brief t12d遥控器数据格式化行程型:[lo, lo+span] → [0, RC_SCALE](越界钳位)
 *
 * @param raw   输入
 * @return int16_t
 */
__STATIC_INLINE int16_t T12DScaleUnipolar(uint16_t raw) {
    return RCScaleUnipolar(raw, T12DRC_CH_VALUE_MIN, T12DRC_CH_VALUE_SPAN);
}

/**
 * @brief 拨杆解析
 *
 * @param   raw
 * @return  Sw_Mode_e
 */
__STATIC_INLINE Sw_Mode_e T12DSwitchModeSet(uint16_t raw) {
    if (raw < 660)
        return RC_SW_UP;  // 标称 326
    if (raw < 1326)
        return RC_SW_MID; // 标称 992

    return RC_SW_DOWN;    // 标称 1659
}

/**
 * @brief T12D 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 * @return 1头校验+尾校验，校验成功 0头校验+尾校验，校验失败
 */
void RCDecodeT12D(const uint8_t *sbus_buf, RC_ctrl_t *rc_ptr) {

    //-------------------------sbus接收-------------------------------------------
    if (sbus_buf[0] != 0x0f || sbus_buf[24] != 0x00) // 头校验+尾校验
    {
        rc_ptr->rc_state = 0;
        return;
    }
    uint16_t ch[16]; // sbus协议通道数据
    ch[0]  = (((sbus_buf[1] >> 0) | (sbus_buf[2] << 8)) & 0x07ff);
    ch[1]  = (((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07ff);
    ch[2]  = (((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) | (sbus_buf[5] << 10)) & 0x07ff);
    ch[3]  = (((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07ff);
    ch[4]  = (((sbus_buf[6] >> 4) | (sbus_buf[7] << 4)) & 0x07ff);
    ch[5]  = (((sbus_buf[7] >> 7) | (sbus_buf[8] << 1) | (sbus_buf[9] << 9)) & 0x07ff);
    ch[6]  = (((sbus_buf[9] >> 2) | (sbus_buf[10] << 6)) & 0x07ff);
    ch[7]  = (((sbus_buf[10] >> 5) | (sbus_buf[11] << 3)) & 0x07ff);
    ch[8]  = (((sbus_buf[13] >> 3) | (sbus_buf[14] << 5)) & 0x07ff);
    ch[9]  = (((sbus_buf[12] >> 0) | (sbus_buf[13] << 8)) & 0x07ff);
    ch[10] = (((sbus_buf[14] >> 6) | (sbus_buf[15] << 2) | (sbus_buf[16] << 10)) & 0x07ff);
    ch[11] = (((sbus_buf[16] >> 1) | (sbus_buf[17] << 7)) & 0x07ff);
    ch[12] = (((sbus_buf[17] >> 4) | (sbus_buf[18] << 4)) & 0x07ff);
    ch[13] = (((sbus_buf[18] >> 7) | (sbus_buf[19] << 1) | (sbus_buf[20] << 9)) & 0x07ff);
    ch[14] = (((sbus_buf[20] >> 2) | (sbus_buf[21] << 6)) & 0x07ff);
    ch[15] = (((sbus_buf[21] >> 5) | (sbus_buf[22] << 3)) & 0x07ff);

    //------------------摇杆（沿用原来的控制方式）----------------------------------

    // 摇杆,直接解算时减去偏置,通道可能需要根据RTT输出结果和实际摇杆映射关系调整
    rc_ptr->rc.rocker_r_ = T12DScaleBipolar(ch[0]);
    rc_ptr->rc.rocker_r1 = -T12DScaleBipolar(ch[1]);
    rc_ptr->rc.rocker_l_ = T12DScaleBipolar(ch[3]);
    rc_ptr->rc.rocker_l1 = -T12DScaleBipolar(ch[2]);

    //------------------四个拨轮----------------------------------
    rc_ptr->rc.vr_a = T12DScaleUnipolar(ch[10]);
    rc_ptr->rc.vr_b = T12DScaleUnipolar(ch[11]);
    rc_ptr->rc.vr_c = T12DScaleUnipolar(ch[8]);
    rc_ptr->rc.vr_d = T12DScaleUnipolar(ch[9]);
    // RectifyRCjoystick(); // 保证摇杆、拨轮的值在范围内
    //------------------四个开关----------------------------------
    rc_ptr->rc.sw_a = T12DSwitchModeSet(ch[4]);
    rc_ptr->rc.sw_b = T12DSwitchModeSet(ch[5]);
    rc_ptr->rc.sw_c = T12DSwitchModeSet(ch[6]);
    rc_ptr->rc.sw_d = T12DSwitchModeSet(ch[7]);

    rc_ptr->recv_type = RCTYPE_T12D;
    rc_ptr->rc_state  = 1;
}
