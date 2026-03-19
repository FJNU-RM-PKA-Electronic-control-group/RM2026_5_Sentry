#ifndef __VISION_TASK_H
#define __VISION_TASK_H

#include "robot_def.h"
#include "stdbool.h"
#include "stdint.h"

typedef enum
{
    AIM_RED         = 0, // 红方
    AIM_BLUE        = 1, // 蓝方
    SMALL_RUNE_RED  = 2,
    SMALL_RUNE_BLUE = 3,
    BIG_RUNE_RED    = 4,
    BIG_RUNE_BLUE   = 5,

} aim_mod_e;

#pragma pack(1)

typedef __packed struct {
    uint8_t header;
    // 视觉
    uint8_t fire_advice;
    float pitch;
    float yaw;
    float distance;
    // 导航;
    float linear_x;
    float linear_y;
    float angular_z;
    Sentry_State_e state;
    uint8_t NC[3]; // 占位符
    uint8_t check_bit;
    uint8_t tail_byte;

} xPC_Data;

typedef __packed struct {
    uint8_t header;
    // 视觉
    uint8_t mode;
    float roll;
    float pitch;
    float yaw;
    // 导航
    float offset;
    uint8_t NC[12]; // 占位符
    uint8_t check_bit;
    uint8_t tail_byte;

} TxPC_Data;

#pragma pack()
// typedef __packed struct {
//     uint8_t frame_header;
//     uint8_t NC; // 占位符
//     float vx;
//     float vy;
//     float wz;
//     bool friction_wheel_scan;
//     uint8_t frame_tail;

// } xGuidance_Data;

typedef __packed struct {
    uint8_t header;
    Enemy_Color_e mode;
    float roll;
    float pitch;
    float yaw;
    uint8_t check_bit;
    uint8_t tail_byte;

} data_to_boardA_t;

void PC_read_data(uint8_t *data);
void VisionInit(void);
void VisionTask(float roll, float pitch, float yaw);

#endif
