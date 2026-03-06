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

typedef __packed struct {
    uint8_t header;
    uint8_t fire_advice;
    float pitch;
    float yaw;
    float distance;
    uint8_t check_bit;
    uint8_t tail_byte;

} xPC_Data;

typedef __packed struct {
    uint8_t frame_header;
    uint8_t NC; // 占位符
    float vx;
    float vy;
    float wz;
    bool friction_wheel_scan;
    uint8_t frame_tail;

} xGuidance_Data;

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
