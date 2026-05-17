#ifndef __VISION_TASK_H
#define __VISION_TASK_H

#include "robot_def.h"
#include "stdbool.h"
#include "stdint.h"

typedef enum
{
    AIM_RED         = 0, // �췽
    AIM_BLUE        = 1, // ����
    SMALL_RUNE_RED  = 2,
    SMALL_RUNE_BLUE = 3,
    BIG_RUNE_RED    = 4,
    BIG_RUNE_BLUE   = 5,

} aim_mod_e;

#pragma pack(1)

typedef __packed struct {
    uint8_t header;
    // �Ӿ�
    uint8_t fire_advice;
    float pitch;
    float yaw;
    float distance;
    // ����;
    float linear_x;
    float linear_y;
    float angular_z;
    Sentry_State_e state;
    uint8_t NC[3]; // ռλ��
    uint8_t check_bit;
    uint8_t tail_byte;

} xPC_Data;

typedef __packed struct {
    uint8_t header;           // 0      帧头 0xFF
    uint8_t mode;             // 1      瞄准颜色
    float roll;               // 2-5    roll
    float pitch;              // 6-10   pitch
    float yaw;                // 10-13  yaw
    float offset;             // 14-17  大yaw正方向与IMU上电时的夹角
    uint16_t game_progress;   // 18-19  比赛阶段
    uint16_t current_HP;      // 20-21  当前血量
    uint16_t bullet_rest;     // 22-23  剩余发弹量
    bool is_combating;        // 24     脱战状态，脱战置1
    bool rfid_fortress;       // 25     堡垒RFID
    bool rfid_supply;         // 26     基地补给区RFID
    uint16_t ally_outpost_HP; // 27-28  己方前哨站血量
    uint8_t NC[1];            // 29     保留位
    uint8_t check_bit;        // 30     校验位，置0
    uint8_t tail_byte;        // 31     帧尾 0x0D

} TxPC_Data;

#pragma pack()
// typedef __packed struct {
//     uint8_t frame_header;
//     uint8_t NC; // ռλ��
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
