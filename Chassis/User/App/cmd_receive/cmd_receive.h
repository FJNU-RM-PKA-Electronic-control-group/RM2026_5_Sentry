#ifndef __CMD_RECEIVE_H
#define __CMD_RECEIVE_H

#include "robot_def.h"
#include "stdint.h"

#pragma pack(1)
// typedef __packed struct {
//     float vx;           // ǰ�������ٶ�
//     float vy;           // ���Ʒ����ٶ�
//     float wz;           // ��ת�ٶ�
//     float offset_angle; // ���̺�������λ�õļн�
//     chassis_mode_e chassis_mode;
// } chassis_data_rev;

#pragma pack()

void CmdRevInit(void);
void CmdRevTask(void);

#endif
