#ifndef __CALIBRATE_H
#define __CALIBRATE_H

#include "stdint.h"
#include "remote_control.h"

#define CALIBRATE_DATA_ADDR ADDR_FLASH_SECTOR_11
#define RC_CALI_VALUE_HOLE 600        //遥控器外八或者内八 阈值判定， 遥控器摇杆最大是 660 只要大于630 就认为到最大

//备用校准数据
#define MAX_PITCH_ALTERNATE -15686.39f
#define MIN_PITCH_ALTERNATE -15747.39f
#define ZERO_PITCH_ALTERNATE 	-15717.39f
#define ZERO_YAW_ALTERNATE 	-5.762939f

typedef enum
{
	CAIL_OFF = 0,
	CAIL_READY,
	CAIL_GIMBAL,
	CAIL_IMU,
}  cail_mods;

typedef struct
{
	uint8_t cali_status;
	float max_pitch;
	float min_pitch;
	float zero_pitch;
	float zero_yaw;
	
} cali_data_t;

cali_data_t *CalibrateInit(void);
void CalibrateTask(RC_ctrl_t *cail_rc_data);

#endif
