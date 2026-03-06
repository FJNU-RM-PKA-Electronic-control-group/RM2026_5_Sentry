#include "bsp_dwt.h"
#include "bsp_flash.h"
#include "buzzer.h"
#include "calibrate.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "message_center.h"
#include "mpu6500.h"
#include "robot_def.h"
#include "stdio.h"
#include "stdlib.h"
#include "tim.h"

cali_data_t cali_data_save; // �����Ǻ͹ؽ�У׼����
cail_mods state = CAIL_OFF;
BuzzzerInstance *buzzer_cali;

static void IMUCalibrateControl(void) {
    mpu_offset_cali();
    AlarmSetStatus(buzzer_cali, ALARM_OFF);
    state = CAIL_OFF;

    // flash_erase_address(CALIBRATE_DATA_ADDR,1);
    // flash_write_single_address(CALIBRATE_DATA_ADDR, (uint32_t *)&cali_data_save,
    // sizeof(cali_data_t)/4);//д��flash
}
void CalibrateTask(RC_ctrl_t *cail_rc_data) {
    switch (state) {
    case CAIL_OFF:
        // CailbrateOffControl(cail_rc_data);
        break;
    case CAIL_READY:
        // 	CalibrateReadyControl(cail_rc_data);
        break;
    case CAIL_JOINTMOTOR:
        // JointCalibrateControl();
        break;
    case CAIL_IMU:
        IMUCalibrateControl();
        break;
    default:
        break;
    }
}
