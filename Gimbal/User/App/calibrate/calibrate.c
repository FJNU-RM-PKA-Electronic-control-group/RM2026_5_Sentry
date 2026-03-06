#include "bsp_dwt.h"
#include "bsp_flash.h"
#include "buzzer.h"
#include "calibrate.h"
#include "message_center.h"
#include "robot_def.h"
#include "stdlib.h"
#include "tim.h"


#include "rmd_motor.h"

static Subscriber_t *gimbal_cali_sub;  // 云台反馈信息订阅者
Gimbal_Upload_Data_s gimbal_cali_data; // 从云台获取的反馈信息
cali_data_t cali_data_save;            // 云台校准数据
cail_mods state = CAIL_OFF;

static void GimbalCalibrateControl(void);

cali_data_t *CalibrateInit(void) {
    gimbal_cali_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    memset(&cali_data_save, 0, sizeof(cali_data_t));

    AlarmSetStatus(ALARM_OFF);

    flash_read(CALIBRATE_DATA_ADDR, (uint32_t *)&cali_data_save, sizeof(cali_data_t) / 4);
    if (cali_data_save.cali_status != 1) // 若无校准数据则先校准
    {
        cali_data_save.max_pitch  = MAX_PITCH_ALTERNATE;
        cali_data_save.min_pitch  = MIN_PITCH_ALTERNATE;
        cali_data_save.zero_pitch = ZERO_PITCH_ALTERNATE;
        cali_data_save.zero_yaw   = ZERO_YAW_ALTERNATE;
        return &cali_data_save;
    }
    else
        return &cali_data_save; // 有数据则返回数据
}

static void CailbrateOffControl(RC_ctrl_t *cail_rc_data) {
    static uint32_t cali_cmd_time;
    static float calibrating_time;
    if (cail_rc_data[TEMP].rc.rocker_l_ > RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_l1 < -RC_CALI_VALUE_HOLE &&
        cail_rc_data[TEMP].rc.rocker_r_ < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r1 < -RC_CALI_VALUE_HOLE) // 下内八开启校准
    {
        if (!cali_cmd_time)
            DWT_GetDeltaT(&cali_cmd_time);
        calibrating_time += DWT_GetDeltaT(&cali_cmd_time);
    }
    else {
        cali_cmd_time    = 0;
        calibrating_time = 0;
    }
    if (calibrating_time > 2) {
        state = CAIL_READY;
    }
}

static void CalibrateReadyControl(RC_ctrl_t *cail_rc_data) {
    static uint32_t cali_cmd_time;
    static float calibrating_time;

    if (cail_rc_data[TEMP].rc.rocker_l_ < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_l1 > RC_CALI_VALUE_HOLE &&
        cail_rc_data[TEMP].rc.rocker_r_ > RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r1 > RC_CALI_VALUE_HOLE) // 上外八校准云台
    {
        if (!cali_cmd_time)
            DWT_GetDeltaT(&cali_cmd_time);
        calibrating_time += DWT_GetDeltaT(&cali_cmd_time);
        if (calibrating_time > 2) // 持续时间大于2s
        {
            state = CAIL_GIMBAL;
        }
    }
    else if (cail_rc_data[TEMP].rc.rocker_l_ < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_l1 < -RC_CALI_VALUE_HOLE &&
             cail_rc_data[TEMP].rc.rocker_r_ > RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r1 < -RC_CALI_VALUE_HOLE) // 下外八校准IMU
    {
        if (!cali_cmd_time)
            DWT_GetDeltaT(&cali_cmd_time);
        calibrating_time += DWT_GetDeltaT(&cali_cmd_time);
        if (calibrating_time > 2) // 持续时间大于2s
        {
            state = CAIL_IMU;
        }
    }
    else {
        cali_cmd_time    = 0;
        calibrating_time = 0;
    }
}

static void GimbalCalibrateControl(void) {
    static float max_pitch_temp = 0, min_pitch_temp = 0;
    static float cali_time = 0;      // 校准持续时间
    static uint32_t cali_start_time; // 校准开始时间戳

    AlarmSetOctave(0.5);
    SubGetMessage(gimbal_cali_sub, &gimbal_cali_data);
    if (cali_time == 0) {
        DWT_GetDeltaT(&cali_start_time);
        //		RMDMotorZeroCali();
    }
    if (!max_pitch_temp && !min_pitch_temp) // 赋初始值
    {
        max_pitch_temp = gimbal_cali_data.pitch_ecd_angle;
        min_pitch_temp = gimbal_cali_data.pitch_ecd_angle;
    }
    if (cali_time < 10) // 校准10s
    {
        cali_time += DWT_GetDeltaT(&cali_start_time);
        if (max_pitch_temp < gimbal_cali_data.pitch_ecd_angle) // 记录最大pitch角度
        {
            max_pitch_temp = gimbal_cali_data.pitch_ecd_angle;
        }
        if (min_pitch_temp > gimbal_cali_data.pitch_ecd_angle) // 记录最小pitch角度
        {
            min_pitch_temp = gimbal_cali_data.pitch_ecd_angle;
        }
    }
    else {
        cali_data_save.max_pitch   = max_pitch_temp;
        cali_data_save.min_pitch   = min_pitch_temp;
        cali_data_save.zero_pitch  = (max_pitch_temp + min_pitch_temp) / 2;
        cali_data_save.zero_yaw    = gimbal_cali_data.yaw2_motor_single_round_angle;
        cali_data_save.cali_status = 1;
        flash_erase_address(CALIBRATE_DATA_ADDR, 1);
        flash_write_single_address(CALIBRATE_DATA_ADDR, (uint32_t *)&cali_data_save, sizeof(cali_data_t) / 4); // 写入flash
        AlarmSetStatus(ALARM_OFF);
        state = CAIL_OFF;
    }
}

void CalibrateTask(RC_ctrl_t *cail_rc_data) {
    switch (state) {
    case CAIL_OFF:
        CailbrateOffControl(cail_rc_data);
        break;
    case CAIL_READY:
        CalibrateReadyControl(cail_rc_data);
        break;
    case CAIL_GIMBAL:
        GimbalCalibrateControl();
        break;
    case CAIL_IMU:
        break;
    default:
        break;
    }
}
