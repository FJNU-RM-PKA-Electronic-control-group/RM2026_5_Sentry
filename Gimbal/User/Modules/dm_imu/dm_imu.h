#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "bsp_can.h"
#include "stm32f4xx_hal.h"

#define ACCEL_CAN_MAX  (58.8f)
#define ACCEL_CAN_MIN  (-58.8f)
#define GYRO_CAN_MAX   (34.88f)
#define GYRO_CAN_MIN   (-34.88f)
#define PITCH_CAN_MAX  (90.0f)
#define PITCH_CAN_MIN  (-90.0f)
#define ROLL_CAN_MAX   (180.0f)
#define ROLL_CAN_MIN   (-180.0f)
#define YAW_CAN_MAX    (180.0f)
#define YAW_CAN_MIN    (-180.0f)
#define TEMP_MIN       (0.0f)
#define TEMP_MAX       (60.0f)
#define Quaternion_MIN (-1.0f)
#define Quaternion_MAX (1.0f)

#define CMD_READ  0
#define CMD_WRITE 1

typedef enum
{
    COM_USB = 0,
    COM_RS485,
    COM_CAN,
    COM_VOFA

} imu_com_port_e;

typedef enum
{
    CAN_BAUD_1M = 0,
    CAN_BAUD_500K,
    CAN_BAUD_400K,
    CAN_BAUD_250K,
    CAN_BAUD_200K,
    CAN_BAUD_100K,
    CAN_BAUD_50K,
    CAN_BAUD_25K

} imu_baudrate_e;

typedef enum
{
    REBOOT_IMU = 0,
    ACCEL_DATA,
    GYRO_DATA,
    EULER_DATA,
    QUAT_DATA,
    SET_ZERO,
    ACCEL_CALI,
    GYRO_CALI,
    MAG_CALI,
    CHANGE_COM,
    SET_DELAY,
    CHANGE_ACTIVE,
    SET_BAUD,
    SET_CAN_ID,
    SET_MST_ID,
    DATA_OUTPUT_SELECTION,
    SAVE_PARAM      = 254,
    RESTORE_SETTING = 255
} reg_id_e;

typedef enum
{
    REQUEST_MODE = 0,
    ACTIVE_MODE  = 1,

} send_mode_e;

typedef struct {
    float pitch;
    float roll;
    float yaw;

    float gyro[3];
    float accel[3];

    float q[4];

    float cur_temp;

} imu_t;

typedef struct {
    CAN_Init_Config_s can_init_config;
    imu_baudrate_e baudrate;
    send_mode_e send_mode;

} DMIMU_Init_Config_s;

typedef struct {
    CANInstance *imu_can_instance;
    imu_com_port_e com_port;
    imu_baudrate_e baudrate;
    send_mode_e send_mode;
    imu_t imu;

} DMIMUInstance;

void imu_restore_settings();
void imu_save_parameters();
void imu_set_can_id(uint8_t can_id);
void imu_set_mst_id(uint8_t mst_id);
void imu_set_baud(DMIMUInstance *dm_imu, imu_baudrate_e baud);
void imu_set_mode(DMIMUInstance *dm_imu, send_mode_e mode);
void imu_change_com_port(DMIMUInstance *dm_imu, imu_com_port_e port);
void imu_set_active_mode_delay(uint32_t delay);
void imu_accel_calibration();
void imu_gyro_calibration();
void imu_reboot();

void DM_IMU_Init(DMIMU_Init_Config_s *config);
imu_t *GetIMUData(void);
void IMU_UpdateData(CANInstance *instance);
// void IMU_RequestData(CAN_HandleTypeDef* hcan,uint16_t can_id,uint8_t reg);
/**
 * @brief 请求函数，使用请求模式时需要配套使用
 *
 */
void imutask(void);
#endif
