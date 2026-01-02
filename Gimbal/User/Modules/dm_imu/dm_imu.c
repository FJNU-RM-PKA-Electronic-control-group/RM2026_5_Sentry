#include "bsp_dwt.h"
#include "dm_imu.h"
#include "string.h"
DMIMUInstance *DM_IMU; // 只有一个实例
// imu_t imu;
uint32_t tick_ms = 0;
float debug111;

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max]
*内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    /* Converts a float to an unsigned int, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max]
*内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /* converts unsigned int to float, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/*******发送部分********/
static void imu_send_cmd(uint8_t reg_id, uint8_t ac, uint32_t data) {
    uint8_t buf[8] = {0xCC, reg_id, ac, 0xDD, 0, 0, 0, 0};

    // 复制数据到正确位置
    memcpy(buf + 4, &data, 4);

    // 复制到CAN发送缓冲区
    memcpy(DM_IMU->imu_can_instance->tx_buff, buf, 8);

    CANTransmit(DM_IMU->imu_can_instance, 1);
    // DM_IMU->imu_can_instance->tx_buff[0]=0xCC;
    // DM_IMU->imu_can_instance->tx_buff[1]=reg_id;
    // DM_IMU->imu_can_instance->tx_buff[2]=ac;
    // DM_IMU->imu_can_instance->tx_buff[3]=0xDD;
    // DM_IMU->imu_can_instance->tx_buff[4]=0;
    // DM_IMU->imu_can_instance->tx_buff[5]=0;
    // DM_IMU->imu_can_instance->tx_buff[6]=0;
    // DM_IMU->imu_can_instance->tx_buff[7]=0;
    // memcpy(DM_IMU->imu_can_instance->tx_buff+4,&data,4);
    //	CANTransmit(DM_IMU->imu_can_instance,1);
}

// 写寄存器
static void imu_write_reg(uint8_t reg_id, uint32_t data) { imu_send_cmd(reg_id, CMD_WRITE, data); }

// 读寄存器
static void imu_read_reg(uint8_t reg_id) { imu_send_cmd(reg_id, CMD_READ, 0); }

// 重启IMU
void imu_reboot() { imu_write_reg(REBOOT_IMU, 0); }

// 加速度计校准
void imu_accel_calibration() { imu_write_reg(ACCEL_CALI, 0); }

// 陀螺仪校准
void imu_gyro_calibration() { imu_write_reg(GYRO_CALI, 0); }

// 改变通信端口
void imu_change_com_port(DMIMUInstance *dm_imu, imu_com_port_e port) {
    dm_imu->com_port = port;
    imu_write_reg(CHANGE_COM, (uint8_t)port);
    HAL_Delay(1);
}

// 设置成主动模式延时
void imu_set_active_mode_delay(uint32_t delay) {
    imu_write_reg(SET_DELAY, delay);
    HAL_Delay(1);
}

// 设置发送模式,1:主动模式,0:请求模式
void imu_set_mode(DMIMUInstance *dm_imu, send_mode_e mode) {
    dm_imu->send_mode = mode;
    imu_write_reg(CHANGE_ACTIVE, (uint8_t)mode);
    HAL_Delay(1);
}
// //设置成主动模式
// void imu_change_to_active()
// {
// 	imu_write_reg(CHANGE_ACTIVE,1);
// }

// //设置成请求模式
// void imu_change_to_request()
// {
// 	imu_write_reg(CHANGE_ACTIVE,0);
// }

// 设置波特率
void imu_set_baud(DMIMUInstance *dm_imu, imu_baudrate_e baud) {
    dm_imu->baudrate = baud;
    imu_write_reg(SET_BAUD, (uint8_t)baud);
    HAL_Delay(1);
}

// 设置CAN ID
void imu_set_can_id(uint8_t can_id) { imu_write_reg(SET_CAN_ID, can_id); }

// 设置主站ID
void imu_set_mst_id(uint8_t mst_id) { imu_write_reg(SET_MST_ID, mst_id); }

// 保存参数
void imu_save_parameters() {
    imu_write_reg(SAVE_PARAM, 0);
    HAL_Delay(1);
}

// 恢复出厂设置
void imu_restore_settings() { imu_write_reg(RESTORE_SETTING, 0); }

//*******读取数据部分********/
// 请求加速度数据
static void imu_request_accel() { imu_read_reg(ACCEL_DATA); }
// 请求陀螺仪数据
static void imu_request_gyro() { imu_read_reg(GYRO_DATA); }
// 请求欧拉角数据
static void imu_request_euler() { imu_read_reg(EULER_DATA); }
// 请求四元数数据
static void imu_request_quat() { imu_read_reg(QUAT_DATA); }

// static void IMU_RequestData(CAN_HandleTypeDef* hcan,uint16_t can_id,uint8_t reg)
// {
// 	uint8_t cmd[4]={(uint8_t)can_id,(uint8_t)(can_id>>8),reg,0xCC};
// 	uint32_t returnBox;

// 	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0)
// 	{
// 		HAL_CAN_AddTxMessage(hcan,&imu_instance->txconf,cmd,&returnBox);
// 	}
// }

static void IMU_UpdateAccel(uint8_t *pData) {
    uint16_t accel[3];

    accel[0] = pData[3] << 8 | pData[2];
    accel[1] = pData[5] << 8 | pData[4];
    accel[2] = pData[7] << 8 | pData[6];

    DM_IMU->imu.accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    DM_IMU->imu.accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    DM_IMU->imu.accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

static void IMU_UpdateGyro(uint8_t *pData) {
    uint16_t gyro[3];

    gyro[0] = pData[3] << 8 | pData[2];
    gyro[1] = pData[5] << 8 | pData[4];
    gyro[2] = pData[7] << 8 | pData[6];

    DM_IMU->imu.gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    DM_IMU->imu.gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    DM_IMU->imu.gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

static void IMU_UpdateEuler(uint8_t *pData) {
    int euler[3];

    euler[0] = pData[3] << 8 | pData[2];
    euler[1] = pData[5] << 8 | pData[4];
    euler[2] = pData[7] << 8 | pData[6];

    DM_IMU->imu.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
    DM_IMU->imu.yaw   = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
    DM_IMU->imu.roll  = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
}

static void IMU_UpdateQuaternion(uint8_t *pData) {
    int w = pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
    int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
    int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | (pData[6] & 0xC0) >> 6;
    int z = (pData[6] & 0x3F) << 8 | pData[7];

    DM_IMU->imu.q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
    DM_IMU->imu.q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
    DM_IMU->imu.q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
    DM_IMU->imu.q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}

static void IMU_UpdateData(CANInstance *instance) {
    // 从CAN实例中获取接收的原始数据（CAN总线上收到的8字节数据）
    uint8_t *pData = instance->rx_buff;
    debug111++;

    // 以下是原解析逻辑（根据数据类型分发）
    switch (pData[0]) {
    case 1: // 加速度数据
        IMU_UpdateAccel(pData);
        break;
    case 2: // 陀螺仪角速度数据
        IMU_UpdateGyro(pData);
        break;
    case 3: // 欧拉角数据
        IMU_UpdateEuler(pData);
        break;
    case 4: // 四元数数据
        IMU_UpdateQuaternion(pData);
        break;
    }
}

static void DM_IMU_Can_Init(void) {
    CAN_Init_Config_s imu_can_config; // 配置结构体
    imu_can_config.can_handle          = &hcan2;
    imu_can_config.tx_id               = 0x01;
    imu_can_config.rx_id               = 0x11;
    imu_can_config.can_module_callback = IMU_UpdateData;
    imu_can_config.id                  = &DM_IMU;
    DM_IMU                             = (DMIMUInstance *)malloc(sizeof(DMIMUInstance));
    memset(DM_IMU, 0, sizeof(DMIMUInstance));
    DM_IMU->imu_can_instance = CANRegister(&imu_can_config); // 注册
    // DM_IMU->imu_can_instance->txconf.IDE=CAN_ID_STD;
    // DM_IMU->imu_can_instance->txconf.RTR=CAN_RTR_DATA;
    // DM_IMU->imu_can_instance->txconf.StdId=0x01;
    CANSetDLC(DM_IMU->imu_can_instance, 8);
}

// 陀螺仪初始化
void DM_IMU_Init(DMIMU_Init_Config_s *config) {
    DM_IMU = (DMIMUInstance *)malloc(sizeof(DMIMUInstance));
    memset(DM_IMU, 0, sizeof(DMIMUInstance));
    config->can_init_config.can_module_callback = IMU_UpdateData;
    DM_IMU->imu_can_instance                    = CANRegister(&config->can_init_config);
    DM_IMU->com_port                            = COM_CAN;          // 用CAN就好了
    DM_IMU->baudrate                            = config->baudrate; // 暂定
    DM_IMU->send_mode                           = config->send_mode;

    //	DWT_Delay_ms(100);
    imu_set_mode(DM_IMU, DM_IMU->send_mode);
    imu_set_baud(DM_IMU, DM_IMU->baudrate);
    imu_change_com_port(DM_IMU, DM_IMU->com_port);
    imu_set_active_mode_delay(10); // 主动模式下5ms发送一次
    // 上电先校准
    imu_accel_calibration();
    HAL_Delay(6000);
    imu_gyro_calibration();
    HAL_Delay(6000);
    imu_save_parameters();

    imu_reboot();
}

DMIMUInstance *GetIMUInstance() { return DM_IMU; }

imu_t *GetIMUData(void) { return &DM_IMU->imu; }

void imutask(void) {
    tick_ms++;

    if (tick_ms % 3 == 0) {
        imu_request_accel();
    }
    else if (tick_ms % 2 == 0) {
        imu_request_gyro();
    }
    else if (tick_ms % 1 == 0) {
        imu_request_euler();
    }
    if (tick_ms > 1000)
        tick_ms = 0;
    //	imu_change_com_port(DM_IMU,DM_IMU->com_port);
}
