void DM_IMU_Init(void)
初始化函数，在robot_def中调用

imu_t *GetIMUData(void)
获取参数，在robot_cmd中调用

void imutask(void)
在robot_def中调用

注意上位机配置要先配正确
2.4的固件
CAN_ID  0x01
MST_ID　0X11
CAN输出
记得保存参数！！！！！





