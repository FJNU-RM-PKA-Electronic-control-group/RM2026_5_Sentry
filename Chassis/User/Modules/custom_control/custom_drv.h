#ifndef __CUSTOM_DRV_H
#define __CUSTOM_DRV_H

#include "usart.h"
#include "referee_protocol.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

#pragma pack(1)

//这个结构体包含自定义控制器数据和键鼠消息
typedef struct 
{
    xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;

    ext_custom_robot_data_t CustomDate; //0x0302

    //键鼠信息还未加入
	uint8_t init_flag;

}custom_info_t;

#pragma pack()

/**
 * @brief 自定义控制器图传通信初始化,该函数会初始化图传串口,开启中断
 *
 * @param custom_usart_handle 串口handle,A板一般用串口8
 * @return custom_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
custom_info_t *CustomDrvInit(UART_HandleTypeDef *custom_usart_handle);

/**
 * @brief 暂定
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */

void CustomSend(uint8_t *send, uint16_t tx_len);






#endif
