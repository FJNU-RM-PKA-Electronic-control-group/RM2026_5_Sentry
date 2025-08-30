#include "custom_drv.h"
#include "main.h"
#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "cmsis_os.h"


#define KEY_RX_BUFFER_SIZE 255U //自定义控制器等键鼠数据接收缓冲区

USARTInstance *custom_usart_instance; //自定义控制器串口实例（线连接图传）
static DaemonInstance *custom_daemon; //自定义控制器守护进程
static custom_info_t custom_info;     //裁判系统传来的自定义控制器数据

/**
 * @brief  读取裁判数据中的自定义控制器数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 * 
 */

 static void CustomReadData(uint8_t *buff)
{
	uint16_t custom_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&custom_info.FrameHeader, buff, LEN_HEADER);

    // 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度(byte),用于CR16校验
			custom_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(buff, custom_length) == TRUE)
			{
				// 2个8位拼成16位int
				custom_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				// 第8个字节开始才是数据 data=7
				switch (custom_info.CmdID)
                {
                    case ID_custom_robot_data : //0x0302
                        memcpy(&custom_info.CustomDate,(buff + DATA_Offset),LEN_custom_robot_data);
                        break;
                }
            }

            // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		    if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + custom_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		    { // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			    CustomReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + custom_info.FrameHeader.DataLength + LEN_TAIL);
            }

        }

    }

}

/*自定义控制器图传串口接收回调函数,解析数据 */
static void CustomRxCallback()
{
//	DaemonReload(custom_daemon);
	CustomReadData(custom_usart_instance->recv_buff);
}

// 自定义控制器丢失回调函数,重新初始化裁判系统串口
static void CustomLostCallback(void *arg)
{
	USARTServiceInit(custom_usart_instance);
}

/* 自定义控制器通信初始化 */
custom_info_t *CustomDrvInit(UART_HandleTypeDef *custom_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = CustomRxCallback;
	conf.usart_handle = custom_usart_handle;
	conf.recv_buff_size = KEY_RX_BUFFER_SIZE; // mx 255(u8)
	custom_usart_instance = USARTRegister(&conf);

//	Daemon_Init_Config_s daemon_conf = {
//		.callback = CustomLostCallback,
//		.owner_id = custom_usart_handle,
//		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
//	};
//	custom_daemon = DaemonRegister(&daemon_conf);

	return &custom_info;
}

/**
 * @brief 自定义控制器数据发送函数
 * @param
 */
void CustomSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(custom_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}