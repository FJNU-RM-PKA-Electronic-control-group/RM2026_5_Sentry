#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dt7.h"
#include "gimbal_referee.h"
#include "referee_task.h"
#include "remote_control.h"
#include "stdlib.h"
#include "string.h"
#include "t12d.h"

#define DT7_CONTROL_FRAME_SIZE  18u // 遥控器接收的buffer大小
#define T12D_CONTROL_FRAME_SIZE 25u

// 遥控器数据
RC_ctrl_t rc_ctrl[2]; //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
uint16_t ch[16];      // wbus协议通道数据

gimbal_referee_info_t *rc_referee_recv;
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位
static RC_Type_e rc_type;

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

static void UpdateKeyCount(void) {
    RC_ctrl_t *now  = &rc_ctrl[TEMP];
    RC_ctrl_t *last = &rc_ctrl[LAST];

    /* 组合键副本:只依赖当前帧 */
    if (now->key[KEY_PRESS].ctrl)
        now->key[KEY_PRESS_WITH_CTRL] = now->key[KEY_PRESS];
    else
        memset(&now->key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (now->key[KEY_PRESS].shift)
        now->key[KEY_PRESS_WITH_SHIFT] = now->key[KEY_PRESS];
    else
        memset(&now->key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++) {
        if (i == 4 || i == 5)
            continue; /* ctrl/shift 位本身跳过 */
        if ((now->key[KEY_PRESS].keys & j) && !(last->key[KEY_PRESS].keys & j) &&
            !(now->key[KEY_PRESS_WITH_CTRL].keys & j) && !(now->key[KEY_PRESS_WITH_SHIFT].keys & j))
            now->key_count[KEY_PRESS][i]++;
        if ((now->key[KEY_PRESS_WITH_CTRL].keys & j) && !(last->key[KEY_PRESS_WITH_CTRL].keys & j))
            now->key_count[KEY_PRESS_WITH_CTRL][i]++;
        if ((now->key[KEY_PRESS_WITH_SHIFT].keys & j) &&
            !(last->key[KEY_PRESS_WITH_SHIFT].keys & j))
            now->key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback() {
    DaemonReload(rc_daemon_instance); // 先喂狗
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP],
           sizeof(RC_ctrl_t));        // 保存上一次的数据,用于按键持续按下和切换的判断
    switch (rc_type) {
    case RCTYPE_DT7:
        RCDecodeDT7(rc_usart_instance->recv_buff, rc_ctrl);
        break;
    case RCTYPE_T12D:
        RCDecodeT12D(rc_usart_instance->recv_buff, rc_ctrl);
        break;
    default:
        break;
    }
    UpdateKeyCount();
    // sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
    // rc_ctrl->rc_state = 1;
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id) {
    if (rc_referee_recv->remote_data.online_state == 1) {
        uint8_t temp[sizeof(RC_ctrl_t)] = {0};
        memcpy(&temp[6], &rc_referee_recv->remote_data, 10);
        switch (rc_type) {
        case RCTYPE_DT7:
            RCDecodeDT7(rc_usart_instance->recv_buff, rc_ctrl);
            break;
        case RCTYPE_T12D:
            RCDecodeT12D(rc_usart_instance->recv_buff, rc_ctrl);
            break;
        default:
            break;
        }
        rc_ctrl->rc_state = 2;
    }
    else {
        memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
        USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
        rc_ctrl->rc_state = 0;
        LOGWARNING("[rc] remote control lost");
    }
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle, RC_Type_e type) {
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle    = rc_usart_handle;
    rc_type              = type;
    conf.recv_buff_size  = T12D_CONTROL_FRAME_SIZE;
    //(rc_type == RCTYPE_T12D) ? T12D_CONTROL_FRAME_SIZE : DT7_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10,   // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback     = RCLostCallback,
        .owner_id     = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);
    rc_referee_recv    = GimbalRefereeTaskInit(&huart1);
    rc_init_flag       = 1;
    return rc_ctrl;
}

RC_Type_e GetRCType(void) { return rc_type; }

uint8_t RemoteControlIsOnline() {
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}
