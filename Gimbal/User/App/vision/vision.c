#include "INS_task.h"
#include "LK9025.h"
#include "message_center.h"
#include "robot_def.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "user_lib.h"
#include "vision.h"

#define COM6_PACKAGE_HEADER    0xFF
#define COM6_GUIDENCE_HEADER   0x5A
#define COM6_PACKAGE_TAIL_BYTE 0x0D
#define PC_DATA_LEN            32

static Publisher_t *vision_pub;                // 自瞄应用消息发布者(自瞄反馈给cmd)
static Publisher_t *guidence_pub;              // 自瞄应用消息发布者(自瞄反馈给cmd)
static Subscriber_t *gimbal_feed_sub;          // 底盘话题消息订阅者(获取底盘夹角)
Vision_Upload_Data_s vision_feedback_data;     // 回传给cmd的云台状态信息
Guidence_Upload_Data_s guidence_feedback_data; // 回传给cmd的导航状态信息
Gimbal_Upload_Data_s gimbal_recv;              // 来自底盘的控制信息(获取底盘夹角)

xPC_Data Data_PC;
TxPC_Data Data_Tx;
// xGuidance_Data Data_Guidance;
uint8_t message[PC_DATA_LEN];
uint8_t data_from_pc[PC_DATA_LEN];

void vision_sendmessage(aim_mod_e mode, float roll, float pitch, float yaw) {
    // 填充帧头，长度，cmd1和speed
    static uint8_t cnt = 0;
    // cnt++;
    SubGetMessage(gimbal_feed_sub, &gimbal_recv); // 获取底盘夹角
    float yaw_offset = gimbal_recv.yaw2_motor_single_round_angle * DEGREE_2_RAD;
    message[0]       = COM6_PACKAGE_HEADER;
    message[1]       = mode;
    memcpy(&message[2], &roll, 4);
    memcpy(&message[6], &pitch, 4);
    memcpy(&message[10], &yaw, 4);
    memcpy(&message[14], &yaw_offset, 4);
    // 发送数据给PC
    message[10] = 0;
    message[31] = COM6_PACKAGE_TAIL_BYTE;
    // if (cnt % 2 == 0) {
    //     cnt = 0;
    //     CDC_Transmit_FS(message, PC_DATA_LEN);
    // }
    memcpy(&Data_Tx, message, sizeof(TxPC_Data));

    CDC_Transmit_FS(message, PC_DATA_LEN);
}

void PC_read_data(uint8_t *data) {
    if (data == NULL) {
        return;
    }

    for (int i = 0; i < 64; i++, data++) {
        if (*data == COM6_PACKAGE_HEADER) {
            if (*(data + sizeof(xPC_Data) - 1) == COM6_PACKAGE_TAIL_BYTE) {
                memcpy(&Data_PC, data, sizeof(xPC_Data));
                break;
            }
            else {
                break;
            }
        }
    }
}

void VisionInit(void) {
    MX_USB_DEVICE_Init();
    vision_pub      = PubRegister("vision_feed", sizeof(Vision_Upload_Data_s));
    guidence_pub    = PubRegister("guidence_feed", sizeof(Guidence_Upload_Data_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
}

void VisionTask(float roll, float pitch, float yaw) {
    vision_sendmessage(AIM_RED, roll, pitch, yaw);
    vision_feedback_data.aim_pitch = DegToRad(Data_PC.pitch);
    vision_feedback_data.aim_yaw   = Data_PC.yaw;
    guidence_feedback_data.vx      = -10000 * Data_PC.linear_y;
    guidence_feedback_data.vy      = 10000 * Data_PC.linear_x;
    guidence_feedback_data.wz      = Data_PC.angular_z;
    guidence_feedback_data.state   = Data_PC.state;
    if (Data_PC.fire_advice == 1)
        vision_feedback_data.fire_advice = SHOOT_ON;
    else
        vision_feedback_data.fire_advice = SHOOT_STOP;
    if (Data_PC.pitch == 0 && Data_PC.yaw == 0)
        vision_feedback_data.vision_state = NO_TARGET;
    else
        vision_feedback_data.vision_state = TARGET_LOCK;
    PubPushMessage(guidence_pub, (void *)&guidence_feedback_data);
    PubPushMessage(vision_pub, (void *)&vision_feedback_data);
}
