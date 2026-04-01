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
#define USE_32_BIT_PC_DATA

#ifdef USE_32_BIT_PC_DATA

#define PC_DATA_LEN 32

static Publisher_t *vision_pub;                // ����Ӧ����Ϣ������(���鷴����cmd)
static Publisher_t *guidence_pub;              // ����Ӧ����Ϣ������(���鷴����cmd)
static Subscriber_t *gimbal_feed_sub;          // ���̻�����Ϣ������(��ȡ���̼н�)
static Subscriber_t *cmd_feed_sub;
Vision_Upload_Data_s vision_feedback_data;     // �ش���cmd����̨״̬��Ϣ
Guidence_Upload_Data_s guidence_feedback_data; // �ش���cmd�ĵ���״̬��Ϣ
Gimbal_Upload_Data_s gimbal_recv;              // ���Ե��̵Ŀ�����Ϣ(��ȡ���̼н�)
Chassis_Upload_Data_s chassis_recv;            // ���Ե��̵Ŀ�����Ϣ(��ȡ���̼н�)

xPC_Data Data_PC;
TxPC_Data Data_Tx;
// xGuidance_Data Data_Guidance;
uint8_t message[PC_DATA_LEN];
uint8_t data_from_pc[PC_DATA_LEN];

void vision_sendmessage(aim_mod_e mode, float roll, float pitch, float yaw) {
    // ���֡ͷ�����ȣ�cmd1��speed
    static uint8_t cnt = 0;
    // cnt++;
    SubGetMessage(gimbal_feed_sub, &gimbal_recv); // ��ȡ���̼н�
    SubGetMessage(cmd_feed_sub, &chassis_recv);
    float yaw_offset       = gimbal_recv.yaw2_motor_single_round_angle * DEGREE_2_RAD;
    uint16_t current_HP    = chassis_recv.current_HP;
    uint16_t game_progress = (uint16_t)chassis_recv.game_progress;

    message[0] = COM6_PACKAGE_HEADER;
    message[1] = mode;
    memcpy(&message[2], &roll, 4);
    memcpy(&message[6], &pitch, 4);
    memcpy(&message[10], &yaw, 4);
    memcpy(&message[14], &yaw_offset, 4);
    memcpy(&message[18], &game_progress, 2);
    message[20] = (uint8_t)chassis_recv.rfid_status;
    memcpy(&message[21], &current_HP, 2);
    // �������ݸ�PC
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
    cmd_feed_sub    = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

void VisionTask(float roll, float pitch, float yaw) {
    vision_sendmessage(AIM_BLUE, roll, pitch, yaw);
    vision_feedback_data.aim_pitch = DegToRad(Data_PC.pitch);
    vision_feedback_data.aim_yaw   = Data_PC.yaw;
    guidence_feedback_data.vx      = -12500 * Data_PC.linear_y;
    guidence_feedback_data.vy      = 12500 * Data_PC.linear_x;
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
#endif

#ifdef USE_16_BIT_PC_DATA
#define PC_DATA_LEN 16
#define PC_DATA_LEN 16

static Publisher_t *vision_pub;                // ����Ӧ����Ϣ������(���鷴����cmd)
static Publisher_t *guidence_pub;              // ����Ӧ����Ϣ������(���鷴����cmd)
Vision_Upload_Data_s vision_feedback_data;     // �ش���cmd����̨״̬��Ϣ
Guidence_Upload_Data_s guidence_feedback_data; // �ش���cmd�ĵ���״̬��Ϣ

xPC_Data Data_PC;
xGuidance_Data Data_Guidance;
uint8_t message[PC_DATA_LEN];
uint8_t data_from_pc[PC_DATA_LEN];

void vision_sendmessage(aim_mod_e mode, float roll, float pitch, float yaw) {
    // ���֡ͷ�����ȣ�cmd1��speed
    message[0] = COM6_PACKAGE_HEADER;
    message[1] = mode;
    memcpy(&message[2], &roll, 4);
    memcpy(&message[6], &pitch, 4);
    memcpy(&message[10], &yaw, 4);
    // �������ݸ�PC
    message[14] = 0;
    message[15] = COM6_PACKAGE_TAIL_BYTE;
    CDC_Transmit_FS(message, PC_DATA_LEN);
}

void PC_read_data(uint8_t *data) {
    if (data == NULL) {
        return;
    }

    for (int i = 0; i < 64; i++, data++) {
        if (*data == COM6_PACKAGE_HEADER) {
            if (*(data + PC_DATA_LEN - 1) == COM6_PACKAGE_TAIL_BYTE) {
                memcpy(&Data_PC, data, PC_DATA_LEN);
                break;
            }
            else {
                break;
            }
        }
        else if (*data == COM6_GUIDENCE_HEADER) {
            if (*(data + PC_DATA_LEN - 1) == COM6_PACKAGE_TAIL_BYTE) {
                memcpy(&Data_Guidance, data, sizeof(xGuidance_Data));
                break;
            }
        }
    }
}

void VisionInit(void) {
    MX_USB_DEVICE_Init();
    vision_pub   = PubRegister("vision_feed", sizeof(Vision_Upload_Data_s));
    guidence_pub = PubRegister("guidence_feed", sizeof(Guidence_Upload_Data_s));
}

void VisionTask(float roll, float pitch, float yaw) {
    vision_sendmessage(AIM_RED, roll, pitch, yaw);
    vision_feedback_data.aim_pitch = DegToRad(Data_PC.pitch);
    vision_feedback_data.aim_yaw   = Data_PC.yaw;
    guidence_feedback_data.vx      = -5000 * Data_Guidance.vy;
    guidence_feedback_data.vy      = 5000 * Data_Guidance.vx;
    guidence_feedback_data.wz      = Data_Guidance.wz;
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
#endif
