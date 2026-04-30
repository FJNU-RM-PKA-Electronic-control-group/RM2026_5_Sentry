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

static Publisher_t *vision_pub;                // �Ӿ��������ݷ�����
static Publisher_t *guidence_pub;              // �����������ݷ�����
static Subscriber_t *gimbal_feed_sub;          // ��̨�ϴ����ݶ�����
static Subscriber_t *cmd_feed_sub;             // �����ϴ����ݶ�����
Vision_Upload_Data_s vision_feedback_data;     // �Ӿ��������ݽṹ��
Guidence_Upload_Data_s guidence_feedback_data; // �����������ݽṹ��
Gimbal_Upload_Data_s gimbal_recv;              // ��̨��������
Chassis_Upload_Data_s chassis_recv;            // ���̽�������

xPC_Data Data_PC;
TxPC_Data Data_Tx;
// xGuidance_Data Data_Guidance;
uint8_t message[PC_DATA_LEN];
uint8_t data_from_pc[PC_DATA_LEN];

void vision_sendmessage(aim_mod_e mode, float roll, float pitch, float yaw) {
    // ��PC������̨����ǶȺ͵���Ѫ����Ϣ
    static uint8_t cnt = 0;
    // cnt++;
    SubGetMessage(gimbal_feed_sub, &gimbal_recv); // ��ȡ��̨��������
    SubGetMessage(cmd_feed_sub, &chassis_recv);
    float yaw_offset       = gimbal_recv.yaw2_motor_single_round_angle * DEGREE_2_RAD;
    uint16_t current_HP    = chassis_recv.current_HP;
    uint16_t game_progress = (uint16_t)chassis_recv.game_progress;
    uint8_t status_flag    = chassis_recv.status_flag;

    Data_Tx.header              = COM6_PACKAGE_HEADER;
    Data_Tx.mode                = mode;
    Data_Tx.roll                = roll;
    Data_Tx.pitch               = pitch;
    Data_Tx.yaw                 = yaw;
    Data_Tx.offset              = yaw_offset;
    Data_Tx.game_progress       = game_progress;
    Data_Tx.rfid_supply_zone    = status_flag & RFID_SUPPLY_ZONE;
    Data_Tx.current_HP          = current_HP;
    Data_Tx.ally_base_HP        = chassis_recv.ally_base_HP;
    Data_Tx.ally_outpost_HP     = chassis_recv.ally_outpost_HP;
    Data_Tx.is_combating        = status_flag & STATUS_COMBATING;
    Data_Tx.rfid_supply_outpost = status_flag & RFID_SUPPLY_OUTPOST;
    Data_Tx.time_gt_2min        = status_flag & TIME_GT_2MIN;
    Data_Tx.check_bit           = 0; // 校验位
    Data_Tx.tail_byte           = COM6_PACKAGE_TAIL_BYTE;

    // message[0] = COM6_PACKAGE_HEADER;
    // message[1] = mode;
    // memcpy(&message[2], &roll, 4);
    // memcpy(&message[6], &pitch, 4);
    // memcpy(&message[10], &yaw, 4);
    // memcpy(&message[14], &yaw_offset, 4);
    // memcpy(&message[18], &game_progress, 2);
    // // message[20] = (uint8_t)chassis_recv.rfid_status;
    // memcpy(&message[21], &current_HP, 2);
    // // ?????????PC
    // message[10] = 0;
    // message[31] = COM6_PACKAGE_TAIL_BYTE;
    // if (cnt % 2 == 0) {
    //     cnt = 0;
    //     CDC_Transmit_FS(message, PC_DATA_LEN);
    // }
    memcpy(&message, &Data_Tx, sizeof(TxPC_Data));

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
    guidence_feedback_data.vx      = -1000 * Data_PC.linear_y;
    guidence_feedback_data.vy      = 1000 * Data_PC.linear_x;
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

static Publisher_t *vision_pub;                // �Ӿ��������ݷ�����
static Publisher_t *guidence_pub;              // �����������ݷ�����
Vision_Upload_Data_s vision_feedback_data;     // �Ӿ��������ݽṹ��
Guidence_Upload_Data_s guidence_feedback_data; // �����������ݽṹ��

xPC_Data Data_PC;
xGuidance_Data Data_Guidance;
uint8_t message[PC_DATA_LEN];
uint8_t data_from_pc[PC_DATA_LEN];

void vision_sendmessage(aim_mod_e mode, float roll, float pitch, float yaw) {
    // ��PC������̨����Ƕ���Ϣ
    message[0] = COM6_PACKAGE_HEADER;
    message[1] = mode;
    memcpy(&message[2], &roll, 4);
    memcpy(&message[6], &pitch, 4);
    memcpy(&message[10], &yaw, 4);
    // Ԥ����PC
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
