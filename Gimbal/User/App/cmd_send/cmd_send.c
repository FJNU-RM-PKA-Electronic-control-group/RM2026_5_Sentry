#include "can_comm.h"
#include "cmd_send.h"
#include "message_center.h"
#include "robot_def.h"
#include "stdlib.h"
#include "string.h"

CANCommInstance *ins;
static Subscriber_t *cmd_send_sub; // ��̨������Ϣ������
static Publisher_t *cmd_send_pub;  // ���ڷ������̵�����
Chassis_Ctrl_Cmd_s cmd_send_data;  // ����̨��ȡ�ķ�����Ϣ
Chassis_Ctrl_Cmd_s data_send;
Chassis_Upload_Data_s *chassis_can_fetch_data;

void CmdRevInit(void) {
    CANComm_Init_Config_s cconfig = {
        .can_config    = {.can_handle = &hcan2, .tx_id = 0x22, .rx_id = 0x33},
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .recv_data_len = sizeof(Chassis_Upload_Data_s)};
    ins          = CANCommInit(&cconfig);
    cmd_send_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    cmd_send_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

void CmdRevTask(void) {
    chassis_can_fetch_data = (Chassis_Upload_Data_s *)CANCommGet(ins);
    SubGetMessage(cmd_send_sub, &cmd_send_data);
    data_send.vx           = cmd_send_data.vx;
    data_send.vy           = cmd_send_data.vy;
    data_send.wz           = cmd_send_data.wz;
    data_send.offset_angle = cmd_send_data.offset_angle;
    data_send.chassis_mode = cmd_send_data.chassis_mode;
    data_send.sentry_state = cmd_send_data.sentry_state;

    CANCommSend(ins, (uint8_t *)&data_send);
    PubPushMessage(cmd_send_pub, chassis_can_fetch_data);
}
