#include "can_comm.h"
#include "cmd_receive.h"
#include "daemon.h"
#include "message_center.h"
#include "stdlib.h"
#include "string.h"

static CANCommInstance *ins;
chassis_data_rev *data_ptr;
static Publisher_t *chassis_cmd_p;   // ���̿�����Ϣ������
static Subscriber_t *chassis_feed_s; // ���̷�����Ϣ������
Chassis_Ctrl_Cmd_s chassis_send;     // ���͸�����Ӧ�õ���Ϣ,����������Ϣ��UI�������
Chassis_Upload_Data_s
    chassis_get_data; // �ӵ���Ӧ�ý��յķ�����Ϣ��Ϣ,���̹���ǹ������������˶�״̬��

void CmdRevInit(void) {
    CANComm_Init_Config_s cconfig = {
        .can_config    = {.can_handle = &hcan2, .tx_id = 0x33, .rx_id = 0x22},
        .send_data_len = sizeof(Chassis_Upload_Data_s),
        .recv_data_len = sizeof(chassis_data_rev)};
    ins = CANCommInit(&cconfig);

    chassis_cmd_p  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_s = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

void CmdRevTask(void) {
    SubGetMessage(chassis_feed_s, &chassis_get_data);
    data_ptr                  = (chassis_data_rev *)CANCommGet(ins);
    chassis_send.vx           = data_ptr->vx;
    chassis_send.vy           = data_ptr->vy;
    chassis_send.wz           = data_ptr->wz;
    chassis_send.offset_angle = data_ptr->offset_angle;
    chassis_send.chassis_mode = data_ptr->chassis_mode;

    CANCommSend(ins, (uint8_t *)&chassis_get_data);
    PubPushMessage(chassis_cmd_p, (void *)&chassis_send);
}
