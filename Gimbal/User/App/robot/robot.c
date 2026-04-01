#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"

#include "INS_task.h"
#include "bsp_delay.h"
#include "bsp_dwt.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "buzzer.h"

#include "cmd_send.h"
#include "gimbal.h"
#include "robot_cmd.h"
#include "shoot.h"
#include "vision.h"

void RobotInit(void) {
    HAL_Delay(100);
    DWT_Init(168);
    while (BMI088_init(1) != BMI088_NO_ERROR) {
    };
    BuzzerInit();
    RobotCmdInit();
    GimbalInit();
    ShootInit();
    VisionInit();
    CmdRevInit();

    OSTaskInit();
}

void RobotTask(void) {
    static uint8_t cnt = 0; // 设定不同任务频率(控制负载)
    RobotCMDTask();
    GimbalTask();
    ShootTask();
    if (cnt % 5 == 0) {
        CmdRevTask();
        cnt = 0;
    }
    cnt++;
}
