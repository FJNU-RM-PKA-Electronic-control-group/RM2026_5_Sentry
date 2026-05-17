#include "motor_task.h"

#include "LK9025.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "feed_forward.h"
#include "rmd_motor.h"

void MotorControlTask(void) {
    static uint8_t cnt = 0; // �趨��ͬ���������Ƶ��
    FeedForwardControl();
    DJIMotorControl();
    if (cnt % 2 == 0) { // 500hz
        DMMotorControl();
        // LKMotorControl();
    }
    //	if(cnt%10==0) //100hz

    cnt++;
}
