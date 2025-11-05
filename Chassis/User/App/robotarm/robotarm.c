#include "dji_motor.h"
#include "dm_motor.h"
#include "message_center.h"
#include "robot_def.h"
#include "robotarm.h"

// #include "feed_forward.h"
#include "MG996.h"
#include "bsp_pwm.h"
#include "bsp_usb.h"
#include "calibrate.h"
#include "motor_def.h"
#include "rmd_motor.h"
#include "unitree_motor.h"
#include "user_lib.h"

// 电机实例注册
// DJIMotorInstance *YawMotor;
// MG996Instance *PitchMotor;

// �??�??的双环pid，感觉写法有点冗�??
PIDInstance pitch_angle_pid, pitch_speed_pid, roll_angle_pid, roll_speed_pid;

// 前�?�实�??
//// FeedForwardInstance *joint;

// 消息发布
static Publisher_t *robotarm_pub;  // 云台应用消息发布�??(云台反�?�给cmd)
static Subscriber_t *robotarm_sub; // cmd控制消息订阅�??
Robotarm_Upload_Data_s robotarm_feedback_data; // 回传给cmd的云台状态信�??
Robotarm_Ctrl_Cmd_s robotarm_cmd_recv;         // 来自cmd的控制信�??

/* 私有函数计算的中介变�??,设为静态避免参数传递的开销 */

float debug_angle[6], debug_speed[6];
void RobotarmInit(void) {

    robotarm_sub = SubRegister("robotarm_cmd", sizeof(Robotarm_Ctrl_Cmd_s));
    robotarm_pub = PubRegister("robotarm_feed", sizeof(Robotarm_Upload_Data_s));
};
void VisionTask();
// /* 机器人云台控制核心任�?? */
void RobotarmTask(void) {

    // 获取云台控制数据
    // 后续增加�??收到数据的�?�理
    SubGetMessage(robotarm_sub, &robotarm_cmd_recv);
    // switch (robotarm_cmd_recv.gimbal_mode) {
    // case GIMBAL_NO_FOLLOW:
    //     DJIMotorChangeFeed(YawMotor, ANGLE_LOOP, MOTOR_FEED);
    //     DJIMotorChangeFeed(YawMotor, SPEED_LOOP, MOTOR_FEED);
    //     DJIMotorSetRef(YawMotor, robotarm_cmd_recv.yaw);
    //     MG996SetRef(PitchMotor, robotarm_cmd_recv.pitch);
    //     DJIMotorEnable(YawMotor);
    //     break;
    // case GIMBAL_FOLLOW_GIMBAL_YAW:
    //     DJIMotorChangeFeed(YawMotor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(YawMotor, SPEED_LOOP, MOTOR_FEED);
    //     DJIMotorSetRef(YawMotor, robotarm_cmd_recv.yaw);
    //     MG996SetRef(PitchMotor, robotarm_cmd_recv.pitch);
    //     DJIMotorEnable(YawMotor);
    //     MG996Enable(PitchMotor);
    //     break;
    // case GIMBAL_ZERO_FORCE:
    // default:
    //     DJIMotorStop(YawMotor);
    //     MG996Stop(PitchMotor);
    // }

    // PWMSetDutyRatio(PitchMotor, 0.0625);
    // robotarm_feedback_data.yaw_relative_angle = YawMotor->measure.angle_single_round *
    // DEGREE_2_RAD; robotarm_feedback_data.yaw_total_angle    = &YawMotor->measure.total_angle;
    // 推送消�??
    PubPushMessage(robotarm_pub, (void *)&robotarm_feedback_data);
}
