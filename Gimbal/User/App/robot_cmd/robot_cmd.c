// app
#include "calibrate.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "vision.h"

// module
#include "INS_task.h"
#include "dji_motor.h"
#include "dm_imu.h"
#include "fliter.h"
#include "message_center.h"
#include "remote_control.h"
#include "user_lib.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "core_cm4.h"

// ??��?,???????????????????
// #define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // ?????????,0-360
// #define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) //
// pitch??????????,0-360
#define USE_VISION_FIRE_CONTROL

#define YAW_ALIGN_ANGLE     0
#define PTICH_HORIZON_ANGLE 0

RC_ctrl_t *rc_data; // ?????????,??????????
attitude_t *ins_data;
imu_t *DM_ins_data;
cali_data_t *calibrate_data;

static Publisher_t *chassis_cmd_pub;        // ????????????????
static Subscriber_t *chassis_feed_sub;      // ????????????????
Chassis_Ctrl_Cmd_s chassis_cmd_send;        // ????????????????,?????????????UI???????
Chassis_Upload_Data_s chassis_fetch_data;   // ?????????????????????,??????????????????????????

static Publisher_t *gimbal_cmd_pub;         // ????????????????
static Subscriber_t *gimbal_feed_sub;       // ????????????????
Gimbal_Ctrl_Cmd_s gimbal_cmd_send;          // ????????????????
Gimbal_Upload_Data_s gimbal_fetch_data;     // ????????????????

static Publisher_t *shoot_cmd_pub;          // ????????????????
static Subscriber_t *shoot_feed_sub;        // ???????????????
Shoot_Ctrl_Cmd_s shoot_cmd_send;            // ????????????????
Shoot_Upload_Data_s shoot_fetch_data;       // ???????????????

static Subscriber_t *vision_feed_sub;       // ????????????????
static Subscriber_t *guidence_feed_sub;
Vision_Upload_Data_s vision_fetch_data;     // ????????????????
Guidence_Upload_Data_s guidence_fetch_data; // ????

static Robot_Status_e robot_state;          // ?????????��????
static Robot_Status_e last_robot_state;     // ?????????��????
static LPFliterInstance yaw_offset_fliter;  // ??????????
static LPFliterInstance vx_fliter;          // ?????????????
static LPFliterInstance vy_fliter;          // ?????????????
speed_mode_e speed_mode = SPEEDMODE_NORMAL;
float speed_offset_angle;
Sentry_State_e sentry_state = STATE_MOVING;

static void CalcOffsetAngle() {
    // ????angle???????,????????????,??????????????????
    static float angle;
    angle = gimbal_fetch_data.yaw1_motor_single_round_angle; // ????????????yaw?????????
#if YAW_ECD_GREATER_THAN_4096                                // ???????180??
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // ��??180??
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = (angle - YAW_ALIGN_ANGLE);
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - PI)
        chassis_cmd_send.offset_angle = (angle - YAW_ALIGN_ANGLE);
    else
        chassis_cmd_send.offset_angle = (angle - YAW_ALIGN_ANGLE + 2 * PI);
#endif
}

static float PitchLimit(float add) {
    if (gimbal_cmd_send.gimbal_mode != GIMBAL_ZERO_FORCE) {
        if (gimbal_fetch_data.pitch_ecd_angle <= (float)PITCH_MAX_ECD &&
            gimbal_fetch_data.pitch_ecd_angle >= (float)PITCH_MIN_ECD) {
            return add;
        }
        else if (add * (gimbal_fetch_data.pitch_ecd_angle - (float)PITCH_MAX_ECD) < 0 ||
                 add * (gimbal_fetch_data.pitch_ecd_angle - (float)PITCH_MIN_ECD) < 0) {
            return add;
        }
    }
    return 0;
}
/**
 * @brief     ????????????????
 * @attention ??????????????????????????????
 *
 * @bug       ??????????  ??  ??  ?????????
 *
 */
void StateControlSet() {
    switch (guidence_fetch_data.state) {
    case STATE_MOVING:
        shoot_cmd_send.shoot_mode    = SHOOT_STOP;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode     = LOAD_STOP;

        break;
    case STATE_ATTACKING:
        chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_ROTATE;
        shoot_cmd_send.friction_mode  = FRICTION_ON;
        if (vision_fetch_data.fire_advice == SHOOT_ON) {
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            shoot_cmd_send.load_mode  = LOAD_REVERSE;
        }
        else {
            shoot_cmd_send.shoot_mode = SHOOT_STOP;
        }
        if (vision_fetch_data.vision_state == NO_TARGET) {
            gimbal_cmd_send.gimbal_mode = GIMBAL_SCAN_MODE;
        }
        else {
            // gimbal_cmd_send.gimbal_mode = GIMBAL_VISION_MODE;
            gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
            gimbal_cmd_send.pitch       = vision_fetch_data.aim_pitch; // 0.027;
            gimbal_cmd_send.yaw         = vision_fetch_data.aim_yaw;
        }

        chassis_cmd_send.vx *= 0.5F;
        chassis_cmd_send.vy *= 0.5F;
        chassis_cmd_send.wz = 20;
        break;
    case STATE_DEFENDING:
        chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_ROTATE;
        shoot_cmd_send.shoot_mode     = SHOOT_STOP;
        shoot_cmd_send.friction_mode  = FRICTION_ON;
        chassis_cmd_send.vx *= 0.3;
        chassis_cmd_send.vy *= 0.3;
        chassis_cmd_send.wz = 100;
    case STATE_NULL:
    default:
        break;
    }
}

void VisionControlSet(void) {
    // if (vision_fetch_data.fire_advice == SHOOT_STOP)
    //     shoot_cmd_send.shoot_mode = SHOOT_STOP;

    // robot_state = ROBOT_AUTO;
    // if (robot_state != last_robot_state && robot_state == ROBOT_AUTO) {
    //     sentry_state = STATE_MOVING;
    // }
    // if (guidence_fetch_data.vx != chassis_cmd_send.vx && chassis_cmd_send.vx == 0 &&
    //     guidence_fetch_data.vy != chassis_cmd_send.vy && chassis_cmd_send.vy == 0) {
    //     StateSwitch();
    // }
    if (vision_fetch_data.vision_state == NO_TARGET) {
        gimbal_cmd_send.gimbal_mode = GIMBAL_SCAN_MODE;
    }
    else {
        // gimbal_cmd_send.gimbal_modse = GIMBAL_VISION_MODE;
        gimbal_cmd_send.pitch = vision_fetch_data.aim_pitch; // 0.027;
        gimbal_cmd_send.yaw   = vision_fetch_data.aim_yaw;
    }

    // if ((chassis_fetch_data.game_progress != GAME_IN_PROGRESS) &&
    //     (chassis_fetch_data.game_progress != GAME_CALCULATING_RESULTS)) {
    //     chassis_cmd_send.wz = 0; // guidence_fetch_data.wz;
    //     return;
    // }

    chassis_cmd_send.vx = guidence_fetch_data.vx;
    chassis_cmd_send.vy = guidence_fetch_data.vy;

    // if (chassis_fetch_data.game_progress == GAME_IN_PROGRESS) {
    //     if (chassis_fetch_data.status_flag & GET_HURT) {
    //         if ((guidence_fetch_data.vx == 0) && (guidence_fetch_data.vy == 0)) {
    //             chassis_cmd_send.wz = 80.0f;
    //         }
    //         else {
    //             chassis_cmd_send.wz = 40.0f;
    //         }
    //     }
    //     else {
    //         chassis_cmd_send.wz = 0.0f;
    //     }
    // }

    // guidence_fetch_data.wz;
    // if (chassis_fetch_data.rfid_status == RFID_CENTER_GAIN) {
    //     chassis_cmd_send.wz = 0;
    // }

    chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_ROTATE;
    chassis_cmd_send.wz           = 0.0f;
    shoot_cmd_send.friction_mode  = FRICTION_ON;

    if (vision_fetch_data.vision_state == NO_TARGET) {
        gimbal_cmd_send.gimbal_mode = GIMBAL_SCAN_MODE;
        return;
    }
    else {
        // gimbal_cmd_send.gimbal_modse = GIMBAL_VISION_MODE;
        gimbal_cmd_send.pitch = vision_fetch_data.aim_pitch; // 0.027;
        gimbal_cmd_send.yaw   = vision_fetch_data.aim_yaw;
    }
    // shoot_cmd_send.shoot_mode = SHOOT_ON;

    // #ifdef USE_VISION_FIRE_CONTROL
    // //自动模式下的发弹逻辑
    // shoot_cmd_send.friction_mode = FRICTION_ON;
    // if (vision_fetch_data.fire_advice == SHOOT_ON) {
    //     shoot_cmd_send.shoot_mode = SHOOT_ON;
    //     shoot_cmd_send.load_mode  = LOAD_BURSTFIRE;
    // }
    // 手动模式下的发弹逻辑
    if (vision_fetch_data.fire_advice != SHOOT_ON) {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.load_mode  = LOAD_STOP;
    }
    // #endif
}

static void RemoteControlSet(void) {
    // static gimbal_mode_e last_gimbal_mode = GIMBAL_ZERO_FORCE;
    // last_robot_state                      = robot_state;
    // if (switch_is_up(rc_data[TEMP].rc.switch_right)) // ???????[??],??????????
    // {
    //     chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_FOLLOW_GIMBAL_YAW;
    //     gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
    //     if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
    //         chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_ROTATE;
    //         chassis_cmd_send.wz           = 50;
    //     }
    // }
    // else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) //
    // ???????[??],????????????,???????????
    // {
    //     chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_NO_FOLLOW;
    //     gimbal_cmd_send.gimbal_mode   = GIMBAL_FREE_MODE;
    //     if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
    //         speed_mode                  = SPEEDMODE_ABSOLUTE;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_SCAN_MODE;
    //         if (last_gimbal_mode != gimbal_cmd_send.gimbal_mode)
    //             speed_offset_angle = DM_ins_data->yaw;
    //         // chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_ROTATE;
    //         // chassis_cmd_send.wz           = -50;
    //     }
    //     else {
    //         speed_mode = SPEEDMODE_NORMAL;
    //     }
    // }
    // if (switch_is_down(rc_data[TEMP].rc.switch_right)) // ???????[??],????
    // {
    //     robot_state                   = ROBOT_STOP;
    //     gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
    //     chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_ZERO_FORCE;
    // }
    // else {
    //     robot_state = ROBOT_READY;
    // }

    // if (switch_is_down(rc_data[TEMP].rc.switch_left) &&
    //     switch_is_down(rc_data[TEMP].rc.switch_right)) // ?????????,???��?????
    // {
    //     CalibrateTask(rc_data);
    // }

    // // ???????,??????????????
    // gimbal_cmd_send.pitch += PitchLimit(0.00001f * (float)rc_data[TEMP].rc.rocker_l1);
    // gimbal_cmd_send.yaw =
    //     theta_format(gimbal_cmd_send.yaw - 0.001f * (float)rc_data[TEMP].rc.rocker_l_);
    // float raw_vx = 20.0f * (float)rc_data[TEMP].rc.rocker_r_; // _??????
    // float raw_vy = 20.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1???????
    // // ???????,????��???��????(????????????��??),??????????
    // // chassis_cmd_send.vx = 20.0f * (float)rc_data[TEMP].rc.rocker_r_; // _??????
    // // chassis_cmd_send.vy = 20.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1???????
    // float sin_theta, cos_theta;
    // if (speed_mode == SPEEDMODE_ABSOLUTE) {
    //     cos_theta = cosf(theta_format(speed_offset_angle - DM_ins_data->yaw) * DEGREE_2_RAD);
    //     sin_theta = sinf(theta_format(speed_offset_angle - DM_ins_data->yaw) * DEGREE_2_RAD);
    //     chassis_cmd_send.vx = raw_vx * cos_theta - raw_vy * sin_theta;
    //     chassis_cmd_send.vy = raw_vx * sin_theta + raw_vy * cos_theta;
    // }
    // else if (speed_mode == SPEEDMODE_NORMAL) {
    //     chassis_cmd_send.vx = raw_vx;
    //     chassis_cmd_send.vy = raw_vy;
    // }
    // if (last_gimbal_mode != gimbal_cmd_send.gimbal_mode) {
    //     switch (gimbal_cmd_send.gimbal_mode) {
    //     case GIMBAL_FREE_MODE:
    //         gimbal_cmd_send.yaw   = ins_data->Yaw * RAD_2_DEGREE;
    //         gimbal_cmd_send.pitch = ins_data->Pitch;
    //         break;
    //     case GIMBAL_GYRO_MODE:
    //         gimbal_cmd_send.yaw   = ins_data->Yaw * RAD_2_DEGREE;
    //         gimbal_cmd_send.pitch = ins_data->Pitch;
    //     case GIMBAL_SCAN_MODE:
    //         gimbal_cmd_send.yaw   = ins_data->Yaw * RAD_2_DEGREE;
    //         gimbal_cmd_send.pitch = ins_data->Pitch;
    //     default:
    //         break;
    //     }
    // }
    // last_gimbal_mode = gimbal_cmd_send.gimbal_mode;

    // // ???????
    // // ????????,????????????,???????
    // if (rc_data[TEMP].rc.dial < -100 || rc_data[TEMP].rc.dial > 100) // ????(??)????100,???????
    // {
    //     shoot_cmd_send.friction_mode = FRICTION_ON;
    // }
    // else {
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    // }
    // // ????????,?????????????????,?????????
    // if (rc_data[TEMP].rc.dial > 500) {
    //     shoot_cmd_send.bullet_speed = TEST;
    //     shoot_cmd_send.load_mode    = LOAD_BURSTFIRE;
    //     shoot_cmd_send.shoot_mode   = SHOOT_ON;
    // }
    // else if (rc_data[TEMP].rc.dial < -500) {
    //     shoot_cmd_send.load_mode  = LOAD_REVERSE;
    //     shoot_cmd_send.shoot_mode = SHOOT_ON;
    // }
    // else {
    //     shoot_cmd_send.load_mode = LOAD_STOP;
    // }
    // if (switch_is_down(rc_data[TEMP].rc.switch_left) &&
    //     robot_state != ROBOT_STOP) // ????????[??],?????
    // {
    //     VisionControlSet();
    // }
    // // ???????,??????1??,????????????????????��?��????,
    // shoot_cmd_send.shoot_rate = 2;
}

/**
 * @brief ???????????????????????
 *
 */
static void MouseKeySet(void) {
    // chassis_cmd_send.vy =
    //     rc_data[TEMP].key[KEY_PRESS].w * 15000 - rc_data[TEMP].key[KEY_PRESS].s * 15000; //
    //     ???????
    // chassis_cmd_send.vx =
    //     rc_data[TEMP].key[KEY_PRESS].d * 15000 - rc_data[TEMP].key[KEY_PRESS].a * 15000;

    // rc_data[TEMP].mouse.x *= -1;
    // rc_data[TEMP].mouse.y *= -1;
    // gimbal_cmd_send.yaw =
    //     RadFormat(gimbal_cmd_send.yaw + (float)rc_data[TEMP].mouse.x / 10000); // ???????
    // gimbal_cmd_send.pitch += PitchLimit((float)rc_data[TEMP].mouse.y / 10000);

    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z?????????
    // {
    // case 0:
    //     shoot_cmd_send.bullet_speed = TEST;
    //     break;
    // case 1:
    //     shoot_cmd_send.bullet_speed = TEST;
    //     break;
    // default:
    //     shoot_cmd_send.bullet_speed = TEST;
    //     break;
    // }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) // E?????��?????
    // {
    // case 0:
    //     shoot_cmd_send.load_mode = LOAD_1_BULLET;
    //     break;
    // default:
    //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    //     break;
    // }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F???????????
    // {
    // case 0:
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    //     break;
    // default:
    //     shoot_cmd_send.friction_mode = FRICTION_ON;
    //     break;
    // }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 3) // V???��??????
    // {
    // case 0:
    //     robot_state = ROBOT_STOP;
    //     break;
    // case 1:
    //     robot_state                   = ROBOT_READY;
    //     gimbal_cmd_send.gimbal_mode   = GIMBAL_FREE_MODE;
    //     chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_NO_FOLLOW;
    //     shoot_cmd_send.shoot_mode     = SHOOT_ON;
    //     break;
    // default:
    //     robot_state                   = ROBOT_READY;
    //     gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
    //     chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_FOLLOW_GIMBAL_YAW;
    //     shoot_cmd_send.shoot_mode     = SHOOT_ON;
    //     break;
    // }
    // switch (rc_data[TEMP].key[KEY_PRESS].ctrl) // ctrl��????
    // {
    // case 1:
    //     chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_ROTATE;
    //     break;

    // default:

    //     break;
    // }
    // switch (rc_data[TEMP].key[KEY_PRESS].shift) // ?????? ??shift?????????? ???????????
    // {
    // case 1:
    //     //        chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_MAXSPEED;
    //     break;

    // default:

    //     break;
    // }
    // switch (rc_data[TEMP].mouse.press_l) {
    // case 1:
    //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    //     break;
    // default:
    //     shoot_cmd_send.load_mode = LOAD_STOP;
    //     break;
    // }
    // switch (rc_data[TEMP].mouse.press_r) {
    // case 1:
    //     VisionControlSet();
    //     break;
    // default:
    //     break;
    // }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 4) // B??reset
    // {
    // case 3:
    //     __disable_irq();
    //     NVIC_SystemReset();
    //     break;
    // default:
    //     break;
    // }
}

/**
 * @brief  ??????,???????????????????/??????????/???????��??
 *         ???????'300'???????????,???????????.
 *
 * @todo
 * ???????????????????????(???????????),?????????????????daemon???
 *
 */
static void EmergencyHandler(void) {
    // ?????????????
    // if (rc_data->rc_state == 0) {
    //     robot_state = ROBOT_STOP;
    // }
    // if (robot_state == ROBOT_STOP) // ?????????????��??????????��?
    // {
    //     gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
    //     chassis_cmd_send.chassis_mode = (uint8_t)CHASSIS_ZERO_FORCE;
    //     shoot_cmd_send.shoot_mode     = SHOOT_OFF;
    //     shoot_cmd_send.friction_mode  = FRICTION_OFF;
    //     shoot_cmd_send.load_mode      = LOAD_STOP;
    //     // LOGERROR("[CMD] emergency stop!");
    // }
    // // ???????????[??],?????s??????
    // if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    //     robot_state               = ROBOT_READY;
    //     shoot_cmd_send.shoot_mode = SHOOT_ON;
    //     // LOGINFO("[CMD] reinstate, robot ready");
    // }
}

void RobotCmdInit(void) {
    rc_data        = RemoteControlInit(&huart3, RCTYPE_T12D);
    ins_data       = GetInsData();
    calibrate_data = CalibrateInit();

    gimbal_cmd_pub    = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub   = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub     = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub    = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    chassis_cmd_pub   = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub  = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    vision_feed_sub   = SubRegister("vision_feed", sizeof(Vision_Upload_Data_s));
    guidence_feed_sub = SubRegister("guidence_feed", sizeof(Guidence_Upload_Data_s));

    Low_Pass_Fliter_Init_Config_s lp_fliter = {
        .RC = 0.0f,
    };
    DM_ins_data = GetIMUData();
    LowPassFliterInit(&yaw_offset_fliter, &lp_fliter);
    lp_fliter.RC = 0.08;
    LowPassFliterInit(&vx_fliter, &lp_fliter);
    LowPassFliterInit(&vy_fliter, &lp_fliter);
    //	gimbal_cmd_send.pitch = 0;		???��?????????
    robot_state = ROBOT_READY;
}

/* ????????????????,200Hz???????(?????????????????) */
void RobotCMDTask(void) {
    // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // ??????????????????
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(vision_feed_sub, &vision_fetch_data);
    SubGetMessage(guidence_feed_sub, &guidence_fetch_data);

    // ????gimbal?????????????????????????��?,?????????,???static??��??????
    CalcOffsetAngle();
    // if (switch_is_up(rc_data[TEMP].rc.switch_left) ||
    //     rc_data->rc_state == 2) // ????????[??],??????
    // {
    //     MouseKeySet();
    // }
    // else {
    RemoteControlSet();
    // }
    EmergencyHandler(); // ????????????????????????????
    // if (robot_state != ROBOT_STOP) {
    //     StateControlSet();
    // }
    // ???????????????,?????????????2????????

    // ???????,??????,???????
    shoot_cmd_send.rest_heat      = chassis_fetch_data.shooter_heat;
    chassis_cmd_send.offset_angle = theta_format(FliterCalcutale(
        (FliterInstance *)&yaw_offset_fliter, gimbal_fetch_data.yaw1_motor_single_round_angle));
    chassis_cmd_send.vx = FliterCalcutale((FliterInstance *)&vx_fliter, chassis_cmd_send.vx);
    chassis_cmd_send.vy = FliterCalcutale((FliterInstance *)&vy_fliter, chassis_cmd_send.vy);

    chassis_cmd_send.vx           = (fabs(chassis_cmd_send.vx) < 0.01f) ? 0 : chassis_cmd_send.vx;
    chassis_cmd_send.vy           = (fabs(chassis_cmd_send.vy) < 0.01f) ? 0 : chassis_cmd_send.vy;
    chassis_cmd_send.sentry_state = (uint8_t)guidence_fetch_data.state;
    // ?????????????????????remotecontrolsetmode??mousekeysetmode?????????

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
