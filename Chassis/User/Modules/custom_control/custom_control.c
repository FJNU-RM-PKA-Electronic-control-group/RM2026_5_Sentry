#include "custom_control.h"
#include "main.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "string.h"
#include "cmsis_os.h"
#include "custom_drv.h"
#include "stdio.h"
#include "math.h"

#define CUSTOM_CONTROL_FRAME_SIZE 30u //暂定
#define CUSTOMANGLE_TO_RAD  0.017444444f //(PI/180 0.017444444)

custom_info_t *custom_recv_info;

                                  
/**
 * @brief 矫正自定义控制器摇杆的值,超过100或者小于-100的值都认为是无效值,(roll的)
 *
 */
static void RectifyRCjoystick(float *angle)
{
    if (fabs(*angle > 100))
        *angle = 0;
}

//图传串口初始化,并且记录初始时的遥控器状态
//这个可以优化一下，每次复位太麻烦了，有没有什么更方便的方法
custom_info_t *CustomControlInit(UART_HandleTypeDef *custom_usart_handle,Custom_ctrl *custom_ctrl)
{
    custom_recv_info = CustomDrvInit(custom_usart_handle); //初始化图传串口
    custom_recv_info->init_flag = 1;
    custom_ctrl->custom_init_angle[yaw_unitree]      = custom_recv_info->CustomDate.angle[yaw_unitree];
    custom_ctrl->custom_init_angle[joint1]   = custom_recv_info->CustomDate.angle[joint1];
    custom_ctrl->custom_init_angle[joint2]   = custom_recv_info->CustomDate.angle[joint2];
    custom_ctrl->custom_init_angle[joint3]   = custom_recv_info->CustomDate.angle[joint3];
    custom_ctrl->custom_init_angle[ee_pitch] = custom_recv_info->CustomDate.angle[ee_pitch];
    return custom_recv_info;

}

//自定义控制器数据获取和解析
void GetCustomAngle(custom_info_t *custom_info,Custom_ctrl *custom_ctrl)
{
    float angle[6];
    float relative_angle[6];

    //将图传数据读入
    angle[yaw_unitree] = custom_info->CustomDate.angle[yaw_unitree];
    angle[joint1]      = custom_info->CustomDate.angle[joint1];
    angle[joint2]      = custom_info->CustomDate.angle[joint2];
    angle[joint3]      = custom_info->CustomDate.angle[joint3];
    angle[ee_pitch]    = custom_info->CustomDate.angle[ee_pitch];
    angle[ee_roll]     = custom_info->CustomDate.s;

    //角度解算（角度制数据）,有进行限位
    relative_angle[yaw_unitree] = angle[yaw_unitree] - custom_ctrl->custom_init_angle[yaw_unitree];
    relative_angle[joint1]      = angle[joint1] - custom_ctrl->custom_init_angle[joint1]*35.0f;
    relative_angle[joint2]      = (angle[joint2] - custom_ctrl->custom_init_angle[joint2]-3)*20+58;
    relative_angle[joint3]      = angle[joint3] - custom_ctrl->custom_init_angle[joint3];
    relative_angle[ee_pitch]    = angle[ee_pitch] - custom_ctrl->custom_init_angle[ee_pitch]*0.8f-0.4f;
    relative_angle[ee_roll]     = angle[ee_roll] - 2048;
    RectifyRCjoystick(&relative_angle[ee_roll]);

    //传入数据（弧度制）
    custom_ctrl->custom_angle[yaw_unitree] = relative_angle[yaw_unitree]*CUSTOMANGLE_TO_RAD;
    custom_ctrl->custom_angle[joint1]      = relative_angle[joint1]*CUSTOMANGLE_TO_RAD;
    custom_ctrl->custom_angle[joint2]      = relative_angle[joint2]*CUSTOMANGLE_TO_RAD;
    custom_ctrl->custom_angle[joint3]      = relative_angle[joint3]*CUSTOMANGLE_TO_RAD;
    custom_ctrl->custom_angle[ee_pitch]    = relative_angle[ee_pitch]*CUSTOMANGLE_TO_RAD;
    custom_ctrl->custom_angle[ee_roll]     = relative_angle[ee_roll]*0.0005f;
    
    //遥控器拨杆开关获取，后续要增加可读性
    custom_ctrl->custom_ch[0] = Key_0_is_up(custom_recv_info->CustomDate.key);
    custom_ctrl->custom_ch[1] = Key_0_is_mid(custom_recv_info->CustomDate.key);
    custom_ctrl->custom_ch[2] = Key_0_is_down(custom_recv_info->CustomDate.key);
    custom_ctrl->custom_ch[3] = Key_1_is_up(custom_recv_info->CustomDate.key);
    custom_ctrl->custom_ch[4] = Key_1_is_mid(custom_recv_info->CustomDate.key);
    custom_ctrl->custom_ch[5] = Key_1_is_down(custom_recv_info->CustomDate.key);


}

void CustomReast(custom_info_t *custom_info,Custom_ctrl *custom_ctrl)
{

    custom_ctrl->custom_init_angle[yaw_unitree] = custom_recv_info->CustomDate.angle[yaw_unitree];
    custom_ctrl->custom_init_angle[joint1]      = custom_recv_info->CustomDate.angle[joint1];
    custom_ctrl->custom_init_angle[joint2]      = custom_recv_info->CustomDate.angle[joint2];
    custom_ctrl->custom_init_angle[joint3]      = custom_recv_info->CustomDate.angle[joint3];
    custom_ctrl->custom_init_angle[ee_pitch]    = custom_recv_info->CustomDate.angle[ee_pitch];
    

}




   
