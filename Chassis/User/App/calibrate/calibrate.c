#include "calibrate.h"
#include "message_center.h"
#include "robot_def.h"
#include "bsp_flash.h"
#include "bsp_dwt.h"
#include "stdlib.h"
#include "buzzer.h"
#include "tim.h"
#include "stdio.h"
#include "mpu6500.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "buzzer.h"
//#include "remote_control.h"

// //���̻�ȡ����������
// static Subscriber_t *chassis_cali_sub;		// ���̷�����Ϣ������
// Chassis_Ctrl_Cmd_s chassis_cali_data; 	// �ӵ��̻�ȡ�ķ�����Ϣ
// //��е�ۻ�ùؽ�У׼����?
// static Subscriber_t *robotarm_cali_sub;		// ��е�۷�����Ϣ������
// Robotarm_Ctrl_Cmd_s robotarm_cali_data;	// ���ݸ���е�۵Ŀ�����Ϣ
// //��е�۷����ؽ�У׼����
// static Subscriber_t *robotarm_cali_sub1;		// ��е�ۿ�����Ϣ������
// Robotarm_Upload_Data_s robotarm_offest_angle;	// ���ݸ���е�۵Ŀ�����Ϣ

cali_data_t cali_data_save;					//�����Ǻ͹ؽ�У׼����
cail_mods state = CAIL_OFF;
BuzzzerInstance *buzzer_cali;
//static void GimbalCalibrateControl(void);

/**
 * @brief У׼��ʼ��
 * @note  ��ʼ��У׼��
 * 
 */
cali_data_t *CalibrateInit(void)
{
	
	// chassis_cali_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
	// robotarm_cali_sub = SubRegister("robotarm_cmd",sizeof(Robotarm_Ctrl_Cmd_s));
	// robotarm_cali_sub1 = SubRegister("robotarm_feed",sizeof(Robotarm_Upload_Data_s));
	
	memset(&cali_data_save,0,sizeof(cali_data_t));

	Buzzer_config_s buzzer_config = {
		.alarm_level = ALARM_LEVEL_MEDIUM,
		.loudness = 0.5,
		.octave = OCTAVE_5,
	};
	buzzer_cali=BuzzerRegister(&buzzer_config);	//ע�������ʵ��?

	//ʹ�ܷ�����
	AlarmSetStatus(buzzer_cali,ALARM_OFF);
	memset(&cali_data_save,0,sizeof(cali_data_save));
	
	//��ʼ����ʱ����?flash����
	flash_read(CALIBRATE_DATA_ADDR,(uint32_t *)&cali_data_save,sizeof(cali_data_t)/4);

	if(cali_data_save.cali_status != 1)//����У׼��������У׼
	{
		//����������Ҫ�޸ģ���Ϊ�����Ǻ͹ؽ�У׼���?
		// cali_data_save.max_pitch  = MAX_PITCH_ALTERNATE;
		// cali_data_save.min_pitch  = MIN_PITCH_ALTERNATE;
		// cali_data_save.zero_pitch = ZERO_PITCH_ALTERNATE;
		// cali_data_save.zero_yaw =	ZERO_YAW_ALTERNATE;
		return &cali_data_save;
	}
	else
		return &cali_data_save;//�������򷵻�����
}

/**
 * @brief ʹ��У׼�����ڰ����룬����CAIL_READY
 * @note  �;ɴ����RC_cmd_to_calibrate�еĵ�һ������ͬ
 */
// static void CailbrateOffControl(RC_ctrl_t *cail_rc_data)
// {
	
// 	static uint32_t cali_cmd_time;
// 	static float calibrating_time;
	
// 	SubGetMessage(robotarm_cali_sub,&robotarm_cali_data);
// 	if(robotarm_cali_data.robotarm_mode == ROBOTARM_CALI )
// 	{
// 		state = CAIL_READY;
// 		cali_data_save.joint_cali_state = JOINT_CALI_INT;
	
// 	}
// 	//���ڰ˿���У׼
// 	if(cail_rc_data[TEMP].rc.rocker_l_ > RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_l1 < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r_ < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r1 < -RC_CALI_VALUE_HOLE)
// 	{
// 		//���û��д��У׼ʱ��?��ȡʱ��
// 		if(!cali_cmd_time)
// 			DWT_GetDeltaT(&cali_cmd_time);
// 		calibrating_time += DWT_GetDeltaT(&cali_cmd_time);
// 	}
// 	else
// 	{
// 		//û�п���У׼����ʱ������
// 		cali_cmd_time = 0;
// 		calibrating_time = 0;
// 	}
// 	//У׼ʱ���������?����У׼׼���׶�
// 	if(calibrating_time > 2)
// 	{
// 		state = CAIL_READY;
// 		AlarmSetStatus(buzzer_cali,ALARM_ON);

// 	}
// }

/**
 * @brief ѡ��У׼����
 * @note  �;ɴ����RC_cmd_to_calibrate�еĵڶ�������ͬ
 * 		  �����У׼������?
 */
// static void CalibrateReadyControl(RC_ctrl_t *cail_rc_data)
// {
// 	SubGetMessage(robotarm_cali_sub,&robotarm_cali_data);
// 	static uint32_t cali_cmd_time;
// 	static float calibrating_time;
	
// 	//�����У׼IMU
// 	if(cail_rc_data[TEMP].rc.rocker_l_ < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_l1 < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r_ > RC_CALI_VALUE_HOLE 
// 		&& cail_rc_data[TEMP].rc.rocker_r1 < -RC_CALI_VALUE_HOLE)
// 	{
// 		if(!cali_cmd_time)
// 			DWT_GetDeltaT(&cali_cmd_time);
// 		calibrating_time += DWT_GetDeltaT(&cali_cmd_time);
// 		if(calibrating_time > 2)//����ʱ�����?2s
// 		{
// 			state = CAIL_IMU;
// 			AlarmSetOctave(buzzer_cali,OCTAVE_2);
// 			AlarmSetStatus(buzzer_cali,ALARM_ON);
// 		}
// 	}
// 	//�²���У׼�ؽڵ��?
// 	if(robotarm_cali_data.robotarm_mode == ROBOTARM_CALI)
// 	{
// 		state = CAIL_JOINTMOTOR;
// 		AlarmSetStatus(buzzer_cali,ALARM_ON);

// 	}
// 	//ң����û�п��ƣ����У׼ʱ��?
// 	else
// 	{
// 		cali_cmd_time = 0;
// 		calibrating_time = 0;
// 	}
	
// }

// float debug_cali_time;
// static void JointCalibrateControl(void)
// {
// 	static float cali_time = 0;		//У׼����ʱ��
// 	static uint32_t cali_start_time;//У׼��ʼʱ���?
	
// 	SubGetMessage(robotarm_cali_sub1,&robotarm_offest_angle);
// 	debug_cali_time = cali_time;
// 	switch(cali_data_save.joint_cali_state)
// 	{
// 		case JOINT_CALI_INT:
// 			if(cali_time == 0)
// 			{
// 				DWT_GetDeltaT(&cali_start_time);
// 				cali_data_save.joint_cali_state = JOINT_CALI_ING;				
// 			}
// 			else
// 			{
// 				cali_time = 0;
// 			}
// 			break;
// 		case JOINT_CALI_ING:
// 			if(cali_time<=2)
// 			{
// 				cali_time += DWT_GetDeltaT(&cali_start_time);
// 			}
// 			else
// 			{
// 				cali_data_save.cali_joint1_angle  = robotarm_offest_angle.joint1_relative_angle;
// 				cali_data_save.cali_joint2_angle  = robotarm_offest_angle.joint2_relative_angle;
// 				cali_data_save.cali_ee1_angle     = robotarm_offest_angle.ee1_relative_angle;
// 				cali_data_save.cali_ee2_angle     = robotarm_offest_angle.ee2_relative_angle;
// 				cali_data_save.cali_unitree_angle = robotarm_offest_angle.unitree_yaw_ralative_angle;
// 				cali_data_save.cali_4310_angle    = robotarm_offest_angle.joint3_relative_angle;
// 				cali_data_save.joint_cali_state   = JOINT_CALI_END;
				
// 			}
// 			break;
// 		case JOINT_CALI_END:

// 			flash_erase_address(CALIBRATE_DATA_ADDR,1);
// 			flash_write_single_address(CALIBRATE_DATA_ADDR, (uint32_t *)&cali_data_save, sizeof(cali_data_t)/4);//д��flash	
// 			cali_data_save.cali_status = 1;
// 			cali_data_save.joint_cali_state = JOINT_READY;
// 			state = CAIL_OFF;
// 			AlarmSetStatus(buzzer_cali,ALARM_OFF);

// 			break;

// 		default:
// 			break;
// 	}
// 	// if(cali_data_save.joint_cali_state!=JOINT_READY)
// 	// {
// 	// 	AlarmSetStatus(ALARM_ON);
// 	// 	AlarmSetOctave(0.5);
// 	// }
// 	// else
// 	// {
// 	// 	AlarmSetStatus(ALARM_OFF);

// 	// }
// }

static void IMUCalibrateControl(void)
{
	mpu_offset_cali();
	AlarmSetStatus(buzzer_cali,ALARM_OFF);
	state = CAIL_OFF;
	
	// flash_erase_address(CALIBRATE_DATA_ADDR,1);
	// flash_write_single_address(CALIBRATE_DATA_ADDR, (uint32_t *)&cali_data_save, sizeof(cali_data_t)/4);//д��flash
}
void CalibrateTask(RC_ctrl_t *cail_rc_data)
{	
	switch(state)
	{
		case CAIL_OFF:
			// CailbrateOffControl(cail_rc_data);
			break;
		case CAIL_READY:
		// 	CalibrateReadyControl(cail_rc_data);
			break;
		case CAIL_JOINTMOTOR:
			// JointCalibrateControl();
			break;
		 case CAIL_IMU:
		 	IMUCalibrateControl();
			break;
		default:
			break;
	}
	
}

