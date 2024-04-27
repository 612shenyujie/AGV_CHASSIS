/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-02-23 16:40:10
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-01 19:36:27
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\GIMBAL\Core\gimbal\gimbal.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef GIMBAL_H_
#define GIMBAL_H_

#include	"pid.h"
#include	"stdint.h"
#include 	"motor.h"


typedef  enum
{
	NO_CALIBRATION = 0,
	CALIBRATING,
	CALIBRATED,
	NORMAL,
}CALIBRATION_STATE_T;

typedef  enum
{
	IMU_MODE = 0,
	ENCODER_MODE,
}GIMBAL_MOTOR_MODE_T;


typedef  enum
{
	GIMBAL_MODE_NO_FORCE = 0,
	GIMBAL_MODE_TOPANGLE,
	GIMBAL_MODE_ABSOLUTE,
	GIMBAL_MODE_PRECISION,
	GIMBAL_MODE_CALI,

}GIMBAL_MODE_T;

typedef  struct 
{
	GIMBAL_MOTOR_MODE_T mode;
	float number_ratio;
  bool online_state;
}YAW_PARAMETER_T,PITCH_PARAMETER_T;

typedef  struct 
{
	float target_angle;
	float target_speed;
	float add_angle;
	float actual_angle;
	
}YAW_COMMAND_T,PITCH_COMMAND_T;


typedef  struct
{
	PID_TypeDef encoder_speed_loop;
	PID_TypeDef encoder_angle_loop;

	PID_TypeDef imu_speed_loop;
	PID_TypeDef imu_angle_loop;

}GIMBAL_PID_T;

typedef  struct
{
	int16_t rounds;
	float total_angle;
	float actual_angle;
	float last_actual_angle;
	float actual_speed;

}IMU_STATUS_T;

typedef  struct
{
	IMU_STATUS_T status;
	
}IMU_PART_T;

typedef	 struct
{
	int16_t rounds;
	float total_angle;
	float actual_angle;

	float actual_speed;
}Yaw_Status_T;

typedef  struct 
{
	GM6020_T motor;
	IMU_PART_T imu;
	GIMBAL_PID_T pid;
	YAW_COMMAND_T command;
	YAW_PARAMETER_T parameter;
	Yaw_Status_T status;
	
	
}GIMBAL_YAW_T,GIMBAL_PITCH_T;

typedef enum
{
	FIVE_METER_DISTANCE=	0x00u,
	SEVEN_METER_DISTANCE=	0x01u,
	TEN_METER_DISTANCE=	0x02u,
	
}PRECISION_DISTANCE_E;


typedef  struct 
{
	GIMBAL_MODE_T mode;
	GIMBAL_MODE_T last_mode;
	CALIBRATION_STATE_T calibration_state;
	PRECISION_DISTANCE_E precision_distance;
	PRECISION_DISTANCE_E last_precision_distance;
}GIMBAL_PARAMETER_T;

typedef enum
{
	AFFILIATED_PITCH_REMOVABLE	=0x01u,
	AFFILIATED_PITCH_UNREMOVABLE	=0x00u,
}AFFILIATED_PITCH_MODE_E;

typedef enum
{
	AFFILIATED_PITCH_LOW_ANGLE	=0x00u,
	AFFILIATED_PITCH_MID_ANGLE	=0x01u,
	AFFILIATED_PITCH_HIGH_ANGLE	=0x02u,

}AFFILIATED_PITCH_STATE_E;

typedef struct
{
	float target_angle;
	float	add_angle;
	float command_angle;
	AFFILIATED_PITCH_MODE_E	mode;
	AFFILIATED_PITCH_STATE_E	state;
	AFFILIATED_PITCH_STATE_E	last_state;
}AFFILIATED_PITCH_T;

typedef  struct 
{

	GIMBAL_YAW_T yaw;
	GIMBAL_PITCH_T pitch;
	GIMBAL_PARAMETER_T parameter;
	AFFILIATED_PITCH_T affiliated_pitch;
}GIMBAL_T;

extern GIMBAL_T gimbal;



void Gimbal_Init(void);

void Gimbal_Task(void);

#endif
