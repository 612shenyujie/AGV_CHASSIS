/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-02-27 21:59:21
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-11 03:28:03
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\GIMBAL\app\fric.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef FRIC_H_
#define FRIC_H_

#include "motor.h"
#include "PID.h"
#include "drv_can.h"


#define FRIC_HIGH_SPEED 7900.f//13.2

//#define FRIC_NORMAL_SPEED 5967.f//15.2_old
#define FRIC_NORMAL_SPEED 5167.f//15.2
#define FRIC_NONE_SPEED 0.f

#define XPOWER_OPEN 121.2f
#define XPOWER_CLOSE 0.f
typedef __packed enum
{
    FRIC_STOP = 0,
    FRIC_RUNNING,
		FRIC_BRUSTING,
    FRIC_ERROR,
}FRIC_MODE_E;

typedef  struct
{
	PID_TypeDef speed_loop;
	PID_TypeDef angle_loop;
}FRIC_PID_T;

typedef  struct 
{
   FRIC_MODE_E mode;
	 int16_t	lr_error;
}FRIC_PARAMETER_T;

typedef  struct 
{
    int16_t target_speed;
	
}FRIC_COMMAND_T;

typedef  struct 
{
    float actual_speed;
    float given_current;
    float temperature;
    float feedback_speed[5];
}FRIC_STATUE_T;

typedef  struct
{
    FRIC_COMMAND_T command;
    FRIC_PID_T pid;
    M3508_T motor;
    FRIC_STATUE_T status;
}FRIC_MOTOR_T;

typedef  struct 
{
    FRIC_PARAMETER_T parameter;
    FRIC_MOTOR_T left_motor;
    FRIC_MOTOR_T right_motor;
}FRIC_T;

typedef __packed enum
{
	XPOWER_STOP = 0,
	XPOWER_RUNNING,
}XPOWER_MODE_E;

typedef  struct 
{
    int16_t target_angle;
	  XPOWER_MODE_E mode;
	
}XPOWER_COMMAND_T;

extern FRIC_T  fric;
extern XPOWER_COMMAND_T xpower;
void Fric_Init(void);
void Fric_Task(void);
void Xpower_Command_Update(void);
void Xpower_Send(void);

#endif // !FRIC_H_

