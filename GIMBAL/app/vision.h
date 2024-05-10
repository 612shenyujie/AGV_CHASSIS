#ifndef VISION_H_
#define VISION_H_


#include "stdint.h"
#include <math.h>





typedef __packed enum{
	
	VISION_OFF		=	0X00U,
	VISION_ON	=	0X01U,
}VISION_MODE;

typedef  struct {

	float yaw_angle;
	float pitch_angle;
	float distance;
	float x;
	float y;
	float z;
	uint8_t	fire_flag;
	int8_t game_state;
	float shoot_speed;
	uint16_t	outpost_hp;
	float s_time;
	float ms_time;	
    
}VISION_COMMAND_T;

typedef  struct VISION_T{
	
	VISION_MODE mode;
	uint8_t target_id;
	VISION_MODE last_mode;
	VISION_COMMAND_T command;
}VISION_T;

void Vision_Angle_Task(float target_yaw_angle,float target_pitch_angle);
void Vision_Aim_Data_Task(float x,float y,float z);
void Vision_Send_Task(void);
void Vision_Auto_Fire_Task(uint8_t	fire_flag);
void Self_aim(float x,float y,float z,float *yaw,float *pitch,float *distance);
extern VISION_T vision_control;
#endif

