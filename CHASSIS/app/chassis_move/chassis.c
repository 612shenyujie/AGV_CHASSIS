/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-09 20:29:02
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-10 16:23:53
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\CHASSIS\CHASSIS\app\chassis_move\chassis.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "chassis.h"

#include "gimbal_connection.h"
#include "kinematic.h"
#include "math.h"
#include "referee.h"

CHASSIS_T chassis;
YAW_T   yaw;
PID_T   yaw_pid;
float yaw_position_loop_data[10]= {0.07f,0.0f,0.f,2.5f,0.0f,1.0f,0.f,0.f,0.f,0.f};

double sShapedRamp(double t, double t0, double k) {
    double sigmoid = 1.0 / (1.0 + exp(-k * (t - t0)));
    return sigmoid * t;
}

void Chassis_Speed_Slow_Motion(CHASSIS_T *chassis)
{
	int Delta; 
	switch (chassis->parameter.speed_slow)
	{
	
		case Uniform_Acceleration:
			Delta = Acceleration;
			
			if (chassis->command.set_vx != 0||chassis->command.vx!=0)
			{
				if (chassis->command.set_vx > chassis->command.vx)
				{
					if (chassis->command.set_vx - chassis->command.vx > Delta)
						 chassis->command.vx += Delta;
					else
						 chassis->command.vx=chassis->command.set_vx;
				}
				if (chassis->command.set_vx < chassis->command.vx)
				{
					if (chassis->command.vx - chassis->command.set_vx > Delta)
						 chassis->command.vx -= Delta;
					else
						 chassis->command.vx=chassis->command.set_vx;
				}
			}
						if (chassis->command.set_vy != 0||chassis->command.vy!=0)
			{
				if (chassis->command.set_vy > chassis->command.vy)
				{
					if (chassis->command.set_vy - chassis->command.vy > Delta)
						 chassis->command.vy += Delta;
					else
						 chassis->command.vy=chassis->command.set_vy;
				}
				if (chassis->command.set_vy < chassis->command.vy)
				{
					if (chassis->command.vy - chassis->command.set_vy > Delta)
						 chassis->command.vy -= Delta;
					else
						 chassis->command.vy=chassis->command.set_vy;
				}
			}
			
		break;
		
		case Ease_Out:
			
			if (chassis->command.set_vx != 0)
			{
				
				float Parameter_C; 
				
					Parameter_C = 0.15;
				
				
				Delta = (chassis->command.set_vx - chassis->command.vx)*Parameter_C;
				
				 chassis->command.vx+=Delta;
			}
			else
				chassis->command.vx=0;
			
				
			if (chassis->command.set_vy != 0)
			{
				
				float Parameter_C; 
				
					Parameter_C = 0.15;
				
			
				Delta = (chassis->command.set_vy - chassis->command.vy)*Parameter_C;
				
				 chassis->command.vy+=Delta;
			}
			else
				chassis->command.vy=0;
			

		break;
		
		case Smoothen_Off:
				chassis->command.vx=chassis->command.set_vx;
				chassis->command.vy=chassis->command.set_vy;
		break;
	}


			
}

void Chassis_Inveter_Judge(void)
{

		chassis.parameter.invert_flag	=	connection.connection_rx.invert.flag;
		chassis.parameter.follow_switch	=	connection.connection_rx.follow.flag;
    if(chassis.parameter.invert_flag==INVERT_ON)
    {
        chassis.command.set_vx = -connection.connection_rx.vx;
        chassis.command.set_vy = -connection.connection_rx.vy;
        chassis.command.set_vw = -connection.connection_rx.vw/10.0f;
    }
    else
    {
        chassis.command.set_vx = connection.connection_rx.vx;
        chassis.command.set_vy = connection.connection_rx.vy;
        chassis.command.set_vw = connection.connection_rx.vw/10.0f;
    }
		Chassis_Speed_Slow_Motion(&chassis);
		
}

void Yaw_Angle_Process(YAW_T  *yaw)
{
		
    yaw->status.total_angle =	yaw->motor.status.total_position_degree/yaw->parameter.number_ratio;
		while(yaw->status.total_angle-yaw->status.rounds*360.0f>360.0f) yaw->status.rounds++;
		while(yaw->status.total_angle-yaw->status.rounds*360.0f<-360.0f) yaw->status.rounds--;
    yaw->status.actual_angle    =   yaw->status.total_angle - yaw->status.rounds*360.0f;
}

void Gimbal_To_Chassis_Relative_Angle_Update(void)
{
    float gimbal_angle,chassis_angle;
    
		if(chassis.parameter.follow_switch	==	FOLLOW_ON)
				{
				chassis_angle   =   yaw.status.actual_angle;
				gimbal_angle    =   GIMBAL_HEAD_ANGLE+180.0f*chassis.parameter.invert_flag;
				chassis.parameter.relative_angle =   chassis_angle-gimbal_angle;
							if(chassis.parameter.mode	==	CHASSIS_SPIN)
							{
							chassis.parameter.relative_angle-=yaw.parameter.yaw_offset;
							}
		
				if(chassis.parameter.relative_angle>180.0f) chassis.parameter.relative_angle-=360.0f;
				if(chassis.parameter.relative_angle<-180.0f) chassis.parameter.relative_angle+=360.0f;
				}
				else
				{
				chassis.parameter.relative_angle =  0;
				}
	
		
}

static float chassis_fllow(void)
{
    float gimbal_angle,chassis_angle;
    
    PID_Calculate(&yaw_pid.angle_loop,chassis.parameter.relative_angle,0);
    return yaw_pid.angle_loop.Output;
    
}

void Chassis_Mode_Command_Update(void)
{
		chassis.parameter.mode=connection.connection_rx.mode;
    switch(chassis.parameter.mode)
    {
        case    CHASSIS_REMOTE_CLOSE:
				case	CHASSIS_PRECISIOUS:
        chassis.command.vx =  0;
        chassis.command.vy =  0;
        chassis.command.vw =  0;
        break;
        case    CHASSIS_NORMAL:
        if(chassis.parameter.follow_switch	==	FOLLOW_ON)
				{
				chassis.command.vw =  -1.0f*chassis_fllow();
				}
				else
				{
				chassis.command.vw =  0;
				}
				
        break;
        case    CHASSIS_SPIN:
        if(chassis.command.vx==0&&chassis.command.vy==0)
        chassis.command.vw =  3.0f;
        else 
				
					chassis.command.vw =  2.0f;
				
				
        break;

    }
}

void Buffer_Limition_Kf_Update(void)
{
	if(JudgeReceive.remainEnergy>50)
		chassis.parameter.buffer_limition_k	=1.0;
	else if(JudgeReceive.remainEnergy>40)
		chassis.parameter.buffer_limition_k	=0.8;
	else if(JudgeReceive.remainEnergy>35)
		chassis.parameter.buffer_limition_k	=0.6;
	else if(JudgeReceive.remainEnergy>30)
		chassis.parameter.buffer_limition_k	=0.4;
	else if(JudgeReceive.remainEnergy>20)
		chassis.parameter.buffer_limition_k	=0.2;
	else if(JudgeReceive.remainEnergy>10)
		chassis.parameter.buffer_limition_k	=0.1;
	else
		chassis.parameter.buffer_limition_k	=0.0;
	
}

void Chassis_Init(void)
{
    chassis.parameter.mode =   CHASSIS_NORMAL;
    chassis.parameter.invert_flag =  1;//1:正向，0：反向
    chassis.parameter.break_mode    =   1;
		chassis.parameter.speed_slow	=	Ease_Out;
    chassis.parameter.relative_angle    =   0.f;
		chassis.parameter.buffer_limition_k	=1;
		chassis.A_motor.zero_position = 0x1b0c;
		chassis.B_motor.zero_position = 0x174c;
		chassis.C_motor.zero_position = 0x0ACE;
		chassis.D_motor.zero_position = 0x19CB;
		chassis.A_motor.active_status=1;
		chassis.B_motor.active_status=1;
		chassis.C_motor.active_status=1;
		chassis.D_motor.active_status=1;
		chassis.A_motor.ID	=	0x1a;
		chassis.B_motor.ID	=	0x1b;
		chassis.C_motor.ID	=	0x1c;
		chassis.D_motor.ID	=	0x1d;
	
}

void Yaw_Init(void)
{
		 yaw.motor.parameter.calibrate_state	=	1;
		yaw.parameter.yaw_offset=0.f;
		yaw.parameter.number_ratio = 2.0f;
    PID_Init(&yaw_pid.angle_loop,yaw_position_loop_data,NONE);
}

void Chassis_Move(void)
{   
    Chassis_Inveter_Judge();
    Gimbal_To_Chassis_Relative_Angle_Update();
    Chassis_Mode_Command_Update();
    Chassis_Speed_Control(&chassis);
}