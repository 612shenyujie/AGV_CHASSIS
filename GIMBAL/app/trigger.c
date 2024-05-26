#include "trigger.h"
#include "math.h"
TRIGGER_T trigger;

float Trigger_Angle_Loop_Data[PID_DATA_LEN]
={20.0f,0.01f,250.0f,30.0f,5.0f,0.1f,1.0f,0.1f,0.6f,0.0f};

float Trigger_Speed_Loop_Data[PID_DATA_LEN]
={0.50f,0.03f,0.0f,20.0f,5.0f,0.0f,1.0f,0.5f,0.0f,0.0f};
//拨弹轮初始化
void Trigger_Init(void)
{
	M3508_Init(&trigger.motor,0x201,50.895f,0.3f);
	trigger.parameter.state	=	TRIGGER_STOP;
	trigger.parameter.shoot_num	=	0;
	trigger.parameter.last_shoot_num	=	trigger.parameter.shoot_num;
	PID_Init(&trigger.pid.angle_loop,Trigger_Angle_Loop_Data,Integral_Limit|ChangingIntegralRate|OutputFilter);
	PID_Init(&trigger.pid.speed_loop,Trigger_Speed_Loop_Data,Integral_Limit|ChangingIntegralRate);
}
//拨弹轮状态更新
void Trigger_Status_Update(void)
{
	 M3508_Status_Update(&trigger.motor);
	
	trigger.status.actual_speed	=	trigger.motor.status.velocity_rpm/trigger.motor.parameter.reduction_rate;
	trigger.status.total_angle =	trigger.motor.status.total_position_degree/trigger.motor.parameter.reduction_rate;
	while(trigger.status.total_angle-trigger.status.rounds*360.0f>360.0f)trigger.status.rounds++;
	while(trigger.status.total_angle-trigger.status.rounds*360.0f<-360.0f) trigger.status.rounds--;
    trigger.status.actual_angle    =   trigger.status.total_angle - trigger.status.rounds*360.0f;
}


//拨弹轮指令更新
int32_t trigger_time=0;
int16_t	error_cnt;
void Trigger_Command_Update(void)
{
	int Delta;
	trigger.command.target_total_position	=	trigger.parameter.shoot_num*(60.0f);
	while(trigger.command.target_position	-	trigger.command.rounds*360.0f>360.0f)	trigger.command.rounds++;
	while(trigger.command.target_position	-	trigger.command.rounds*360.0f<-360.0f)	trigger.command.rounds--;
	trigger.command.target_position	=	trigger.command.target_total_position	-	trigger.command.rounds*360.0f	;
	trigger_time++;
	if(trigger.parameter.state!=TRIGGER_BRUSTING)
	{
		
		
		
	if(trigger_time>50)
	{
		trigger_time=0;
	if(trigger.command.target_total_position-trigger.command.actual_target_position >= 0.8f)
	  {
	    trigger.command.actual_target_position +=(trigger.command.target_total_position-trigger.command.actual_target_position)*0.09f;
	  }
		else
			trigger.command.actual_target_position=trigger.command.target_total_position;
	}
	if(trigger.command.actual_target_position+trigger.status.total_angle>10.f)
			error_cnt++;
		else
			error_cnt=0;
		if(error_cnt>100)
		{
			trigger.command.actual_target_position=-trigger.status.total_angle-20.f;
		}
}
	else
	{
			trigger.command.actual_target_position=trigger.command.target_total_position;
	}
	
	PID_Calculate(&trigger.pid.angle_loop,trigger.status.total_angle,-trigger.command.actual_target_position);
	trigger.command.target_speed	=	trigger.pid.angle_loop.Output;
//		trigger.command.target_speed	=	-10;
	PID_Calculate(&trigger.pid.speed_loop,trigger.status.actual_speed,trigger.command.target_speed);
	M3508_Command_Update(&trigger.motor,trigger.pid.speed_loop.Output);
	
}
//拨弹轮缓冲区数据更新
void Trigger_Send_Command_Update(void)
{
	if(trigger.parameter.reactive_flag)
	{
	switch(trigger.parameter.state)
	{
		case TRIGGER_STOP:
			memset(&CAN2_0x200_Tx_Data,0,8);
			break;
		case TRIGGER_RUNNING	:
		case TRIGGER_BRUSTING	:
			CAN2_0x200_Tx_Data[0]=trigger.motor.command.give_current_lsb>>8;
			CAN2_0x200_Tx_Data[1]=trigger.motor.command.give_current_lsb;
			break;
		
	}
	}
	else
	{
	memset(&CAN2_0x200_Tx_Data,0,8);
	}
	
}

void Trigger_Calculate_Task(void)
{
	while(trigger.command.target_total_position+trigger.status.total_angle>-60.f&&trigger.parameter.shoot_num>=0)
	{
		trigger.parameter.shoot_num--;
		trigger.command.target_total_position	=	trigger.parameter.shoot_num*(60.0f);
	}
}

void	Trigger_Task(void)
{
	// 更新拨弹轮状态
	Trigger_Status_Update();
	// 更新拨弹轮命令
	Trigger_Command_Update();
	// 更新拨弹轮发送命令
	Trigger_Send_Command_Update();
}