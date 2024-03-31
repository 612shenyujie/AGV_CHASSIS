/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-02-27 21:59:10
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-02 16:28:23
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\GIMBAL\Core\fric\fric.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "fric.h"
#include "can_connection.h"
#include "tim.h"

FRIC_T  fric;
XPOWER_COMMAND_T xpower;

//left-fric-PID
float left_fric_data[PID_DATA_LEN]
	={0.042f,0.0007f,0.0f,5.0f,0.01f,0.0f,500.0f,100.0f,0.5f,0.01f,0.0f,0.0f,900.0f,1800.0f};
//right-fric-PID
float right_fric_data[PID_DATA_LEN]
	={0.044f,0.0003f,0.0f,5.0f,0.01f,0.0f,220.0f,100.0f,0.5f,0.01f,0.0f,0.0f,900.0f,1800.0f};
//float right_fric_data[PID_DATA_LEN]
//	={0.042f,0.0007f,0.0f,20.0f,0.01f,0.0f,500.0f,100.0f,0.5f,0.0f};

	
		float meanFilter(float* data, int size)
{
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }

    return sum / size;
}
	
void Fric_Filter_Data_Update(void)
{
	for(int i=0;i<4;i++)
	{
		fric.left_motor.status.feedback_speed[i+1]=fric.left_motor.status.feedback_speed[i];
		fric.right_motor.status.feedback_speed[i+1]=fric.right_motor.status.feedback_speed[i];
	}
	fric.left_motor.status.feedback_speed[0]=fric.left_motor.motor.status.velocity_rpm;
	fric.right_motor.status.feedback_speed[0]=fric.right_motor.motor.status.velocity_rpm;
	
}
	

	
	//摩擦轮初始化
void Fric_Init(void)
{
    fric.parameter.mode =   FRIC_STOP;
    M3508_Init(&fric.left_motor.motor,0x202,1,0.3);
    M3508_Init(&fric.left_motor.motor,0x201,1,0.3);
    

    PID_Init(&fric.left_motor.pid.speed_loop,left_fric_data,Integral_Limit|ChangingIntegralRate|OutputFilter);
	PID_Init(&fric.right_motor.pid.speed_loop,right_fric_data,Integral_Limit|ChangingIntegralRate|OutputFilter);

}
//摩擦轮状态更新
float watch_left;
float watch_right;
void Fric_Status_Update(void)
{
    M3508_Status_Update(&fric.left_motor.motor);
    M3508_Status_Update(&fric.right_motor.motor);
	  Fric_Filter_Data_Update();
    fric.left_motor.status.actual_speed =   meanFilter(fric.left_motor.status.feedback_speed,5);
    fric.right_motor.status.actual_speed =   meanFilter(fric.right_motor.status.feedback_speed,5);
		watch_left=fric.left_motor.status.actual_speed;
		watch_right=-fric.right_motor.status.actual_speed;
//    fric.left_motor.status.actual_speed =  fric.left_motor.motor.feedback.velocity_lsb;
//    fric.right_motor.status.actual_speed =   fric.right_motor.motor.feedback.velocity_lsb;
    fric.left_motor.status.given_current =   fric.left_motor.motor.status.given_current;
    fric.right_motor.status.given_current =   fric.right_motor.motor.status.given_current;
    fric.left_motor.status.temperature =   fric.left_motor.motor.status.temperature;
    fric.right_motor.status.temperature =   fric.right_motor.motor.status.temperature;
		fric.parameter.lr_error=fric.left_motor.status.actual_speed-fric.right_motor.status.actual_speed;
}
//摩擦轮命令更新
void Fric_Command_Update(void)
{
    switch(fric.parameter.mode)
    {
        case FRIC_RUNNING  :
            fric.left_motor.command.target_speed =   FRIC_HIGH_SPEED;
            fric.right_motor.command.target_speed =  - FRIC_HIGH_SPEED;
						chassis.send.fric_speed=fric.left_motor.command.target_speed;
        break;
        case FRIC_STOP  :
            fric.left_motor.command.target_speed =    FRIC_NONE_SPEED;
            fric.right_motor.command.target_speed =   FRIC_NONE_SPEED;
						chassis.send.fric_speed=fric.left_motor.command.target_speed;
        break;
    }
}
//摩擦轮缓冲区更新
void Fric_Current_Update(void)
{
		PID_Calculate(&fric.left_motor.pid.speed_loop, fric.left_motor.status.actual_speed,   fric.left_motor.command.target_speed);
		PID_Calculate(&fric.right_motor.pid.speed_loop, fric.right_motor.status.actual_speed,   fric.right_motor.command.target_speed);
    
     M3508_Command_Update(&fric.left_motor.motor,fric.left_motor.pid.speed_loop.Output);
     M3508_Command_Update(&fric.right_motor.motor,fric.right_motor.pid.speed_loop.Output);      
	
	switch(fric.parameter.mode)
    {
        case FRIC_STOP  :

		 CAN1_0x200_Tx_Data[0]=fric.right_motor.motor.command.give_current_lsb>>8;
		 CAN1_0x200_Tx_Data[1]=fric.right_motor.motor.command.give_current_lsb;
		 CAN1_0x200_Tx_Data[2]=fric.left_motor.motor.command.give_current_lsb>>8;
		 CAN1_0x200_Tx_Data[3]=fric.left_motor.motor.command.give_current_lsb;
        break;
        case FRIC_RUNNING  :
            

     CAN1_0x200_Tx_Data[0]=fric.right_motor.motor.command.give_current_lsb>>8;
		 CAN1_0x200_Tx_Data[1]=fric.right_motor.motor.command.give_current_lsb;
		 CAN1_0x200_Tx_Data[2]=fric.left_motor.motor.command.give_current_lsb>>8;
		 CAN1_0x200_Tx_Data[3]=fric.left_motor.motor.command.give_current_lsb;
        break;
    }			
}


//更新倍镜Xpower命令
void Xpower_Command_Update(void)
{
	switch(xpower.mode)
	{
		case XPOWER_RUNNING  :
			xpower.target_angle = XPOWER_OPEN;
		break;
		case XPOWER_STOP   :
			xpower.target_angle = XPOWER_CLOSE;
		break;
	}
}
//更新倍镜Xpower电流
void Xpower_Send(void)
{
	switch(xpower.mode)
	{
		case XPOWER_RUNNING   :
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500+(XPOWER_OPEN/180)*2000);
		break;
		case XPOWER_STOP   :
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500+(XPOWER_CLOSE/180)*2000);
		break;
	}
}

// 更新Fric任务函数
void Fric_Task(void)
{
    // 更新Fric状态
    Fric_Status_Update();
    // 更新Fric命令
    Fric_Command_Update();
    // 更新Fric电流
    Fric_Current_Update();
	  //更新倍镜Xpower命令
	  Xpower_Command_Update();
	  //更新倍镜Xpower电流
	  Xpower_Send();
}