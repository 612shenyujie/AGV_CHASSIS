#include "task_schedule.h"
#include "ins_task.h"


TASK_TIME_T gimbal_time;
float temp;
int16_t cnt;
uint16_t pwmVal=0;



void Init_Task(void)
{
		
		//初始化CAN连接
		Can_Connection_Init();
		//初始化IMU
		INS_Init();
		//初始化遥控器
		Remote_Init();
		//初始化摩擦轮
		Fric_Init();
		//初始化云台
		Gimbal_Init();
		//初始化拨弹轮
		Trigger_Init();
		//初始化底盘通信
		Chassis_Connection_Init();
	  buzzer_init_example();
		buzzer_setTask(&buzzer, BUZZER_DJI_STARTUP_PRIORITY);
	 
}

void Error_State_Judge(void)
{
	//判断是否是错误状态
	
	//如果20ms一次，则计数器加1
	if(gimbal_time.ms_count%16==0)
	{
		
		//如果上一次接收时间小于当前接收时间，则清空键盘，鼠标和rc接收数据
		if(RC.rc_receive.receive_ms_time+RC.rc_receive.receive_s_time*1000-gimbal_time.ms_count-gimbal_time.s_count*1000<-1000)
		{
			memset(&RC.rc_receive.key_board,0,sizeof(RC.rc_receive.key_board));
			memset(&RC.rc_receive.rc,0,sizeof(RC.rc_receive.rc));
			memset(&RC.rc_receive.mouse,0,sizeof(RC.rc_receive.mouse));
			gimbal.parameter.mode=GIMBAL_MODE_NO_FORCE;
			chassis.send.mode	=	CHASSIS_MODE_NOFORCE;
			fric.parameter.mode=	FRIC_STOP;
//			buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
		}
	}
	if(gimbal_time.ms_count%16==1)
	{
		
		//如果上一次接收时间小于当前接收时间，则蜂鸣器离线音响起
		if(fric.left_motor.motor.parameter.receive_ms_time+fric.left_motor.motor.parameter.receive_s_time*1000-gimbal_time.ms_count-gimbal_time.s_count*1000<-1000)
		{
			buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
		}
	}
	if(gimbal_time.ms_count%16==2)
	{
		
		//如果上一次接收时间小于当前接收时间，则蜂鸣器离线音响起
		if(fric.right_motor.motor.parameter.receive_ms_time+fric.right_motor.motor.parameter.receive_s_time*1000-gimbal_time.ms_count-gimbal_time.s_count*1000<-1000)
		{
			buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
		}
	}
	if(gimbal_time.ms_count%16==3)
	{
		
		//如果上一次接收时间小于当前接收时间，则蜂鸣器离线音响起
		if(trigger.motor.parameter.receive_ms_time+trigger.motor.parameter.receive_s_time*1000-gimbal_time.ms_count-gimbal_time.s_count*1000<-1000)
		{
			buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
			trigger.parameter.online_state	=	0;
			Trigger_Calculate_Task();
		}
	}
  if(gimbal_time.ms_count%16==4)
	{
		
		//如果上一次接收时间小于当前接收时间，则蜂鸣器离线音响起
		if(gimbal.pitch.motor.parameter.receive_ms_time+gimbal.pitch.motor.parameter.receive_s_time*1000-gimbal_time.ms_count-gimbal_time.s_count*1000<-1000)
		{
			buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
		}
	}
	if(gimbal_time.ms_count%16==5)
	{
		
		//如果上一次接收时间小于当前接收时间，则蜂鸣器离线音响起
		if(gimbal.yaw.motor.parameter.receive_ms_time+gimbal.yaw.motor.parameter.receive_s_time*1000-gimbal_time.ms_count-gimbal_time.s_count*1000<-1000)
		{
			buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
		}
	}
}

void Time_Count_Task(void)
{
	// 增加毫秒计数
	gimbal_time.ms_count++;
	// 如果毫秒计数大于等于1000，增加秒计数
	if(gimbal_time.ms_count>=1000)
	{
		gimbal_time.s_count++;
		gimbal_time.ms_count=0;
	}
	// 如果秒计数等于3，且状态为初始状态，改变状态为运行状态
	if(gimbal_time.s_count	==	3	&&	gimbal_time.state	==	INIT_STATE)
		gimbal_time.state=RUNNING_STATE;
	// 如果延迟计数等于1，改变标定状态为已标定
	if(delay_time.gimbal_cali_cnt== 1)	gimbal.parameter.calibration_state=CALIBRATED ;
	if(gimbal_time.s_count<10)	chassis.send.ui_init_flag=0;
	if(gimbal_time.s_count==10)	chassis.send.ui_init_flag=1;
	if(gimbal_time.s_count>=10&&delay_time.ui_mode_cnt==0)chassis.send.ui_init_flag=1;
};
	


void task_schedule()
{
	//如果定时器是TIM3
	if(htim->Instance==TIM3)
	{
		//根据gimbal_time.state的状态来执行不同的任务
		switch(gimbal_time.state)
	{
		//初始化状态
		case INIT_STATE :
			
			break;
		//运行状态
		case RUNNING_STATE :
			
			//执行移动任务
			Remote_Task();
			//执行云台任务

			Gimbal_Task();
			//执行摩擦轮任务
			Fric_Task();
			//执行底盘通信任务
			Chassis_Connection_Task();
			//执行拨弹轮任务
			Trigger_Task();
			//发送数据
				if(gimbal_time.ms_count%5==0)
				Vision_Send_Task();
				
			
			break;
		//默认状态
		default :
			
			break;
		 
				
	}
//	buzzer_taskScheduler(&buzzer);
	//执行时间计数任务
	Time_Count_Task();
	//执行延迟计数任务
	Delay_Cnt_Task();	
	//判断错误状态
	if(gimbal.parameter.calibration_state==NORMAL)
			Error_State_Judge();
	//发送数据
	Can_Send_Task(gimbal_time.ms_count);
	//计数器加1
	cnt++;
	}
	//如果定时器不是TIM3
	else
	{
		//执行INS任务
		INS_Task();
		temp=arm_sin_f32(3.14/2.f);
	}
	
};



