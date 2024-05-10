#include "gimbal_connection.h"
#include "referee.h"
GIMBAL_CONNECTION_T connection;

float	actual_speed;
uint16_t outpost_hp;

void Chassis_Flag_Update(GIMBAL_CONNECTION_T *connection)
{
	connection->connection_rx.invert.last_flag=connection->connection_rx.invert.flag;
	connection->connection_rx.follow.last_flag=connection->connection_rx.follow.flag;
	connection->connection_rx.fric.last_flag=connection->connection_rx.fric.flag;
	connection->connection_rx.vision.last_flag=connection->connection_rx.vision.flag;
	connection->connection_rx.Graphic_Init.last_flag=connection->connection_rx.Graphic_Init.flag;
	
}

void Chassis_Speed_Command_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[])
{
				connection->connection_rx.vx = (int16_t)(data[0]<<8|data[1]);
        connection->connection_rx.vy = (int16_t)(data[2]<<8|data[3]);
        connection->connection_rx.vw = (int16_t)(data[4]<<8|data[5]);	
}

void Chassis_Control_Mode_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[])
{
				
				connection->connection_rx.mode = data[0];
        connection->connection_rx.invert.flag = data[1];
        connection->connection_rx.follow.flag = data[2];
				connection->connection_rx.fric.flag	=	data[3];
				connection->connection_rx.vision.flag	=	data[4];
				connection->connection_rx.Graphic_Init.flag=data[5];
				connection->connection_rx.spin_direction_flag=data[6];
}

void Fric_Speed_And_Pitch_Angle_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[])
{
				memcpy(&connection->connection_rx.fric_speed,&data[0],2);
				memcpy(&connection->connection_rx.pitch_angle,&data[2],4);
}

void	Shoot_Speed_Update(void)
{
			if(actual_speed==0&&JudgeReceive.shoot_data.initial_speed==0)
				actual_speed=15.2f;
			if(JudgeReceive.shoot_data.initial_speed!=0)
				actual_speed=JudgeReceive.shoot_data.initial_speed;
}

void Outpost_HP_Update(void)
{
			if(JudgeReceive.robot_state.robot_id<=11)
				outpost_hp=JudgeReceive.robot_hp.blue_outpost_HP;
			else
				outpost_hp=JudgeReceive.robot_hp.red_outpost_HP;
}

void Send_Speed_And_State_Task(void)
{
				CAN1_0xxf1_Tx_Data[0]=JudgeReceive.game_status.game_progress;
				Shoot_Speed_Update();
				memcpy(&CAN1_0xxf1_Tx_Data[1],&actual_speed,4);
				memcpy(&CAN1_0xxf1_Tx_Data[5],&outpost_hp,2);
				CAN_Send_Data(&hcan1,0xf1,CAN1_0xxf1_Tx_Data,8);
				
}