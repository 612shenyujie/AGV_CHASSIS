#include "agv_control.h"
#include "chassis.h"
#include "referee.h"
 chassis_power_control_t chassis_power_control;



void Set_AGV_Velocity_Vector_Data_Update(uint8_t tx_data[],int16_t	angle,int16_t speed,float power_limition)
{
	
		memcpy(&tx_data[0],&angle,2);
		memcpy(&tx_data[2],&speed,2);
		memcpy(&tx_data[4],&power_limition,4);
		
	
	
}
void AGV_connoection(int ms_cnt)
{
	 if( ms_cnt%12==0)
    {
        
        if(chassis.parameter.mode != CHASSIS_REMOTE_CLOSE&&!chassis.A_motor.active_status)
        {
								memset(AGV_A_Tx_Data,0,8);
								chassis.A_motor.active_status=1;
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.A_motor.ID,0,0x01),AGV_A_Tx_Data,8);
        }
				else if(chassis.parameter.mode == CHASSIS_REMOTE_CLOSE&&chassis.A_motor.active_status)
				{
								memset(AGV_A_Tx_Data,0,8);
								chassis.A_motor.active_status=0;
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.A_motor.ID,0,0x00),AGV_A_Tx_Data,8);
				}
				else  
        {          
							  Set_AGV_Velocity_Vector_Data_Update(AGV_A_Tx_Data, chassis.A_motor.target_angle, chassis.A_motor.target_speed.output, chassis_power_control.scaled_power_32[0]);
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.A_motor.ID,(int16_t)(chassis.parameter.power_limition_k*100),0x03),AGV_A_Tx_Data,8);     
        
				}
				   
    }
		if( ms_cnt%12==3)
    {
        if(chassis.parameter.mode != CHASSIS_REMOTE_CLOSE&&!chassis.B_motor.active_status)
        {
								memset(AGV_B_Tx_Data,0,8);
								chassis.B_motor.active_status=1;
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.B_motor.ID,0,0x01),AGV_B_Tx_Data,8);
        }
				else if(chassis.parameter.mode == CHASSIS_REMOTE_CLOSE&&chassis.B_motor.active_status)
				{
								memset(AGV_B_Tx_Data,0,8);
								chassis.B_motor.active_status=0;
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.B_motor.ID,0,0x00),AGV_B_Tx_Data,8);
				}
				else  
        {          
							  Set_AGV_Velocity_Vector_Data_Update(AGV_B_Tx_Data, chassis.B_motor.target_angle, chassis.B_motor.target_speed.output, chassis_power_control.scaled_power_32[1]);
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.B_motor.ID,(int16_t)(chassis.parameter.power_limition_k*100),0x03),AGV_B_Tx_Data,8);     
        
				}
    }
		if( ms_cnt%12==6)
    {
        if(chassis.parameter.mode != CHASSIS_REMOTE_CLOSE&&!chassis.C_motor.active_status)
        {
								memset(AGV_C_Tx_Data,0,8);
								chassis.C_motor.active_status=1;
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.C_motor.ID,0,0x01),AGV_C_Tx_Data,8);
        }
				else if(chassis.parameter.mode == CHASSIS_REMOTE_CLOSE&&chassis.C_motor.active_status)
				{
								memset(AGV_C_Tx_Data,0,8);
								chassis.C_motor.active_status=0;
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.C_motor.ID,0,0x00),AGV_C_Tx_Data,8);
				}
				else  
        {          
							  Set_AGV_Velocity_Vector_Data_Update(AGV_C_Tx_Data, chassis.C_motor.target_angle, chassis.C_motor.target_speed.output, chassis_power_control.scaled_power_32[2]);
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.C_motor.ID,(int16_t)(chassis.parameter.power_limition_k*100),0x03),AGV_C_Tx_Data,8);     
        
				}  
    }
		if( ms_cnt%12==9)
    {
        if(chassis.parameter.mode != CHASSIS_REMOTE_CLOSE&&!chassis.D_motor.active_status)
        {
								memset(AGV_D_Tx_Data,0,8);
								chassis.D_motor.active_status=1;
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.D_motor.ID,0,0x01),AGV_D_Tx_Data,8);
        }
				else if(chassis.parameter.mode == CHASSIS_REMOTE_CLOSE&&chassis.D_motor.active_status)
				{
								memset(AGV_D_Tx_Data,0,8);
								chassis.D_motor.active_status=0;
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.D_motor.ID,0,0x00),AGV_D_Tx_Data,8);
				}
				else  
        {          
							  Set_AGV_Velocity_Vector_Data_Update(AGV_D_Tx_Data, chassis.D_motor.target_angle, chassis.D_motor.target_speed.output, chassis_power_control.scaled_power_32[3]);
								CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.D_motor.ID,(int16_t)(chassis.parameter.power_limition_k*100),0x03),AGV_D_Tx_Data,8);     
        
				}
    }
		
	
}



float total_power;


void calculate_true_power(void)
{
    
	float sum = 0;
		
	switch(chassis.parameter.power_loop)
	{
		case SUPERCAP_LOOP:
			if(chassis.supercap.supercap_per>80.f)
				chassis_power_control.power_limit_max   =   JudgeReceive.robot_state.MaxPower+20.f;
			else if(chassis.supercap.supercap_per>40.f)
				chassis_power_control.power_limit_max   =   JudgeReceive.robot_state.MaxPower+(chassis.supercap.supercap_per-40.f)*20.f;
			else
				chassis_power_control.power_limit_max   =   JudgeReceive.robot_state.MaxPower;
			break;
		case BUFFER_LOOP:
			chassis_power_control.power_limit_max   =   JudgeReceive.robot_state.MaxPower;
			break;
	}
	
			


	if(chassis_power_control.all_mscb_ready_flag&0xf)
	{
    for (uint8_t i = 0; i < 4; i++)
    {
       if(chassis_power_control.expect_power_32[i]<0)
				 continue;
			sum+=chassis_power_control.expect_power_32[i];
    }
		chassis_power_control.total_expect_power=sum;
    chassis_power_control.scaled_power_coefficient_32 = (chassis_power_control.power_limit_max) / sum;

    if (chassis_power_control.scaled_power_coefficient_32<=1)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_power_control.scaled_power_32[i] = chassis_power_control.scaled_power_coefficient_32 * chassis_power_control.expect_power_32[i];
        }
    }
    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_power_control.scaled_power_32[i] = chassis_power_control.expect_power_32[i];
        }

    }
    chassis_power_control.all_mscb_ready_flag   =   0;
	}  
}

void Chassis_Power_Control_Init(void)
{
   for(int i=0;i<4;i++)
	{
		chassis_power_control.scaled_power_32[i]=10.0f;
	}
}