
#include "SW_control_task.h"
#include "chassis_power_control.h"
steering_wheel_t steering_wheel;

int16_t probe;
void SW_control_task(void)
{
    if(steering_wheel.parameter.enable)
	{
        
       
				if(steering_wheel.parameter.receive_ms_count+steering_wheel.parameter.receive_s_count*1000-ms_count-s_count*1000<-1000)
				steering_wheel.parameter.connection_state=0;
				Steering_Wheel_CommandUpdate(&steering_wheel);
        Steering_Wheel_StatusUpdate(&steering_wheel);
				Steering_Wheel_CommandTransmit(&steering_wheel);
		
		
		//ɾ������
		//probe = steering_wheel.directive_part.motor.M3508_kit.parameter.bus->motor[1].feedback.LSB_rotor_rpm;


   }
		else
		{
		steering_wheel.motion_part.motor.M3508_kit.command.torque=0;
			steering_wheel.directive_part.motor.M3508_kit.command.torque=0;
		Steering_Wheel_CommandTransmit(&steering_wheel);
		}
        
     
    

}

void SW_subscribe_task(void)
{
    steering_communication_SubscribeList_Scheduler(&steering_wheel);
}

void SW_control_task_init(void)
{

    Steering_Wheel_HandleInit(&steering_wheel);
	steering_communication_init();
}
