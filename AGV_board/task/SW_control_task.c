
#include "SW_control_task.h"
#include "chassis_power_control.h"
steering_wheel_t steering_wheel;

int16_t probe;
void SW_control_task(void)
{
  #ifndef motor_power_test  
	if(steering_wheel.parameter.enable)
	{
        
       
				if(steering_wheel.parameter.receive_ms_count+steering_wheel.parameter.receive_s_count*1000-ms_count-s_count*1000<-1000)
				steering_wheel.parameter.connection_state=0;
				Steering_Wheel_CommandUpdate(&steering_wheel);
        Steering_Wheel_StatusUpdate(&steering_wheel);
				Steering_Wheel_CommandTransmit(&steering_wheel);
		
		
		//É¾³ý²âÊÔ
		//probe = steering_wheel.directive_part.motor.M3508_kit.parameter.bus->motor[1].feedback.LSB_rotor_rpm;


   }
		else
		{
		steering_wheel.motion_part.motor.M3508_kit.command.torque=0;
		steering_wheel.directive_part.motor.M3508_kit.command.torque=0;
		Steering_Wheel_CommandTransmit(&steering_wheel);
			
		}
      #endif  
    #ifdef motor_power_test
		//step-1
//		briter_encoder_set_baud_rate(&steering_wheel.directive_part.encoder.briter_encoder,BRITER_ENCODER_SET_CAN_BAUD_RATE_1M);
		//step-2
//		briter_encoder_set_callback_mode(&steering_wheel.directive_part.encoder.briter_encoder,BRITER_ENCODER_SET_CALLBACK_REQUEST);
//		//step-3
		briter_encoder_set_increment_direction(&steering_wheel.directive_part.encoder.briter_encoder,BRITER_ENCODER_INCREMENT_DIRECTION_CW);
//		//step-4
//		if(ms_count%10==0)
//			steering_wheel.directive_part.encoder.briter_encoder.parameter.CAN_ID=0x01;
//		briter_encoder_set_CAN_ID(&steering_wheel.directive_part.encoder.briter_encoder,0x0B);
//		if(ms_count%10==0)
//		briter_encoder_request_tatal_angle(&steering_wheel.directive_part.encoder.briter_encoder);
		#endif
    

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
