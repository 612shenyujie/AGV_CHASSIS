#include "vision.h"


VISION_T vision_control;

const float g = 10.2 ; 
const float bullet_v = 15.0; 

float x_offset;
float y_offset;
float z_offset=0.235;

void Vision_Angle_Task(float target_yaw_angle,float target_pitch_angle)
{

	vision_control.command.yaw_angle	=	-target_yaw_angle;
	vision_control.command.pitch_angle	=	-target_pitch_angle;

};

void Vision_Aim_Data_Task(float x,float y,float z)
{
	vision_control.command.x	=	x+x_offset;
	vision_control.command.y	=	y+y_offset;
	vision_control.command.z	=	z+z_offset;
//	vision_control.command.x	=	x;
//	vision_control.command.y	=	y;
//	vision_control.command.z	=	z;
	
}

void Vision_Send_Task(void)
{
	DMA_Send();

}

/**
 * ??????????(yaw)?
 * 
 * @param x ???x??
 * @param y ???y??
 * @param z ???z??(???)
 * @return ????????(??????)
 */
float calc_yaw(float x, float y, float z) {
    // ?? atan2f ????????,?????????
    float yaw;
		if(x==0 &&y>0) yaw=3.1415926/2.0f;
		else if(x==0 &&y<0) yaw=-3.1415926/2.0f;
		else  yaw = atan2f(y, x);
    // ??????????????
    yaw = -(yaw * 180 / 3.1415926); // ????,????

    return yaw;
}

/**
 * ??????????????
 * 
 * @param x ???x??
 * @param y ???y??
 * @param z ???z??
 * @return ???????????
 */
float calc_distance(float x, float y, float z) {
    // ?????????,??????????????
    float distance = sqrtf(x * x + y * y + z * z);
		if(x==0&&y==0&&z==0)
			distance=0;
    return distance;
}

/**
 * ??????????(pitch)?
 * 
 * @param x ???x??
 * @param y ???y??
 * @param z ???z??
 * @return ????????(??????)
 */
float calc_pitch(float x, float y, float z) {
    // ?? x?y ????????????? z ?????????,?????????
  float pitch;  
	float temp_distance;
	if(x==0&&z>0)pitch=3.1415926/2.0f;
	else if(x==0&&z<0)pitch=-3.1415926/2.0f;
	 else pitch = atan2f(z, sqrtf(x * x ));
		
    // ????????????????
    for (int16_t i = 0; i < 20; i++) {
        float v_x = bullet_v * cosf(pitch);
        float v_y = bullet_v * sinf(pitch);

        float t = sqrtf(x * x + y * y) / v_x;
        float h = v_y * t - 0.5 * g * t * t;
        float dz = z - h;

        if (fabsf(dz) < 0.01) {
            break;
        }

        // ?? dz ?????????????????????,??????
				temp_distance=calc_distance(x, y, z);
				if(temp_distance==0)
					break;
        pitch += asinf(dz /temp_distance );
    }

    // ??????????????
    pitch = -(pitch * 180 / 3.1415926); // ????,????

    return pitch;
}
/**
 * ????yaw,pitch
 * 
 * @param x ???x??
 * @param y ???y??
 * @param z ???z??
 * @return ????????(??????)
 */
float temp_pitch=1.8f;
void Self_aim(float x,float y,float z,float *yaw,float *pitch,float *distance)
{
    *yaw = calc_yaw(x, y, z);
    *pitch = calc_pitch(x, y, z);
    *distance = calc_distance(x, y, z);
}