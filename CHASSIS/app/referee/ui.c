#include "ui.h"
#include "referee.h"
#include "usart.h"
#include	"gimbal_connection.h"

unsigned char JudgeSend[SEND_MAX_SIZE];
int Char_Change_Array[7];					//0表示没变化，非0表示有变化
int Graphic_Change_Array[7];
float relative_angle,last_relative_angle;
float cap_per,last_cap_per;
char	spin_mode,last_spin_mode;
char LowHP_Flag,lastLowHP_Flag;									//低血量警报
ext_student_interactive_char_header_data_t custom_char_draw;
ext_student_interactive_header_data_t custom_grapic_draw;
char change_cnt[7];
	int32_t temp_pitch;
	int32_t temp_supercap_per;
	int32_t	last_temp_supercap_per;
	char	precision_state,last_precision_state;
	float	distance_1,distance_2,distance_3,delta_x,delta_y;
float c_pos_x[12] = {0.15,0.15,0.15,0.71,0.71, 0.71,0.55, 0.54,0.40,0.53,0.3,0.4};
float c_pos_y[12] = {0.8,0.7,0.6,0.8 ,0.75, 0.7,0.5, 0.05,0.1 ,0.15,0.5,0.7};
float g_pos_x[CAP_GRAPHIC_NUM] = {0.2,0.2,0.2,0.5,0.75,0.459,0.62,0.5,0.3,0.7,0.3,0.3,0.684};
float g_pos_y[CAP_GRAPHIC_NUM] = {0.74,0.64,0.54,0.8,0.64,0.36,0.1,0.8,0.3,0.75,0.15,0.14,0.16};
float g_line_x[20]={0.657,0.541,0.343,0.459,0.5,0.5,0.49,0.51,0.48,0.52,0.47,0.53,0.46,0.54,0.478,0.490,0.484,0.484};
float g_line_y[20]={0.0,0.36,0.0,0.36,0.0,1.0,0.618,0.618,0.585,0.585,0.551,0.551,0.444,0.444,0.538,0.538,0.544,0.533};
/*瞄准线偏移量*/

float target_x_1,target_x_2,target_x_3,target_y_1,target_y_2,target_y_3;

float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}		

int Graphic_Change_Check(void)
{
	int i;
	/*用于初始化基本图形，如车道线，框线，和辅助瞄准线等*/
	if(connection.connection_rx.Graphic_Init.flag == 0)		
	{
		return Op_Init;	//返回Init,会一直发送Add，添加所有图层
	}
	
	
		cap_per=chassis.supercap.supercap_per;
		if(JudgeReceive.robot_state.robot_id<101)
		{
			target_x_1=19.01f;
			target_y_1=7.45f;
			target_x_2=12.85f;
			target_y_2=15.22f;
			target_x_3=10.44f;
			target_y_3=14.09f;
		}
		else
		{
			target_x_1=9.98f;
			target_y_1=8.54f;
			target_x_2=16.29f;
			target_y_2=1.13f;
			target_x_3=18.61f;
			target_y_3=2.40f;
		}
		delta_x=JudgeReceive.robot_pos_t.x-target_x_1;
		delta_y=JudgeReceive.robot_pos_t.y-target_y_1;
	distance_1=Sqrt(delta_x*delta_x+delta_y*delta_y);
	delta_x=JudgeReceive.robot_pos_t.x-target_x_2;
		delta_y=JudgeReceive.robot_pos_t.y-target_y_2;
	distance_2=Sqrt(delta_x*delta_x+delta_y*delta_y);
		delta_x=JudgeReceive.robot_pos_t.x-target_x_3;
		delta_y=JudgeReceive.robot_pos_t.y-target_y_3;
	distance_3=Sqrt(delta_x*delta_x+delta_y*delta_y);
	if(distance_2<0.25||distance_1<0.25||distance_3<0.25)
		precision_state	=	1;
	else
		precision_state	=	0;
	if(connection.connection_rx.fric.flag!=connection.connection_rx.fric.last_flag)
	{
		Graphic_Change_Array[0]=Op_Change;
		change_cnt[0]=5;
	
	}
	
		if(connection.connection_rx.vision.flag!=connection.connection_rx.vision.last_flag)
	{
		Graphic_Change_Array[1]=Op_Change;
		change_cnt[1]=5;
	
	}
	if(connection.connection_rx.follow.flag!=connection.connection_rx.follow.last_flag)
	{
		Graphic_Change_Array[2]=Op_Change;
		change_cnt[2]=5;
	}
	
		if(fabs(relative_angle-last_relative_angle)>0.01f)
	{
		last_relative_angle=relative_angle;
		Graphic_Change_Array[3]=Op_Change;
		change_cnt[3]=10;
	}
	if(cap_per-last_cap_per>0.5f||cap_per-last_cap_per<-0.5f)
	{
		last_cap_per=cap_per;
		Graphic_Change_Array[6]=Op_Change;
		change_cnt[6]=10;
	}
	
	if(connection.connection_rx.mode==CHASSIS_SPIN)
	spin_mode=1;
	else
		spin_mode=0;
	
	
	if(spin_mode!=last_spin_mode)
	{
		Graphic_Change_Array[4]=Op_Change;
		change_cnt[4]=5;
	}
	
		if(precision_state!=last_precision_state)
	{
		Graphic_Change_Array[5]=Op_Change;
		change_cnt[5]=5;
	
	}
	last_spin_mode=spin_mode;
	last_precision_state=precision_state;
	for(i = 0;i<7;i++)
	{
		if(Graphic_Change_Array[i] == Op_Change)
			return Op_Change;
	}
	return Op_None;	//返回空操作
}

int Char_Change_Check(void)
{
	int i;



	/*用于图形界面初始化*/
	if(connection.connection_rx.Graphic_Init.flag == 0)		
	{

		return Op_Init;	//返回Init,会使一直发送Add，添加所有图层
	}

	
		
	/*读取云台发送的各种状态*/
	
	LowHP_Flag = JudgeReceive.robot_state.maxHP * 0.35 > JudgeReceive.robot_state.remainHP ? 1:0;
	
	
	/*有变化，标志各个位*/


	temp_pitch=-connection.connection_rx.pitch_angle*1000;
	if(fabs(connection.connection_rx.pitch_angle-connection.connection_rx.last_pitch_angle)>0.01)
	{
		connection.connection_rx.last_pitch_angle=connection.connection_rx.pitch_angle;
		Char_Change_Array[3]=Op_Change;
		change_cnt[3]=5;
	}


	
	/*保存这次标志和上次比较*/

	lastLowHP_Flag = LowHP_Flag;
	
	
	
	/*检索有没有发生变化，如果有变化则返回修改图层*/
	for(i = 0;i<7;i++)
	{
		if(Char_Change_Array[i] == Op_Change)
			return Op_Change;
	}
	
	return Op_None;	//否则返回空操作，此时不会发送东西
}

void referee_data_load_String(int Op_type)
{
	static int tick=0;
	static char Fric_State[2][6] = {"CLOSE","OPEN"};
	static char Vision_State[2][6] = {"CLOSE","OPEN"};
	static char Chassis_State[4][9] = {"NOForce","Normal","SPIN","PRECISION"};
	static char Gimbal_State[4][9] = {"NOForce","Normal","SPIN","PRECISION"};
	static char Power_State[2][4] = {"Bat","Cap"};

	/*初始化操作，轮流生成图层*/
	if(Op_type == Op_Init)
	{
		switch(tick%11)
		{
			/*静态字符*/
			
			case 0:
			/*******************************Fric Mode 字符*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 1;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=1;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("Fric Mode:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[0]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[0]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"Fric Mode:");
			break;
			case 1:
			/*******************************Auto Aim Mode 字符*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 2;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=2;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange; 
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen(" Auto Aim:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[1]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[1]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data," Auto Aim:");
			break;
			case	2:
				/*******************************Follow Mode 字符*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 3;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=3;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange; 
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("Follow Mode:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[2]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[2]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"Follow Mode:");
			break;
			case	3:
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 4;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=4;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange; 
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("Pitch:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[3]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[3]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"Pitch:");
			break;
			case 4:
				custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 7;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=1;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("Spin Mode:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[5]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[5]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"Spin Mode:");
				break;
				
			case 5:
			PITCH_:	
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 5;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=5;
			custom_char_draw.char_custom.grapic_data_struct.layer=4;
			custom_char_draw.char_custom.grapic_data_struct.color=Purple; 
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[4]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[4]*SCREEN_WIDTH;
			custom_char_draw.char_custom.grapic_data_struct.radius=temp_pitch&0x3ff;
			custom_char_draw.char_custom.grapic_data_struct.end_x=temp_pitch>>10&0x7ff;
			custom_char_draw.char_custom.grapic_data_struct.end_y=temp_pitch>>21&0x7ff;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			break;
			case	6:
			
			break;
			
			default:
			break;
		}
		tick++;
		return ;
		
	}else if(Op_type == Op_Change)		//如果是标志为修改
	{
		/*寻找是哪个发生了变化*/
	if(Char_Change_Array[0] == Op_Change)  
		{
			if(change_cnt[0]>0)
			{
			 change_cnt[0] -- ;
			}
			else
			{
			Char_Change_Array[0] = Op_None;
			}
		}
			if(Char_Change_Array[1] == Op_Change)  
		{
			if(change_cnt[1]>0)
			{
			 change_cnt[1] -- ;
			}
			else
			{
			Char_Change_Array[1] = Op_None;
			}
		}
			if(Char_Change_Array[2] == Op_Change)  
		{
			if(change_cnt[2]>0)
			{
			 change_cnt[2] -- ;
			}
			else
			{
			Char_Change_Array[2] = Op_None;
			}
		}
			if(Char_Change_Array[3] == Op_Change)  
		{
			if(change_cnt[3]>0)
			{
			 change_cnt[3] -- ;
				goto PITCH_;
			}
			else
			{
			Char_Change_Array[3] = Op_None;
			}
		}
		if(Char_Change_Array[4] == Op_Change)  
		{
			if(change_cnt[4]>0)
			{
			 change_cnt[4] -- ;
			
			}
			else
			{
			Char_Change_Array[4] = Op_None;
			}
		}
		if(Char_Change_Array[5] == Op_Change)  
		{
			if(change_cnt[5]>0)
			{
			 change_cnt[5] -- ;
			}
			else
			{
			Char_Change_Array[5] = Op_None;
			}
			
		}
		
	
	}
}

void referee_data_load_Graphic(int Op_type)
{
	static int pack_tick = 0;			//数据包计数器
	static int i;
	static float start_angle,end_angle;
	int packed_tick = 0;							//装包计数器
	/*初始化操作，轮流生成图层*/
	if(Op_type == Op_Init)
	{
		switch(pack_tick % PACK_NUM)
		{
		case 0:
			/*******************************Fric Mode 图案*********************************/
		FRIC_MODE:		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=1;
				if(connection.connection_rx.fric.flag==2)
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Purple;		
				else if(connection.connection_rx.fric.flag==1)
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Green;
				else
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=g_pos_x[0] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=g_pos_y[0] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius=20;
				if(Op_type == Op_Change) goto CONT_0;	
				/*******************************Auto Aim Mode 图案*********************************/
		AUTO_AIM:		custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=2;
				if(connection.connection_rx.vision.flag)
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=g_pos_x[1] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=g_pos_y[1] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].radius=20;
			if(Op_type == Op_Change) goto CONT_1;
				
				/*瞄准线基础版*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=g_line_x[4] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=g_line_y[4] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_x=g_line_x[5] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_y=g_line_y[5] * SCREEN_WIDTH;
				/*1m瞄准线*//*3m瞄准线*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[1] = 4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_x=g_line_x[6] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_y=g_line_y[6] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_x=g_line_x[7] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_y=g_line_y[7] * SCREEN_WIDTH;
				
				/*5m瞄准线*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[1] = 5;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_x=g_line_x[8] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_y=g_line_y[8] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_x=g_line_x[9] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_y=g_line_y[9] * SCREEN_WIDTH;
				
				/*7m瞄准线*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[1] = 6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_x=g_line_x[10] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_y=g_line_y[10] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_x=g_line_x[11] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_y=g_line_y[11] * SCREEN_WIDTH;
				
				/*11m瞄准线*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[1] = 7;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_x=g_line_x[12] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_y=g_line_y[12] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_x=g_line_x[13] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_y=g_line_y[13] * SCREEN_WIDTH;
				break;
			case 1:
			
			/*停车线基础版*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=g_line_x[0] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=g_line_y[0] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x=g_line_x[1] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y=g_line_y[1] * SCREEN_WIDTH;
			
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=g_line_x[2] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=g_line_y[2] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_x=g_line_x[3] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_y=g_line_y[3] * SCREEN_WIDTH;
			/*Follow Mode图案*/
			FOLLOW_MODE:		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=3;
				if(connection.connection_rx.follow.flag)
			custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=g_pos_x[2] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=g_pos_y[2] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].radius=20;
			if(Op_type == Op_Change) goto CONT_2;
		
			SPIN_MODE:		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[1] = 4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].layer=4;
				if(spin_mode)
			custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_x=g_pos_x[4] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_y=g_pos_y[4] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].radius=20;
			if(Op_type == Op_Change) goto CONT_4;
			PRECISION:
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[1] = 5;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_tpye=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].layer=6;
			if(precision_state)
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].color=Red_Blue;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_x=g_pos_x[8] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_y=g_pos_y[8] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_x=g_pos_x[9] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_y=g_pos_y[9] * SCREEN_WIDTH;
				if(Op_type == Op_Change) goto CONT_5;
			
//							/*公路区瞄准线*/
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[0] = 1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[1] = 6;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[2] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_tpye=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].layer=6;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].color=Green;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].width=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_x=g_line_x[14] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_y=g_line_y[14] * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_x=g_line_x[15] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_y=g_line_y[15] * SCREEN_WIDTH;
//				
//				/*11m瞄准线*/
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[0] = 1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[1] = 7;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[2] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_tpye=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].layer=6;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].color=Green;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].width=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_x=g_line_x[16] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_y=g_line_y[16] * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_x=g_line_x[17] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_y=g_line_y[17] * SCREEN_WIDTH;
				break;
				break;
			case 2:
					RELATIVE_:

			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 3;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 1;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=4;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=5;
			if(connection.connection_rx.invert.flag)
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Cyan; 
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Yellow; 
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=10;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=g_pos_x[3]*SCREEN_LENGTH;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=g_pos_y[3]*SCREEN_WIDTH;
			start_angle=relative_angle-50.f;
			if(start_angle>180.f)start_angle-=360.f;
			if(start_angle<-180.f)start_angle+=360.f;
			end_angle=relative_angle+50.f;
			if(end_angle>180.f)end_angle-=360.f;
			if(end_angle<-180.f)end_angle+=360.f;
			if(start_angle<0)
				start_angle+=360.f;
			if(end_angle<0)
				end_angle+=360.f;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle=start_angle;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle=end_angle;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x=40;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y=40;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
				if(Op_type == Op_Change) goto CONT_3;
		
			CAP_:
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 3;
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 2;
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 0;
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=Op_type;
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=0;
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=5;
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=Green;  
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=20;
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=g_pos_x[10]*SCREEN_LENGTH;
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=g_pos_y[10]*SCREEN_WIDTH;
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_x=g_pos_x[10]*SCREEN_LENGTH+cap_per*0.004f*SCREEN_LENGTH;
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_y=g_pos_y[10]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
				if(Op_type == Op_Change) goto CONT_6;
			custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Pink;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=g_pos_x[11] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=g_pos_y[11] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_x=g_pos_x[12] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_y=g_pos_y[12] * SCREEN_WIDTH;
				break;
			
			default:
				break;
		}
		pack_tick++;
	}
	else if(Op_type == Op_Change)
	{
			if(Graphic_Change_Array[0] == Op_Change)  
		{
			goto FRIC_MODE;
	CONT_0:
			if(change_cnt[0])
				change_cnt[0]--;
			else
			Graphic_Change_Array[0]=Op_None;
		}
			if(Graphic_Change_Array[1] == Op_Change)  
		{
			goto AUTO_AIM;
			CONT_1:
			if(change_cnt[1])
				change_cnt[1]--;
			else
			Graphic_Change_Array[1]=Op_None;
		}
		if(Graphic_Change_Array[2] == Op_Change)  
		{
			goto FOLLOW_MODE;
	CONT_2:
			if(change_cnt[2])
				change_cnt[2]--;
			else
			Graphic_Change_Array[2]=Op_None;
		}
		if(Graphic_Change_Array[3] == Op_Change)  
		{
			goto RELATIVE_;
	CONT_3:
			if(change_cnt[3])
				change_cnt[3]--;
			else
			Graphic_Change_Array[3]=Op_None;
		}
		if(Graphic_Change_Array[4] == Op_Change)  
		{
			goto SPIN_MODE;
	CONT_4:
			if(change_cnt[4])
				change_cnt[4]--;
			else
			Graphic_Change_Array[4]=Op_None;
		}
		if(Graphic_Change_Array[5] == Op_Change)  
		{
			goto PRECISION;
	CONT_5:
			if(change_cnt[5])
				change_cnt[5]--;
			else
			Graphic_Change_Array[5]=Op_None;
		}
		if(Graphic_Change_Array[6] == Op_Change)  
		{
			goto CAP_;
	CONT_6:
			if(change_cnt[6])
				change_cnt[6]--;
			else
			Graphic_Change_Array[6]=Op_None;
		}
	}
}

void JudgementCustomizeChar(int Op_type)
{
		custom_char_draw.data_cmd_id=0x0110;//绘制字符

		custom_char_draw.sender_ID=JudgeReceive.robot_state.robot_id;//发送者ID，机器人对应ID
		if(JudgeReceive.robot_state.robot_id == 101)
				custom_char_draw.receiver_ID = 0x0165;
		if(JudgeReceive.robot_state.robot_id == 1)
				custom_char_draw.receiver_ID = 0x0101;
		if(JudgeReceive.robot_state.robot_id == 103)
				custom_char_draw.receiver_ID = 0x0167;
		if(JudgeReceive.robot_state.robot_id == 104)
				custom_char_draw.receiver_ID = 0x0168;
		if(JudgeReceive.robot_state.robot_id == 105)
				custom_char_draw.receiver_ID = 0x0169;
		if(JudgeReceive.robot_state.robot_id == 3)
				custom_char_draw.receiver_ID = 0x0103;	
		if(JudgeReceive.robot_state.robot_id == 4)
				custom_char_draw.receiver_ID = 0x0104;
		if(JudgeReceive.robot_state.robot_id == 5)
				custom_char_draw.receiver_ID = 0x0105;

/*********************************自定义字符数据***********************************/
		referee_data_load_String(Op_type);
}

void JudgementCustomizeGraphics(int Op_type)
{
		custom_grapic_draw.data_cmd_id=0x0104;//绘制七个图形（内容ID，查询裁判系统手册）

		custom_grapic_draw.sender_ID=JudgeReceive.robot_state.robot_id;//发送者ID，机器人对应ID
		if(JudgeReceive.robot_state.robot_id == 101)
				custom_grapic_draw.receiver_ID = 0x0165;
		if(JudgeReceive.robot_state.robot_id == 1)
				custom_grapic_draw.receiver_ID = 0x0101;
		if(JudgeReceive.robot_state.robot_id == 103)
				custom_grapic_draw.receiver_ID = 0x0167;
		if(JudgeReceive.robot_state.robot_id == 104)
				custom_grapic_draw.receiver_ID = 0x0168;
		if(JudgeReceive.robot_state.robot_id == 105)
				custom_grapic_draw.receiver_ID = 0x0169;
		if(JudgeReceive.robot_state.robot_id == 3)
				custom_grapic_draw.receiver_ID = 0x0103;	
		if(JudgeReceive.robot_state.robot_id == 4)
				custom_grapic_draw.receiver_ID = 0x0104;
		if(JudgeReceive.robot_state.robot_id == 5)
				custom_grapic_draw.receiver_ID = 0x0105;
			

/*********************************自定义图像数据***********************************/
		referee_data_load_Graphic(Op_type);
}

uint8_t seq = 0;
void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //数据帧长度	

	memset(JudgeSend,0,frame_length);  //存储数据的数组清零
	
	/*****帧头打包*****/
	JudgeSend[0] = sof;//数据帧起始字节
	memcpy(&JudgeSend[1],(uint8_t*)&len, sizeof(len));//数据帧中data的长度
	JudgeSend[3] = seq;//包序号
	Append_CRC8_Check_Sum(JudgeSend,frameheader_len);  //帧头校验CRC8

	/*****命令码打包*****/
	memcpy(&JudgeSend[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****数据打包*****/
	memcpy(&JudgeSend[frameheader_len+cmd_len], p_data, len);
	Append_CRC16_Check_Sum(JudgeSend,frame_length);  //一帧数据校验CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****数据上传*****/

   __HAL_UART_CLEAR_FLAG(&huart6,UART_FLAG_TC);
//	 HAL_UART_Transmit_DMA(&huart6, JudgeSend,frame_length);	
	HAL_UART_Transmit(&huart6, JudgeSend,frame_length,150);
	while (__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC) == RESET); 


	
}

int char_change_state,graphic_change_state;

void UI_Send_Graphic_Task(void)
{
			graphic_change_state = Graphic_Change_Check();
			if(graphic_change_state)
			{
				JudgementCustomizeGraphics(graphic_change_state);
				if(graphic_change_state != Op_None)
				referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_grapic_draw,sizeof(custom_grapic_draw));
			}
}

void UI_Send_Char_Task(void)
{
		char_change_state = Char_Change_Check();
			if(char_change_state)			//检查有没有变化，没有变化就不发，节省带宽
			{
				JudgementCustomizeChar(char_change_state);
				if(char_change_state != Op_None)
					referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_char_draw,sizeof(custom_char_draw));
			}
}