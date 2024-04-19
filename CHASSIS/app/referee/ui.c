#include "ui.h"
#include "referee.h"
#include "usart.h"
#include	"gimbal_connection.h"

unsigned char JudgeSend[SEND_MAX_SIZE];
int Char_Change_Array[7];					//0表示没变化，非0表示有变化
int Graphic_Change_Array[7];
float relative_angle,last_relative_angle;
char	spin_mode,last_spin_mode;
char LowHP_Flag,lastLowHP_Flag;									//低血量警报
ext_student_interactive_char_header_data_t custom_char_draw;
ext_student_interactive_header_data_t custom_grapic_draw;
char change_cnt[7];
	int32_t temp_pitch;
float c_pos_x[12] = {0.01,0.01,0.01,0.9,0.9, 0.01,0.64, 0.54,0.40,0.53,0.3,0.4};
float c_pos_y[12] = {0.8,0.7,0.6,0.8 ,0.75, 0.5,0.1, 0.05,0.1 ,0.15,0.5,0.7};
float g_pos_x[CAP_GRAPHIC_NUM] = {0.05,0.05,0.05,0.5,0.05,0.459,0.62,0.5,0.42};
float g_pos_y[CAP_GRAPHIC_NUM] = {0.74,0.64,0.54,0.5,0.44,0.36,0.1,0.8,0.1};
float g_line_x[20]={0.657,0.541,0.343,0.459,0.5,0.5,0.47,0.53};
float g_line_y[20]={0.0,0.36,0.0,0.36,0.0,0.5,0.392,0.392};
/*瞄准线偏移量*/
int AIM_bias_x = 0;
int AIM_bias_y = 0;
int placece_x[14]={0  , 50, 30,  30, 30,  10,  7,  7,  7,  10,  7,  7,  7 ,10 };
int placece_y[15]={-80,-320,-80,-100,-120,-140,-160,-180,-200,-220,-240,-260,-280,10, 10 };
int Graphic_Change_Check(void)
{
	int i;
	/*用于初始化基本图形，如车道线，框线，和辅助瞄准线等*/
	if(connection.connection_rx.Graphic_Init.flag == 0)		
	{
		return Op_Init;	//返回Init,会一直发送Add，添加所有图层
	}
	
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
	
		if(fabs(relative_angle-last_relative_angle)>0.1f)
	{
		last_relative_angle=relative_angle;
		Graphic_Change_Array[3]=Op_Change;
		change_cnt[3]=10;
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
	
	
	last_spin_mode=spin_mode;
	
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
	if(fabs(connection.connection_rx.pitch_angle-connection.connection_rx.last_pitch_angle)>0.2)
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
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
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
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
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
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
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
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
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
				PITCH_:	
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 5;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=5;
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
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
			case 5:
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 6;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("Spin Mode:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[5]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[5]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"Spin Mode:");
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
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=3;
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
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=4;
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
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=g_line_x[4] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=g_line_y[4] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_x=g_line_x[5] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_y=g_line_y[5] * SCREEN_WIDTH;
				/*1m瞄准线*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[1] = 4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].layer=3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_x=g_line_x[6] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_y=g_line_y[6] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_x=g_line_x[7] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_y=g_line_y[7] * SCREEN_WIDTH;
				
				/*3m瞄准线*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[1] = 5;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].layer=3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_x=g_line_x[6] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_y=g_line_y[6] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_x=g_line_x[7] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_y=g_line_y[7] * SCREEN_WIDTH;
				
				/*5m瞄准线*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[1] = 6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].layer=3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_x=g_line_x[6] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_y=g_line_y[6] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_x=g_line_x[7] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_y=g_line_y[7] * SCREEN_WIDTH;
				
				/*7m瞄准线*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[1] = 7;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].layer=3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_x=g_line_x[6] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_y=g_line_y[6] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_x=g_line_x[7] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_y=g_line_y[7] * SCREEN_WIDTH;
				break;
			case 1:
			
			/*停车线基础版*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=2;
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
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=3;
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
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=4;
				if(connection.connection_rx.follow.flag)
			custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=g_pos_x[2] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=g_pos_y[2] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].radius=20;
			if(Op_type == Op_Change) goto CONT_2;
			break;
			SPIN_MODE:		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=4;
				if(spin_mode)
			custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=g_pos_x[4] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=g_pos_y[4] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].radius=20;
			if(Op_type == Op_Change) goto CONT_4;
			case 2:
					RELATIVE_:

			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 0;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 6;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=4;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=9;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Cyan; 
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=10;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=g_pos_x[3]*SCREEN_LENGTH;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=g_pos_x[3]*SCREEN_WIDTH;
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