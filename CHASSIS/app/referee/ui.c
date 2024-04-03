#include "ui.h"
#include "referee.h"
#include "usart.h"
#include	"gimbal_connection.h"

unsigned char JudgeSend[SEND_MAX_SIZE];
int Char_Change_Array[7];					//0��ʾû�仯����0��ʾ�б仯
int Graphic_Change_Array[7];
char LowHP_Flag,lastLowHP_Flag;									//��Ѫ������
char Chassis_State,Chassis_lastState;
ext_student_interactive_char_header_data_t custom_char_draw;
ext_student_interactive_header_data_t custom_grapic_draw;
char change_cnt[7];
float c_pos_x[12] = {0.01,0.01,0.4,0.54,0.3, 0.41,0.64, 0.54,0.40,0.53,0.3,0.4};
float c_pos_y[12] = {0.8,0.7,0.05,0.1 ,0.1, 0.15,0.1, 0.05,0.1 ,0.15,0.5,0.7};
float g_pos_x[CAP_GRAPHIC_NUM] = {0.05,0.05,0.4,0.52,0.34,0.42,0.62,0.5,0.42};
float g_pos_y[CAP_GRAPHIC_NUM] = {0.74,0.64,0.8,0.1,0.1,0.15,0.1,0.8,0.1};
/*��׼��ƫ����*/
int AIM_bias_x = 0;
int AIM_bias_y = 0;
int placece_x[14]={0  , 50, 30,  30, 30,  10,  7,  7,  7,  10,  7,  7,  7 ,10 };
int placece_y[15]={-80,-320,-80,-100,-120,-140,-160,-180,-200,-220,-240,-260,-280,10, 10 };
int Graphic_Change_Check(void)
{
	int i;
	/*���ڳ�ʼ������ͼ�Σ��糵���ߣ����ߣ��͸�����׼�ߵ�*/
	if(connection.connection_rx.Graphic_Init.flag == 0)		
	{
		return Op_Init;	//����Init,��һֱ����Add���������ͼ��
	}
	
	if(connection.connection_rx.fric.flag!=connection.connection_rx.fric.last_flag)
	{
		Graphic_Change_Array[0]=Op_Change;
		change_cnt[0]=5;
	
	}
	
		if(connection.connection_rx.vision.flag!=connection.connection_rx.vision.last_flag)
	{
		Graphic_Change_Array[1]=Op_Change;
		change_cnt[0]=5;
	
	}
	
	for(i = 0;i<7;i++)
	{
		if(Graphic_Change_Array[i] == Op_Change)
			return Op_Change;
	}
	return Op_None;	//���ؿղ���
}

int Char_Change_Check(void)
{
	int i;



	/*����ͼ�ν����ʼ��*/
//	if(connection.connection_rx.Graphic_Init.flag == 0)		
//	{

		return Op_Init;	//����Init,��ʹһֱ����Add���������ͼ��
//	}

	
		
	/*��ȡ��̨���͵ĸ���״̬*/
	
	LowHP_Flag = JudgeReceive.maxHP * 0.35 > JudgeReceive.remainHP ? 1:0;
	Chassis_State=connection.connection_rx.mode;
	
	/*�б仯����־����λ*/

	if(Chassis_lastState!=Chassis_State)
	{
		Char_Change_Array[0]=Op_Change;
		change_cnt[0]	=2;
	}
	
	if(connection.connection_rx.fric.flag!=connection.connection_rx.fric.last_flag)
	{
		Char_Change_Array[1]=Op_Change;
		change_cnt[1]	=2;
	
	}
		if(connection.connection_rx.follow.flag!=connection.connection_rx.follow.last_flag)
	{
		Char_Change_Array[2]=Op_Change;
		change_cnt[2]	=2;
	
	}
			if(connection.connection_rx.invert.flag!=connection.connection_rx.invert.last_flag)
	{
		Char_Change_Array[3]=Op_Change;
		change_cnt[3]	=2;
	
	}
				if(connection.connection_rx.vision.flag!=connection.connection_rx.vision.last_flag)
	{
		Char_Change_Array[4]=Op_Change;
		change_cnt[4]	=2;
	
	}

	if(LowHP_Flag != lastLowHP_Flag)
	{
		Char_Change_Array[5]=Op_Change;
		change_cnt[5] = 2;	  
	}
	
	/*������α�־���ϴαȽ�*/

	lastLowHP_Flag = LowHP_Flag;
	Chassis_lastState=Chassis_State;
	
	
	/*������û�з����仯������б仯�򷵻��޸�ͼ��*/
	for(i = 0;i<7;i++)
	{
		if(Char_Change_Array[i] == Op_Change)
			return Op_Change;
	}
	
	return Op_None;	//���򷵻ؿղ�������ʱ���ᷢ�Ͷ���
}

void referee_data_load_String(int Op_type)
{
	static int tick=0;
	static char Fric_State[2][6] = {"CLOSE","OPEN"};
	static char Vision_State[2][6] = {"CLOSE","OPEN"};
	static char Chassis_State[4][9] = {"NOForce","Normal","SPIN","PRECISION"};
	static char Gimbal_State[4][9] = {"NOForce","Normal","SPIN","PRECISION"};
	static char Power_State[2][4] = {"Bat","Cap"};

	/*��ʼ����������������ͼ��*/
	if(Op_type == Op_Init)
	{
		switch(tick%11)
		{
			/*��̬�ַ�*/
			
			case 0:
			/*******************************Fric Mode �ַ�*********************************/
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
			/*******************************Auto Aim Mode �ַ�*********************************/
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

		
			default:
			break;
		}
		tick++;
		return ;
		
	}else if(Op_type == Op_Change)		//����Ǳ�־Ϊ�޸�
	{
		/*Ѱ�����ĸ������˱仯*/
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
	static int pack_tick = 0;			//���ݰ�������
	static int i;
	int packed_tick = 0;							//װ��������
	/*��ʼ����������������ͼ��*/
	if(Op_type == Op_Init)
	{
		switch(pack_tick % PACK_NUM)
		{
		case 0:
			/*******************************Fric Mode ͼ��*********************************/
		FRIC_MODE:		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=3;
				if(connection.connection_rx.fric.flag)
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=g_pos_x[0] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=g_pos_y[0] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius=20;
				if(Op_type == Op_Change) goto CONT_0;		
				break;
			case 1:
			/*******************************Auto Aim Mode ͼ��*********************************/
							AUTO_AIM:		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=4;
				if(connection.connection_rx.vision.flag)
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=g_pos_x[1] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=g_pos_y[1] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius=20;
			if(Op_type == Op_Change) goto CONT_1;
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
	}
}

void JudgementCustomizeChar(int Op_type)
{
		custom_char_draw.data_cmd_id=0x0110;//�����ַ�

		custom_char_draw.sender_ID=JudgeReceive.robot_id;//������ID�������˶�ӦID
		if(JudgeReceive.robot_id == 101)
				custom_char_draw.receiver_ID = 0x0165;
		if(JudgeReceive.robot_id == 1)
				custom_char_draw.receiver_ID = 0x0101;
		if(JudgeReceive.robot_id == 103)
				custom_char_draw.receiver_ID = 0x0167;
		if(JudgeReceive.robot_id == 104)
				custom_char_draw.receiver_ID = 0x0168;
		if(JudgeReceive.robot_id == 105)
				custom_char_draw.receiver_ID = 0x0169;
		if(JudgeReceive.robot_id == 3)
				custom_char_draw.receiver_ID = 0x0103;	
		if(JudgeReceive.robot_id == 4)
				custom_char_draw.receiver_ID = 0x0104;
		if(JudgeReceive.robot_id == 5)
				custom_char_draw.receiver_ID = 0x0105;

/*********************************�Զ����ַ�����***********************************/
		referee_data_load_String(Op_type);
}

void JudgementCustomizeGraphics(int Op_type)
{
		custom_grapic_draw.data_cmd_id=0x0104;//�����߸�ͼ�Σ�����ID����ѯ����ϵͳ�ֲᣩ

		custom_grapic_draw.sender_ID=JudgeReceive.robot_id;//������ID�������˶�ӦID
		if(JudgeReceive.robot_id == 101)
				custom_grapic_draw.receiver_ID = 0x0165;
		if(JudgeReceive.robot_id == 1)
				custom_grapic_draw.receiver_ID = 0x0101;
		if(JudgeReceive.robot_id == 103)
				custom_grapic_draw.receiver_ID = 0x0167;
		if(JudgeReceive.robot_id == 104)
				custom_grapic_draw.receiver_ID = 0x0168;
		if(JudgeReceive.robot_id == 105)
				custom_grapic_draw.receiver_ID = 0x0169;
		if(JudgeReceive.robot_id == 3)
				custom_grapic_draw.receiver_ID = 0x0103;	
		if(JudgeReceive.robot_id == 4)
				custom_grapic_draw.receiver_ID = 0x0104;
		if(JudgeReceive.robot_id == 5)
				custom_grapic_draw.receiver_ID = 0x0105;
			

/*********************************�Զ���ͼ������***********************************/
		referee_data_load_Graphic(Op_type);
}

uint8_t seq = 0;
void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //����֡����	

	memset(JudgeSend,0,frame_length);  //�洢���ݵ���������
	
	/*****֡ͷ���*****/
	JudgeSend[0] = sof;//����֡��ʼ�ֽ�
	memcpy(&JudgeSend[1],(uint8_t*)&len, sizeof(len));//����֡��data�ĳ���
	JudgeSend[3] = seq;//�����
	Append_CRC8_Check_Sum(JudgeSend,frameheader_len);  //֡ͷУ��CRC8

	/*****��������*****/
	memcpy(&JudgeSend[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****���ݴ��*****/
	memcpy(&JudgeSend[frameheader_len+cmd_len], p_data, len);
	Append_CRC16_Check_Sum(JudgeSend,frame_length);  //һ֡����У��CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****�����ϴ�*****/
		




   __HAL_UART_CLEAR_FLAG(&huart6,UART_FLAG_TC);
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
			if(char_change_state)			//�����û�б仯��û�б仯�Ͳ�������ʡ����
			{
				JudgementCustomizeChar(char_change_state);
				if(char_change_state != Op_None)
					referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_char_draw,sizeof(custom_char_draw));
			}
}