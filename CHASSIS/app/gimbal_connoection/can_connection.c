/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-10 15:38:20
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-10 16:46:28
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\CHASSIS\CHASSIS\app\gimbal_connoection\can_connection.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "can_connection.h"
#include "agv_control.h"


void CAN1_Call_Back(struct Struct_CAN_Rx_Buffer *rx)
{
    switch(rx->Header.StdId)
    {
        case 0x150:
				Chassis_Speed_Command_Update(&connection,rx->Data);
        break;

        case 0x152:
        Chassis_Control_Mode_Update(&connection,rx->Data);
        break;

        case 0x206:
        GM6020_Feedback_Update(&yaw.motor,rx->Data);
        break;
    }
     switch(rx->Header.ExtId)
    {
        
    }
}

void CAN2_Call_Back(struct Struct_CAN_Rx_Buffer *rx)
{
    switch(rx->Header.StdId)
    {

    }
    switch(rx->Header.ExtId&0xff)
    {
        case 0x1a:
					memcpy(&chassis_power_control.expect_power_32[0],rx->Data,4);
                    chassis_power_control.all_mscb_ready_flag   = chassis_power_control.all_mscb_ready_flag | 0x1;
        break;
        case 0x1b:
					memcpy(&chassis_power_control.expect_power_32[0],rx->Data,4);
                     chassis_power_control.all_mscb_ready_flag   = chassis_power_control.all_mscb_ready_flag | 0x2;
        break;
        case 0x1c:
					memcpy(&chassis_power_control.expect_power_32[0],rx->Data,4);
                     chassis_power_control.all_mscb_ready_flag   = chassis_power_control.all_mscb_ready_flag | 0x4;
        break;
        case 0x1d:
					memcpy(&chassis_power_control.expect_power_32[0],rx->Data,4);
                     chassis_power_control.all_mscb_ready_flag   = chassis_power_control.all_mscb_ready_flag | 0x8;
        break;
    }
}

void Can_Connection_Init(void)
{
    CAN_Init(&hcan1,CAN1_Call_Back);
    CAN_Init(&hcan2,CAN2_Call_Back);
}