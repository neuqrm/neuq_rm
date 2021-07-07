
#include "referee.h"



frame_header Referee_Data_Head;
frame_header Referee_Send_head;


ext_game_state 			Game_State;
ext_game_result 		Game_Result;
ext_game_robot_HP 	Game_Robot_HP;
ext_ICRA_buff_debuff_zone_status	ICRA_Zone_Status;

ext_event_data 									Field_Event;
ext_supply_projectile_action 		Supply_Projectile_Action;
ext_supply_projectile_booking 	Supply_Projectile_Booking;
ext_referee_warning 						Referee_Warning;
ext_dart_remaining_time					Dart_Remaining_Time;

ext_game_robot_state 			Robot_State;
ext_power_heat_data 			Power_Heat_Data;
ext_game_robot_pos	 			Game_Robot_Pos;
ext_buff_musk 						Buff_Musk;
aerial_robot_energy 			Robot_Energy;
ext_robot_hurt 						Robot_Hurt;
ext_shoot_data 						Shoot_Data;
ext_bullet_remaining 			Bullet_Remaining;
ext_rfid_status						RFID_Status;
ext_dart_clinet_cmd				Dart_Client_Cmd;
ext_robot_command					Robot_Command;
ext_student_interactive_head 		Student_Interactive_Head;
ext_client_send_struct		Client_Send_charData;
ext_client_send_struct		Client_Send_graphicData;

char graphic1[3]="01";
char graphic2[3]="02";
char graphic3[3]="03";
char graphic4[3]="04";
char graphic5[3]="05";

ext_char_data_struct Graphic_Data1 = {0};
ext_char_data_struct Graphic_Data2 = {0};
ext_char_data_struct Graphic_Data3 = {0};
ext_char_data_struct Graphic_Data4 = {0};
ext_char_data_struct Graphic_Data5 = {0};

void m_memcpy(void *pd,const void *ps,uint16_t len)
{
	const uint8_t* tempps = ps;
	uint8_t* temppd = pd;
	uint16_t i;
	for (i=0;i<len;i++)
	{temppd[i] = tempps[i];}
}

/******************************************************
* @fn Init_Referee_Struct_Data
*
* @brief 初始裁判系统回传数结构体
* @pData None
* @return None.
* @note 在主函数开始前调用一次
*/
void Init_Referee_Struct_Data(void)
{
    memset(&Referee_Data_Head, 0, sizeof(frame_header));
	
    memset(&Game_State, 0, sizeof(ext_game_state));
    memset(&Game_Result, 0, sizeof(ext_game_result));
    memset(&Game_Robot_HP, 0, sizeof(ext_game_robot_HP));
		memset(&ICRA_Zone_Status, 0, sizeof(ext_ICRA_buff_debuff_zone_status));

    memset(&Field_Event, 0, sizeof(ext_event_data));
    memset(&Supply_Projectile_Action, 0, sizeof(ext_supply_projectile_action));
    memset(&Supply_Projectile_Booking, 0, sizeof(ext_supply_projectile_booking));
    memset(&Referee_Warning, 0, sizeof(ext_referee_warning));
		memset(&Dart_Remaining_Time, 0, sizeof(ext_dart_remaining_time));
	
    memset(&Robot_State, 0, sizeof(ext_game_robot_state));
    memset(&Power_Heat_Data, 0, sizeof(ext_power_heat_data));
    memset(&Game_Robot_Pos, 0, sizeof(ext_game_robot_pos));
    memset(&Buff_Musk, 0, sizeof(ext_buff_musk));
    memset(&Robot_Energy, 0, sizeof(aerial_robot_energy));
    memset(&Robot_Hurt, 0, sizeof(ext_robot_hurt));
    memset(&Shoot_Data, 0, sizeof(ext_shoot_data));
    memset(&Bullet_Remaining, 0, sizeof(ext_bullet_remaining));
		memset(&RFID_Status, 0, sizeof(ext_rfid_status));
		memset(&Dart_Client_Cmd, 0, sizeof(ext_dart_clinet_cmd));
		memset(&Robot_Command, 0, sizeof(ext_robot_command));

		Referee_Data_Send_Init(&Graphic_Data1,UI_XLENGTH-300,UI_YLENGTH+50,graphic1);
		Referee_Data_Send_Init(&Graphic_Data2,UI_XLENGTH-300,UI_YLENGTH+25,graphic2);
		Referee_Data_Send_Init(&Graphic_Data3,UI_XLENGTH-300,UI_YLENGTH,graphic3);
		Referee_Data_Send_Init(&Graphic_Data4,UI_XLENGTH-300,UI_YLENGTH-25,graphic4);
		Referee_Data_Send_Init(&Graphic_Data5,UI_XLENGTH-300,UI_YLENGTH-50,graphic5);
//    memset(&Student_Interactive_Data, 0, sizeof(ext_student_interactive_data));
}

/******************************************************
* @fn Referee_Data_Solve
*
* @brief 解析接收数据 放入对应结构体中
* @pData 接收数据的指针
* @return None.
* @note DMA中断中调用
*/
uint16_t DataLength[300]={0};
uint8_t DataLengthSEQ[300]={0};
uint16_t Datacmdid[300]={0};
void Referee_Data_Solve(uint8_t *frame)
{

  uint16_t cmd_id = 0;
	uint32_t judge_length=0;
  uint8_t index = 0;
	static uint16_t count=0;
	if (count==299)
		count=0;
	if (frame[PROGRESS_UNSTART]==HEADSOF)
	{
  memcpy(&Referee_Data_Head, frame, sizeof(frame_header));
//	judge_flag = verify_CRC8_check_sum(frame,REF_PROTOCOL_HEADER_SIZE);
	if (verify_CRC8_check_sum(frame,REF_PROTOCOL_HEADER_SIZE)==1)
		{
    index += sizeof(frame_header);
		judge_length = Referee_Data_Head.data_length + REF_HEADER_CRC_CMDID_LEN;
		if (verify_CRC16_check_sum(frame,judge_length)==1)
		{
			memcpy(&cmd_id, frame + index, sizeof(uint16_t));
			DataLengthSEQ[count] = Referee_Data_Head.seq;
			Datacmdid[count] = cmd_id;
			DataLength[count++]=Referee_Data_Head.data_length;
			index += sizeof(uint16_t);
			switch (cmd_id)
			{
					case GAME_STATE_CMD_ID:
					{
							memcpy(&Game_State, frame + index, sizeof(ext_game_state));
					}
					break;
					case GAME_RESULT_CMD_ID:
					{
							memcpy(&Game_Result, frame + index, sizeof(ext_game_result));
					}
					break;
					case GAME_ROBOT_HP_CMD_ID:
					{
							memcpy(&Game_Robot_HP, frame + index, sizeof(ext_game_robot_HP));
					}
					break;
					case ICRA_BUFF_CMD_ID:
					{
						memcpy(&ICRA_Zone_Status, frame + index, sizeof(ext_ICRA_buff_debuff_zone_status));
					}break;
					case FIELD_EVENTS_CMD_ID:
					{
							memcpy(&Field_Event, frame + index, sizeof(ext_event_data));
					}
					break;
					case SUPPLY_PROJECTILE_ACTION_CMD_ID:
					{
							memcpy(&Supply_Projectile_Action, frame + index, sizeof(ext_supply_projectile_action));
					}
					break;
					case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
					{
							memcpy(&Supply_Projectile_Booking, frame + index, sizeof(ext_supply_projectile_booking));
					}
					break;
					case REFEREE_WARNING_CMD_ID:
					{
							memcpy(&Referee_Warning, frame + index, sizeof(ext_referee_warning));
					}
					break;
					
					case DART_REMAINING_CMD_ID:
					{
						memcpy(&Dart_Remaining_Time, frame + index, sizeof(ext_dart_remaining_time));
					}break;
					case ROBOT_STATE_CMD_ID:
					{
							memcpy(&Robot_State, frame + index, sizeof(ext_game_robot_state));
					}
					break;
					case POWER_HEAT_DATA_CMD_ID:
					{
							memcpy(&Power_Heat_Data, frame + index, sizeof(ext_power_heat_data));
					}
					break;
					case ROBOT_POS_CMD_ID:
					{
							memcpy(&Game_Robot_Pos, frame + index, sizeof(ext_game_robot_pos));
					}
					break;
					case BUFF_MUSK_CMD_ID:
					{
							memcpy(&Buff_Musk, frame + index, sizeof(ext_buff_musk));
					}
					break;
					case AERIAL_ROBOT_ENERGY_CMD_ID:
					{
							memcpy(&Robot_Energy, frame + index, sizeof(aerial_robot_energy));
					}
					break;
					case ROBOT_HURT_CMD_ID:
					{
							memcpy(&Robot_Hurt, frame + index, sizeof(ext_robot_hurt));
					}
					break;
					case SHOOT_DATA_CMD_ID:
					{
							memcpy(&Shoot_Data, frame + index, sizeof(ext_shoot_data));
					}
					break;
					case BULLET_REMAINING_CMD_ID:
					{
							memcpy(&Bullet_Remaining, frame + index, sizeof(ext_bullet_remaining));
					}
					break;
					case RFIS_STATUS_CMD_ID:
					{
						memcpy(&RFID_Status, frame + index, sizeof(ext_rfid_status));
					}break;
					case DART_CLINET_CMS_ID:
					{
						memcpy(&Dart_Client_Cmd, frame + index, sizeof(ext_dart_clinet_cmd));
					}break;
					case ROBOT_COMMAND:
					{
						memcpy(&Robot_Command, frame + index, sizeof(ext_robot_command));
					}break;
//					case STUDENT_INTERACTIVE_DATA_CMD_ID:
//					{
//							memcpy(&Student_Interactive_Data, frame + index, sizeof(ext_student_interactive_data));
//					}
//					break;
					default:
					{
							break;
					}
				}
			}
    }
	}
}

/******************************************************
* @fn Referee_Data_Send
*
* @brief //客户端返回机器人状态数据
* @pData uc_cmd 数据头
* @return None
* @note None.
*/
void Referee_Data_Send_Init(ext_char_data_struct *tempTCB,int16_t pisitionX,int16_t pisitionY,char *ucgraphic1)
{
	tempTCB->graphic_name[0] = ucgraphic1[0];
	tempTCB->graphic_name[1] = ucgraphic1[1];
	tempTCB->graphic_name[2] = ucgraphic1[2];
	tempTCB->operate_type = 2;							//修改图层
	tempTCB->graphic_type = 5;									//浮点数据
	tempTCB->layer = 0;													//图层0
	tempTCB->color = 0;													//红蓝主色
	tempTCB->start_angle = 30;									//字体大小
	tempTCB->end_angle = 3;											//小数位数
	tempTCB->width = 3;													//线宽
	tempTCB->start_x = pisitionX;//UI_XLENGTH/2-50;											//X坐标
	tempTCB->start_y = pisitionY;//UI_YLENGTH/2-50;											//Y坐标
	tempTCB->float_data = 0;								//数据
}

/******************************************************
* @fn Referee_Data_Send
*
* @brief //客户端返回机器人状态数据
* @pData uc_cmd 数据头
* @return None
* @note None.
*/
void Referee_Data_Send(uint16_t uc_cmd)
{
	uint8_t Send_Data[128];
	uint8_t ucdata_length=0,i;
	uint32_t ucdata=0;
	static uint8 data_seq=0;
	Client_Send_charData.Send_Haed.SOF = HEADSOF;
	Client_Send_charData.Send_Haed.data_length = sizeof(ext_student_interactive_head)+5*sizeof(ext_graphic_data_struct);
	Client_Send_charData.Send_Haed.seq = data_seq;
	data_seq++;
	if (data_seq==0xFF)
		data_seq=0;
	Client_Send_charData.Send_Haed.CRC8 = 0;
	memcpy(Send_Data,&Client_Send_charData.Send_Haed,sizeof(frame_header));
	append_CRC8_check_sum(Send_Data,sizeof(frame_header));
	Client_Send_charData.Send_Haed.CRC8 = Send_Data[4];
	Client_Send_charData.cmd_id = 0x0301;
	Client_Send_charData.Send_ClientHead.data_cmd_id = uc_cmd;
	Client_Send_charData.Send_ClientHead.send_ID = Robot_State.robot_id;
	Client_Send_charData.Send_ClientHead.receiver_ID = Robot_State.robot_id+(0x0100);
	ucdata = Get_Chassis_PowerBuff();
	Graphic_Data1.float_data = ucdata*1000;
	Client_Send_charData.Send_ClientData1 = Graphic_Data1;
	
	ucdata = Get_Shoot_Heat();
	Graphic_Data2.float_data = ucdata*1000;
	Client_Send_charData.Send_ClientData2 = Graphic_Data2;
	
	ucdata = Motion_Gimbal_Pitch.Pitch_rad;
	Graphic_Data3.float_data = ucdata*1000;
	Client_Send_charData.Send_ClientData3 = Graphic_Data3;
	
	ucdata = Motion_Gimbal_Pitch.Pitch_rad;
	Graphic_Data4.float_data = ucdata*1000;
	Client_Send_charData.Send_ClientData4 = Graphic_Data4;
	
	ucdata = SuperCap_RealVoltage;
	Graphic_Data5.float_data = ucdata*1000;
	Client_Send_charData.Send_ClientData5 = Graphic_Data5;
	
	ucdata_length = sizeof(Client_Send_charData);
	memcpy(Send_Data,&Client_Send_charData,ucdata_length);
	append_CRC16_check_sum(Send_Data,ucdata_length);
	ucdata_length = sizeof(Client_Send_charData);
	Client_Send_charData.FrameTail = ((Send_Data[ucdata_length-2]<<8)&0xFF00) | Send_Data[ucdata_length-1];
	for (i=0;i<ucdata_length;i++)
	{
		REFEREE_SendByte(Send_Data[i]);
	}
}

/******************************************************
* @fn Get_Robot_HP
*
* @brief //得到机器人血量
* @pData None
* @return 机器人血量 uint16
* @note None.
*/
uint16_t Get_Robot_HP(void) 					
{
	return Robot_State.remain_HP;
}

/******************************************************
* @fn Get_Shoot_Heatlimit
*
* @brief 得到机器人热量上限
* @pData None
* @return 热量上限 uint16
* @note None.
*/
uint16_t Get_Shoot_Heatlimit(void)	//
{
	return Robot_State.shooter_id1_17mm_cooling_limit;
}

/******************************************************
* @fn Get_Chassis_Powerlimit
*
* @brief 得到机器人底盘功率上限
* @pData None
* @return 功率上限uint16
* @note None.
*/
uint16_t Get_Chassis_Powerlimit(void)		
{
	return Robot_State.chassis_power_limit;
}


/******************************************************
* @fn Get_Shoot_Coolrate
*
* @brief 得到机器人枪口冷却速率
* @pData None
* @return 冷却速率uint16
* @note None.
*/
uint16_t Get_Shoot_Coolrate(void)
{
	return Robot_State.shooter_id1_17mm_cooling_rate;
}

/******************************************************
* @fn Get_Shoot_Speedlimit
*
* @brief 得到机器人枪口发射速率
* @pData None
* @return 冷却速率uint16
* @note None.
*/
uint16_t Get_Shoot_Speedlimit(void)
{
	return Robot_State.shooter_id1_17mm_speed_limit;
}
/******************************************************
* @fn Get_Chassis_Power
*
* @brief 得到机器人底盘功率
* @pData None
* @return 底盘功率 float
* @note None.
*/
float Get_Chassis_Power(void)					
{
	return Power_Heat_Data.chassis_power;
}

/******************************************************
* @fn Get_Chassis_Current
*
* @brief 得到机器人底盘电流
* @pData None
* @return 底盘电流 float
* @note None.
*/
float Get_Chassis_Current(void)					
{
	return Power_Heat_Data.chassis_current;
}

/******************************************************
* @fn Get_Chassis_PowerBuff
*
* @brief 得到机器人功率缓冲
* @pData None
* @return 功率缓冲 uint16
* @note None.
*/
uint16_t Get_Chassis_PowerBuff(void)	//
{
	return Power_Heat_Data.chassis_power_buffer;
}

/******************************************************
* @fn Get_Shoot_Heat
*
* @brief 得到机器人枪口热量
* @pData None
* @return 枪口热量 uint16
* @note None.
*/
uint16_t Get_Shoot_Heat(void)					//
{
	return Power_Heat_Data.shooter_id1_17mm_cooling_heat;
}
/******************************************************
* @fn Get_HP_Type
*
* @brief 得到机器人血量变化类型
* @pData None
* @return 血量变化类型 uint8
* @note None.
*/
uint8_t Get_HP_Type(void)							//
{
	uint8_t temp_flag;
	temp_flag = Robot_Hurt.hurt_type>>4;
	return temp_flag;
}

