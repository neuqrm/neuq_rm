#ifndef REFEREE_H
#define REFEREE_H

#include "stm32f4xx.h"
#include "bsp_supercap_usart.h"
#include "bsp_referee_usart.h"
#include "crc8_crc16.h"
#include "motion.h"
#include "string.h"
#include "stdio.h"

#define HEADSOF		0xA5					//数据帧起始字节
#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#define UI_XLENGTH	960			//1920/2=960
#define UI_YLENGTH	540			//1080/2=540

#pragma pack(push, 1) //设置结构体字节对齐方式 为1字节对齐 不然解码会造成错乱！！！

typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header;

typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,
    GAME_RESULT_CMD_ID                = 0x0002,
    GAME_ROBOT_HP_CMD_ID              = 0x0003,
		ICRA_BUFF_CMD_ID									= 0x0005,
    FIELD_EVENTS_CMD_ID               = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    REFEREE_WARNING_CMD_ID            = 0x0104,
		DART_REMAINING_CMD_ID							= 0x0105,
    ROBOT_STATE_CMD_ID                = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
    ROBOT_POS_CMD_ID                  = 0x0203,
    BUFF_MUSK_CMD_ID                  = 0x0204,
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,
    ROBOT_HURT_CMD_ID                 = 0x0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    BULLET_REMAINING_CMD_ID           = 0x0208,
		RFIS_STATUS_CMD_ID								= 0x0209,
		DART_CLINET_CMS_ID								= 0x020A,
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,
		ROBOT_COMMAND											= 0x0303,
    IDCustomData,
}referee_cmd_id;

#pragma pack(pop)

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
		RED_RADAR				= 9,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
		BLUE_RADAR			= 109,
} robot_id;

typedef enum
{
		CLIENT_RED_HERO				 = 0x0101,
		CLIENT_RED_ENGINEER    = 0x0102,
    CLIENT_RED_STANDARD_1  = 0x0103,
    CLIENT_RED_STANDARD_2  = 0x0104,
    CLIENT_RED_STANDARD_3  = 0x0105,
    CLIENT_RED_AERIAL      = 0x0106,
    CLIENT_BLUE_HERO       = 0x0165,
    CLIENT_BLUE_ENGINEER   = 0x0166,
    CLIENT_BLUE_STANDARD_1 = 0x0167,
    CLIENT_BLUE_STANDARD_2 = 0x0168,
    CLIENT_BLUE_STANDARD_3 = 0x0169,
    CLIENT_BLUE_AERIAL     = 0x016A,
} client_id;

typedef enum
{
	DELET_GRAPHIC						= 0x0100,
	CHAR_GRAPHIC						= 0x0110,
} client_cmd_id;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress;

typedef  struct //0x0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
		uint64_t SynTimeStamp;
} ext_game_state;

typedef  struct //0x0002
{
    uint8_t winner;
} ext_game_result;

typedef  struct	//0x0003
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
		uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
		uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
	
} ext_game_robot_HP;
typedef  struct		//0x0005
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3; 
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3; 
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3; 
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3; 
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3; 
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
 uint16_t blue1_bullet_left;
 uint16_t blue2_bullet_left;
}	ext_ICRA_buff_debuff_zone_status;
typedef  struct //0x0101
{
    uint32_t event_type;
} ext_event_data;

typedef  struct //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action;


typedef  struct //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking;

typedef  struct	//0x0104
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning;

typedef  struct	//0x0105
{
	uint8_t dart_remaining_time;
}	ext_dart_remaining_time;

typedef  struct //0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
	
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
		uint16_t shooter_id1_17mm_speed_limit;

		uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
		uint16_t shooter_id2_17mm_speed_limit;
	
		uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
		uint16_t shooter_id1_42mm_speed_limit;
	
		uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state;

typedef  struct //0x0202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float 	 chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
		uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data;

typedef  struct //0x0203
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos;

typedef  struct //0x0204
{
    uint8_t power_rune_buff;
} ext_buff_musk;

typedef  struct //0x0205
{
    uint8_t attack_time;
} aerial_robot_energy;

typedef  struct //0x0206
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt;

typedef  struct //0x0207
{
    uint8_t bullet_type;
		uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data;

typedef  struct	//0x0208
{
    uint16_t bullet_remaining_num_17mm;
		uint16_t bullet_remaining_num_42mm;
		uint16_t coin_remaining_num;
} ext_bullet_remaining;

typedef	 struct	//0x0209
{
	uint32_t rfid_status;
}ext_rfid_status;

typedef struct	//0x020A
{
	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint16_t operate_launch_cmd_time;
}ext_dart_clinet_cmd;

typedef  struct //0x0301
{
    uint16_t data_cmd_id;	
    uint16_t send_ID;
    uint16_t receiver_ID;
} ext_student_interactive_head;

typedef  struct
{
	uint8_t graphic_name[3];
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
} ext_graphic_data_struct;

typedef  struct
{
	uint8_t graphic_name[3];
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t float_data;
} ext_char_data_struct;

typedef  struct
{
	frame_header 						Send_Haed;	//帧头				
	uint16_t cmd_id;
	ext_student_interactive_head	Send_ClientHead;	//数据段头 包含命令码 接收&发射者id
	ext_char_data_struct Send_ClientData1;  //字符/图形1
	ext_char_data_struct Send_ClientData2;	//字符/图形2
	ext_char_data_struct Send_ClientData3;  //字符/图形3
	ext_char_data_struct Send_ClientData4;	//字符/图形4
	ext_char_data_struct Send_ClientData5;	//字符/图形5
	uint16_t		 						FrameTail;//帧尾
} ext_client_send_struct;


//typedef  struct
//{
//    float data1;
//    float data2;
//    float data3;
//    uint8_t data4;
//} custom_data;


//typedef  struct
//{
//    uint8_t data[64];
//} ext_up_stream_data;

//typedef  struct
//{
//    uint8_t data[32];
//} ext_download_stream_data;

typedef struct		//0x0303
{
	float traget_position_x;
	float traget_position_y;
	float traget_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
}ext_robot_command;

typedef enum
{
	ARMOUR	= 0x00,	//装甲伤害
	OFFLINR	= 0x01,	//模块离线
	SHOOTHEAT	= 0x02,	//枪口热量超限
	POWER		= 0x03,		//底盘功率超限
}hp_tupe;//血量变化类型

extern void Init_Referee_Struct_Data(void);
//extern void Referee_Data_Solve(uint8_t *pData,uint16_t Length);
extern void Referee_Data_Solve(uint8_t *frame);
extern void Referee_Data_Send(uint16_t uc_cmd);		//客户端返回状态数据
extern void Referee_Data_Send_Init(ext_char_data_struct *tempTCB,int16_t pisitionX,int16_t pisitionY,char *ucgraphic1);//数据初始化
extern uint16_t Get_Robot_HP(void); 					//得到机器人血量
extern uint16_t Get_Shoot_Heatlimit(void);		//得到机器人枪口热量上限
extern uint16_t Get_Shoot_Speedlimit(void);		//得到机器人枪口热量上限
extern uint16_t Get_Chassis_Powerlimit(void);		//得到机器人底盘功率上限
extern uint16_t Get_Shoot_Coolrate(void);			//得到机器人枪口冷却速率
extern float Get_Chassis_Power(void);					//得到机器人底盘功率
extern float Get_Chassis_Current(void);				//得到机器人底盘电流
extern uint16_t Get_Chassis_PowerBuff(void);	//得到机器人功率缓冲
extern uint16_t Get_Shoot_Heat(void);					//得到机器人枪口热量
extern uint8_t Get_HP_Type(void);							//得到机器人血量变化类型

#endif /*REFEREE_H*/
