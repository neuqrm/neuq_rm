#ifndef REFEREE_H
#define REFEREE_H

#include "stm32f4xx.h"
#include "string.h"
#include "stdio.h"

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
    FIELD_EVENTS_CMD_ID               = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    REFEREE_WARNING_CMD_ID            = 0x0104,
    ROBOT_STATE_CMD_ID                = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
    ROBOT_POS_CMD_ID                  = 0x0203,
    BUFF_MUSK_CMD_ID                  = 0x0204,
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,
    ROBOT_HURT_CMD_ID                 = 0x0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    BULLET_REMAINING_CMD_ID           = 0x0208,
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,
    IDCustomData,
}referee_cmd_id;

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress;

typedef  struct //0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} ext_game_state;

typedef  struct //0002
{
    uint8_t winner;
} ext_game_result;

typedef  struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP;

typedef  struct //0101
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

typedef  struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning;
typedef  struct //0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_heat0_cooling_rate;
    uint16_t shooter_heat0_cooling_limit;
    uint16_t shooter_heat1_cooling_rate;
    uint16_t shooter_heat1_cooling_limit;
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
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
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
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy;

typedef  struct //0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt;

typedef  struct //0x0207
{
    uint8_t bullet_type;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data;

typedef  struct
{
    uint8_t bullet_remaining_num;
} ext_bullet_remaining;

typedef  struct //0x0301
{
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint16_t data_cmd_id;
    uint16_t data_len;
    uint8_t *data;
} ext_student_interactive_data;

typedef  struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data;


typedef  struct
{
    uint8_t data[64];
} ext_up_stream_data;

typedef  struct
{
    uint8_t data[32];
} ext_download_stream_data;

#pragma pack(pop)

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

extern ext_power_heat_data Power_Heat_Data;

extern uint16_t Get_Robot_HP(void); 					//得到机器人血量
extern uint16_t Get_Shoot_Heatlimit(void);		//得到机器人热量上限
extern float Get_Chassis_Power(void);					//得到机器人底盘功率
extern uint16_t Get_Chassis_PowerBuff(void);	//得到机器人功率缓冲
extern uint16_t Get_Shoot_Heat(void);					//得到机器人枪口热量
extern uint8_t Get_HP_Type(void);							//得到机器人血量变化类型
extern  uint8_t Get_Robot_Stats(void);
#endif /*REFEREE_H*/
