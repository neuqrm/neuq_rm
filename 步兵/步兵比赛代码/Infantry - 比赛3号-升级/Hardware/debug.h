#ifndef __DEBUG_H
#define	__DEBUG_H


#include "bsp_debug_usart.h"
#include "string.h"
#include "stdio.h"

/***匿名数传通信格式 数据头0xAA 硬件地址0xAF 上位机****/
#define ANONY_HEAD	0xAA
#define ANONY_ADDR	0xAF
#define ANONY_ID1		0xF1	//数传灵活帧 ID范围从0xF1~0xFA
#define ANONY_ID2		0xF2
#define ANONY_ID3		0xF3
#define ANONY_ID4		0xF4
#define ANONY_ID5		0xF5
#define ANONY_ID6		0xF6
#define ANONY_ID7		0xF7
#define ANONY_ID8		0xF8
#define ANONY_ID9		0xF9
#define ANONY_IDA		0xFA
#define ANONY_LEN		40		//数传数据段长度 最长40
#define STM32_ADDR	0xFF	//下位机接收地址
#define STM32_WRITE 0xE2	//下位机写入ID
#define STM32_LEN		6			//写入数长度
#define RETURN_ID	0x00		//返回帧ID

#pragma pack(push, 1) 
typedef struct 
{
	uint16_t PAR_ID;
	int32_t PAR_VAL;
}Debug_receive;

typedef struct 
{
	uint16_t DEBUG_ID;
	float DEBUG_VAL;
}Debug_data;
#pragma pack(pop)

extern void DEBUG_Anony16Send(uint8_t id,int16_t *pst);
extern void DEBUG_Anony32Send(uint8_t id,int32_t *pst);
extern void DEBUG_Anonyreturn(uint8_t *pst);
extern void DEBUG_Data_solve(uint8_t ucData);

#endif /**__DEBUG_H**/
