
/*****     ���ļ�ר�������Ÿ�����ʼ������    *****/

#include "My_Init.h"
#include "stm32f4xx_rcc.h"
#include "delay.h"
#include "key.h"
#include "LED.h"
#include "myflash.h"
#include "timer.h"
#include "can.h"
#include "power.h"
#include "fric.h"
#include "motor.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "DJi_remote.h"
#include "remote_code.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"
#include "gimbal.h"
#include "bsp_imu_usart.h"
#include "power_limitation.h"
#include "referee.h"
#include "steering_engine.h"
RCC_ClocksTypeDef get_rcc_clock;		//ϵͳʱ�ӽṹ��

extern void Referee_USART_Config(void);
// ����: All_Init()
// ����: ���������в�����ʼ��
// ��������
// �������
void All_Init()
{
	motor5.round_cnt=0;
	Stm32_Clock_Init(360,12,2,8);				//����ʱ��,180Mhz = 12M / 25 * 350 / 2
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	
	delay_init(180);									//��ʱ��ʼ��
	
	power_init();										  //�����Դ��ʼ����Ĭ��Ϊ��
	power_open_all();									//�������е����Դ
	
	Dji_Remote_Init();									//��ң������ʼ��

	led_init();											//led��ʼ��
	key_init();											//������ʼ��

	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_4tq,5,CAN_Mode_Normal);				//can��ʼ���������ʼ���� 45M/(4+4+1)/5
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_4tq,5,CAN_Mode_Normal);				//can��ʼ���������ʼ���� 45M/(4+4+1)/5

	Debug_USART_Config();           //ͨ�Ŵ��ڳ�ʼ����USART2��
	JSON_USART_Config();           	//���ӿ�ͨ�Ŵ��ڳ�ʼ����UART7��
	bsp_imu_usart_init();
  Referee_USART_Config();
	motor_init();		//���������ʼ��
	fric_PWM_configuration();       //Ħ���ֵ��pwm��ʼ��   TIM1 ����
	TIM_Steering_Engine_PWM_Init();
	TIM1_GIMBAL_Init();
	TIM3_Int_Init(10-1,9000-1);		  //��ʱ��ʱ��90M��9000������90M/9000=10Khz�ļ���Ƶ�ʣ�����10��Ϊ1ms  ��1khz
  //TIM4_Int_Init(100-1,9000-1);   //������ʱ��
	VPID_Init_All();								//�ٶ�pid������ʼ��
	APID_Init_All();								//�Ƕ�pid������ʼ��
	stop_chassis_motor();									//�õ�����ֵ�ǰ�Ƕȣ����������
	remote_ch_init();
	power_limitation_init();
	
	gimbal_y.actual_angle = BASIC_YAW_ANGLE_CAN;
	gimbal_p.actual_angle = BASIC_PITCH_ANGLE_CAN;
	Power_Heat_Data.chassis_power_buffer=60; //��ʼ����������

	RCC_GetClocksFreq(&get_rcc_clock); 			//�鿴ϵͳʱ��(watch��)
	
	
}
