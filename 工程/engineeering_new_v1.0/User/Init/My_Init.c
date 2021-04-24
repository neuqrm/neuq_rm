
/*****     ���ļ�ר�������Ÿ�����ʼ������    *****/

#include "My_Init.h"
#include "stm32f4xx_rcc.h"
#include "delay.h"
#include "key.h"
#include "LED.h"
#include "timer.h"

#include "can.h"
#include "power.h"

#include "motor.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "DJi_remote.h"

#include "remote_code.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"

#include "bsp_imu_usart.h"
#include "steering_engine.h"
#include "stepper_motor.h"
#include "solenoid_valve.h"
#include "limit_switch.h"

RCC_ClocksTypeDef get_rcc_clock;		//ϵͳʱ�ӽṹ��

// ����: All_Init()
// ����: ���������в�����ʼ��
// ��������
// �������
void All_Init()
{
	Stm32_Clock_Init(360,12,2,8);				//����ʱ��,180Mhz = 12M / 25 * 350 / 2
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	
	delay_init(180);									//��ʱ��ʼ��
	
	power_init();										  //�����Դ��ʼ����Ĭ��Ϊ��
	power_open_all();									//�������е����Դ
	power_close_motor(1);
	Dji_Remote_Init();									//��ң������ʼ��

	led_init();											//led��ʼ��
	key_init();											//������ʼ��

	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_4tq,5,CAN_Mode_Normal);				//can��ʼ���������ʼ���� 45M/(4+4+1)/5

	Debug_USART_Config();           //ͨ�Ŵ��ڳ�ʼ����USART2��
	


	motor_init();		//���������ʼ��

	TIM3_Int_Init(10-1,9000-1);		  //��ʱ��ʱ��90M��9000������90M/9000=10Khz�ļ���Ƶ�ʣ�����10��Ϊ1ms  ��1khz
	VPID_Init_All();								//�ٶ�pid������ʼ��
	APID_Init_All();								//�Ƕ�pid������ʼ��
	ap_pid_flag = ang_pid;									//λ�ýǶȱջ�ѡ���־
	stop_chassis_motor();									//�õ�����ֵ�ǰ�Ƕȣ����������
	step_motor_init();
	
	TIM_Steering_Engine_PWM_Init();
	
	TIM_Stepper_Motor_PWM_Init();
	Stepper_Motor_EN_GPIO_Init();
	Stepper_Motor_DIR_GPIO_Init();
	
	
	solenoid_valve_init();
	
	limit_switch_NVIC_Configuration();
	
	RCC_GetClocksFreq(&get_rcc_clock); 			//�鿴ϵͳʱ��(watch��)
	
	
}
