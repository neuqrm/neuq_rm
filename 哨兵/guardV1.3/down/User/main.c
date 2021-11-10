/**
  ******************************************************************************
  * @file    Project/USER/main.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
      ..................NEUQ_SUDO..................

  ...........������δ�ɹ���ͬ־����Ŭ��...........
*/  
#include "My_Init.h"
#include "control.h"  //ʵ�飬���ڸ�control_flagֵ
/**
  *@brief  ��������ʼ��������ѭ���ȴ��ж�
  */
//extern unsigned char left_ucRxFinish; //����6������ɱ�־
//extern unsigned char right_ucRxFinish;//����8������ɱ�־

extern DJi_RC rc;
int flag2=0;
int main()
{
 	All_Init();												//������Ӳ�����ṹ���ʼ����ϵͳʱ��180MHz
	pid_init();                       //��ʼ��pid���������ֵ
	mode_init();                      //��ʼ��������ģʽ����
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);

	while(1)                          //����ѭ��
	{
		
		/****    ����λ����������   *****/
		if(MSG_SEND_EN)
		{
			//hit_judge();//�Ƿ��ܻ�
			send_infantry_info_by_json();		   
			//hit_flag	=	0;  //��֤���ظ��ϴ�
			delay_ms(100);//������ʱ��ֹ���ֳ����ܷ�
		}
		
	}
	
}

/**
  * @brief  ��ʱ��3�жϷ�����			1ms
  */
void TIM3_IRQHandler(void)
{
	static int time_count=1;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) 	//����ж�
	{ 
		time_count++;
		
		/*****   ң��������    ******/
	  if(Remote_control_mode == chassis_CH_width)  //�󿪹����м�
		{
			
			if(control_flag == 1)//�������
			{
				control_flag = 0;
				Kinematics.yaw.target_angular = 0;  //��������
				Kinematics.pitch.target_angular = 0;
			}
			
			control_mode = DJi_Remote_Control;
		}
		
		else if(Remote_control_mode == 2) //��������
		{//�������
			control_flag = 1;
			cruise_mode();
		}
		
		else   //�������ϻ���ң�����ر�
		{
			control_mode = auto_control;
			//control_flag = 0;		//�ֶ�����ʱ�Ÿ��㣬�Զ�ʱ����λ�����͵�ֵ
			cruise_mode();
		}

		if(control_mode == DJi_Remote_Control) //�ж��Ƿ�Ϊң��ģʽ
		Remote_Control();				//ң�������ƴ���
		
		/*****  ROS��λ������  *****/
		if(control_mode == auto_control) //�ж��Ƿ�Ϊ�Զ�����ģʽ
	{
		if(flag_command_recieved == 1)	//ÿһ������һ���Ƿ��յ�����ָ�����ʶ��Ϊһ˵����Ӧָ�������������ͬ��
		{
			resolve_json_chassis_command(); //����ָ��
			flag_command_recieved = 0;	
		}
   
	 if(flag_command_recieved1 == 1)
		{
			resolve_json_gimbal_speed_command();  //��̨�ٶ�ָ��
			flag_command_recieved1 = 0;	
		}
	
	 	if(flag_command_recieved2 == 1)
		{
		  resolve_json_trigger_command();  //������ָ��
			resolve_json_fric_command();     //Ħ����ָ��
			flag_command_recieved2 = 0;	
		}
		if(flag_command_recieved3 == 1)  
		{
			resolve_json_gimbal_speed_command();
		  caclulate_pwm_pulse();		
			flag_command_recieved3 = 0;
		}
		if(flag_command_recieved4 == 1)	
		{
			resolve_json_control_command();//������λ���ź�
			flag_command_recieved4 = 0;	
		}
		
			if(flag_command_recieved5 == 1)	
		{
				resolve_json_gimbal_speed_command();
			  flag_command_recieved5 = 0;	
		}
	}
		/****  �������˶�����  *****/
		if(time_count%7 ==0)		//7ms
		//	chassis_behavior();
			Robo_Move();
		
		/*****    ����ɨ��   ******/
		if(time_count%31 == 0)		//31ms  ����
			Debug_Key();
		
		/*****    ����״ָ̬ʾ��   ******/
		if(time_count%500 == 0)			//500ms
		{
			LED1=!LED1;							//��Ƭ����������״ָ̬ʾ��
			time_count = 0;
		}
		
		if(time_count%4 == 0)		//4ms  ����
			Get_Base_Velocities();		//�����������ʵ���ٶ�
		  Get_Gimbal_Angle();
			Get_referee_info();


		if(time_count>=1000)			//���������־    1s
			time_count=1;
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}

