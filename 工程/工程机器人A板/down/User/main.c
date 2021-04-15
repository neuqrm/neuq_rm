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
/**
  *@brief  ��������ʼ��������ѭ���ȴ��ж�
  */
int main()
{
 	All_Init();												//�������������ó�ʼ��	
	pid_init();                     //��ʼ��pid���������ֵ
	Kinematics.handle_L.target_angular=-570;
	while(1)                          //����ѭ��
	{
		LED0=!LED0;
		delay_ms(500);
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
		Remote_Control();				//ң�������ƴ���
		
		/****  ROS��λ������  *****/
	/*	if(flag_command_recieved == 1)	//ÿһ������һ���Ƿ��յ�����ָ��
		{
			//����Զ����Ʋſ��Ը������Ŀ���ٶȸ�ֵ
			if(1)
			//resolve_json_chassis_command();
		
			flag_command_recieved = 0;	//������ձ�־λ����
		}
   
	 if(flag_command_recieved1 == 1)
		{
		if(1)//(Control_Mode & auto_control) == auto_control
			resolve_json_gimbal_speed_command();
			flag_command_recieved1 = 0;	//������ձ�־λ����
		}
	 
	 	if(flag_command_recieved2 == 1)
		{
		if(1)//(Control_Mode & auto_control) == auto_control
			//resolve_json_trigger_command();
      //resolve_json_fric_command();
	
			flag_command_recieved2 = 0;	//������ձ�־λ����
		}
		if(flag_command_recieved3 == 1)  //1���������λ�����0����������λ������
		{
		if(1)//(Control_Mode & auto_control) == auto_control
			//resolve_json_gimbal_speed_command();
		  //caclulate_pwm_pulse();		
			flag_command_recieved3 = 0;	//������ձ�־λ����
		}
		*/
		  if(Solenoid_handle_flag == 1)
			power_open_motor(2);
		  if(Solenoid_handle_flag == 0)
			power_close_motor(2);
			if(Solenoid_valve_flag == 1)	//ÿһ������һ���Ƿ��յ�����ָ��
				power_open_motor(1);
		  if(Solenoid_valve_flag == 0)
			power_close_motor(1);
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
		
		
		if(time_count%20 == 0)		//20ms��50Hz 		
  {   
        send_chassis_info_by_json();
	   //send_gimbal_info_by_json();
		 //send_infantry_info_by_json();
    //send_info_by_json();
		 //send_infantry_info_by_json();
  } 		   
		                            
		
		if(time_count>=1000)			//���������־    1s
			time_count=1;
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}

