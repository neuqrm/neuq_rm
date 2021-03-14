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
	mode_init();                      //��ʼ��������ģʽ����
 	All_Init();												//������Ӳ�����ṹ���ʼ��
	pid_init();                       //��ʼ��pid���������ֵ
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
		if(control_mode == DJi_Remote_Control) //�ж��Ƿ�Ϊң��ģʽ
		Remote_Control();				//ң�������ƴ���
		
		/****  ROS��λ������  *****/
		if(control_mode == auto_control) //�ж��Ƿ�Ϊ�Զ�����ģʽ
	{
		if(flag_command_recieved == 1)	//ÿһ������һ���Ƿ��յ�����ָ�����ʶ��Ϊһ˵����Ӧָ�������������ͬ��
		{
			resolve_json_chassis_command(); //����ָ��
			flag_command_recieved = 0;	
		}
   
	 if(flag_command_recieved1 == 1)
		{
			resolve_json_gimbal_angle_command();  //��̨�ٶ�ָ��
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
	    //resolve_json_pidparam_command(); //������λ��pid��������
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
		
		/****    ����λ����������   *****/
		if(MSG_SEND_EN)
		{
		if(time_count%20 == 0)		//20ms��50Hz 		
     {   
       send_chassis_info_by_json();
	     send_gimbal_info_by_json();
		   send_infantry_info_by_json();
     } 		   
	  }                          
		
		if(time_count>=1000)			//���������־    1s
			time_count=1;
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}

