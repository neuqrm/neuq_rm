/**
  ******************************************************************************
  * @file    Project/USER/Tim3_Events.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   ��ʱ��3��غ���
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include "Tim3_Events.h"
int flag_1=0;

/**
  * @breif �˶����ƺ���
	* @param  ��pid����
	*/
void Robo_Move()
{
	if(stop_flag_1 && ap_pid_flag == ang_pid)			//�����ʱ�ٶ�Ϊ0��ֹͣ�������Զ�����ʱû��������λ�ñջ�   ��ô�Ƕȱջ�
	{	
		break_jugement();
		if(stop_flag_3 && 1)
		{
			stop_chassis_motor();
		}
		apid_chassic_realize(0.2,0.05,0);			
	
	}
  if(1) //(Control_Mode) == 0x03 Control_Mode & auto_control) == auto_control)
	{
		//chassic_test();
		chassic_speed_control(Liner_X, Liner_Y, Angular_Z);		 
		//handle_angle_control(Handle_L, Handle_R);      //ʹ����̨�ٶȻ�����
    handle_speed_control(Handle_speed,Handle_speed);	 
	}
 
	
   	vpid_chassic_realize(v_chassic_p,v_chassic_i,v_chassic_d);			//�ٶȱջ�2  0.05
	  //apid_handle_realize(a_handle_p,a_handle_i,a_handle_d);      
	  vpid_handle_realize(v_handle_p,v_handle_i,v_handle_d);     
	  set_chassis_current();		//�趨�������
	  set_handle_current();

	  TIM_SetCompare1(TIM1,pwm_pulse_p);
		TIM_SetCompare2(TIM1,pwm_pulse_y);
	if(flag_1==0)
	{
		pwm_pulse_2=1400;
  	pwm_pulse_1=1800;
	}
	if(flag_1==1)
	{
		pwm_pulse_2=2200;
  	pwm_pulse_1=800;

	}
    TIM_SetCompare2(TIM2,pwm_pulse_1);
    TIM_SetCompare3(TIM2,pwm_pulse_2);
}



/**
  * @breif �����õİ���������Ƭ���ϵİ�ɫ����
	* @param key_flag
	*/
void Debug_Key()
{
	static int key_flag = unpressed;		//���ڿ��������ж��ڰ��µĹ�����ֻ����һ��
	if( key_press() && key_flag == unpressed)		//�������������
	{
		LED4=!LED4;												//LED4��ת������״ָ̬ʾ
		key_flag = pressed;			          //����������
		// your codes below
		
		//NRF24L01_TxPacket(send_data);			//nrf�����ַ���
	}
	else if(!key_press())
		key_flag=unpressed;		//����δ������
}

