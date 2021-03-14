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

/**
  * @breif �˶����ƺ���
	* @param  ��pid����
	*/
void Robo_Move()
{
	if(CHASSIS_BREAK_EN)          //����ɲ��ʹ��
	{
	 if(stop_flag_chassis==1)			//�����ʱ�ٶ�Ϊ0��ֹͣ����ô�Ƕȱջ�
		apid_chassis_realize(a_chassis_p,a_chassis_i,a_chassis_d);		//����λ�û�����	
  }
 /***** ���˶����ƺ��� ******/
		chassis_speed_control(Liner_X, Liner_Y, Angular_Z);		 //�����ٶȿ���
		trigger_control(Trigger_Speed);	                       //�������ٶȿ���
	
    if(gimbal_modes == gimbal_can_mode)	          //�������̨ģʽ��
		{
	    if(GIMBAL_POS_EN)                             //��̨λ�û�ʹ��
		   gimbal_angle_control(Angle_Yaw,Angle_Pitch);  //��̨λ�û�����
		  else
		   gimbal_speed_control(Angular_Yaw, Angular_Pitch);      //��̨�ٶȻ�����	
	  }
		//auto_fire();//��λ�����ͱ�־λ  ����
	 
 /***** pid���㼰������ֵ *****/
   	vpid_chassis_realize(v_chassis_p,v_chassis_i,v_chassis_d);			//�ٶȱջ�2  0.05
	  vpid_trigger_realize(v_trigger_p,v_trigger_i,v_trigger_d);      //�������ٶȱջ�  ����δȷ��   2.5  0.05
		if(GIMBAL_POS_EN)
     apid_gimbal_realize(a_yaw_p,a_yaw_i,a_yaw_d,a_pitch_p,a_pitch_i,a_pitch_d);	        //��̨λ�û�����
		
	  vpid_gimbal_realize(v_yaw_p,v_yaw_i,v_yaw_d,v_pitch_p,v_pitch_i,v_pitch_d);     //  ��̨�ٶȻ��趨

 /***** �趨���������ѹֵ *****/	
	  set_chassis_current();		
	  set_trigger_current();
	  set_gimbal_current();

 /***** ������� *****/
		if(gimbal_modes == gimbal_pwm_mode)
		{
	   TIM_SetCompare1(TIM1,pwm_pulse_p);                              //pitch��
		 TIM_SetCompare2(TIM1,pwm_pulse_y);                              //yaw��
		}
		 fric1_on(FRIC_Speed);                                           //Ħ���ֿ���
		 fric2_on(FRIC_Speed);
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

