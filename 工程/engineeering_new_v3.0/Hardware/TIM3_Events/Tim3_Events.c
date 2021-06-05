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
	//if(off_line_flag==1)   //��������״̬		
		
	/*------ �˶����ƺ��� ------*/
	chassic_speed_control(Liner_X, Liner_Y, Angular_Z);	
	
	if(HANDLE_POS_EN)	
		handle_angle_control(Handle_Left_Angel,Handle_Right_Angle);
	else
		handle_speed_control(Handle_Left_Angular,Handle_Right_Angular);
	
	//void stepper_motor_control(int Distance);//����ǲ�����������˶����ƺ���
	
	//stepper_motor_control(Stepper_Left_Angle,Stepper_Right_Angle);
	
	/*------ pid���� ------*/	
	vpid_chassic_realize(v_chassic_p,v_chassic_i,v_chassic_d);		
	if(HANDLE_POS_EN)
	apid_handle_realize(a_handle_p,a_handle_i,a_handle_d);
	vpid_handle_realize(v_handle_p,v_handle_i,v_handle_d);
  /*------ ������ֵ ------*/	
	set_chassis_current();	
  //handle_current_control();	
	set_handle_current();
	
	
	/*------ ������� ------*/
	//TIM_SetCompare1();
	
	/*------ ��ŷ����� ------*/
  switch(Solenoid_valve_flag)
	{
		case 0:
			power_close_motor(3);
			break;
		case 1:
			power_open_motor(3);
			break;
		default:break;
	}
	  switch(Solenoid_handle_flag)
	{
		case 0:
			power_close_motor(4);
			break;
		case 1:
			power_open_motor(4);
			break;
		default:break;
	}

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
		
	}
	else if(!key_press())
		key_flag=unpressed;		//����δ������
}

