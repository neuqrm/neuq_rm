
#include "motor.h"

Motor_Chassis_Rpm_stc  	Motor_Chassis_Rpm		= {0};
Motor_Gimbal_Yaw_stc  	Motor_Gimbal_Yaw	= {0,YAW_RESETANGLE};
Motor_Gimbal_Pitch_stc 	Motor_Gimbal_Pitch	=	{0,PITCH_RESETANGLE};
Motor_Gimbal_Trigger_stc	Motor_Gimbal_Trigger	=	{0};



/******************************************************
* @fn Motor_PID_init
*
* @brief ��ʼ������Ŀ��ƽṹ��
* @pData None
* @return None.
* @note ����������ʼǰ����һ��
*/
void Motor_PID_init(void)
{
	uint8_t i=0;
	for (i=0;i<4;i++)
	{
		pid_init(&pid_wheel_speed[i],15000,500,8.5,0.6,0);	//�Ѿ����Ե����� set��get ͬһ���� P=8
		pid_init(&pid_wheel_current[i],15000,500,1,1,0);//�¼��������
	}
	pid_init(&pid_trigger_speed,10000,500,2.95,0.06,0.2); //�Ѿ����Ե����� set��get��С 2.95 0.06 0.2
	pid_init(&pid_trigger_angle,2160,360,0.0025,0.01,0);			//�����ת�����90 �޷�10 ���ٱ�36
//	pid_init(&pid_pitch_speed,30000,500,1.5,2.5,3);
//	pid_init(&pid_pitch_angle,30000,500,50,1.4,0);		//�Ѿ����Ե����� �Ƕ�ֵ��Ӧ 0~8191
	pid_init(&pid_pitch_speed,30000,7300,310,45,5);
//	pid_init(&pid_pitch_angle,30000,800,660,10.2,0.4);		//�����ǶȻ� 420,5,0//660 10.2 0.4
	pid_init(&pid_pitch_angle,60,20,0.055,0.0007,0);					//�Ƕ�˫��
	pid_init(&pid_yaw_speed,32000,500,220,10,9);
//	pid_init(&pid_yaw_angle,30000,3000,380,4,9);					//65 1.4 �����ǶȻ�
	pid_init(&pid_yaw_angle,350,50,0.04,0,0);									//�Ƕ�˫��65 1.4
	pid_init(&pid_test_moto,10000,500,0.001,0.0001,0); 	//������Բ���
}//yaw_speed 150 6 0 No_1

/******************************************************
* @fn Urgency_Brake
*
* @brief ����ɲ�� ȫ�����ͣת
* @pData None
* @return None.
* @note ���ص��� ��Ҫ�ֶ���λ
*/
void Urgency_Brake(void)
{
	while(1)
	{
		TIM_Cmd(BASIC_TIM, DISABLE);	
		TIM_SetCompare1(TIM8,1000);
		TIM_SetCompare2(TIM8,1000);
		CAN_Send_Chassis_Msg(0,0,0,0);
		CAN_Send_Gimball_Msg(0,0,0,0);
		CAN_Send_Extra_Msg(0,0,0,0);
		LEDR_OFF;
		LEDG_ON;
		LED_Control(0x66);
	}//������ѭ��
}


/******************************************************
* @fn Motot_Control
*
* @brief ������س���
* @pData None
* @return None.
* @note ��Ҫ���ݵ�������������� ����ʵ�ʿ�����
*/
//uint16_t STEP = 200;
float WheelLook[4] = {0},PitchLook =0 ,M3508LookA = 0;
int16_t Debug_datamotor[10]={0};
int16_t Debug_dataspan[10];
int16_t Lookcircle_count=0;
void Motot_Control(void)			//�������������
{
	volatile uint16_t Fric_rpm = FRICRPM_RESET;
	int16_t Current_Wheel[4] = {0}, Current_Trigger = 0;  //���ٵ�� ���Ƶ���ֵ
	int16_t Voltage_Yaw = 0 , Voltage_Pitch = 0 , Volayge_unkoown = 0;	//������Ƶ�ѹֵ
	int16_t Speed_Yaw = 0,Speed_Pitch = 0,Speed_Trigger = 0;					//�ǶȻ������ٶ��м���
	uint16_t Trigger_reset_angle,Trigger_stepcount = TRIGGER_TOOTH;		//�����ֵ����ر���
	uint16_t Fric_pluse = FRICRPM_RESET;
	uint32_t Gimbal_Trigger_get_angle=0;
	int16_t Z_angle = 0,Z_rad = 0,Yaw_setRPM = 0,Yaw_realRPM = 0,Yaw_realAngle = 0;
	int16_t Wheelgetrpm[4] = {0},Wheelgetcur[4] = {0};
	float Wheelsetrpm[4] = {0},Wheelsetcur[4],Singal_rpm=0;
	uint8_t i;
	int16_t circle=0;
	static int16_t circle_count=0;
	/**********3508��PID����*********************/
	Wheelsetrpm[0] = Motor_Chassis_Rpm.MoterID1_rpm;
	Wheelsetrpm[1] = Motor_Chassis_Rpm.MoterID2_rpm;
	Wheelsetrpm[2] = Motor_Chassis_Rpm.MoterID3_rpm;
	Wheelsetrpm[3] = Motor_Chassis_Rpm.MoterID4_rpm;
//	for (i=0;i<4;i++) //���̵��˶�����
//	{
//	WheelLook[i] = Wheelsetrpm[i];
//	}
	Wheelgetrpm[0] = M3508_ID1_Data.rotation;
	Wheelgetrpm[1] = M3508_ID2_Data.rotation;
	Wheelgetrpm[2] = M3508_ID3_Data.rotation;
	Wheelgetrpm[3] = M3508_ID4_Data.rotation;
	Wheelgetcur[0] = M3508_ID1_Data.current;
	Wheelgetcur[1] = M3508_ID2_Data.current;
	Wheelgetcur[2] = M3508_ID3_Data.current;
	Wheelgetcur[3] = M3508_ID4_Data.current;
	for(i=0;i<4;i++)
	{
		Wheelsetcur[i] = (rc.sw-DJI_CHMID)*30;//���������Ը���ֵ
		Debug_datamotor[i] = Wheelsetcur[i];
		//Wheelsetcur[i] = pid_increase_calc(&pid_wheel_speed[i],Wheelgetrpm[i],Wheelsetrpm[i]);
		//Current_Wheel[i] = pid_increase_calc(&pid_wheel_current[i],Wheelgetcur[i],Wheelsetcur[i]);
		Current_Wheel[i] = pid_increase_calc(&pid_wheel_speed[i],Wheelgetrpm[i],Wheelsetrpm[i]);
		Debug_datamotor[i+4] = Current_Wheel[i];
		WheelLook[i] = Current_Wheel[i];
	}
	
	/**********GM6020Yaw��PID����********************/
/***********************************˫�ջ�����*****************************************************/
	if(Gimbal_Yaw_Setangle != MAXANGLE)
	{
		Speed_Yaw = pid_increase_calc(&pid_yaw_angle,GM6020_Yaw_Data.angle,Gimbal_Yaw_Setangle);
	}
	else
	{
		Speed_Yaw = Gimbal_Yaw_Setrpm;
	}
	Voltage_Yaw = pid_increase_calc(&pid_yaw_speed,GM6020_Yaw_Data.rotation,Speed_Yaw);
/***********************************���ջ�����*****************************************************/
//	if(Gimbal_Yaw_Setangle == MAXANGLE)
//	{
////		if (Gimbal_Yaw_Setangle > YAW_MAXANGLE && GM6020_Yaw_Data.rotation >= 0)					//�ٶȿ����£�Yaw����Ƕȳ��� ���
////			Voltage_Yaw = pid_increase_calc(&pid_yaw_angle,GM6020_Yaw_Data.angle,YAW_MAXANGLE);
////		else if (Gimbal_Yaw_Setangle < YAW_MINANGLE && GM6020_Yaw_Data.rotation <= 0) 		//�ٶȿ����£�Yaw����Ƕȳ��� ��С
////			Voltage_Yaw = pid_increase_calc(&pid_yaw_angle,GM6020_Yaw_Data.angle,YAW_MINANGLE);
////		else 
//		Voltage_Yaw = pid_increase_calc(&pid_yaw_speed,GM6020_Yaw_Data.rotation,Gimbal_Yaw_Setrpm);
////		Z_angle = 100*IMUAngle.imuYaw;
////		Z_rad = 100*IMUGyro.Wz*60/360;
////		Yaw_setRPM = 100*Gimbal_Yaw_Setrpm;
////		Yaw_realRPM = 100*GM6020_Yaw_Data.rotation;
////		Yaw_realAngle = 100*GM6020_Yaw_Data.angle/8192*360;
//	}
//	else
//	{
//		Voltage_Yaw = pid_increase_calc(&pid_yaw_angle,GM6020_Yaw_Data.angle,Gimbal_Yaw_Setangle);
//	}
	/**********GM6020Pitch��PID����********************/	
/***********************************˫�ջ�����*****************************************************/
	if(Gimbal_Pitch_Setangle != MAXANGLE)
	{
		Speed_Pitch = pid_posation_calc(&pid_pitch_angle,GM6020_Pitch_Data.angle,Gimbal_Pitch_Setangle);
	}
	else
	{
		Speed_Pitch = Gimbal_Pitch_Setrpm;
	}
	Voltage_Pitch = pid_posation_calc(&pid_pitch_speed,GM6020_Pitch_Data.rotation,Speed_Pitch);
/***********************************���ջ�����*****************************************************/
//	if (Gimbal_Pitch_Setangle == MAXANGLE)
//	{
//		if (Gimbal_Pitch_Setangle > PITCH_MAXANGLE && GM6020_Pitch_Data.rotation >= 0) 		//�ٶȿ����£�Pitch����Ƕȳ��� ���
//			Voltage_Pitch = pid_increase_calc(&pid_pitch_angle,GM6020_Pitch_Data.angle,PITCH_MAXANGLE);
//		else if(Gimbal_Pitch_Setangle < PITCH_MINANGLE && GM6020_Pitch_Data.rotation <= 0)	//�ٶȿ����£�Pitch����Ƕȳ��� ��С
//			Voltage_Pitch = pid_increase_calc(&pid_pitch_angle,GM6020_Pitch_Data.angle,PITCH_MINANGLE);
//		else
//		Voltage_Pitch = pid_increase_calc(&pid_pitch_speed,GM6020_Pitch_Data.rotation,Gimbal_Pitch_Setrpm);
//	}
//	else
//	{
//		Voltage_Pitch = pid_increase_calc(&pid_pitch_angle,GM6020_Pitch_Data.angle,Gimbal_Pitch_Setangle);
//	}
	/**********M2006�����PID����********************/	
	if(Gimbal_Trigger_Setcircle == 0)
	{
		circle_count=0;
		Current_Trigger = pid_increase_calc(&pid_trigger_speed,M2006_Data.rotation,Gimbal_Trigger_Setrpm);
	}
	else
	{  
		circle = Circle_Calculate(M2006_Data.angle);
		circle_count += circle;
		Motor_Gimbal_Trigger.Trigger_circle = Motor_Gimbal_Trigger.Trigger_circle-circle;
		Lookcircle_count=circle_count;
		Gimbal_Trigger_get_angle = circle_count*MAXANGLE+M2006_Data.angle;
		Speed_Trigger = pid_increase_calc(&pid_trigger_angle,Gimbal_Trigger_get_angle,Gimbal_Trigger_set_angle);
		Current_Trigger = pid_increase_calc(&pid_trigger_speed,M2006_Data.rotation,Speed_Trigger);
//		Current_Trigger = pid_increase_calc(&pid_test_moto,Gimbal_Trigger_get_angle,Gimbal_Trigger_set_angle);
		if ((Gimbal_Trigger_set_angle-Gimbal_Trigger_get_angle)<10||(Gimbal_Trigger_get_angle-Gimbal_Trigger_set_angle)<10)
		{
			circle_count = 0;
			Motor_Gimbal_Trigger.Trigger_circle = 0;
			Motor_Gimbal_Trigger.Trigger_angle 	= M2006_Data.angle;
		}
//		if (Trigger_stepcount < TRIGGER_TOOTH)
			
		
//		{
//			Current_Trigger = pid_increase_calc(&pid_trigger_speed,M2006_Data.rotation,TRIGGER_SHOOTSPEED);
//			if((M2006_Data.angle - Trigger_reset_angle > TRIGGER_ANGLESTEP) || (Trigger_reset_angle - M2006_Data.angle < TRIGGER_ANGLESTEP*3))
//			{
//				Trigger_reset_angle = M2006_Data.angle;
//				Trigger_stepcount ++;
//			}
//		}//�����ת����ֵ�жϣ��ж�һ�μ�1��ֱ������Ԥ��λ�� Ĭ�ϵ����ת
//		else 
//		{
//		Current_Trigger = pid_increase_calc(&pid_trigger_angle,M2006_Data.angle,Trigger_reset_angle);	
//		}
	}
	/**************Snail2305�ļ���****************************************/
	Fric_rpm=Gimbal_Fric_Setrpm;
	Fric_pluse=Fric_rpm;
	Debug_dataspan[0]=100*Chassis_Span_Stc.ZAngle;
	Debug_dataspan[1]=100*IMUGyro.Wz*60/360;
	Debug_dataspan[2]=100*Motor_Gimbal_Yaw.Yaw_rpm;
	Debug_dataspan[3]=100*IMUAngle.imuYaw;
//	Debug_dataspan[4]=100*Chassis_Span_Stc.ZreAngle;
//	Debug_dataspan[5]=100*Motion_Chassis_Rpm.Xrpm;
//	Debug_dataspan[6]=100*Motion_Chassis_Rpm.Yrpm;
//	Debug_dataspan[7]=100*Motion_Chassis_Rpm.Zrpm;
//	Debug_dataspan[8]=(rc.ch1-DJI_CHMID)*100;
//	Debug_dataspan[9]=(rc.ch0-DJI_CHMID)*100;
//	Debug_dataspan[0]=Gimbal_Trigger_Setrpm;
//	Debug_dataspan[1]=M2006_Data.rotation;
//	Debug_dataspan[2]=Current_Trigger;
//	Debug_dataspan[3]=pid_trigger_speed.p*100;
//	Debug_dataspan[4]=pid_trigger_speed.i*100;
//	Debug_dataspan[5]=circle_count;
//	Debug_dataspan[6]=M2006_Data.angle;
//	Debug_dataspan[7]=Gimbal_Trigger_set_angle;
//	Debug_dataspan[8]=Gimbal_Trigger_get_angle;
////	Debug_dataspan[9]=Current_Trigger;
//	Debug_dataspan[0]=Gimbal_Yaw_Setangle*5;
//	Debug_dataspan[1]=GM6020_Yaw_Data.angle*5;
//	Debug_dataspan[2]=Voltage_Yaw/10;
//	Debug_dataspan[3]=GM6020_Yaw_Data.rotation*100;
//	Debug_dataspan[4]=Gimbal_Yaw_Setrpm*100;
//	Debug_dataspan[5]=Gimbal_Pitch_Setangle*5;
//	Debug_dataspan[6]=GM6020_Pitch_Data.angle*5;
//	Debug_dataspan[7]=Voltage_Pitch/10;
//	Debug_dataspan[8]=GM6020_Pitch_Data.rotation*1000;
//	Debug_dataspan[9]=Gimbal_Pitch_Setrpm*1000;
//	Debug_dataspan[5]=Speed_Yaw*100;
//	Debug_dataspan[5]=circle_count;
//	Debug_dataspan[6]=M2006_Data.angle;
//	Debug_dataspan[7]=Gimbal_Trigger_set_angle;
//	Debug_dataspan[8]=Gimbal_Trigger_get_angle;
//	Debug_dataspan[9]=Current_Trigger;
	
	//DEBUG_Anony16Send(ANONY_ID1,Debug_dataspan);
	//DEBUG_Anony16Send(ANONY_ID3,Debug_datamotor);
	TIM_SetCompare1(TIM1,Fric_pluse);
	TIM_SetCompare2(TIM1,Fric_pluse);
	TIM_SetCompare3(TIM1,Fric_pluse);
	TIM_SetCompare4(TIM1,Fric_pluse);
	TIM_SetCompare1(TIM8,Helm_pluse);
	TIM_SetCompare2(TIM8,Helm_pluse);
	TIM_SetCompare3(TIM8,Helm_pluse);
	TIM_SetCompare4(TIM8,Helm_pluse);
	CAN_Send_Chassis_Msg(Current_Wheel[0],Current_Wheel[1],Current_Wheel[2],Current_Wheel[3]);
	CAN_Send_Gimball_Msg(0,Voltage_Pitch,Current_Trigger,Voltage_Yaw);
//	CAN_Send_Gimball_Msg(0,Voltage_Pitch,0,0);
	CAN_Send_Extra_Msg(0,0,0,0);
	
}

