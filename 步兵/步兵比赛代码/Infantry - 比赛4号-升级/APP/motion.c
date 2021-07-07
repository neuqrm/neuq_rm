
#include "motion.h"

Chassis_Rpm_stc  	Motion_Chassis_Rpm		=	{0};
Gimbal_Yaw_stc  	Motion_Gimbal_Yaw		=	{0};
Gimbal_Pitch_stc 	Motion_Gimbal_Pitch	=	{0};
Gimbal_Trigger_stc	Motion_Gimbal_Trigger	=	{0};
Motion_Chassis_Span_Stc	Chassis_Span_Stc = {0};


/*****************�˶�����������*******************************/
/**
 *@brief �������˶�����
 */
int16_t Debug_data0[10]={0};
void Motion_Ctrl(void)
{
	Motion_Gimbal_Ctrl();
	Motion_Chassis_Ctrl();
	Motion_Shoot_Ctrl();
	Debug_data0[0]=Motor_Chassis_Rpm.MoterID1_rpm;
	Debug_data0[1]=Motor_Chassis_Rpm.MoterID2_rpm;
	Debug_data0[2]=Motor_Chassis_Rpm.MoterID3_rpm;
	Debug_data0[3]=Motor_Chassis_Rpm.MoterID4_rpm;
	Debug_data0[4]=Motor_Gimbal_Yaw.Yaw_rpm;
	Debug_data0[5]=Motor_Gimbal_Yaw.Yaw_angle;
//	Debug_data0[6]=Motor_Gimbal_Pitch.Pitch_rpm;
//	Debug_data0[7]=Motor_Gimbal_Pitch.Pitch_angle;
	Debug_data0[6]=8000;
	Debug_data0[8]=Motor_Gimbal_Trigger.Trigger_rpm;
	Debug_data0[9]=Motor_Gimbal_Trigger.Fric_rpm;
	//DEBUG_Anony16Send(ANONY_ID1,Debug_data0);
}

/***************С����ģʽ�µ��˶�����*************************/
/**
  *@brief С����ģʽ�½�������˶�
  */
void Span_Move_Calculate(float ucAngle)
{
	float Zangle = 0.0f;
	double vx_set = 0,vy_set = 0,sin_yaw = 0,cos_yaw = 0;
	if (IMUAngle.imuYaw>ucAngle)
		Zangle = IMUAngle.imuYaw-ucAngle;
	else
		Zangle = IMUAngle.imuYaw-ucAngle+360.0f;
	sin_yaw = sin(Zangle/180.0f*pi);
	cos_yaw = cos(Zangle/180.0f*pi);
	vx_set = Remote_Chassis_Rpm.Xrpm;
	vy_set = Remote_Chassis_Rpm.Yrpm;
	Chassis_Span_Stc.Xrpm =  sin_yaw * vy_set + cos_yaw * vx_set;
  Chassis_Span_Stc.Yrpm =  cos_yaw * vy_set - sin_yaw * vx_set;
	Chassis_Span_Stc.ZreAngle = ucAngle;
	Chassis_Span_Stc.ZAngle = Zangle;
	Motion_Chassis_Rpm.Xrpm = Chassis_Span_Stc.Xrpm ;
	Motion_Chassis_Rpm.Yrpm = Chassis_Span_Stc.Yrpm;
}

/***************�������************************/
/**
  *@brief ���书�� trigger���ƽ���
  */
void Singal_Shoot_Calculate(void)
{
	uint32_t temp_Angle=0;
	uint8_t temp_circle=0;
	if (Motor_Gimbal_Trigger.Trigger_circle==0)
	temp_Angle = M2006_Data.angle;
	else
	temp_Angle = Motor_Gimbal_Trigger.Trigger_angle ;
	temp_Angle = TRIGGER_RADSTEP*RAD2ANGLE*M2006VAL + temp_Angle;
	temp_circle = temp_Angle/MAXANGLE;
	Motor_Gimbal_Trigger.Trigger_angle = temp_Angle;	//-temp_circle*MAXANGLE;
	temp_circle++; 
	Motor_Gimbal_Trigger.Trigger_circle = temp_circle;
	
}
/******************************************************
* @fn Motion_Gimbal_Ctrl
*
* @brief ��̨�������ָ�� ������̨���ת��or�ǶȲ��� 
* @pData ��λrpm  ���� Remote_Gimbal �ṹ��
* @return None.
* @note �ⲿ���� ���ת��Ϊ�����ʵ��ת�٣����ת��Ϊ�����ʵ��ת��
*/
void Motion_Gimbal_Ctrl(void)//��̨������ƺ���
{
	float temp_Yrpm=0,temp_Prpm=0;
	float temp_Yrad=YAW_RESETRAD,temp_Prad=PITCH_RESETRAD;
	if (AIMMODE==AUTOAIM&&CHASSISMODE==NORMALCHASSIS)
	{
		temp_Yrad	= Json_Gimbal_Yaw.Yaw_rad;
		temp_Prad	= Json_Gimbal_Pitch.Pitch_rad;
	}//�Զ���׼ �����Ƕ� �Ƕȿ���
	if (AIMMODE==REMOTEAIM&&CHASSISMODE==NORMALCHASSIS)
	{
		temp_Yrad	= Remote_Gimbal_Yaw.Yaw_rad;
		temp_Prad	= Remote_Gimbal_Pitch.Pitch_rad;
	}//�ֶ���׼ �Ƕȿ���
	if (CHASSISMODE==SPANCHASSIS)
	{
		temp_Yrad	= MAXANGLE;
		temp_Prad	= MAXANGLE;
		temp_Yrpm	= Remote_Gimbal_Yaw.Yaw_rpm;
		temp_Prpm	= Remote_Gimbal_Pitch.Pitch_rpm;
	}
//	if (DJI_Action_Flag==DJI_Action_Trigger)
//		temp_Prad = 20;
//	if (DJI_Action_Flag==DJI_Action_ShootSingal)
//		temp_Prad = -20;
	Motion_Gimbal_Yaw.Yaw_rad	= temp_Yrad;
	Motion_Gimbal_Yaw.Yaw_rpm = temp_Yrpm;
	Motion_Gimbal_Pitch.Pitch_rad	= temp_Prad;
	Motion_Gimbal_Pitch.Pitch_rpm = temp_Prpm;
	if (temp_Yrad!=MAXANGLE)
	{
		if (temp_Prad>0) //Pitch��ĸ����ǿ��ܲ��Գ�
			Motor_Gimbal_Pitch.Pitch_angle = temp_Prad*(PITCH_MAXANGLE-PITCH_RESETANGLE)/PITCH_MAXRAD+PITCH_RESETANGLE;
		else
			Motor_Gimbal_Pitch.Pitch_angle = -temp_Prad*(PITCH_MINANGLE-PITCH_RESETANGLE)/PITCH_MAXRAD+PITCH_RESETANGLE;
		Motor_Gimbal_Yaw.Yaw_angle = temp_Yrad*(YAW_MAXANGLE-YAW_RESETANGLE)/YAW_MAXRAD+YAW_RESETANGLE;
	}
	else 
	{
		Motor_Gimbal_Pitch.Pitch_angle = temp_Prad;
		Motor_Gimbal_Yaw.Yaw_angle = temp_Yrad;
	}
	Motor_Gimbal_Pitch.Pitch_rpm = temp_Prpm;
	Motor_Gimbal_Yaw.Yaw_rpm = temp_Yrpm;
}
/******************************************************
* @fn Motion_Chassis_Ctrl
*
* @brief �����ٶȿ��Ƽ��㺯��
* @pData  ��λrpm ����Remot_Chassis �ṹ��
* @return None.
* @note �ⲿ���ã��ı�����ٶ�
*/
void Motion_Chassis_Ctrl()		//���̵�����ƺ���
{
	volatile float Wheelrpm[4]={0},MaxRpm=0,ParaRpm=1; //����Ϊ�ֲ�������������ʹ�ã�
	uint8_t i=0;
	Motion_Chassis_Rpm.Xrpm = Remote_Chassis_Rpm.Xrpm*M3508AVAL;
	Motion_Chassis_Rpm.Yrpm = Remote_Chassis_Rpm.Yrpm*M3508AVAL;
	Motion_Chassis_Rpm.Zrpm = Remote_Chassis_Rpm.Zrpm*M3508AVAL;
	Wheelrpm[0] =  Motion_Chassis_Rpm.Xrpm + Motion_Chassis_Rpm.Yrpm + Motion_Chassis_Rpm.Zrpm*(CHASSIS_LONG + CHASSIS_WIDE)/MECANUM_DIAMETER;
	Wheelrpm[1] = -Motion_Chassis_Rpm.Xrpm + Motion_Chassis_Rpm.Yrpm + Motion_Chassis_Rpm.Zrpm*(CHASSIS_LONG + CHASSIS_WIDE)/MECANUM_DIAMETER;
	Wheelrpm[2] =  Motion_Chassis_Rpm.Xrpm - Motion_Chassis_Rpm.Yrpm + Motion_Chassis_Rpm.Zrpm*(CHASSIS_LONG + CHASSIS_WIDE)/MECANUM_DIAMETER;
	Wheelrpm[3] = -Motion_Chassis_Rpm.Xrpm - Motion_Chassis_Rpm.Yrpm + Motion_Chassis_Rpm.Zrpm*(CHASSIS_LONG + CHASSIS_WIDE)/MECANUM_DIAMETER;
//���̵��˶�����
//	V1 =  Vx + Vy + Wz*(a+b);
//	V2 = -Vx + Vy + Wz*(a+b);
//	V3 =  Vx - Vy + Wz*(a+b);
//	V4 = -Vx - Vy + Wz*(a+b);
// a��b�ֱ�ΪС�������һ��	VΪ���ٶ� WΪ���ٶ�
	MaxRpm = fabs(Wheelrpm[0]);//fabs();�Ը���������ȡ����ֵ
	for (i=0;i<4;i++)
	{
		if (fabs(Wheelrpm[i])>MaxRpm)
			MaxRpm=fabs(Wheelrpm[i]);
			if (MaxRpm>MAXSPEED*M3508AVAL)
		{
			ParaRpm=MAXSPEED*M3508AVAL/MaxRpm;
		}
	}
	for (i=0;i<4;i++)
	{
		Wheelrpm[i]=ParaRpm*Wheelrpm[i];
	}
	//�����ٶȹ�һ�� ȡ����ֵϵ��
	
	Motor_Chassis_Rpm.MoterID1_rpm = Wheelrpm[0]; 
	Motor_Chassis_Rpm.MoterID2_rpm = Wheelrpm[1]; 
	Motor_Chassis_Rpm.MoterID3_rpm = Wheelrpm[2]; 
	Motor_Chassis_Rpm.MoterID4_rpm = Wheelrpm[3]; 
}//�����ٶ������ش��ٶȵ�ӳ��


/******************************************************
* @fn Motion_Shoot_Ctrl
*
* @brief �����غ���
* @pData 
* @return None.
* @note �ⲿ����
*/

void Motion_Shoot_Ctrl()
{
	int16_t temp_Trpm=0;
	uint16_t temp_Frpm=FRICRPM_RESET;
	temp_Trpm = Remote_Gimbal_Trigger.Trigger_rpm;
	temp_Frpm = Remote_Gimbal_Trigger.Fric_rpm;
	Motor_Gimbal_Trigger.Trigger_rpm = temp_Trpm*M2006VAL;
	Motor_Gimbal_Trigger.Fric_rpm = temp_Frpm;
}










