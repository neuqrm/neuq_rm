#ifndef __REMOTE_H
#define	__REMOTE_H


#include "bsp_dbus.h"
#include "bsp_led.h"
#include "motor.h"
#include "motion.h"
#include "imu.h"
#include "math.h"
#include "basicdata.h"
#include "limit.h"


/*********************************************************************************
*��ؿ��Ʋ����궨��
*�ٶȿ�������float �Ƕȿ�������uint6_t
*/ 

#define pi	3.1415926f

#define XRPM_RESET  100.0f	//ս���ٶȳ�ʼֵ X
#define YRPM_RESET  100.0f	//ս���ٶȳ�ʼֵ Y
#define ZRPM_RESET  100.0f	//ս���ٶȳ�ʼֵ Z
#define RPM_STEP		1		//ս������or��������ֵ
#define XYRPM_MAX		300.0f	//ս��ֱ���ٶ����ֵ
#define ZRPM_MAX 		200.0f	//ս�����ٶ����ֵ
#define ZRPM_SPAN		40.0f  //С����ת��

#define REMOTEAIM		0xf0 //ң����׼
#define AUTOAIM			0x0f //����
extern uint8_t AIMMODE;

#define SPANCHASSIS	0xf0 //С���ݵ���
#define NORMALCHASSIS 0x0f	//��������
extern uint8_t CHASSISMODE;

//#define SINGALSHOOT 0xf0	//�����־
//#define CONTINUESHOOT 0x0f	//�����־
//extern uint8_t SHOOTMODE;
/**********************************************************************************
ң����ͨ��ֵ
ͨ��0~3 CH0~3
MAX	=	1684��MID	=	1024��MIN	=	364
ͨ��4~5	S1~2 ��S1 ��S2
��	=	1��		��	=	3��		��	=	2��
ͨ��6		SW
MAX	=	1684��MID	=	1024��MIN	=	364
�ٶȵ�λ rpm		�Ƕȵ�λ 360��
***********************************************************************************/
#define DJI_CHMID	1024
#define DJI_MAX		1684
#define DJI_MIN		364
#define DJI_MID		660
#define DJI_XVAL 	3.0f		//X�����ٶ�ϵ��
#define DJI_YVAL 	3.0f		//Y�����ٶ�ϵ��
#define DJI_ZVAL	4.0f		//Z�����ٶ�ϵ��
//#define DJI_YawVAL 10.0f		//Yaw���ٶ�ϵ��
//#define DJI_PitchVAL 30.0f	//Pitch���ٶ�ϵ��
#define SINGAL_SHOOTANGLE	682.0f							//�����ֲ���ֵ ����� 360�� = 0~8192
#define	DJI_Motion_X	(rc.ch1)						//ʹ��������ֱң�п��Ƶ���ǰ��
#define	DJI_Motion_Y	(rc.ch0)						//ʹ������ˮƽҡ�˿��Ƶ�������
#define	DJI_Motion_Z	(rc.ch2)						//ʹ������ˮƽҡ�˿��Ƶ���ת��
#define	DJI_Motion_Pitch	(rc.ch3)				//ʹ��������ֱҡ�˿�����̨pitch
#define	DJI_Motion_Yaw		(rc.sw)					//ʹ�����ֻ��ֿ�����̨yaw
#define DJI_Action_Flag		((rc.s1<<4)|rc.s2)	//�Ƹ�ѡ����̨����
#define	DJI_Action_Reset			((3<<4)|3)	//��λ״̬ �޶���
#define	DJI_Action_Change			((3<<4)|1)	//ң�����ͼ����л�
#define	DJI_Action_Brake			((3<<4)|2)		//����ɲ��
#define	DJI_Action_Fric				((1<<4)|3)	//��Ħ����
#define	DJI_Action_Trigger		((1<<4)|2)	//��������
#define	DJI_Action_ShootSingal	((1<<4)|1)	//����
#define	DJI_Action_AUTOAIM		((2<<4)|2)	//����
#define	DJI_Action_Unknown2		((2<<4)|1)	//
#define	DJI_Action_Span				((2<<4)|3)	//С������ת
//#define	DJI_Action_Reset			0x33	//��λ״̬ �޶���
//#define	DJI_Action_Change			0x31	//ң�����ͼ����л�
//#define	DJI_Action_Brake			0x32		//����ɲ��
//#define	DJI_Action_Fric				0x13	//��Ħ����
//#define	DJI_Action_Trigger		0x12	//��������
//#define	DJI_Action_ShootSingal	0x11	//����
//#define	DJI_Action_AUTOAIM		0x22	//����
//#define	DJI_Action_Unknown2		0x21	//
//#define	DJI_Action_Span				0x23	//С������ת


extern Chassis_Rpm_stc  	Remote_Chassis_Rpm;
extern Gimbal_Yaw_stc  		Remote_Gimbal_Yaw;
extern Gimbal_Pitch_stc 	Remote_Gimbal_Pitch;
extern Gimbal_Trigger_stc	Remote_Gimbal_Trigger;

extern void DJI_Remote_Control(void);
extern void DJI_Remote_Chassis(void);
extern void DJI_Remote_Gimball(void);
extern void DJI_Remote_Action(void);

/*************************************************************************************
PCͨ��ֵ
���
�ռ������� X��Y��Z������ٶ�ֵ
MAX	=	+32767��static	=	0��MIN	=	-32768
������Ҽ�
���£�1		δ���£�0
����
0~7bit ÿλ��Ӧһ������״̬
W	S	A	D	Q	E	shift	ctrl
*************************************************************************************/

#define PC_KEY_W			((uint16_t)0x01<<0)
#define PC_KEY_S			((uint16_t)0x01<<1)
#define PC_KEY_A			((uint16_t)0x01<<2)
#define PC_KEY_D			((uint16_t)0x01<<3)
#define PC_KEY_Shift	((uint16_t)0x01<<4)
#define PC_KEY_Ctrl		((uint16_t)0x01<<5)
#define PC_KEY_Q			((uint16_t)0x01<<6)
#define PC_KEY_E			((uint16_t)0x01<<7)
#define PC_KEY_Flag		(RC_Ctl.key.v)
#define PC_Press_W			(((uint16_t)0x01<<0)&RC_Ctl.key.v)
#define PC_Press_S			(((uint16_t)0x01<<1)&RC_Ctl.key.v)
#define PC_Press_A			(((uint16_t)0x01<<2)&RC_Ctl.key.v)
#define PC_Press_D			(((uint16_t)0x01<<3)&RC_Ctl.key.v)
#define PC_Press_Shift	(((uint16_t)0x01<<4)&RC_Ctl.key.v)
#define PC_Press_Ctrl		(((uint16_t)0x01<<5)&RC_Ctl.key.v)
#define PC_Press_Q			(((uint16_t)0x01<<6)&RC_Ctl.key.v)
#define PC_Press_E 			(((uint16_t)0x01<<7)&RC_Ctl.key.v)
#define PC_Motion_XP	(PC_KEY_W)
#define PC_Motion_XN	(PC_KEY_S)
#define PC_Motion_YP	(PC_KEY_A)
#define PC_Motion_YN	(PC_KEY_D)
#define PC_Motion_ZP	(PC_KEY_E)
#define PC_Motion_ZN	(PC_KEY_Q)
#define PC_Motion_WS	(PC_KEY_W|PC_KEY_Shift)
#define PC_Motion_SS	(PC_KEY_S|PC_KEY_Shift)
#define PC_Motion_ES	(PC_KEY_E|PC_KEY_Shift)
#define PC_Motion_QS	(PC_KEY_Q|PC_KEY_Shift)
#define PC_Motion_Span	(PC_KEY_Shift)
#define PC_Motion_Fire	(PC_KEY_Ctrl)
//#define PC_Motion_ZP	(PC_KEY_W|PC_KEY_A)
//#define PC_Motion_ZN	(PC_KEY_W|PC_KEY_D)
//#define PC_Motion_XPup	(PC_KEY_W|PC_KEY_Shift)
//#define PC_Motion_XNup	(PC_KEY_S|PC_KEY_Shift)
//#define PC_Motion_YPup	(PC_KEY_A|PC_KEY_Shift)
//#define PC_Motion_YNup	(PC_KEY_D|PC_KEY_Shift)
//#define PC_Motion_ZPup	(PC_KEY_W|PC_KEY_A|PC_KEY_Shift)
//#define PC_Motion_ZNup	(PC_KEY_W|PC_KEY_D|PC_KEY_Shift)
//#define PC_Motion_XPdown	(PC_KEY_W|PC_KEY_Ctrl)
//#define PC_Motion_XNdown	(PC_KEY_S|PC_KEY_Ctrl)
//#define PC_Motion_YPdown	(PC_KEY_A|PC_KEY_Ctrl)
//#define PC_Motion_YNdown	(PC_KEY_D|PC_KEY_Ctrl)
//#define PC_Motion_ZPdown	(PC_KEY_W|PC_KEY_A|PC_KEY_Ctrl)
//#define PC_Motion_ZNdown	(PC_KEY_W|PC_KEY_D|PC_KEY_Ctrl)
//#define PC_Motion_Reset		(PC_KEY_E)
//#define PC_Motion_Stop		(PC_KEY_E|PC_KEY_Shift)
#define PC_Action_Reset		(PC_KEY_Shift|PC_KEY_Ctrl)
#define PC_Mouse_X		(RC_Ctl.mouse.x)
#define PC_Mouse_Y		(RC_Ctl.mouse.y)
#define PC_Mouse_Z		(RC_Ctl.mouse.z)
#define PC_Mouse_Left			(RC_Ctl.mouse.press_l)
#define PC_Mouse_Right		(RC_Ctl.mouse.press_r)

#define PC_YawVAL 45000.0f
#define PC_PitchVAL	75000.0f

#define XY_Flag 1
#define Z_Flag 0

extern void PC_Remote_Control(void);
extern void PC_Remote_Chassis(void);
extern void PC_Remote_Gimball(void);
extern void PC_Remote_Action(void);


#endif /* __REMOTE_H */
