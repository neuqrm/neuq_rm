#ifndef _KINEMATIC_H
#define _KINEMATIC_H

#define wheel_diameter  10.000000f			//����ֱ��
#define half_width  25.000000f		//���̰��
#define half_length  35.000000f		//���̰볤

#define PI 			3.141593f
#define RPM2RAD 0.104720f										//ת��ת���ٶ�		1 rpm = 2pi/60 rad/s 
#define RPM2VEL 0.523599f										//ת��ת���ٶ�		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f										//���ٶ�תת��
#define M2006_REDUCTION_RATIO 36.000000f		//��������ٱ�
#define M3508_REDUCTION_RATIO 19.000000f		//��������ٱ�

#define MAX_MOTOR_SPEED   15336				//������ת�٣��궨�巽���޸�   ��Χ0 - 10000   15336   
#define MAX_BASE_LINEAR_SPEED    217.817f    //�������ƽ���ٶȣ���λcm/s   
#define MAX_BASE_ROTATIONAL_SPEED    7.260570f    //���������ת�ٶȣ���λrad/s    


#define set_trigger_speed(motor5_speed) \
				do{                                   \
						motor5.vpid.target_speed = motor5_speed; \
	          motor5.target_speed = motor5_speed;	     \
        }while(0)                                    \

#define set_gimbal_y_motor_speed(gimbal_y_speed) \
				do{                                    \
			     gimbal_y.vpid.target_speed = gimbal_y_speed; \
	         gimbal_y.target_speed = gimbal_y_speed;		  \
        }while(0)                                     \

#define trigger_to_motor(trigger_angular)	\
				do{                               \
					motor5.target_speed =(int)(trigger_angular*M2006_REDUCTION_RATIO); \
				}while(0)                                                            \
												
#define set_gimbal_angle(yaw_angle,pitch_angle) \
				do{ \
					gimbal_y.apid.target_angle = yaw_angle; \
          gimbal_p.apid.target_angle = pitch_angle; \
				}while(0)                                   \

				
typedef struct
{
	float linear_vel;			//���ٶ�
	float rpm;						//ת��Ȧÿ����
}Speed_t;

typedef struct
{
	Speed_t target_speed;			
	Speed_t actual_speed;						
}Wheel_t;

//���̼������ĵ���/���ٶ�
typedef struct
{
	float linear_x;	//m/s
	float linear_y;
	float angular_z; //���ٶ�rpm
}Velocities_t;

typedef struct
{
	float target_angular;
	float actual_angular;
  float target_angle;
	float actual_angle;
}Application_t;

typedef struct
{
	Wheel_t wheel1;
	Wheel_t wheel2;
	Wheel_t wheel3;
	Wheel_t wheel4;
	
	Velocities_t target_velocities;		//Ŀ�����ٶ�
	Velocities_t actual_velocities; 	//ʵ�����ٶ�
	Application_t trigger;
	Application_t fric;
	Application_t pitch;
	Application_t yaw;
}Kinematics_t;




extern Kinematics_t Kinematics;
extern float max_base_linear_speed;
extern float max_base_rotational_speed;

extern int stop_flag_1;
extern int stop_flag_2;
extern int stop_flag_3; //�¼���ģ���ֹɲ����Ӳ����by LUO   666  by Q

void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
void Get_Base_Velocities(void);
void chassic_speed_control(float speed_x,float speed_y,float speed_r);		//�����������ٶ�ת��Ϊ���ת��
void trigger_control(float trigger_angular);
void gimbal_speed_control(float gimbal_y_angle,float gimbal_p_angle);
void break_jugement(void);  // by luo
void gimbal_angle_control(float yaw_angle,float pitch_angle);

#endif


