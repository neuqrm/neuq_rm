#ifndef MOTOR_H
#define MOTOR_H


//can stdid
#define CAN_LoopBack_ID				 0x200		//用于环回模式自检调试
#define	CAN_3508Motor1_ID  	   0x201
#define	CAN_3508Motor2_ID      0x202
#define	CAN_3508Motor3_ID      0x203
#define	CAN_3508Motor4_ID      0x204
#define CAN_TRIGGER_ID         0x207
#define CAN_GIMBAL_Y_ID         0x209
#define CAN_GIMBAL_P_ID         0x20A


//电机转速pid参数结构体
typedef struct{
	int err;
	int last_err;
	int err_integration;
	int target_speed;
	int actual_speed;
	
	int P_OUT;
	int I_OUT;
	int D_OUT;
	int PID_OUT;
	
	int pid_count;           //pid变量，用于计算平均误差
	float average_err;           //平均误差，使误差曲线更加平滑
	float last_average_err;      //上一次平均误差
	int pitch_err[10];
}VPID_t;

//电机机械角度参数
typedef struct{
	
	int err;
	int last_err;
	int err_integration;
	int actual_angle;
	int target_angle;
	int P_OUT;
	int I_OUT;
	int D_OUT;
	int PID_OUT;
	int trigger_first_total_angle_storage;
	/*int actual_speed;
	int target_speed;*/
}APID_t;
//电机参数结构体
typedef struct{
	
	int start_angle;			//电机初始角度值
	int start_angle_flag;	//记录电机初始角度值的flag
	int stop_angle;				//发送停止命令时候的角度值
	int target_angle;
	
	float actual_angle;			//当前真实角度值
	int last_angle;				//上一次返回的角度值
	int round_cnt;				//相对开机时转过的圈数
	int total_angle;			//总共转过的计数
	int last_bullet_angle;		//上次发射子弹时拨弹轮的位置
	
	float actual_speed;			//电机真实速度,rpm
	int target_speed;			//电机目标速度,rpm  转/min
	int last_speed;       //电机上一次回传的速度值
	int actual_current;		//电机真实电流
	int target_current;		//电机目标电流
	//int temp;							//电机温度（2006电机不支持，3508支持）
	VPID_t vpid;
	APID_t apid;
}MOTOR_t;

extern MOTOR_t motor1,motor2,motor3,motor4,motor5,motor6,gimbal_y,gimbal_p;//**********************************

extern int max_motor_speed;
extern float max_base_linear_speed;
extern float max_base_rotational_speed;

//电机电流参数结构体
typedef struct{			
	
	int motor1_current;
	int motor2_current;
	int motor3_current;
	int motor4_current;
	int motor5_current;
	int motor6_current;
	int gimbal_y_current;
	int gimbal_p_current;
	
}LOOPBACK;

extern LOOPBACK loopback;
	
void record_gimbal_callback(MOTOR_t *motor, unsigned short angle, short speed, short current);
void record_motor_callback(MOTOR_t *motor, unsigned short angle, short speed, short current);
void motor_init(void);			//电机初始化
void set_chassis_current(void);	//设置电机电流
void set_trigger_current(void);
void set_gimbal_current(void);
void stop_chassis_motor(void);	//将电机角度固定在当前值
void stop_trigger_motor(void);
void set_gimbal_current(void); //设置云台电流 2020.7.24
void record_trigger_callback(MOTOR_t *motor, unsigned short angle, short speed, short current);
#endif



