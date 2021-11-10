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

  ...........革命尚未成功，同志仍需努力...........
*/  
#include "My_Init.h"
#include "control.h"  //实验，用于改control_flag值
/**
  *@brief  主函数初始化，进入循环等待中断
  */
//extern unsigned char left_ucRxFinish; //串口6接受完成标志
//extern unsigned char right_ucRxFinish;//串口8接受完成标志

extern DJi_RC rc;
int flag2=0;
int main()
{
 	All_Init();												//机器人硬件及结构体初始化，系统时钟180MHz
	pid_init();                       //初始化pid各项参数的值
	mode_init();                      //初始化机器人模式控制
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);

	while(1)                          //进入循环
	{
		
		/****    向上位机发送数据   *****/
		if(MSG_SEND_EN)
		{
			//hit_judge();//是否受击
			send_infantry_info_by_json();		   
			//hit_flag	=	0;  //保证不重复上传
			delay_ms(100);//设置延时防止出现程序跑飞
		}
		
	}
	
}

/**
  * @brief  定时器3中断服务函数			1ms
  */
void TIM3_IRQHandler(void)
{
	static int time_count=1;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) 	//溢出中断
	{ 
		time_count++;
		
		/*****   遥控器控制    ******/
	  if(Remote_control_mode == chassis_CH_width)  //左开关在中间
		{
			
			if(control_flag == 1)//试验代码
			{
				control_flag = 0;
				Kinematics.yaw.target_angular = 0;  //锁死！！
				Kinematics.pitch.target_angular = 0;
			}
			
			control_mode = DJi_Remote_Control;
		}
		
		else if(Remote_control_mode == 2) //开关在下
		{//试验代码
			control_flag = 1;
			cruise_mode();
		}
		
		else   //开关在上或者遥控器关闭
		{
			control_mode = auto_control;
			//control_flag = 0;		//手动控制时才赋零，自动时看上位机发送的值
			cruise_mode();
		}

		if(control_mode == DJi_Remote_Control) //判断是否为遥控模式
		Remote_Control();				//遥控器控制代码
		
		/*****  ROS上位机控制  *****/
		if(control_mode == auto_control) //判断是否为自动控制模式
	{
		if(flag_command_recieved == 1)	//每一毫秒检查一次是否收到控制指令，若标识符为一说明对应指令待解析，下述同理
		{
			resolve_json_chassis_command(); //底盘指令
			flag_command_recieved = 0;	
		}
   
	 if(flag_command_recieved1 == 1)
		{
			resolve_json_gimbal_speed_command();  //云台速度指令
			flag_command_recieved1 = 0;	
		}
	
	 	if(flag_command_recieved2 == 1)
		{
		  resolve_json_trigger_command();  //拨弹轮指令
			resolve_json_fric_command();     //摩擦轮指令
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
			resolve_json_control_command();//接收上位机信号
			flag_command_recieved4 = 0;	
		}
		
			if(flag_command_recieved5 == 1)	
		{
				resolve_json_gimbal_speed_command();
			  flag_command_recieved5 = 0;	
		}
	}
		/****  机器人运动控制  *****/
		if(time_count%7 ==0)		//7ms
		//	chassis_behavior();
			Robo_Move();
		
		/*****    按键扫描   ******/
		if(time_count%31 == 0)		//31ms  消抖
			Debug_Key();
		
		/*****    工作状态指示灯   ******/
		if(time_count%500 == 0)			//500ms
		{
			LED1=!LED1;							//单片机正常工作状态指示灯
			time_count = 0;
		}
		
		if(time_count%4 == 0)		//4ms  测速
			Get_Base_Velocities();		//计算底盘中心实际速度
		  Get_Gimbal_Angle();
			Get_referee_info();


		if(time_count>=1000)			//清除计数标志    1s
			time_count=1;
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}

