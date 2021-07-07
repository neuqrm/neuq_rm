/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	基于RoboMaster二次开发
 ***************************************************************************/

#include "pid.h"

//云台电机 PID 结构体定义
pid_parameter pid_yaw_angle     	= { 0 };
pid_parameter pid_pitch_angle     = { 0 };
pid_parameter pid_yaw_speed     	= { 0 }; //yaw 轴速度环
pid_parameter pid_pitch_speed     = { 0 }; //pitch 轴速度环
//底盘电机 PID 结构体定义
pid_parameter pid_wheel_speed[4]  = { 0 };
pid_parameter pid_wheel_current[4]	=	{0};

//拨弹电机 PID 结构体定义
pid_parameter pid_trigger_angle = { 0 };
pid_parameter pid_trigger_speed = { 0 };
pid_parameter pid_trigger_current = { 0 };


pid_parameter pid_test_moto     = { 0 };

static void abs_limit(float *a, float ABS_MAX)
{
  if (*a > ABS_MAX)
      *a = ABS_MAX;
  if (*a < -ABS_MAX)
      *a = -ABS_MAX;
}

/**
  * @brief     PID 初始化函数
  * @param[in] pid: PID 结构体
  * @param[in] max_out: 最大输出
  * @param[in] intergral_limit: 积分限幅
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
void pid_init(pid_parameter *pid, uint32_t max_out, uint32_t intergral_limit, float kp, float ki, float kd)
{
  pid->integral_limit = intergral_limit;
  pid->max_output     = max_out;
	pid->err[NOW] = 0;
	pid->err[LAST] = 0;
	pid->err[PAST] = 0;
  pid->p = kp;
  pid->i = ki;
  pid->d = kd;
}

/**
  * @brief     PID 计算函数，使用位置式 PID 计算
  * @param[in] pid: PID 结构体
  * @param[in] get: 反馈数据
  * @param[in] set: 目标数据
  * @retval    PID 计算输出
  */
float pid_posation_calc(pid_parameter *pid, float get, float set)
{
  pid->get = get;
  pid->set = set;
  pid->err[NOW] = set - get; 
  
  pid->pout = pid->p * pid->err[NOW];
  pid->iout += pid->i * pid->err[NOW];
  pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

  abs_limit(&(pid->iout), pid->integral_limit);
  pid->out = pid->pout + pid->iout + pid->dout;
  abs_limit(&(pid->out), pid->max_output);

  pid->err[LAST]  = pid->err[NOW];
  
  return pid->out;
}

/**
  * @brief     PID 计算函数，使用增量式 PID 计算
  * @param[in] pid: PID 结构体
  * @param[in] get: 反馈数据
  * @param[in] set: 目标数据
  * @retval    PID 计算输出 输出有累积 pid->out = pid->out + pid->last 适用速度环
  */
float pid_increase_calc(pid_parameter *pid, float get, float set)
{
  pid->get = get;
  pid->set = set;
  pid->err[NOW] = set - get; 
  
  pid->pout = pid->p * (pid->err[NOW]-pid->err[LAST]);
  pid->iout = pid->i * pid->err[NOW];
  pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[PAST]);

  abs_limit(&(pid->iout), pid->integral_limit);
  pid->out = pid->pout + pid->iout + pid->dout + pid->last_out;
  abs_limit(&(pid->out), pid->max_output);	
	pid->last_out = pid->out;
	pid->err[PAST]  = pid->err[LAST];
  pid->err[LAST]  = pid->err[NOW];
  
  return pid->out;
}

/**
  * @brief     PID 计算函数，使用增量式 PID 计算
  * @param[in] pid: PID 结构体
  * @param[in] get: 反馈数据
  * @param[in] set: 目标数据
  * @retval    PID 计算输出 输出无累积 pid->out = pid->out 适用位置环
  */
float pid_increase_count(pid_parameter *pid, float get, float set)
{
  pid->get = get;
  pid->set = set;
  pid->err[NOW] = set - get; 
  
  pid->pout = pid->p * (pid->err[NOW]-pid->err[LAST]);
  pid->iout = pid->i * pid->err[NOW];
  pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[PAST]);

  abs_limit(&(pid->iout), pid->integral_limit);
  pid->out = pid->pout + pid->iout + pid->dout;
  abs_limit(&(pid->out), pid->max_output);	
	pid->err[PAST]  = pid->err[LAST];
  pid->err[LAST]  = pid->err[NOW];
  
  return pid->out;
}
/**
  * @brief     PID 参数复位函数
  * @param[in] pid: PID 结构体
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
void pid_reset(pid_parameter *pid, float kp, float ki, float kd)
{
  pid->p = kp;
  pid->i = ki;
  pid->d = kd;
  pid->err[NOW] = 0;
	pid->err[LAST] = 0;
	pid->err[PAST] = 0;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out  = 0;
	pid->last_out = 0;
}

/**
  * @brief     PID 参数清零函数
  * @param[in] pid: PID 结构体
  * @param[in] pid结构体
  */
void pid_clear(pid_parameter *pid)
{
	pid->err[NOW] = 0;
	pid->err[LAST] = 0;
	pid->err[PAST] = 0;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out  = 0;
	pid->last_out = 0;
}
