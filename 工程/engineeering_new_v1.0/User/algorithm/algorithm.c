/**
  ******************************************************************************
  * @file    Project/USER/main.c 
  * @author  Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention 此文件存放各种自定义的数学函数以及一些算法
  ******************************************************************************
      ..................NEUQ_SUDO..................
*/
#include "algorithm.h"


/**
  *@brief  自定义求绝对值函数
  */
int abs(int input)
{
	if(input<0)
		input = -input;
	return input;
}

/**
  *@brief  卡尔曼滤波函数
           Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
           R:测量噪声，R增大，动态响应变慢，收敛稳定性变好       
  */
float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
                                        
{
        float R = MeasureNoise_R;
        float Q = ProcessNiose_Q;

        static        float x_last = 0;

        float x_mid = x_last;
        float x_now;

        static        float p_last = 0;

        float p_mid ;
        float p_now;
        float kg;       

        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
        kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
        x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
               
        p_now=(1-kg)*p_mid;//最优值对应的covariance       

        p_last = p_now; //更新covariance值
        x_last = x_now; //更新系统状态值

        return x_now;               
}

/*-------------------------------------------------------------------------------------------------------------*/



