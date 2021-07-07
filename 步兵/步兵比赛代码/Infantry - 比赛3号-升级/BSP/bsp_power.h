#ifndef __CTRL_H
#define	__CTRL_H

#include "stm32f4xx.h"

//���Ŷ���
/*******************************************************/
#define POWER_GPIO_CLK            RCC_AHB1Periph_GPIOH
#define POWER_GPIO_PORT           GPIOH
#define POWER1_PIN								GPIO_Pin_2   
#define POWER2_PIN								GPIO_Pin_3   
#define POWER3_PIN								GPIO_Pin_4  
#define POWER4_PIN								GPIO_Pin_5   

/***���Ƶ�Դ���صĵĺ꣬
	* ��Դ�͵�ƽ��������ON=0��OFF=1
	* ����Դ�ߵ�ƽ�����Ѻ����ó�ON=1 ��OFF=0 ����
	*/
#define POWER_ON  1
#define POWER_ONOFF 0

/* ���κ꣬��������������һ��ʹ�� */
#define POWER1(a)	if (a)	\
					GPIO_SetBits(POWER_GPIO_PORT,POWER1_PIN);\
					else		\
					GPIO_ResetBits(POWER_GPIO_PORT,POWER1_PIN)

#define POWER2(a)	if (a)	\
					GPIO_SetBits(POWER_GPIO_PORT,POWER2_PIN);\
					else		\
					GPIO_ResetBits(POWER_GPIO_PORT,POWER2_PIN)
					
#define POWER3(a)	if (a)	\
					GPIO_SetBits(POWER_GPIO_PORT,POWER3_PIN);\
					else		\
					GPIO_ResetBits(POWER_GPIO_PORT,POWER3_PIN)
					
#define POWER4(a)	if (a)	\
					GPIO_SetBits(POWER_GPIO_PORT,POWER4_PIN);\
					else		\
					GPIO_ResetBits(POWER_GPIO_PORT,POWER4_PIN)
					
					
/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRRL=i;}			  //����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BSRRH=i;}				//����͵�ƽ


/* �������IO�ĺ� */
#define POWER1_OFF		digitalHi(POWER_GPIO_PORT,POWER1_PIN)
#define POWER1_ON			digitalLo(POWER_GPIO_PORT,POWER1_PIN)

#define POWER2_OFF		digitalHi(POWER_GPIO_PORT,POWER2_PIN)
#define POWER2_ON			digitalLo(POWER_GPIO_PORT,POWER2_PIN)

#define POWER3_OFF		digitalHi(POWER_GPIO_PORT,POWER3_PIN)
#define POWER3_ON			digitalLo(POWER_GPIO_PORT,POWER3_PIN)

#define POWER4_OFF		digitalHi(POWER_GPIO_PORT,POWER4_PIN)
#define POWER4_ON			digitalLo(POWER_GPIO_PORT,POWER4_PIN)

void POWER_GPIO_Config(void);

#endif /* __CTRL_H */
