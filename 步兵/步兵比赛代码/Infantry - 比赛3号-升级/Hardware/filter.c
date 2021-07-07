
#include "filter.h"

/**************±àÂëÆ÷¼ÆÈ¦******************/
uint16_t Angle[4]={MIDANGLE,MIDANGLE,MIDANGLE,MIDANGLE};
int8_t Circle_Calculate(uint16_t ucAngel)
{
//	static uint16_t Angle[4]={MIDANGLE};
	uint8_t i;
	for (i=3;i>0;i--)
	{
		Angle[i]=Angle[i-1];
	}
	Angle[0] = ucAngel;
	if (Angle[3]>MIDANGLE&&Angle[2]>MIDANGLE&&Angle[1]<MIDANGLE&&Angle[0]<MIDANGLE&&(Angle[2]-Angle[1])>MIDANGLE)
		return 1;
//	else if (Angle[3]<MIDANGLE&&Angle[2]<MIDANGLE&&Angle[1]>MIDANGLE&&Angle[0]>MIDANGLE&&(Angle[1]-Angle[2])>MIDANGLE)
//		return -1;
	else
		return 0;
}

//float IMU_Filter_Calculate()
//{
//	
//}
