#include "filter.h"
#include "delay.h"
#define FILTER_N 6

int Filter(float filter_buf[FILTER_N]) 
{
	int i=0;
	int j=0;
	int filter_temp=0;
	for(j = 0; j < FILTER_N - 1; j++)
	{
		for(i = 0; i < FILTER_N - 1 - j; i++)
		{
			if(filter_buf[i] > filter_buf[i + 1]) 
			{
				filter_temp = filter_buf[i];
				filter_buf[i] = filter_buf[i + 1];
				filter_buf[i + 1] = filter_temp;
			}
		}
	}
	return filter_buf[(FILTER_N - 1) / 2];
}
