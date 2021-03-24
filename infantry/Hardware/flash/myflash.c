#include "myflash.h"
#include "stmflash.h"

float xp,xi,xd,yp,yi,yd,rp,ri,rd;
int a[9];

#define FLASH_SAVE_ADDR  0X0800C004 	

void Write_PID_Address(void)    
{
	int off=0;
	a[0]=xp*100;
	a[1]=xi*100;
	a[2]=xd*100;
	a[3]=yp*100;
	a[4]=yi*100;
	a[5]=yd*100;
	a[6]=rp*100;
	a[7]=ri*100;
	a[8]=rd*100;
	STMFLASH_Write(FLASH_SAVE_ADDR+off,(u32*)a,sizeof(a[0])/4*9);
}


void Read_PID_Address()
{
	STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)a,sizeof(a[0])/4*9);
	xp=a[0]/100.0;
	xi=a[1]/100.0;
	xd=a[2]/100.0;
	yp=a[3]/100.0;
	yi=a[4]/100.0;
	yd=a[5]/100.0;
	rp=a[6]/100.0;
	ri=a[7]/100.0;
	rd=a[8]/100.0;
}




