#include "intrins.h"
#include "main.h"
#include "stdio.h"
#define FOSC 11059200L          //系统频率
#define BAUD 115200             //串口1  串口2 共同波特率
u32 a;
extern		u32 temp[3];
extern u8 crc[16];
extern float TEMP1;
extern float DEEP;
extern float xdata Pressure;
u8 i;
void delayus(u8 i)   //@stc15-11.0592MHz定时器实测，带入数值低时候误差1，高于50后面误差0
{
   while(i)
	{  _nop_();
		 _nop_(); _nop_();
	    i--;
	}
}
void delayms(u16 i)   //@stc15-11.0592MHz定时器实测，误差千分1
{    unsigned char a,b;
	while(i)
	{  i--;
	a = 11;
	b = 189;
	do
	{
		while (--b);
	} while (--a);
	}
}
bit busy ;
void Send1(u8 dat)						//STC15  串口1的发送函数。
{
    while (busy);               //等待前面的数据发送完成
    ACC = dat;                  //获取校验位P (PSW.0)   
    busy = 1;                    //
	SBUF = ACC;                 //写数据到UART数据寄存器，SBUF它会自动发送串行数据，发送完成后busy会归零=0
}
void chuankou1()//与串口2共用T2定时器，共同波特率
{   P_SW1|=0x00;				//|=0x00[3.0/Rxd,3.1/Txd];|=0x40[3.6/Rxd,3.7/Txd];|=0x80[1.6/Rxd,1.7/Txd]
	SCON= 0x50; 
    T2L = (65536 - (FOSC/4/BAUD));   //设置波特率重装值
    T2H = (65536 - (FOSC/4/BAUD))>>8;
    AUXR |= 0x14;                //bit4启动（关闭）定时器2，bit3 T2用做计数器,
    AUXR |= 0x01;               //bit0定时器2（定时器1）为串口1的波特率发生器
    ES = 1;                     //使能串口1中断
    EA = 1;
}
void main()
{	
	delayus(1);
	delayms(500);
	  P0M0 = 0x00; //这些都是定义单片机针脚的四种功能，不同功能的功率不同，阻抗不同，做AD检测的阻抗越高越好
    P0M1 = 0x00; 
    P1M0 = 0x00;
    P1M1 = 0x00; 
    P2M0 = 0x00;
    P2M1 = 0x00;
    P3M0 = 0x00;
    P3M1 = 0x00;
    P4M0 = 0x00;
    P4M1 = 0x00;
	chuankou1();
	Send1('a');
	MS5837_IIC_Init();
	delayms(100);
	MS583703BA_RESET();	 // Reset Device  复位MS5837
	delayms(500);       //复位后延时（注意这个延时是一定必要的，可以缩短但似乎不能少于20ms）
	MS5837_init();	     //初始化MS5837
	
	
	while(1)
	{
	
		
	MS583703BA_getTemperature();//获取温度
	MS583703BA_getPressure();   //获取大气压
	  printf("	Temp : %5.3f\r\n",TEMP1);               //串口输出原始数据
		printf("	Pressure : %5.3f\r\n",Pressure); //串口输出原始数据
		printf("	Deep : %5.3f\r\n",DEEP);               //串口输出原始数据	
		printf("	     \r\n"); 
 /* Send1('a');
	Send1(a/100000000%10+48);
		Send1(a/10000000%10+48);
		Send1(a/1000000%10+48);
		Send1(a/100000%10+48);
		Send1(a/10000%10+48);
		Send1(a/1000%10+48);
		Send1(a/100%10+48);
		Send1(a/10%10+48);
		Send1(a%10+48);
		Send1('\n');*/
/*	Send1(temp[0]/100%10+48);
	Send1(temp[0]/10%10+48);	
	Send1(temp[0]%10+48);		
  Send1(temp[1]/100%10+48);
  Send1(temp[1]/10%10+48);		
	Send1(temp[1]%10+48);	
	Send1(temp[2]/100%10+48);
	Send1(temp[2]/10%10+48);
	Send1(temp[2]%10+48);
	Send1('/');*/
/*	for(i=0;i<8;i++)
{ 
	Send1(crc[i]/10000%10+48);
	Send1(crc[i]/1000%10+48);
  Send1(crc[i]/100%10+48);
	Send1(crc[i]/10%10+48);
	Send1(crc[i]%10+48);
}		
	Send1('\n');
		*/
	delayms(200);
	
	}		
}/*
void Uart() interrupt 4 using 1	 //STC15   串口1中断处理程序
{static u8 z;
    if (RI)			            //清除RI位
    {	RI = 0; 
	   a=SBUF;		  //获取串口SBUF数据
		if(SBUF==0x7f){z++;if(z>=10){z=0;IAP_CONTR |= 0x60;}}
     }
    if (TI)
    {
        TI = 0;                 //清除TI位
        busy = 0;               //清忙标志
    }
}
*/
















