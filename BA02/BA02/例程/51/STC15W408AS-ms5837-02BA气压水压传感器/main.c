#include "intrins.h"
#include "main.h"
#include "stdio.h"
#define FOSC 11059200L          //ϵͳƵ��
#define BAUD 115200             //����1  ����2 ��ͬ������
u32 a;
extern		u32 temp[3];
extern u8 crc[16];
extern float TEMP1;
extern float DEEP;
extern float xdata Pressure;
u8 i;
void delayus(u8 i)   //@stc15-11.0592MHz��ʱ��ʵ�⣬������ֵ��ʱ�����1������50�������0
{
   while(i)
	{  _nop_();
		 _nop_(); _nop_();
	    i--;
	}
}
void delayms(u16 i)   //@stc15-11.0592MHz��ʱ��ʵ�⣬���ǧ��1
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
void Send1(u8 dat)						//STC15  ����1�ķ��ͺ�����
{
    while (busy);               //�ȴ�ǰ������ݷ������
    ACC = dat;                  //��ȡУ��λP (PSW.0)   
    busy = 1;                    //
	SBUF = ACC;                 //д���ݵ�UART���ݼĴ�����SBUF�����Զ����ʹ������ݣ�������ɺ�busy�����=0
}
void chuankou1()//�봮��2����T2��ʱ������ͬ������
{   P_SW1|=0x00;				//|=0x00[3.0/Rxd,3.1/Txd];|=0x40[3.6/Rxd,3.7/Txd];|=0x80[1.6/Rxd,1.7/Txd]
	SCON= 0x50; 
    T2L = (65536 - (FOSC/4/BAUD));   //���ò�������װֵ
    T2H = (65536 - (FOSC/4/BAUD))>>8;
    AUXR |= 0x14;                //bit4�������رգ���ʱ��2��bit3 T2����������,
    AUXR |= 0x01;               //bit0��ʱ��2����ʱ��1��Ϊ����1�Ĳ����ʷ�����
    ES = 1;                     //ʹ�ܴ���1�ж�
    EA = 1;
}
void main()
{	
	delayus(1);
	delayms(500);
	  P0M0 = 0x00; //��Щ���Ƕ��嵥Ƭ����ŵ����ֹ��ܣ���ͬ���ܵĹ��ʲ�ͬ���迹��ͬ����AD�����迹Խ��Խ��
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
	MS583703BA_RESET();	 // Reset Device  ��λMS5837
	delayms(500);       //��λ����ʱ��ע�������ʱ��һ����Ҫ�ģ��������̵��ƺ���������20ms��
	MS5837_init();	     //��ʼ��MS5837
	
	
	while(1)
	{
	
		
	MS583703BA_getTemperature();//��ȡ�¶�
	MS583703BA_getPressure();   //��ȡ����ѹ
	  printf("	Temp : %5.3f\r\n",TEMP1);               //�������ԭʼ����
		printf("	Pressure : %5.3f\r\n",Pressure); //�������ԭʼ����
		printf("	Deep : %5.3f\r\n",DEEP);               //�������ԭʼ����	
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
void Uart() interrupt 4 using 1	 //STC15   ����1�жϴ������
{static u8 z;
    if (RI)			            //���RIλ
    {	RI = 0; 
	   a=SBUF;		  //��ȡ����SBUF����
		if(SBUF==0x7f){z++;if(z>=10){z=0;IAP_CONTR |= 0x60;}}
     }
    if (TI)
    {
        TI = 0;                 //���TIλ
        busy = 0;               //��æ��־
    }
}
*/
















