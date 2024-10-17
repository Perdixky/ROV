#include "MS5837.h"
#define ms5837_IIC_SCL P15
#define ms5837_IIC_SDA P14
extern u8  a,b,c,k,i; 
/*
C1 压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3	温度压力灵敏度系数 TCS
C4	温度系数的压力补偿 TCO
C5	参考温度 T|REF
C6 	温度系数的温度 TEMPSENS
*/
u32  Cal_C[7];	        //用于存放PROM中的6组数据1-6
u32 xdata D1,D2;	// 数字压力值,数字温度值
float dT,TEMP;
float Aux;
float OFF_;
/*
dT 实际和参考温度之间的差异
TEMP 实际温度	
*/
float dT,TEMP,TEMP1;
float xdata OFF,SENS;
/*
OFF 实际温度补偿
SENS 实际温度灵敏度
*/
float p;
float DEEP;
float xdata Pressure;				//大气压
float xdata  T2,OFF2,SENS2;	//温度校验值

float depth();



//初始化IIC
void ms5837_IIC_Init(void)
{					     
	ms5837_IIC_SCL=1;
	ms5837_IIC_SDA=1;
}
//产生IIC起始信号
void ms5837_IIC_Start(void)
{
	
	ms5837_IIC_SDA=1;	  	  
	ms5837_IIC_SCL=1;
	delayus(4);
 	ms5837_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delayus(4);
	ms5837_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void ms5837_IIC_Stop(void)
{

	ms5837_IIC_SCL=0;
	ms5837_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delayus(4);
	ms5837_IIC_SCL=1; 
	ms5837_IIC_SDA=1;//发送I2C总线结束信号
	delayus(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 ms5837_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	
	ms5837_IIC_SDA=1;
	delayus(4);	   
	ms5837_IIC_SCL=1;
	delayus(1);	 
	while(ms5837_IIC_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			ms5837_IIC_Stop();
			return 1;
		}
	}
	ms5837_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
 
void ms5837_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  
    ms5837_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        ms5837_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delayus(2);   //对TEA5767这三个延时都是必须的
		ms5837_IIC_SCL=1;
		delayus(2); 
		ms5837_IIC_SCL=0;	
		delayus(2);
    }		
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 ms5837_IIC_Read_Byte(u8 ack)
{
	unsigned char i,receive=0;
	ms5837_IIC_SDA=1;
    for(i=0;i<8;i++ )
	{
        delayus(2);
		    ms5837_IIC_SCL=1;
        receive<<=1;
        if(ms5837_IIC_SDA)receive++;   
		delayus(1); 
		ms5837_IIC_SCL=0; 
    }	
	
if (ack==0)
{	ms5837_IIC_SCL=0;
	ms5837_IIC_SDA=1;
	delayus(2);
	
	ms5837_IIC_SCL=1;
	delayus(2);
 ms5837_IIC_SCL=0;}//发送nACK
else{		
  ms5837_IIC_SCL=0;
	ms5837_IIC_SDA=0;
	delayus(2);
	
	ms5837_IIC_SCL=1;
	delayus(2);
	ms5837_IIC_SCL=0;
		}         

    return receive;
}

void MS583703BA_RESET(void)
{
		ms5837_IIC_Start();
		ms5837_IIC_Send_Byte(0xEC);//CSB接地，主机地址：0XEE，否则 0X77
	  ms5837_IIC_Wait_Ack();
    ms5837_IIC_Send_Byte(0x1E);//发送复位命令
	  ms5837_IIC_Wait_Ack();
    ms5837_IIC_Stop();
	
}

void MS5837_init(void)
 {	 
  u16  inth,intl;
  for (i=0;i<=6;i++) 
	{ 
		ms5837_IIC_Start();
    ms5837_IIC_Send_Byte(0xEC);
		ms5837_IIC_Wait_Ack();
		ms5837_IIC_Send_Byte(0xA0 + (i*2));
		ms5837_IIC_Wait_Ack();
    ms5837_IIC_Stop();		
		delayus(5);
		ms5837_IIC_Start();
		ms5837_IIC_Send_Byte(0xEC+0x01);  //进入接收模式
		delayus(1);
		ms5837_IIC_Wait_Ack();
		inth = ms5837_IIC_Read_Byte(1);  		//带ACK的读数据
		intl = ms5837_IIC_Read_Byte(0); 			//最后一个字节NACK
		ms5837_IIC_Stop();
    Cal_C[i] = (((u16)inth << 8) | intl);
	}
	 
 }


/**************************实现函数********************************************
*函数原型:unsigned long MS561101BA_getConversion(void)
*功　　能:    读取 MS5837 的转换结果 
*******************************************************************************/
u32  MS583703BA_getConversion(u8 command)
{
			unsigned long conversion = 0;
	u32 temp[3];
	
	    ms5837_IIC_Start();
			ms5837_IIC_Send_Byte(0xEC); 		//写地址
			ms5837_IIC_Wait_Ack();
			ms5837_IIC_Send_Byte(command); //写转换命令
			ms5837_IIC_Wait_Ack();
			ms5837_IIC_Stop();

			delayms(20);
			ms5837_IIC_Start();
			ms5837_IIC_Send_Byte(0xEC); 		//写地址
			ms5837_IIC_Wait_Ack();
			ms5837_IIC_Send_Byte(0);				// start read sequence
			ms5837_IIC_Wait_Ack();
			ms5837_IIC_Stop();
		 
			ms5837_IIC_Start();
			ms5837_IIC_Send_Byte(0xEC+0x01);  //进入接收模式
			ms5837_IIC_Wait_Ack();
			temp[0] = ms5837_IIC_Read_Byte(1);  //带ACK的读数据  bit 23-16
			temp[1] = ms5837_IIC_Read_Byte(1);  //带ACK的读数据  bit 8-15
			temp[2] = ms5837_IIC_Read_Byte(0);  //带NACK的读数据 bit 0-7
			ms5837_IIC_Stop();
			conversion = temp[0] * 65536 + temp[1] * 256 + temp[2];
		
return  conversion;
}
void MS583703BA_getPressure(void)
{
	D1= MS583703BA_getConversion(0x48);
	delayms(10);	
//	OFF=Cal_C[2]*65536.0+Cal_C[4]*(dT/128.0);	
//	SENS=Cal_C[1]*32678.0+Cal_C[3]*(dT/256.0);
	OFF=Cal_C[2]*131072.0+Cal_C[4]*(dT/64.0);	
	SENS=Cal_C[1]*65536.0+Cal_C[3]*(dT/128.0);
	P=((D1*SENS/2097152.0)*OFF)/32768.0;
	
	if(TEMP<2000)  // low temp
	{
		Aux = (2000-TEMP)*(2000-TEMP);
		T2 = (dT/3333.0)*(dT/214748.3648); 
		
		OFF2 = 1.5*(2000-TEMP)*(2000-TEMP);
		SENS2 = 5*((2000-TEMP)*(2000-TEMP))/8;
		OFF_ = OFF_ - OFF2;
		SENS = SENS - SENS2;			
	}
else {	 
	T2=(dT/100000)*(dT/31236.12579);	
		OFF2 = 1*Aux/16;
		SENS2 = 0;
		OFF_ = OFF_ - OFF2;
		SENS = SENS - SENS2;		 
	   }
  Pressure= (D1*SENS/2097152.0-OFF_)/81920;
	 TEMP=(TEMP-T2)/100;
	 DEEP=Pressure/9794.4;
}
void MS583703BA_getTemperature(void)
{	
	D2 = MS583703BA_getConversion(0x58);
	delayms(10);
	dT=D2 - (Cal_C[5]*256);
	TEMP=2000.0+dT*(Cal_C[6]/8388608.0);
	TEMP1 = TEMP/100.0f;
}