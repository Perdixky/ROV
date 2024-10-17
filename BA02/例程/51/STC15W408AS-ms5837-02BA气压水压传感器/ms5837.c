#include "MS5837.h"
#define ms5837_IIC_SCL P15
#define ms5837_IIC_SDA P14
extern u8  a,b,c,k,i; 
/*
C1 ѹ�������� SENS|T1
C2  ѹ������  OFF|T1
C3	�¶�ѹ��������ϵ�� TCS
C4	�¶�ϵ����ѹ������ TCO
C5	�ο��¶� T|REF
C6 	�¶�ϵ�����¶� TEMPSENS
*/
u32  Cal_C[7];	        //���ڴ��PROM�е�6������1-6
u32 xdata D1,D2;	// ����ѹ��ֵ,�����¶�ֵ
float dT,TEMP;
float Aux;
float OFF_;
/*
dT ʵ�ʺͲο��¶�֮��Ĳ���
TEMP ʵ���¶�	
*/
float dT,TEMP,TEMP1;
float xdata OFF,SENS;
/*
OFF ʵ���¶Ȳ���
SENS ʵ���¶�������
*/
float p;
float DEEP;
float xdata Pressure;				//����ѹ
float xdata  T2,OFF2,SENS2;	//�¶�У��ֵ

float depth();



//��ʼ��IIC
void ms5837_IIC_Init(void)
{					     
	ms5837_IIC_SCL=1;
	ms5837_IIC_SDA=1;
}
//����IIC��ʼ�ź�
void ms5837_IIC_Start(void)
{
	
	ms5837_IIC_SDA=1;	  	  
	ms5837_IIC_SCL=1;
	delayus(4);
 	ms5837_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delayus(4);
	ms5837_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void ms5837_IIC_Stop(void)
{

	ms5837_IIC_SCL=0;
	ms5837_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delayus(4);
	ms5837_IIC_SCL=1; 
	ms5837_IIC_SDA=1;//����I2C���߽����ź�
	delayus(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
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
	ms5837_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
 
void ms5837_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  
    ms5837_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        ms5837_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delayus(2);   //��TEA5767��������ʱ���Ǳ����
		ms5837_IIC_SCL=1;
		delayus(2); 
		ms5837_IIC_SCL=0;	
		delayus(2);
    }		
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
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
 ms5837_IIC_SCL=0;}//����nACK
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
		ms5837_IIC_Send_Byte(0xEC);//CSB�ӵأ�������ַ��0XEE������ 0X77
	  ms5837_IIC_Wait_Ack();
    ms5837_IIC_Send_Byte(0x1E);//���͸�λ����
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
		ms5837_IIC_Send_Byte(0xEC+0x01);  //�������ģʽ
		delayus(1);
		ms5837_IIC_Wait_Ack();
		inth = ms5837_IIC_Read_Byte(1);  		//��ACK�Ķ�����
		intl = ms5837_IIC_Read_Byte(0); 			//���һ���ֽ�NACK
		ms5837_IIC_Stop();
    Cal_C[i] = (((u16)inth << 8) | intl);
	}
	 
 }


/**************************ʵ�ֺ���********************************************
*����ԭ��:unsigned long MS561101BA_getConversion(void)
*��������:    ��ȡ MS5837 ��ת����� 
*******************************************************************************/
u32  MS583703BA_getConversion(u8 command)
{
			unsigned long conversion = 0;
	u32 temp[3];
	
	    ms5837_IIC_Start();
			ms5837_IIC_Send_Byte(0xEC); 		//д��ַ
			ms5837_IIC_Wait_Ack();
			ms5837_IIC_Send_Byte(command); //дת������
			ms5837_IIC_Wait_Ack();
			ms5837_IIC_Stop();

			delayms(20);
			ms5837_IIC_Start();
			ms5837_IIC_Send_Byte(0xEC); 		//д��ַ
			ms5837_IIC_Wait_Ack();
			ms5837_IIC_Send_Byte(0);				// start read sequence
			ms5837_IIC_Wait_Ack();
			ms5837_IIC_Stop();
		 
			ms5837_IIC_Start();
			ms5837_IIC_Send_Byte(0xEC+0x01);  //�������ģʽ
			ms5837_IIC_Wait_Ack();
			temp[0] = ms5837_IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 23-16
			temp[1] = ms5837_IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 8-15
			temp[2] = ms5837_IIC_Read_Byte(0);  //��NACK�Ķ����� bit 0-7
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