#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
/////////////////////////////////////////////////////////////////////////////////////////		 
//B30 ��ȴ�������������
//�����壺BlueTest STM32
//������̳: www.Bluerobots.cn ��BlueRobots ˮ�»�����������
//�޸�����: 2019/4/30
//���̰汾��V1.2
//��ϵ���䣺info@bluerobots.cn
//�ر���������������Դ�����磬��BlueRobots ���������޸ĺ����ڽ�������ʹ�������ге�һ�к����
/////////////////////////////////////////////////////////////////////////////////////////	
   	   		   
////IO��������, 
//#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=8<<28;} // PC11 = SDA ��Ϊ����ʱ 
//#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=3<<28;} // PC11 = SDA ��Ϊ���ʱ

////IO��������	 
//#define IIC_SCL    PBout(6) //SCL = PC10
//#define IIC_SDA    PBout(7) //SDA = PC11 ��Ϊ���ʱ
//#define READ_SDA   PBin(7)  //SDA = PC11 ��Ϊ����ʱ
 



//IO��������, 
#define SDA_IN()  {GPIOA->CRH&=0XFFFF0FFF;GPIOA->CRH|=8<<12;} // PC11 = SDA ��Ϊ����ʱ 
#define SDA_OUT() {GPIOA->CRH&=0XFFFF0FFF;GPIOA->CRH|=3<<12;} // PC11 = SDA ��Ϊ���ʱ

//IO��������	 
#define IIC_SCL    PAout(12) //SCL = PC10
#define IIC_SDA    PAout(11) //SDA = PC11 ��Ϊ���ʱ
#define READ_SDA   PAin(11)  //SDA = PC11 ��Ϊ����ʱ




//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);								//����IIC��ʼ�ź�
void IIC_Stop(void);	  						//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);					//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 							//IIC�ȴ�ACK�ź�
void IIC_Ack(void);									//IIC����ACK�ź�
void IIC_NAck(void);								//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  

#endif

// BlueRobots Lab














