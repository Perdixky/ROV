/////////////////////////////////////////////////////////////////////////////////////////		 
//B30 ��ȴ�������������
//�����壺BlueTest STM32
//������̳: www.Bluerobots.cn ��BlueRobots ˮ�»�����������
//�޸�����: 2019/4/30
//���̰汾��V1.2
//��ϵ���䣺info@bluerobots.cn
//�ر���������������Դ�����磬��BlueRobots ���������޸ĺ����ڽ�������ʹ�������ге�һ�к����
/////////////////////////////////////////////////////////////////////////////////////////	

#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "myiic.h"
#include "MS5837.h"

MS5837_DATATYPE MS5837;
float pressure,temp;
int k;

 int main(void)
 { 

	SystemInit(); 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                          // �����ж����ȼ�����2
	delay_init();	    	                                                     //��ʱ������ʼ��	  
	uart_init(115200);	                                                     //���ڳ�ʼ��
	delay_ms(100);
	IIC_Init();	                                                             //��ʼ��BlueTest STM32������ IIC��(ģ��IIC)
	delay_ms(100);

	 while(1)
	 {
	 MS5837_30BA_ReSet();	                                                   //��λMS5837
   delay_ms(20); 
	 MS5837_30BA_PROM();                                                     //��ʼ��MS5837
	 delay_ms(20);
			if(!MS5837_30BA_Crc4())                                               //CRCУ��
					{
					  printf("  ��ʼ��ʧ��\r\n"); 
						printf("  ��������Ƿ���ȷ\r\n");
					break;
					}
			else 
			{
			      printf("  ��ʼ���ɹ�\r\n"); 
				    printf("  ��⵽MS5837_30BA\r\n\r\n"); 
				    break;
			}
	}			
		  while(1)
		       {	
	             MS5837_30BA_GetData();                                      //��ȡ���� 
		           delay_ms(20);
//		           printf("  Welcome to BlueRobots Community! \r\n");          //�������ԭʼ����
//		           printf("  Temperature : %.2f C \r\n",Temperature);          //�������ԭʼ����
						 
						 MS5837.st_data.pressure=(float)Pressure;
						 MS5837.st_data.water_temp=(float)Temperature;
						 Usart_SendByte(USART3,0xab);
						 Usart_SendByte(USART3,0x01);
//						 
						for( k=0;k<8;k++)
						 {
//							 Usart_SendByte(USART3,k);
						 Usart_SendByte(USART3,MS5837.byte[k]);
						 }
						 
						 
						 
//						 printf("  Temperature : %f C \r\n",temp); 
//		           printf("  Pressure : %f mBar \r\n\r\n\r\n",pressure);       //�������ԭʼ����
						
					 }
		
        
}
// BlueRobots Lab	

	
