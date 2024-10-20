#include "my_function.h"
#include "IIC.h"
#include "thread_manager.h"
#include "gpio.h"
#include "stdio.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MS5837.h"
#include "usart.h"
#include "my_adc.h"
#include "motion_control.h"




float pitch_1, roll_1, yaw_1;           //欧拉角
short aacx, aacy, aacz;         //加速度传感器原始数据
short gyrox, gyroy, gyroz;      //陀螺仪原始数据
float temp;                     //温度
extern uint16_t IIC_SCL_PIN;
extern uint16_t IIC_SDA_PIN;
JY901B_DataType jy901b;
void my_limit(float *p)
{
if(*p>2400)
*p=2400;
else if(*p<600)
*p=600;
}



uint8_t procese_buf[50];//数据处理区
uint8_t buf;
uint8_t sound_buf;

//**************************************线程1********************************/
void led1_thread_entry(void *parameter)
{
    while (1)
    {

        rt_thread_delay(500);   /* 延时500个tick */

//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        rt_thread_delay(500);   /* 延时500个tick */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
    }
}




//**************************************线程2********************************/
MPU6050DATATYPE mpu6050;


void mpu6050_thread_entry(void *parameter)
{

    rt_mq_t queue = rt_mq_create("my_queue", sizeof(float), 10, RT_IPC_FLAG_FIFO);
    if (queue == RT_NULL)
    {
        printf("Failed to create message queue!\n");
        rt_thread_delay(500);   /* 延时500个tick */

    }
    IIC_SCL_PIN =     SCL_1_Pin;
    IIC_SDA_PIN =     SDA_1_Pin;
		while(MPU_Init()+mpu_dmp_init()!=0)
//    uint8_t a = MPU_Init();         //MPU6050初始化
//    int b = mpu_dmp_init();     //dmp初始化
//printf("b=%d",b);
    rt_thread_delay(500);   /* 延时500个tick */

    while (1)
    {


//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        while (mpu_dmp_get_data(&roll_1, &pitch_1, &yaw_1)); //必须要用while等待，才能读取成功
//        mpu6050.st_data.pitch =  jy901b.byte[0];
//        mpu6050.st_data.roll = roll_1;
//        mpu6050.st_data.yaw = yaw_1;
				mpu6050.st_data.pitch =  ((jy901b.byte[1]<<8)|jy901b.byte[0])/32768*180;
        mpu6050.st_data.roll = ((jy901b.byte[1]<<8)|jy901b.byte[0])/32768*180;
        mpu6050.st_data.yaw = ((jy901b.byte[1]<<8)|jy901b.byte[0])/32768*180;
        mpu6050.st_data.Temp = temp / 100;
        MPU_Get_Accelerometer(&aacx, &aacy, &aacz);     //得到加速度传感器数据
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);      //得到陀螺仪数据
        temp = MPU_Get_Temperature();                   //得到温度信息
//        printf("mpu1：X:%.1f°  Y:%.1f°  Z:%.1f°  %.2f°C\r\n",roll_1,pitch_1,yaw_1,temp/100);//串口1输出采集信息
//   rt_thread_delay(50);   /* 延时500个tick */
//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
//   rt_thread_delay(50);   /* 延时500个tick */
        rt_thread_delay(50);   /* 延时500个tick */




    }
}

\

//**************************************线程3********************************/
void MS5837_thread_entry(void *parameter)
{
    while (1)
    {
//        IIC_SCL_PIN =     MS5837_SCL_Pin;
//        IIC_SDA_PIN =     MS5837_SDA_Pin;
        MS5837_30BA_ReSet();                                                    //复位MS5837
        rt_thread_mdelay(20);

        MS5837_30BA_PROM();                                                     //初始化MS5837
        rt_thread_mdelay(20);
        if (!MS5837_30BA_Crc4())                                              //CRC校验
        {
            printf("  初始化失败\r\n");
            printf("  请检查接线是否正确\r\n");

        }
        else
        {
            printf("  初始化成功\r\n");
            printf("  检测到MS5837_30BA\r\n\r\n");
            break;
        }

    }

    while (1)
    {
        MS5837_30BA_GetData();                                      //获取数据
        rt_thread_mdelay(200);
        printf("  Welcome to BlueRobots Community! \r\n");          //串口输出原始数据
        printf("  Temperature : %.2f C \r\n", Temperature);         //串口输出原始数据
        printf("  Pressure : %u mBar \r\n\r\n\r\n", Pressure);      //串口输出原始数据


    }
}


//**************************************线程4********************************/
uint8_t buf;
uint8_t len, head, tail, id;
extern UART_HandleTypeDef huart2;
ps2DATATYPE PS_2;

//uint8_t rx_buf[100]={0xAA,0X02,0X10,0X00,0X00,0X00,0X00,   0XCD,0XCC,0X8C,0X3F,    0XCD,0XCC,0X0C,0X40,  0X33,0X33,0X53,0X40, 0X7B};//串口接收缓存区
uint8_t rx_buf[100];
uint8_t data=0;
void data_receive_thread_entry(void *parameter)
{
//HAL_UART_Receive_IT(&huart1, rx_buf, 140);


    while (1)
    {
			#if old
			HAL_UART_Receive_IT(&huart1, rx_buf, 140);
			#else
						HAL_UART_Receive_IT(&huart1, &data, 1);
			#endif
//rt_interrupt_enter();

//rt_interrupt_leave();


        rt_thread_mdelay(20);

    }
}


//**************************************线程5********************************/
void SendData_thread_entry(void *parameter)
{
    while (1)
    {

        Send_Data_Task();
        rt_thread_mdelay(5);

    }
}

//**************************************线程6：超声波********************************/
uint8_t enable = 1;

uint8_t high_data, low_data;
uint8_t sound_rx_buf[8];
uint8_t sound_sum = 0;
int distance;
soundDATATYPE Ultrasound;

unsigned char uart_buff[4];
unsigned char uart_temp[1];unsigned int uart_rx_cnt = 0;

void Ultrasound_thread_entry(void *parameter)
{
    while (1)
    {
     HAL_UART_Receive_IT(&huart5,(uint8_t *)uart_temp, 1);

        HAL_UART_Receive_IT(&huart4, &sound_buf, 1);

//        HAL_UART_Transmit(&huart4, &enable, sizeof(enable), 0xff);
    HAL_UART_Transmit(&huart4, (uint8_t *)&enable, 1, 0xFFFF);


        for (int i = 0; i < 8; i++)
        {
            if (sound_rx_buf[i] == 0xff)
            {
//printf("%d %d %d %d\t\t",sound_rx_buf[i],sound_rx_buf[i+1],sound_rx_buf[i+2],sound_rx_buf[i+3]);

                high_data = sound_rx_buf[i + 1];
                low_data = sound_rx_buf[i + 2];

                sound_sum = 0xff + high_data + low_data;
                if (sound_sum == sound_rx_buf[i + 3])
                {
                    distance = high_data * 256 + low_data;
                    Ultrasound.st_data.distance = (float)distance;

//                    printf("distance=%d\n",distance);
                    break;
                }
                else
                {
                    printf("error");

                }

            }


        }


        rt_thread_mdelay(50);

    }
}
//**************************************线程7:PH计********************************/

extern ADC_HandleTypeDef hadc1;
extern float volot[4];
phDATATYPE ph;

extern float v;
extern int adc_num;

uint8_t i;
float adcBuf[2];//存放ADC

void PH_thread_entry(void *parameter)
{
    while (1)
    {


        i = 0;
        while (i < 2)
        {
            HAL_ADC_Start(&hadc1);//启动ADC
            HAL_ADC_PollForConversion(&hadc1, 10); //表示等待转换完成，第二个参数表示超时时间，单位ms.
            //HAL_ADC_GetState(&hadc1)为换取ADC状态，HAL_ADC_STATE_REG_EOC表示转换完成标志位，转换数据可用。
            if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) //就是判断转换完成标志位是否设置,HAL_ADC_STATE_REG_EOC表示转换完成标志位，转换数据可用
            {
                //读取ADC转换数据，数据为12位。查看数据手册可知，寄存器为16位存储转换数据，数据右对齐，则转换的数据范围为0~2^12-1,即0~4095.
                adcBuf[i] = HAL_ADC_GetValue(&hadc1) * 3.3 / 4096;
//   printf("\nadc%d=%4.0d,电压=%1.4f",i,adcBuf[i],adcBuf[i]*3.3f/65536);
                i++;
            }
        }

        HAL_ADC_Stop(&hadc1);

//        ph.st_data.ph = adcBuf[1]*-5.7541+16.654;
				ph.st_data.ph = 6.5-adcBuf[1];
        ph.st_data.quality = adcBuf[0];
				ph.st_data.quality = ph.st_data.quality*ph.st_data.quality*ph.st_data.quality*66.71-127.93*ph.st_data.quality*ph.st_data.quality+428.7*ph.st_data.quality;

//				ph.st_data.quality=jj++;
				
				//printf("v1=%f,v2=%f\n",adcBuf[0],adcBuf[1]);


        rt_thread_mdelay(50);


    }
}



//**************************************线程8：水质传感器********************************/

void water_quality_thread_entry(void *parameter)
{
    while (1)
    {


        rt_thread_mdelay(50);

    }
}


//**************************************线程9：运动控制********************************/
uint8_t lock=1;
float jiaodu[10000];
int ii=0;

			float angle_1,angle_2,angle_3,angle_4;
void motion_control_thread_entry(void *parameter)
{
				
	
	
    while (1)
    {
			


# if old
	
			control_motion(2-PS_2.st_data.ch4,PS_2.st_data.ch3,2-PS_2.st_data.ch2);//考试前一直用的
# else
			control_motion(1.01-PS_2.st_data.ch4,PS_2.st_data.ch3-1,1.01-PS_2.st_data.ch2,PS_2.st_data.ch5/2);//考试后改进的算法,第四个参数是速度控制

			angle_1=1500-PS_2.st_data.yaw_2*1000/90;
			angle_2=2500-PS_2.st_data.roll_1*1000/90;
			angle_3=1500-(PS_2.st_data.roll_2-PS_2.st_data.roll_1)*1000/90;
			angle_4=1500+(PS_2.st_data.yaw_3-PS_2.st_data.yaw_2)*1000/90;
			my_limit(&angle_1);
			my_limit(&angle_2);
			my_limit(&angle_3);
			my_limit(&angle_4);


			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, angle_1);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, angle_2);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, angle_3);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, angle_4);
					
//					jiaodu[ii]=angle_1;ii++;
//					if(ii>10000)
//				ii=10000;
//control_motion(   0.5  ,    0.5  ,     0.5 ,1  );
//control_motion(   0.5  ,    -0.5  ,     0.5 ,1  );

//control_motion(   -0.5   ,   0.5  ,     0.5 ,1   );
//control_motion(   -0.5   ,   -0.5  ,     0.5 ,1   );

//control_motion(   -0.5   ,   0.1  ,     -0.5   ,1 );
//control_motion(   0.5   ,   0.1  ,     -0.5   ,1 );


			if((lock==0)&&(PS_2.st_data.ch7>0.8))
			{
			
				lock=1;
				
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

				
				
			
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1492);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1491);
	
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1493);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1494);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1495);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1496);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1497);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1498);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1499);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1500);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1501);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1502);
		
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1504);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1503);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1502);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1501);			
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1506);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1508);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1510);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1512);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1514);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1516);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1518);			
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1520);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1524);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1526);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1528);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1530);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,1532);
	
		
		
		
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1500);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1499);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1498);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1497);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1496);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1495);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1494);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1493);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1492);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1491);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1490);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1489);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1488);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,1487);
		
		
		
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1500);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1499);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1498);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1497);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1496);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1495);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1494);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1493);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1492);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1491);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1490);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1489);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1488);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,1487);
		
		
		
HAL_Delay(3000);			
				
				


					
			}
			
			if((lock==1)&&(PS_2.st_data.ch8>0.8))
			{
			
				lock=0;
				
							
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);			
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);		
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);							
					
			}			


				        rt_thread_mdelay(10);

# endif
    }
}
//***************************************回调函数区***********************************/
uint8_t rx_num = 0;
uint8_t sound_i = 0;
uint8_t buff=0;
uint8_t cnt=0,buf_sum=0;
    uint8_t sum = 0;
MS5837_DATATYPE ms5837;

//extern unsigned int distance;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART1) // ???????????? USART1
    {
			
			
			
			#if old
//        HAL_UART_Receive_IT(&huart1, &buf, 1);

			        for (int i = 0; i < sizeof(rx_buf) / 2; i++) //遍历接受数组找出一帧
        {

            if ((rx_buf[i] == 0xAA)&&(rx_buf[i+1] == 0x00))//看见帧头
            {

                head = rx_buf[i];//aa
                id = rx_buf[i + 1];//00
                len = rx_buf[i + 2];//44
                sum = head + id + len;
//                if (id != 0x01)
//                {
//                    break;
//                    printf("id err");
//                }


                for (int j = 0; j < len ; j++)//此循环用于算出一帧内的校验和
                {
                    if ((j % 4) == 3)
                    {
                        sum += rx_buf[i + 3 + j];
                    }
                }
                if (sum == rx_buf[i + 3 + len])//判断是否通过校验
                {

                    switch (id)//校验通过，根据id处理数据
                    case 0x00: ////***************以后要在这个地方通过不同的id把数据写到不同的共用体数组里面，在function.h里面定义好datatype，然后更改下面的东西*********************/
                    for (int k = 0; k < len; k++)
                    {
                        PS_2.byte[k] = rx_buf[i + 3 + k];
                    }







                    printf("ch1=%f,ch2=%f,ch3=%f,ch4=%f\n", PS_2.st_data.ch1, PS_2.st_data.ch2, PS_2.st_data.ch3, PS_2.st_data.ch4);
                    break;

                }
                else
                {
                    printf("4_error");
									break;
                }

            }

        }
				HAL_UART_Receive_IT(&huart1, rx_buf, 140);	
				#else
				
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;u8 check=1;
	static u8 DT_RxBuffer[256];
HAL_UART_Receive_IT(&huart1, &data, 1);
	//判断帧头是否满足匿名协议的0xAA
	if (rxstate == 0 && data == 0xAA)
	{
		rxstate = 1;
		head = data;
		sum+=data;
	}
	//判断id
	else if (rxstate == 1 && (data==0x00))
	{
		rxstate = 3;
		 id = data;
	  sum+=data;
	}

//懒得改源码了，直接到3
	//接收数据长度字节
	else if (rxstate == 3 && data < 250)//len
	{
		rxstate = 4;
		len = data;
		_data_len = data;
		_data_cnt = 0;
		sum+=data;
	}
	//接收数据区
	else if (rxstate == 4 && _data_len > 0)//float*8
	{
		_data_len--;
		DT_RxBuffer[ _data_cnt++] = data;
	
		if(_data_cnt%4==0)
		{sum+=data;}
		
		if (_data_len == 0)
			rxstate = 5;
		
	}
	//接收sum
	else if (rxstate == 5)
	{
		rxstate = 6;
		check = data;
	}
	//判断校验是否通过
	else if (rxstate == 6)
	{
		rxstate = 0;
		if(sum==sum)
		{ 
			for (int k = 0; k < len; k++)
                    {
                        PS_2.byte[k] = DT_RxBuffer[ k ];
                    }
		}
		else
		{
			
		printf("error");
		}
		sum=0;

	}
	else
	{
		rxstate = 0;
	}
				#endif
			
		
	 }		
//			//////////////////

   
//    if (huart->Instance == UART5) // ???????????? Uart5   ms5837
//    {


//	static u8 _data_len = 0, _cnt = 0;
//	static u8 rx_state = 0;u8 check=1;
//	static u8 DT_RxBuffer[256];


//	if (rx_state == 0 && data == 0xAB)
//	{
//		rx_state = 1;
//		head = data;

//	}
//	//判断id
//	else if (rx_state == 1 && (data==0x01))
//	{
//		rx_state = 2;
//		 id = data;

//	}
//	else if (rx_state == 2 )
//	{
//		
//		 ms5837.byte[_cnt++]= data;
//if(_cnt>=8)
//{
//_cnt=0;
//rx_state=0;
//}
//	}
//	else
//	{
//	rx_state=0;
//	}

//HAL_UART_Receive_IT(&huart5, &data, 1);

//    }
		
		
		
		
		
////////////////////////////////////////////////////
    if (huart->Instance == UART4) // ???????????? USART1
    {

        sound_rx_buf[sound_i++] = sound_buf;
        if (sound_i > 8)
        {
            sound_i = 0;
        }
        HAL_UART_Receive_IT(&huart4, &sound_buf, 1);
    }
		
		if (huart->Instance == UART5)//jy901b数据的接收
    {
        static u8 _data_len = 0, _cnt = 0;
        static u8 rx_state = 0;

        if (rx_state == 0 && data == 0x55)//帧头是0x55
        {
            rx_state = 1;
            head = data;
        }
        else if (rx_state == 1 && (data == 0x53))
        {
            rx_state = 2;
            id = data;
        }
        else if (rx_state == 2)
        {

            jy901b.byte[_cnt++] = data;
            if (_cnt >= 6)
            {
                _cnt = 0;
                rx_state = 3;
            }
        }
        else
        {
            rx_state = 0;
        }

        HAL_UART_Receive_IT(&huart5, &data, 1);
    }

}
	

