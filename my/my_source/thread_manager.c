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



float pitch_1, roll_1, yaw_1;           //?????
short aacx, aacy, aacz;         //????????????????
short gyrox, gyroy, gyroz;      //????????????
float temp;                     //???
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



uint8_t procese_buf[50];//?????????
uint8_t buf;
uint8_t sound_buf;

//**************************************???1********************************/
void led1_thread_entry(void *parameter)
{
    while (1)
    {

        rt_thread_delay(500);   /* ???500??tick */

//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        rt_thread_delay(500);   /* ???500??tick */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
    }
}




//**************************************???2********************************/
MPU6050DATATYPE mpu6050;


void mpu6050_thread_entry(void *parameter)
{

    rt_mq_t queue = rt_mq_create("my_queue", sizeof(float), 10, RT_IPC_FLAG_FIFO);
    if (queue == RT_NULL)
    {
        printf("Failed to create message queue!\n");
        rt_thread_delay(500);   /* ???500??tick */

    }
    IIC_SCL_PIN =     SCL_1_Pin;
    IIC_SDA_PIN =     SDA_1_Pin;
		while(MPU_Init()+mpu_dmp_init()!=0)
//    uint8_t a = MPU_Init();         //MPU6050?????
//    int b = mpu_dmp_init();     //dmp?????
//printf("b=%d",b);
    rt_thread_delay(500);   /* ???500??tick */

    while (1)
    {


//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        while (mpu_dmp_get_data(&roll_1, &pitch_1, &yaw_1)); //???????while??????????????
//        mpu6050.st_data.pitch =  jy901b.byte[0];
//        mpu6050.st_data.roll = roll_1;
//        mpu6050.st_data.yaw = yaw_1;
				mpu6050.st_data.pitch =  ((jy901b.byte[1]<<8)|jy901b.byte[0])/32768*180;
        mpu6050.st_data.roll = ((jy901b.byte[1]<<8)|jy901b.byte[0])/32768*180;
        mpu6050.st_data.yaw = ((jy901b.byte[1]<<8)|jy901b.byte[0])/32768*180;
        mpu6050.st_data.Temp = temp / 100;
        MPU_Get_Accelerometer(&aacx, &aacy, &aacz);     //?????????????????
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);      //?????????????
        temp = MPU_Get_Temperature();                   //?????????
//        printf("mpu1??X:%.1f??  Y:%.1f??  Z:%.1f??  %.2f??C\r\n",roll_1,pitch_1,yaw_1,temp/100);//????1?????????
//   rt_thread_delay(50);   /* ???500??tick */
//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
//   rt_thread_delay(50);   /* ???500??tick */
        rt_thread_delay(50);   /* ???500??tick */




    }
}

\

//**************************************???3********************************/
void MS5837_thread_entry(void *parameter)
{
    while (1)
    {
//        IIC_SCL_PIN =     MS5837_SCL_Pin;
//        IIC_SDA_PIN =     MS5837_SDA_Pin;
        MS5837_30BA_ReSet();                                                    //??¦ËMS5837
        rt_thread_mdelay(20);

        MS5837_30BA_PROM();                                                     //?????MS5837
        rt_thread_mdelay(20);
        if (!MS5837_30BA_Crc4())                                              //CRC§µ??
        {
            printf("  ????????\r\n");
            printf("  ?????????????\r\n");

        }
        else
        {
            printf("  ????????\r\n");
            printf("  ???MS5837_30BA\r\n\r\n");
            break;
        }

    }

    while (1)
    {
        MS5837_30BA_GetData();                                      //???????
        rt_thread_mdelay(200);
        printf("  Welcome to BlueRobots Community! \r\n");          //?????????????
        printf("  Temperature : %.2f C \r\n", Temperature);         //?????????????
        printf("  Pressure : %u mBar \r\n\r\n\r\n", Pressure);      //?????????????


    }
}


//**************************************???4********************************/
uint8_t buf;
uint8_t len, head, tail, id;
extern UART_HandleTypeDef huart2;
ps2DATATYPE PS_2;

//uint8_t rx_buf[100]={0xAA,0X02,0X10,0X00,0X00,0X00,0X00,   0XCD,0XCC,0X8C,0X3F,    0XCD,0XCC,0X0C,0X40,  0X33,0X33,0X53,0X40, 0X7B};//????????????
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


//**************************************???5********************************/
void SendData_thread_entry(void *parameter)
{
    while (1)
    {

        Send_Data_Task();
        rt_thread_mdelay(5);

    }
}

//**************************************???6????????********************************/
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
//**************************************???7:PH??********************************/

extern ADC_HandleTypeDef hadc1;
extern float volot[4];
phDATATYPE ph;

extern float v;
extern int adc_num;

uint8_t i;
float adcBuf[2];//???ADC

void PH_thread_entry(void *parameter)
{
    while (1)
    {


        i = 0;
        while (i < 2)
        {
            HAL_ADC_Start(&hadc1);//????ADC
            HAL_ADC_PollForConversion(&hadc1, 10); //?????????????????????????????????¦Ëms.
            //HAL_ADC_GetState(&hadc1)????ADC????HAL_ADC_STATE_REG_EOC???????????¦Ë???????????¨¢?
            if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) //?????§Ø?????????¦Ë???????,HAL_ADC_STATE_REG_EOC???????????¦Ë????????????
            {
                //???ADC?????????????12¦Ë?????????????????????16¦Ë?›¥?????????????????????????????¦¶?0~2^12-1,??0~4095.
                adcBuf[i] = HAL_ADC_GetValue(&hadc1) * 3.3 / 4096;
//   printf("\nadc%d=%4.0d,???=%1.4f",i,adcBuf[i],adcBuf[i]*3.3f/65536);
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



//**************************************???8??????????********************************/

void water_quality_thread_entry(void *parameter)
{
    while (1)
    {


        rt_thread_mdelay(50);

    }
}


//**************************************???9?????????********************************/
uint8_t lock=1;
float jiaodu[10000];
int ii=0;

			float angle_1,angle_2,angle_3,angle_4;
void motion_control_thread_entry(void *parameter)
{
				
	
	
    while (1)
    {
			


# if old
	
			control_motion(2-PS_2.st_data.ch4,PS_2.st_data.ch3,2-PS_2.st_data.ch2);//??????????
# else
			control_motion(1.01-PS_2.st_data.ch4,PS_2.st_data.ch3-1,1.01-PS_2.st_data.ch2,PS_2.st_data.ch5/2);//???????????,?????????????????

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
//***************************************?????????***********************************/
uint8_t rx_num = 0;
uint8_t sound_i = 0;
uint8_t buff=0;
uint8_t cnt=0,buf_sum=0;
    uint8_t sum = 0;
MS5837_DATATYPE ms5837;

//extern unsigned int distance;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t data = 0;
    
    // USART1 ???? PS2 ????

    if (huart->Instance == USART1) // ???????????? USART1
    {
        static uint8_t rxState1 = 0;       /**< ?????????? */
        static uint8_t dataLen1 = 0;       /**< ??????? */
        static uint8_t dataCnt1 = 0;       /**< ??????? */
        static uint8_t sum1 = 0;           /**< §µ??? */
        static uint8_t rxBuffer1[256];     /**< ????????? */
        
        // ?????????????
        data = rxByte_USART1;
        
        switch (rxState1)
        {
            case 0: // ?????
                if (data == 0xAA)
                {
                    rxState1 = 1;
                    rxBuffer1[0] = data;
                    sum1 = data;
                }
                break;
            
            case 1: // ???? ID
                rxBuffer1[1] = data;
                sum1 += data;
                if (data == 0x00)
                {
                    rxState1 = 2;
                }
                else
                {
                    rxState1 = 0; // ??§¹ ID????????
                }
                break;
            
            case 2: // ???????????
                rxBuffer1[2] = data;
                if (data < 250)
                {
                    dataLen1 = data;
                    dataCnt1 = 0;
                    sum1 += data;
                    rxState1 = 3;
                }
                else
                {
                    rxState1 = 0; // ??§¹???????????
                }
                break;
            
            case 3: // ????????
                rxBuffer1[3 + dataCnt1++] = data;
                if ((dataCnt1 % 4) == 3) // ?????4??????§Ö??4?????
                {
                    sum1 += data;
                }
                if (--dataLen1 == 0)
                {
                    rxState1 = 4;
                }
                break;
            
            case 4: // ????§µ?????
                rxBuffer1[3 + dataCnt1] = data;
                rxState1 = 0; // ??????
                if (sum1 == data) // §µ??????
                {
                    // ???????????????? PS2Data ????
                    memcpy(&ps2Data.channel1, &rxBuffer1[3], sizeof(float));
                    memcpy(&ps2Data.channel2, &rxBuffer1[7], sizeof(float));
                    memcpy(&ps2Data.channel3, &rxBuffer1[11], sizeof(float));
                    memcpy(&ps2Data.channel4, &rxBuffer1[15], sizeof(float));
                    // ?????????????????????
                    
                    printf("ch1=%.2f, ch2=%.2f, ch3=%.2f, ch4=%.2f\n", 
                           ps2Data.channel1, ps2Data.channel2, ps2Data.channel3, ps2Data.channel4);
                }
                else
                {
                    printf("USART1 §µ?????\n");
                }
                sum1 = 0; // ????§µ???
                break;
            
            default:
                rxState1 = 0; // ??????
                break;
        }
        
        // ???????? UART1 ??????§Ø?
        HAL_UART_Receive_IT(&huart1, &rxByte_USART1, 1);
    }
    
    // UART5 ???? MS5837 ????
    else if (huart->Instance == UART5)
    {
        static u8 _data_len = 0, _cnt = 0;
        static u8 rx_state = 0;

        if (rx_state == 0 && data == 0x55)//????0x55
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
	

