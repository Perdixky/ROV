
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



float pitch_1, roll_1, yaw_1;           //ŷ����
short aacx, aacy, aacz;         //���ٶȴ�����ԭʼ����
short gyrox, gyroy, gyroz;      //������ԭʼ����
float temp;                     //�¶�
extern uint16_t IIC_SCL_PIN;
extern uint16_t IIC_SDA_PIN;
void my_limit(float *p)
{
if(*p>2400)
*p=2400;
else if(*p<600)
*p=600;
}



uint8_t procese_buf[50];//���ݴ�����
uint8_t buf;
uint8_t sound_buf;

//**************************************�߳�1********************************/
void led1_thread_entry(void *parameter)
{
    while (1)
    {

        rt_thread_delay(500);   /* ��ʱ500��tick */

//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        rt_thread_delay(500);   /* ��ʱ500��tick */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
    }
}




//**************************************�߳�2********************************/
MPU6050DATATYPE mpu6050;


void mpu6050_thread_entry(void *parameter)
{

    rt_mq_t queue = rt_mq_create("my_queue", sizeof(float), 10, RT_IPC_FLAG_FIFO);
    if (queue == RT_NULL)
    {
        printf("Failed to create message queue!\n");
        rt_thread_delay(500);   /* ��ʱ500��tick */

    }
    IIC_SCL_PIN =     SCL_1_Pin;
    IIC_SDA_PIN =     SDA_1_Pin;
		while(MPU_Init()+mpu_dmp_init()!=0)
//    uint8_t a = MPU_Init();         //MPU6050��ʼ��
//    int b = mpu_dmp_init();     //dmp��ʼ��
//printf("b=%d",b);
    rt_thread_delay(500);   /* ��ʱ500��tick */

    while (1)
    {


//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        while (mpu_dmp_get_data(&roll_1, &pitch_1, &yaw_1)); //����Ҫ��while�ȴ������ܶ�ȡ�ɹ�
        mpu6050.st_data.pitch = pitch_1;
        mpu6050.st_data.roll = roll_1;
        mpu6050.st_data.yaw = yaw_1;
        mpu6050.st_data.Temp = temp / 100;
        MPU_Get_Accelerometer(&aacx, &aacy, &aacz);     //�õ����ٶȴ���������
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);      //�õ�����������
        temp = MPU_Get_Temperature();                   //�õ��¶���Ϣ
//        printf("mpu1��X:%.1f��  Y:%.1f��  Z:%.1f��  %.2f��C\r\n",roll_1,pitch_1,yaw_1,temp/100);//����1����ɼ���Ϣ
//   rt_thread_delay(50);   /* ��ʱ500��tick */
//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
//   rt_thread_delay(50);   /* ��ʱ500��tick */
        rt_thread_delay(50);   /* ��ʱ500��tick */




    }
}

\

//**************************************�߳�3********************************/
void MS5837_thread_entry(void *parameter)
{
    while (1)
    {
//        IIC_SCL_PIN =     MS5837_SCL_Pin;
//        IIC_SDA_PIN =     MS5837_SDA_Pin;
        MS5837_30BA_ReSet();                                                    //��λMS5837
        rt_thread_mdelay(20);

        MS5837_30BA_PROM();                                                     //��ʼ��MS5837
        rt_thread_mdelay(20);
        if (!MS5837_30BA_Crc4())                                              //CRCУ��
        {
            printf("  ��ʼ��ʧ��\r\n");
            printf("  ��������Ƿ���ȷ\r\n");

        }
        else
        {
            printf("  ��ʼ���ɹ�\r\n");
            printf("  ��⵽MS5837_30BA\r\n\r\n");
            break;
        }

    }

    while (1)
    {
        MS5837_30BA_GetData();                                      //��ȡ����
        rt_thread_mdelay(200);
        printf("  Welcome to BlueRobots Community! \r\n");          //�������ԭʼ����
        printf("  Temperature : %.2f C \r\n", Temperature);         //�������ԭʼ����
        printf("  Pressure : %u mBar \r\n\r\n\r\n", Pressure);      //�������ԭʼ����


    }
}


//**************************************�߳�4********************************/
uint8_t buf;
uint8_t len, head, tail, id;
extern UART_HandleTypeDef huart2;
ps2DATATYPE PS_2;

//uint8_t rx_buf[100]={0xAA,0X02,0X10,0X00,0X00,0X00,0X00,   0XCD,0XCC,0X8C,0X3F,    0XCD,0XCC,0X0C,0X40,  0X33,0X33,0X53,0X40, 0X7B};//���ڽ��ջ�����
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


//**************************************�߳�5********************************/
void SendData_thread_entry(void *parameter)
{
    while (1)
    {

        Send_Data_Task();
        rt_thread_mdelay(5);

    }
}

//**************************************�߳�6��������********************************/
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
//**************************************�߳�7:PH��********************************/

extern ADC_HandleTypeDef hadc1;
extern float volot[4];
phDATATYPE ph;

extern float v;
extern int adc_num;

uint8_t i;
float adcBuf[2];//���ADC

void PH_thread_entry(void *parameter)
{
    while (1)
    {


        i = 0;
        while (i < 2)
        {
            HAL_ADC_Start(&hadc1);//����ADC
            HAL_ADC_PollForConversion(&hadc1, 10); //��ʾ�ȴ�ת����ɣ��ڶ���������ʾ��ʱʱ�䣬��λms.
            //HAL_ADC_GetState(&hadc1)Ϊ��ȡADC״̬��HAL_ADC_STATE_REG_EOC��ʾת����ɱ�־λ��ת�����ݿ��á�
            if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) //�����ж�ת����ɱ�־λ�Ƿ�����,HAL_ADC_STATE_REG_EOC��ʾת����ɱ�־λ��ת�����ݿ���
            {
                //��ȡADCת�����ݣ�����Ϊ12λ���鿴�����ֲ��֪���Ĵ���Ϊ16λ�洢ת�����ݣ������Ҷ��룬��ת�������ݷ�ΧΪ0~2^12-1,��0~4095.
                adcBuf[i] = HAL_ADC_GetValue(&hadc1) * 3.3 / 4096;
//   printf("\nadc%d=%4.0d,��ѹ=%1.4f",i,adcBuf[i],adcBuf[i]*3.3f/65536);
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



//**************************************�߳�8��ˮ�ʴ�����********************************/

void water_quality_thread_entry(void *parameter)
{
    while (1)
    {


        rt_thread_mdelay(50);

    }
}


//**************************************�߳�9���˶�����********************************/
uint8_t lock = 1;
float jiaodu[10000];
int ii = 0;
float angle_1, angle_2, angle_3, angle_4;

void motion_control_thread_entry(void *parameter)
{
    while (1)
    {
        #if old
        control_motion(2 - PS_2.st_data.ch4, PS_2.st_data.ch3, 2 - PS_2.st_data.ch2); // ����ǰһֱ�õ�
        #else
        control_motion(1.01 - PS_2.st_data.ch4, PS_2.st_data.ch3 - 1, 1.01 - PS_2.st_data.ch2, PS_2.st_data.ch5 / 2); // ���Ժ�Ľ����㷨, ���ĸ��������ٶȿ���

        angle_1 = 1500 - PS_2.st_data.yaw_2 * 1000 / 90;
        angle_2 = 2500 - PS_2.st_data.roll_1 * 1000 / 90;
        angle_3 = 1500 - (PS_2.st_data.roll_2 - PS_2.st_data.roll_1) * 1000 / 90;
        angle_4 = 1500 + (PS_2.st_data.yaw_3 - PS_2.st_data.yaw_2) * 1000 / 90;

        my_limit(&angle_1);
        my_limit(&angle_2);
        my_limit(&angle_3);
        my_limit(&angle_4);

        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, angle_1);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, angle_2);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, angle_3);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, angle_4);

        if ((lock == 0) && (PS_2.st_data.ch7 > 0.8))
        {
            lock = 1;

            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

			// ������Ķ������
            for (int i = 1492; i <= 1532; i += 2)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
            }

            for (int i = 1500; i >= 1487; i--)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, i);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, i);
            }

            HAL_Delay(3000);
        }

        if ((lock == 1) && (PS_2.st_data.ch8 > 0.8))
        {
            lock = 0;
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
        }

        rt_thread_mdelay(10);
        #endif
    }
}

//***************************************�ص�������***********************************/
uint8_t rx_num = 0;
uint8_t sound_i = 0;
uint8_t buff = 0;
uint8_t cnt = 0, buf_sum = 0;
uint8_t sum = 0;
MS5837_DATATYPE ms5837;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t data = 0;
    
    // USART1 ���� PS2 ����
    if (huart->Instance == USART1)
    {
        static uint8_t rxState1 = 0;       /**< ����״̬��״̬ */
        static uint8_t dataLen1 = 0;       /**< ���ݳ��� */
        static uint8_t dataCnt1 = 0;       /**< ���ݼ��� */
        static uint8_t sum1 = 0;           /**< У��� */
        static uint8_t rxBuffer1[256];     /**< ���ջ����� */
        
        // ��ȡ���յ����ֽ�
        data = rxByte_USART1;
        
        switch (rxState1)
        {
            case 0: // �ȴ�֡ͷ
                if (data == 0xAA)
                {
                    rxState1 = 1;
                    rxBuffer1[0] = data;
                    sum1 = data;
                }
                break;
            
            case 1: // ���� ID
                rxBuffer1[1] = data;
                sum1 += data;
                if (data == 0x00)
                {
                    rxState1 = 2;
                }
                else
                {
                    rxState1 = 0; // ��Ч ID������״̬
                }
                break;
            
            case 2: // �������ݳ���
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
                    rxState1 = 0; // ��Ч���ȣ�����״̬
                }
                break;
            
            case 3: // ��������
                rxBuffer1[3 + dataCnt1++] = data;
                if ((dataCnt1 % 4) == 3) // ÿ����4���ֽ��еĵ�4���ֽ�
                {
                    sum1 += data;
                }
                if (--dataLen1 == 0)
                {
                    rxState1 = 4;
                }
                break;
            
            case 4: // ����У���ֽ�
                rxBuffer1[3 + dataCnt1] = data;
                rxState1 = 0; // ����״̬
                if (sum1 == data) // У���ƥ��
                {
                    // �����յ���������䵽 PS2Data �ṹ��
                    memcpy(&ps2Data.channel1, &rxBuffer1[3], sizeof(float));
                    memcpy(&ps2Data.channel2, &rxBuffer1[7], sizeof(float));
                    memcpy(&ps2Data.channel3, &rxBuffer1[11], sizeof(float));
                    memcpy(&ps2Data.channel4, &rxBuffer1[15], sizeof(float));
                    // ������Ҫ������������ֶ�
                    
                    printf("ch1=%.2f, ch2=%.2f, ch3=%.2f, ch4=%.2f\n", 
                           ps2Data.channel1, ps2Data.channel2, ps2Data.channel3, ps2Data.channel4);
                }
                else
                {
                    printf("USART1 У�����\n");
                }
                sum1 = 0; // ����У���
                break;
            
            default:
                rxState1 = 0; // ����״̬
                break;
        }
        
        // �������� UART1 �Ľ����ж�
        HAL_UART_Receive_IT(&huart1, &rxByte_USART1, 1);
    }
    
    // UART5 ���� MS5837 ����
    else if (huart->Instance == UART5)
    {
        static uint8_t rxState5 = 0;       /**< ����״̬��״̬ */
        static uint8_t dataCnt5 = 0;       /**< ���ݼ��� */
        static uint8_t rxBuffer5[256];     /**< ���ջ����� */
        
        // ��ȡ���յ����ֽ�
        data = rxByte_UART5;
        
        switch (rxState5)
        {
            case 0: // �ȴ�֡ͷ
                if (data == 0xAB)
                {
                    rxState5 = 1;
                    rxBuffer5[0] = data;
                }
                break;
            
            case 1: // ���� ID
                rxBuffer5[1] = data;
                if (data == 0x01)
                {
                    rxState5 = 2;
                }
                else
                {
                    rxState5 = 0; // ��Ч ID������״̬
                }
                break;
            
            case 2: // ��������
                rxBuffer5[2 + dataCnt5++] = data;
                if (dataCnt5 >= sizeof(MS5837Data))
                {
                    // �����յ���������䵽 MS5837Data �ṹ��
                    memcpy(&ms5837Data.pressure, &rxBuffer5[2], sizeof(float));
                    memcpy(&ms5837Data.water_temperature, &rxBuffer5[6], sizeof(float));
                    
                    printf("Pressure=%.2f, Water Temp=%.2f\n", 
                           ms5837Data.pressure, ms5837Data.water_temperature);
                    
                    rxState5 = 0; // ����״̬
                    dataCnt5 = 0;
                }
                break;
            
            default:
                rxState5 = 0; // ����״̬
                break;
        }
        
        // �������� UART5 �Ľ����ж�
        HAL_UART_Receive_IT(&huart5, &rxByte_UART5, 1);
    }
    
    // UART4 ���� Sound ����
    else if (huart->Instance == UART4)
    {
        static uint8_t dataCnt4 = 0;       /**< ���ݼ��� */
        static uint8_t rxBuffer4[4];       /**< ���ջ����������� SoundData ֻ��һ�� float */
        
        // ��ȡ���յ����ֽ�
        data = rxByte_UART4;
        
        rxBuffer4[dataCnt4++] = data;
        if (dataCnt4 >= sizeof(SoundData))
        {
            // �����յ���������䵽 SoundData �ṹ��
            memcpy(&soundData.distance, rxBuffer4, sizeof(float));
            
            printf("Distance=%.2f\n", soundData.distance);
            
            dataCnt4 = 0; // ���ü���
        }
        
        // �������� UART4 �Ľ����ж�
        HAL_UART_Receive_IT(&huart4, &rxByte_UART4, 1);
    }
}
