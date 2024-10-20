
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
        mpu6050.st_data.pitch = pitch_1;
        mpu6050.st_data.roll = roll_1;
        mpu6050.st_data.yaw = yaw_1;
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
uint8_t lock = 1;
float jiaodu[10000];
int ii = 0;
float angle_1, angle_2, angle_3, angle_4;

void motion_control_thread_entry(void *parameter)
{
    while (1)
    {
        #if old
        control_motion(2 - PS_2.st_data.ch4, PS_2.st_data.ch3, 2 - PS_2.st_data.ch2); // 考试前一直用的
        #else
        control_motion(1.01 - PS_2.st_data.ch4, PS_2.st_data.ch3 - 1, 1.01 - PS_2.st_data.ch2, PS_2.st_data.ch5 / 2); // 考试后改进的算法, 第四个参数是速度控制

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

			// 解锁后的舵机动作
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

//***************************************回调函数区***********************************/
uint8_t rx_num = 0;
uint8_t sound_i = 0;
uint8_t buff = 0;
uint8_t cnt = 0, buf_sum = 0;
uint8_t sum = 0;
MS5837_DATATYPE ms5837;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t data = 0;
    
    // USART1 接收 PS2 数据
    if (huart->Instance == USART1)
    {
        static uint8_t rxState1 = 0;       /**< 接收状态机状态 */
        static uint8_t dataLen1 = 0;       /**< 数据长度 */
        static uint8_t dataCnt1 = 0;       /**< 数据计数 */
        static uint8_t sum1 = 0;           /**< 校验和 */
        static uint8_t rxBuffer1[256];     /**< 接收缓冲区 */
        
        // 获取接收到的字节
        data = rxByte_USART1;
        
        switch (rxState1)
        {
            case 0: // 等待帧头
                if (data == 0xAA)
                {
                    rxState1 = 1;
                    rxBuffer1[0] = data;
                    sum1 = data;
                }
                break;
            
            case 1: // 接收 ID
                rxBuffer1[1] = data;
                sum1 += data;
                if (data == 0x00)
                {
                    rxState1 = 2;
                }
                else
                {
                    rxState1 = 0; // 无效 ID，重置状态
                }
                break;
            
            case 2: // 接收数据长度
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
                    rxState1 = 0; // 无效长度，重置状态
                }
                break;
            
            case 3: // 接收数据
                rxBuffer1[3 + dataCnt1++] = data;
                if ((dataCnt1 % 4) == 3) // 每接收4个字节中的第4个字节
                {
                    sum1 += data;
                }
                if (--dataLen1 == 0)
                {
                    rxState1 = 4;
                }
                break;
            
            case 4: // 接收校验字节
                rxBuffer1[3 + dataCnt1] = data;
                rxState1 = 0; // 重置状态
                if (sum1 == data) // 校验和匹配
                {
                    // 将接收到的数据填充到 PS2Data 结构体
                    memcpy(&ps2Data.channel1, &rxBuffer1[3], sizeof(float));
                    memcpy(&ps2Data.channel2, &rxBuffer1[7], sizeof(float));
                    memcpy(&ps2Data.channel3, &rxBuffer1[11], sizeof(float));
                    memcpy(&ps2Data.channel4, &rxBuffer1[15], sizeof(float));
                    // 根据需要继续填充其他字段
                    
                    printf("ch1=%.2f, ch2=%.2f, ch3=%.2f, ch4=%.2f\n", 
                           ps2Data.channel1, ps2Data.channel2, ps2Data.channel3, ps2Data.channel4);
                }
                else
                {
                    printf("USART1 校验错误\n");
                }
                sum1 = 0; // 重置校验和
                break;
            
            default:
                rxState1 = 0; // 重置状态
                break;
        }
        
        // 重新启动 UART1 的接收中断
        HAL_UART_Receive_IT(&huart1, &rxByte_USART1, 1);
    }
    
    // UART5 接收 MS5837 数据
    else if (huart->Instance == UART5)
    {
        static uint8_t rxState5 = 0;       /**< 接收状态机状态 */
        static uint8_t dataCnt5 = 0;       /**< 数据计数 */
        static uint8_t rxBuffer5[256];     /**< 接收缓冲区 */
        
        // 获取接收到的字节
        data = rxByte_UART5;
        
        switch (rxState5)
        {
            case 0: // 等待帧头
                if (data == 0xAB)
                {
                    rxState5 = 1;
                    rxBuffer5[0] = data;
                }
                break;
            
            case 1: // 接收 ID
                rxBuffer5[1] = data;
                if (data == 0x01)
                {
                    rxState5 = 2;
                }
                else
                {
                    rxState5 = 0; // 无效 ID，重置状态
                }
                break;
            
            case 2: // 接收数据
                rxBuffer5[2 + dataCnt5++] = data;
                if (dataCnt5 >= sizeof(MS5837Data))
                {
                    // 将接收到的数据填充到 MS5837Data 结构体
                    memcpy(&ms5837Data.pressure, &rxBuffer5[2], sizeof(float));
                    memcpy(&ms5837Data.water_temperature, &rxBuffer5[6], sizeof(float));
                    
                    printf("Pressure=%.2f, Water Temp=%.2f\n", 
                           ms5837Data.pressure, ms5837Data.water_temperature);
                    
                    rxState5 = 0; // 重置状态
                    dataCnt5 = 0;
                }
                break;
            
            default:
                rxState5 = 0; // 重置状态
                break;
        }
        
        // 重新启动 UART5 的接收中断
        HAL_UART_Receive_IT(&huart5, &rxByte_UART5, 1);
    }
    
    // UART4 接收 Sound 数据
    else if (huart->Instance == UART4)
    {
        static uint8_t dataCnt4 = 0;       /**< 数据计数 */
        static uint8_t rxBuffer4[4];       /**< 接收缓冲区，假设 SoundData 只有一个 float */
        
        // 获取接收到的字节
        data = rxByte_UART4;
        
        rxBuffer4[dataCnt4++] = data;
        if (dataCnt4 >= sizeof(SoundData))
        {
            // 将接收到的数据填充到 SoundData 结构体
            memcpy(&soundData.distance, rxBuffer4, sizeof(float));
            
            printf("Distance=%.2f\n", soundData.distance);
            
            dataCnt4 = 0; // 重置计数
        }
        
        // 重新启动 UART4 的接收中断
        HAL_UART_Receive_IT(&huart4, &rxByte_UART4, 1);
    }
}
