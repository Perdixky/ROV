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

// 全局变量，用于存储MPU6050数据
float pitch_1, roll_1, yaw_1;           // 俯仰角、横滚角、偏航角
short aacx, aacy, aacz;                 // 加速度计数据
short gyrox, gyroy, gyroz;              // 陀螺仪数据
float temp;                             // 温度

extern uint16_t IIC_SCL_PIN;            // I2C SCL引脚
extern uint16_t IIC_SDA_PIN;            // I2C SDA引脚

JY901B_DataType jy901b;                 // JY901B传感器数据类型

// 限制值的函数，限制范围在600到2400之间
void my_limit(float *p)
{
    if(*p > 2400)
        *p = 2400;
    else if(*p < 600)
        *p = 600;
}

uint8_t procese_buf[50];    // 数据处理缓冲区
uint8_t buf;
uint8_t sound_buf;

//************************************** 线程1: LED闪烁 ********************************/
void led1_thread_entry(void *parameter)
{
    while (1)
    {
        rt_thread_delay(500);   /* 延迟500个tick */

        // 点亮连接在GPIOB引脚4的LED
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

        rt_thread_delay(500);   /* 延迟500个tick */

        // 熄灭连接在GPIOB引脚4的LED
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    }
}

//************************************** 线程2: MPU6050传感器 ********************************/
MPU6050DATATYPE mpu6050;

void mpu6050_thread_entry(void *parameter)
{
    // 创建一个名为“my_queue”的消息队列
    rt_mq_t queue = rt_mq_create("my_queue", sizeof(float), 10, RT_IPC_FLAG_FIFO);
    if (queue == RT_NULL)
    {
        printf("消息队列创建失败！\n");
        rt_thread_delay(500);   /* 延迟500个tick */
    }

    // 设置MPU6050的I2C引脚
    IIC_SCL_PIN = SCL_1_Pin;
    IIC_SDA_PIN = SDA_1_Pin;

    // 初始化MPU6050和DMP（数字运动处理器）
    while (MPU_Init() + mpu_dmp_init() != 0)
        rt_thread_delay(500);   /* 延迟500个tick */

    while (1)
    {
        // 使用DMP获取MPU6050的数据
        while (mpu_dmp_get_data(&roll_1, &pitch_1, &yaw_1)); // 不断获取数据，直到成功

        // 将原始数据转换为角度（度数）
        mpu6050.st_data.pitch = ((jy901b.byte[1] << 8) | jy901b.byte[0]) / 32768 * 180;
        mpu6050.st_data.roll = ((jy901b.byte[1] << 8) | jy901b.byte[0]) / 32768 * 180;
        mpu6050.st_data.yaw = ((jy901b.byte[1] << 8) | jy901b.byte[0]) / 32768 * 180;
        mpu6050.st_data.Temp = temp / 100;

        // 获取加速度计和陀螺仪数据
        MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);

        // 获取温度
        temp = MPU_Get_Temperature();

        rt_thread_delay(50);   /* 延迟50个tick */
    }
}

//************************************** 线程3: MS5837传感器 ********************************/
void MS5837_thread_entry(void *parameter)
{
    while (1)
    {
        // 重置MS5837传感器
        MS5837_30BA_ReSet();
        rt_thread_mdelay(20);

        // 读取校准数据
        MS5837_30BA_PROM();
        rt_thread_mdelay(20);

        // 进行CRC校验
        if (!MS5837_30BA_Crc4())
        {
            printf("  CRC校验失败\r\n");
            printf("  重新初始化MS5837传感器\r\n");
        }
        else
        {
            printf("  CRC校验通过\r\n");
            printf("  MS5837_30BA传感器已初始化\r\n\r\n");
            break;
        }
    }

    while (1)
    {
        // 从MS5837传感器获取数据
        MS5837_30BA_GetData();
        rt_thread_mdelay(200);

        // 打印传感器数据
        printf("  欢迎来到BlueRobots社区！\r\n");
        printf("  温度: %.2f C \r\n", Temperature);
        printf("  压力: %u mBar \r\n\r\n\r\n", Pressure);
    }
}

//************************************** 线程4: 数据接收 ********************************/
uint8_t buf;
uint8_t len, head, tail, id;
extern UART_HandleTypeDef huart2;
ps2DATATYPE PS_2;

uint8_t rx_buf[100];
uint8_t data = 0;

void data_receive_thread_entry(void *parameter)
{
    while (1)
    {
        #if old
            // 使用中断接收140字节数据，通过UART1
            HAL_UART_Receive_IT(&huart1, rx_buf, 140);
        #else
            // 使用中断接收一个字节数据，通过UART1
            HAL_UART_Receive_IT(&huart1, &data, 1);
        #endif

        rt_thread_mdelay(20);
    }
}

//************************************** 线程5: 数据发送 ********************************/
void SendData_thread_entry(void *parameter)
{
    while (1)
    {
        Send_Data_Task(); // 数据发送任务
        rt_thread_mdelay(5);
    }
}

//************************************** 线程6: 超声波传感器 ********************************/
uint8_t enable = 1;

uint8_t high_data, low_data;
uint8_t sound_rx_buf[8];
uint8_t sound_sum = 0;
int distance;
soundDATATYPE Ultrasound;

unsigned char uart_buff[4];
unsigned char uart_temp[1];
unsigned int uart_rx_cnt = 0;

void Ultrasound_thread_entry(void *parameter)
{
    while (1)
    {
        // 使用中断接收UART5和UART4数据
        HAL_UART_Receive_IT(&huart5, (uint8_t *)uart_temp, 1);
        HAL_UART_Receive_IT(&huart4, &sound_buf, 1);

        // 发送使能命令给超声波传感器
        HAL_UART_Transmit(&huart4, (uint8_t *)&enable, 1, 0xFFFF);

        for (int i = 0; i < 8; i++)
        {
            if (sound_rx_buf[i] == 0xff)
            {
                // 提取高位和低位数据字节
                high_data = sound_rx_buf[i + 1];
                low_data = sound_rx_buf[i + 2];

                // 计算校验和
                sound_sum = 0xff + high_data + low_data;
                if (sound_sum == sound_rx_buf[i + 3])
                {
                    // 计算距离
                    distance = high_data * 256 + low_data;
                    Ultrasound.st_data.distance = (float)distance;

                    break;
                }
                else
                {
                    printf("错误");
                }
            }
        }

        rt_thread_mdelay(50);
    }
}

//************************************** 线程7: PH传感器 ********************************/
extern ADC_HandleTypeDef hadc1;
extern float volot[4];
phDATATYPE ph;

extern float v;
extern int adc_num;

uint8_t i;
float adcBuf[2]; // 存储ADC转换结果

void PH_thread_entry(void *parameter)
{
    while (1)
    {
        i = 0;
        while (i < 2)
        {
            HAL_ADC_Start(&hadc1); // 启动ADC转换
            HAL_ADC_PollForConversion(&hadc1, 10); // 等待转换完成，超时时间10毫秒

            if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) // 如果转换完成
            {
                // 获取ADC值并转换为电压
                adcBuf[i] = HAL_ADC_GetValue(&hadc1) * 3.3 / 4096;
                i++;
            }
        }

        HAL_ADC_Stop(&hadc1);

        // 计算PH值
        ph.st_data.ph = 6.5 - adcBuf[1];

        // 使用多项式公式计算水质质量
        ph.st_data.quality = adcBuf[0];
        ph.st_data.quality = ph.st_data.quality * ph.st_data.quality * ph.st_data.quality * 66.71 
                             - 127.93 * ph.st_data.quality * ph.st_data.quality 
                             + 428.7 * ph.st_data.quality;

        rt_thread_mdelay(50);
    }
}

//************************************** 线程8: 水质传感器 ********************************/
void water_quality_thread_entry(void *parameter)
{
    while (1)
    {
        rt_thread_mdelay(50);
    }
}

//************************************** 线程9: 运动控制 ********************************/
uint8_t lock = 1;
float jiaodu[10000];
int ii = 0;

float angle_1, angle_2, angle_3, angle_4;
void motion_control_thread_entry(void *parameter)
{
    while (1)
    {
        #if old
            // 使用旧的控制函数
            control_motion(2 - PS_2.st_data.ch4, PS_2.st_data.ch3, 2 - PS_2.st_data.ch2);
        #else
            // 使用新的控制函数
            // 根据PS_2数据调整控制参数
            control_motion(1.01 - PS_2.st_data.ch4, PS_2.st_data.ch3 - 1, 1.01 - PS_2.st_data.ch2, PS_2.st_data.ch5 / 2);

            // 计算舵机角度
            angle_1 = 1500 - PS_2.st_data.yaw_2 * 1000 / 90;
            angle_2 = 2500 - PS_2.st_data.roll_1 * 1000 / 90;
            angle_3 = 1500 - (PS_2.st_data.roll_2 - PS_2.st_data.roll_1) * 1000 / 90;
            angle_4 = 1500 + (PS_2.st_data.yaw_3 - PS_2.st_data.yaw_2) * 1000 / 90;

            // 限制角度到可接受的范围
            my_limit(&angle_1);
            my_limit(&angle_2);
            my_limit(&angle_3);
            my_limit(&angle_4);

            // 设置舵机的PWM比较值
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, angle_1);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, angle_2);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, angle_3);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, angle_4);

            // 锁定机制
            if ((lock == 0) && (PS_2.st_data.ch7 > 0.8))
            {
                lock = 1;

                // 启动PWM，控制htim3的通道
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

                // 渐进增加TIM_CHANNEL_1的PWM比较值
                for (int value = 1492; value <= 1532; value++)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, value);
                }

                // 渐进减少TIM_CHANNEL_2和TIM_CHANNEL_3的PWM比较值
                for (int value = 1500; value >= 1487; value--)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, value);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, value);
                }

                HAL_Delay(3000); // 延迟3秒
            }

            if ((lock == 1) && (PS_2.st_data.ch8 > 0.8))
            {
                lock = 0;

                // 停止htim3的通道PWM
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
            }

            rt_thread_mdelay(10);
        #endif
    }
}

//*************************************** UART接收回调 ***********************************/
uint8_t rx_num = 0;
uint8_t sound_i = 0;
uint8_t buff = 0;
uint8_t cnt = 0, buf_sum = 0;
uint8_t sum = 0;
MS5837_DATATYPE ms5837;

// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t data = 0;

    // USART1接收PS2数据
    if (huart->Instance == USART1) // 检查是否为USART1
    {
        static uint8_t rxState1 = 0;       /**< 接收状态 */
        static uint8_t dataLen1 = 0;       /**< 数据长度 */
        static uint8_t dataCnt1 = 0;       /**< 数据计数 */
        static uint8_t sum1 = 0;           /**< 校验和 */
        static uint8_t rxBuffer1[256];     /**< 接收缓冲区 */

        // 读取接收到的数据
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

            case 1: // 等待ID
                rxBuffer1[1] = data;
                sum1 += data;
                if (data == 0x00)
                {
                    rxState1 = 2;
                }
                else
                {
                    rxState1 = 0; // ID错误，重置
                }
                break;

            case 2: // 等待数据长度
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
                    rxState1 = 0; // 数据长度无效，重置
                }
                break;

            case 3: // 接收数据
                rxBuffer1[3 + dataCnt1++] = data;
                if ((dataCnt1 % 4) == 3) // 每4字节（float数据）处理一次
                {
                    sum1 += data;
                }
                if (--dataLen1 == 0)
                {
                    rxState1 = 4;
                }
                break;

            case 4: // 等待校验和
                rxBuffer1[3 + dataCnt1] = data;
                rxState1 = 0; // 重置状态
                if (sum1 == data) // 校验和验证通过
                {
                    // 将接收到的数据解析到PS2Data结构中
                    memcpy(&ps2Data.channel1, &rxBuffer1[3], sizeof(float));
                    memcpy(&ps2Data.channel2, &rxBuffer1[7], sizeof(float));
                    memcpy(&ps2Data.channel3, &rxBuffer1[11], sizeof(float));
                    memcpy(&ps2Data.channel4, &rxBuffer1[15], sizeof(float));

                    // 处理接收到的数据
                    printf("ch1=%.2f, ch2=%.2f, ch3=%.2f, ch4=%.2f\n", 
                           ps2Data.channel1, ps2Data.channel2, ps2Data.channel3, ps2Data.channel4);
                }
                else
                {
                    printf("USART1校验和错误\n");
                }
                sum1 = 0; // 重置校验和
                break;

            default:
                rxState1 = 0; // 重置状态
                break;
        }

        // 重新启动UART1接收中断
        HAL_UART_Receive_IT(&huart1, &rxByte_USART1, 1);
    }

    // UART5接收MS5837数据
    else if (huart->Instance == UART5)
    {
        static uint8_t _data_len = 0, _cnt = 0;
        static uint8_t rx_state = 0;

        if (rx_state == 0 && data == 0x55) // 检查帧头0x55
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
