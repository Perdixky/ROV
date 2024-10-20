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

// ȫ�ֱ��������ڴ洢MPU6050����
float pitch_1, roll_1, yaw_1;           // �����ǡ�����ǡ�ƫ����
short aacx, aacy, aacz;                 // ���ٶȼ�����
short gyrox, gyroy, gyroz;              // ����������
float temp;                             // �¶�

extern uint16_t IIC_SCL_PIN;            // I2C SCL����
extern uint16_t IIC_SDA_PIN;            // I2C SDA����

JY901B_DataType jy901b;                 // JY901B��������������

// ����ֵ�ĺ��������Ʒ�Χ��600��2400֮��
void my_limit(float *p)
{
    if(*p > 2400)
        *p = 2400;
    else if(*p < 600)
        *p = 600;
}

uint8_t procese_buf[50];    // ���ݴ�������
uint8_t buf;
uint8_t sound_buf;

//************************************** �߳�1: LED��˸ ********************************/
void led1_thread_entry(void *parameter)
{
    while (1)
    {
        rt_thread_delay(500);   /* �ӳ�500��tick */

        // ����������GPIOB����4��LED
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

        rt_thread_delay(500);   /* �ӳ�500��tick */

        // Ϩ��������GPIOB����4��LED
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    }
}

//************************************** �߳�2: MPU6050������ ********************************/
MPU6050DATATYPE mpu6050;

void mpu6050_thread_entry(void *parameter)
{
    // ����һ����Ϊ��my_queue������Ϣ����
    rt_mq_t queue = rt_mq_create("my_queue", sizeof(float), 10, RT_IPC_FLAG_FIFO);
    if (queue == RT_NULL)
    {
        printf("��Ϣ���д���ʧ�ܣ�\n");
        rt_thread_delay(500);   /* �ӳ�500��tick */
    }

    // ����MPU6050��I2C����
    IIC_SCL_PIN = SCL_1_Pin;
    IIC_SDA_PIN = SDA_1_Pin;

    // ��ʼ��MPU6050��DMP�������˶���������
    while (MPU_Init() + mpu_dmp_init() != 0)
        rt_thread_delay(500);   /* �ӳ�500��tick */

    while (1)
    {
        // ʹ��DMP��ȡMPU6050������
        while (mpu_dmp_get_data(&roll_1, &pitch_1, &yaw_1)); // ���ϻ�ȡ���ݣ�ֱ���ɹ�

        // ��ԭʼ����ת��Ϊ�Ƕȣ�������
        mpu6050.st_data.pitch = ((jy901b.byte[1] << 8) | jy901b.byte[0]) / 32768 * 180;
        mpu6050.st_data.roll = ((jy901b.byte[1] << 8) | jy901b.byte[0]) / 32768 * 180;
        mpu6050.st_data.yaw = ((jy901b.byte[1] << 8) | jy901b.byte[0]) / 32768 * 180;
        mpu6050.st_data.Temp = temp / 100;

        // ��ȡ���ٶȼƺ�����������
        MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);

        // ��ȡ�¶�
        temp = MPU_Get_Temperature();

        rt_thread_delay(50);   /* �ӳ�50��tick */
    }
}

//************************************** �߳�3: MS5837������ ********************************/
void MS5837_thread_entry(void *parameter)
{
    while (1)
    {
        // ����MS5837������
        MS5837_30BA_ReSet();
        rt_thread_mdelay(20);

        // ��ȡУ׼����
        MS5837_30BA_PROM();
        rt_thread_mdelay(20);

        // ����CRCУ��
        if (!MS5837_30BA_Crc4())
        {
            printf("  CRCУ��ʧ��\r\n");
            printf("  ���³�ʼ��MS5837������\r\n");
        }
        else
        {
            printf("  CRCУ��ͨ��\r\n");
            printf("  MS5837_30BA�������ѳ�ʼ��\r\n\r\n");
            break;
        }
    }

    while (1)
    {
        // ��MS5837��������ȡ����
        MS5837_30BA_GetData();
        rt_thread_mdelay(200);

        // ��ӡ����������
        printf("  ��ӭ����BlueRobots������\r\n");
        printf("  �¶�: %.2f C \r\n", Temperature);
        printf("  ѹ��: %u mBar \r\n\r\n\r\n", Pressure);
    }
}

//************************************** �߳�4: ���ݽ��� ********************************/
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
            // ʹ���жϽ���140�ֽ����ݣ�ͨ��UART1
            HAL_UART_Receive_IT(&huart1, rx_buf, 140);
        #else
            // ʹ���жϽ���һ���ֽ����ݣ�ͨ��UART1
            HAL_UART_Receive_IT(&huart1, &data, 1);
        #endif

        rt_thread_mdelay(20);
    }
}

//************************************** �߳�5: ���ݷ��� ********************************/
void SendData_thread_entry(void *parameter)
{
    while (1)
    {
        Send_Data_Task(); // ���ݷ�������
        rt_thread_mdelay(5);
    }
}

//************************************** �߳�6: ������������ ********************************/
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
        // ʹ���жϽ���UART5��UART4����
        HAL_UART_Receive_IT(&huart5, (uint8_t *)uart_temp, 1);
        HAL_UART_Receive_IT(&huart4, &sound_buf, 1);

        // ����ʹ�������������������
        HAL_UART_Transmit(&huart4, (uint8_t *)&enable, 1, 0xFFFF);

        for (int i = 0; i < 8; i++)
        {
            if (sound_rx_buf[i] == 0xff)
            {
                // ��ȡ��λ�͵�λ�����ֽ�
                high_data = sound_rx_buf[i + 1];
                low_data = sound_rx_buf[i + 2];

                // ����У���
                sound_sum = 0xff + high_data + low_data;
                if (sound_sum == sound_rx_buf[i + 3])
                {
                    // �������
                    distance = high_data * 256 + low_data;
                    Ultrasound.st_data.distance = (float)distance;

                    break;
                }
                else
                {
                    printf("����");
                }
            }
        }

        rt_thread_mdelay(50);
    }
}

//************************************** �߳�7: PH������ ********************************/
extern ADC_HandleTypeDef hadc1;
extern float volot[4];
phDATATYPE ph;

extern float v;
extern int adc_num;

uint8_t i;
float adcBuf[2]; // �洢ADCת�����

void PH_thread_entry(void *parameter)
{
    while (1)
    {
        i = 0;
        while (i < 2)
        {
            HAL_ADC_Start(&hadc1); // ����ADCת��
            HAL_ADC_PollForConversion(&hadc1, 10); // �ȴ�ת����ɣ���ʱʱ��10����

            if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) // ���ת�����
            {
                // ��ȡADCֵ��ת��Ϊ��ѹ
                adcBuf[i] = HAL_ADC_GetValue(&hadc1) * 3.3 / 4096;
                i++;
            }
        }

        HAL_ADC_Stop(&hadc1);

        // ����PHֵ
        ph.st_data.ph = 6.5 - adcBuf[1];

        // ʹ�ö���ʽ��ʽ����ˮ������
        ph.st_data.quality = adcBuf[0];
        ph.st_data.quality = ph.st_data.quality * ph.st_data.quality * ph.st_data.quality * 66.71 
                             - 127.93 * ph.st_data.quality * ph.st_data.quality 
                             + 428.7 * ph.st_data.quality;

        rt_thread_mdelay(50);
    }
}

//************************************** �߳�8: ˮ�ʴ����� ********************************/
void water_quality_thread_entry(void *parameter)
{
    while (1)
    {
        rt_thread_mdelay(50);
    }
}

//************************************** �߳�9: �˶����� ********************************/
uint8_t lock = 1;
float jiaodu[10000];
int ii = 0;

float angle_1, angle_2, angle_3, angle_4;
void motion_control_thread_entry(void *parameter)
{
    while (1)
    {
        #if old
            // ʹ�þɵĿ��ƺ���
            control_motion(2 - PS_2.st_data.ch4, PS_2.st_data.ch3, 2 - PS_2.st_data.ch2);
        #else
            // ʹ���µĿ��ƺ���
            // ����PS_2���ݵ������Ʋ���
            control_motion(1.01 - PS_2.st_data.ch4, PS_2.st_data.ch3 - 1, 1.01 - PS_2.st_data.ch2, PS_2.st_data.ch5 / 2);

            // �������Ƕ�
            angle_1 = 1500 - PS_2.st_data.yaw_2 * 1000 / 90;
            angle_2 = 2500 - PS_2.st_data.roll_1 * 1000 / 90;
            angle_3 = 1500 - (PS_2.st_data.roll_2 - PS_2.st_data.roll_1) * 1000 / 90;
            angle_4 = 1500 + (PS_2.st_data.yaw_3 - PS_2.st_data.yaw_2) * 1000 / 90;

            // ���ƽǶȵ��ɽ��ܵķ�Χ
            my_limit(&angle_1);
            my_limit(&angle_2);
            my_limit(&angle_3);
            my_limit(&angle_4);

            // ���ö����PWM�Ƚ�ֵ
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, angle_1);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, angle_2);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, angle_3);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, angle_4);

            // ��������
            if ((lock == 0) && (PS_2.st_data.ch7 > 0.8))
            {
                lock = 1;

                // ����PWM������htim3��ͨ��
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

                // ��������TIM_CHANNEL_1��PWM�Ƚ�ֵ
                for (int value = 1492; value <= 1532; value++)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, value);
                }

                // ��������TIM_CHANNEL_2��TIM_CHANNEL_3��PWM�Ƚ�ֵ
                for (int value = 1500; value >= 1487; value--)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, value);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, value);
                }

                HAL_Delay(3000); // �ӳ�3��
            }

            if ((lock == 1) && (PS_2.st_data.ch8 > 0.8))
            {
                lock = 0;

                // ֹͣhtim3��ͨ��PWM
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
            }

            rt_thread_mdelay(10);
        #endif
    }
}

//*************************************** UART���ջص� ***********************************/
uint8_t rx_num = 0;
uint8_t sound_i = 0;
uint8_t buff = 0;
uint8_t cnt = 0, buf_sum = 0;
uint8_t sum = 0;
MS5837_DATATYPE ms5837;

// UART������ɻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t data = 0;

    // USART1����PS2����
    if (huart->Instance == USART1) // ����Ƿ�ΪUSART1
    {
        static uint8_t rxState1 = 0;       /**< ����״̬ */
        static uint8_t dataLen1 = 0;       /**< ���ݳ��� */
        static uint8_t dataCnt1 = 0;       /**< ���ݼ��� */
        static uint8_t sum1 = 0;           /**< У��� */
        static uint8_t rxBuffer1[256];     /**< ���ջ����� */

        // ��ȡ���յ�������
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

            case 1: // �ȴ�ID
                rxBuffer1[1] = data;
                sum1 += data;
                if (data == 0x00)
                {
                    rxState1 = 2;
                }
                else
                {
                    rxState1 = 0; // ID��������
                }
                break;

            case 2: // �ȴ����ݳ���
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
                    rxState1 = 0; // ���ݳ�����Ч������
                }
                break;

            case 3: // ��������
                rxBuffer1[3 + dataCnt1++] = data;
                if ((dataCnt1 % 4) == 3) // ÿ4�ֽڣ�float���ݣ�����һ��
                {
                    sum1 += data;
                }
                if (--dataLen1 == 0)
                {
                    rxState1 = 4;
                }
                break;

            case 4: // �ȴ�У���
                rxBuffer1[3 + dataCnt1] = data;
                rxState1 = 0; // ����״̬
                if (sum1 == data) // У�����֤ͨ��
                {
                    // �����յ������ݽ�����PS2Data�ṹ��
                    memcpy(&ps2Data.channel1, &rxBuffer1[3], sizeof(float));
                    memcpy(&ps2Data.channel2, &rxBuffer1[7], sizeof(float));
                    memcpy(&ps2Data.channel3, &rxBuffer1[11], sizeof(float));
                    memcpy(&ps2Data.channel4, &rxBuffer1[15], sizeof(float));

                    // ������յ�������
                    printf("ch1=%.2f, ch2=%.2f, ch3=%.2f, ch4=%.2f\n", 
                           ps2Data.channel1, ps2Data.channel2, ps2Data.channel3, ps2Data.channel4);
                }
                else
                {
                    printf("USART1У��ʹ���\n");
                }
                sum1 = 0; // ����У���
                break;

            default:
                rxState1 = 0; // ����״̬
                break;
        }

        // ��������UART1�����ж�
        HAL_UART_Receive_IT(&huart1, &rxByte_USART1, 1);
    }

    // UART5����MS5837����
    else if (huart->Instance == UART5)
    {
        static uint8_t _data_len = 0, _cnt = 0;
        static uint8_t rx_state = 0;

        if (rx_state == 0 && data == 0x55) // ���֡ͷ0x55
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
