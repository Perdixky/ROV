#include "main.h"
#include "tim.h"
#include "my_function.h"
extern uint32_t encoder_count_1;
extern uint32_t encoder_count_2;
extern uint8_t  encoder_flag;
//������pc13���ӱ�������A�࣬pc14����B�࣬ʹ�õ�������ķ�������ȡ������
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case GPIO_PIN_7:
        ;
        break;
    case GPIO_PIN_1:
        ;
        break;
    case GPIO_PIN_2:
        ;
        break;
    case GPIO_PIN_3:
        ;
        break;
    case GPIO_PIN_4:
        ;
        break;
    case GPIO_PIN_5:
        ;
        break;




    case GPIO_PIN_13:
        if (encoder_flag++ == 0) //���ڳ�ʼ��ʱ��flag��0��ȷ���Ǹոտ�ʼ���������Խ�num���㣬�����ʵҲ���Բ���
        {
            encoder_count_1 = 0;

        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == 1)
            encoder_count_1++;
        else
            encoder_count_1--;
        break;
    }
}



float v1,v2;
























