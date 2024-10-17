#include "main.h"
#include "tim.h"
#include "my_function.h"
extern uint32_t encoder_count_1;
extern uint32_t encoder_count_2;
extern uint8_t  encoder_flag;
//假设是pc13连接编码器的A相，pc14连接B相，使用单纯捕获的方法来读取编码器
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
        if (encoder_flag++ == 0) //由于初始化时候flag是0，确保是刚刚开始启动，所以将num置零，这句其实也可以不加
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
























