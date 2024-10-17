#include "my_adc.h"

 float volot[40];
 float v;
 int adc_num=0;

float ADC_IN_1(void) //ADC采集程序
{
	HAL_ADC_Start(&hadc1);//开始ADC采集
	HAL_ADC_PollForConversion(&hadc1,50);//等待采集结束
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))//读取ADC完成标志位
	{
		return HAL_ADC_GetValue(&hadc1)*3.0/65535;//读出ADC数值
	}
	return 0;
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

//v=ADC_IN_1();
	if(hadc == &hadc1)
	{

		volot[adc_num++]= HAL_ADC_GetValue(hadc)*3.3/4096;

if(adc_num==40)
adc_num=0;
		// 重新开启ADC中断
		HAL_ADC_Start_IT(&hadc1);
	}
}
