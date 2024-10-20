/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t encoder_count_1 = 0; //编码器计数值
uint32_t encoder_count_2 = 0; //编码器计数值
uint8_t  encoder_flag = 0; //编码器是否第一次工作的标志
MYPID PID_angle;
MYPID PID_speed;

extern uint8_t buff;

extern uint16_t IIC_SCL_PIN;
extern uint16_t IIC_SDA_PIN;
extern uint8_t rx_buf[100];//串口接收缓存区
extern uint8_t buf;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_UART7_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
DT_TX_P_Init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		
		
		





//***         代码调试区      ****/////////




//while(1)
//{
////		
////////////	HAL_Delay(1000);
//	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,500);
//	
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,500);
//	
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,500);
//	
//			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,500);
//	HAL_Delay(1000);
//		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1500);
//	
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,1500);
//	
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,1500);
//	
//			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,1500);
//		HAL_Delay(1000);
//			  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1500);
//	
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,2500);
//	
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,2500);
//	
//			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,2500);
//		HAL_Delay(1000);
//}
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1350);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1400);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1450);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1500);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1550);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1600);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1650);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1700);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1750);
////	
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1500);
////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1950);
////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,2000);
////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,2100);
////		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,2200);
////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,2300);
////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,2400);

//	
////////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,1700);
////////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,1500);
////////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,1700);
////////	HAL_Delay(1000);
////  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,600);
////////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,1500);
////////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,700);
////////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,1500);
//////	
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,2400);
//////		HAL_Delay(1000);
//////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,1700);
//////		HAL_Delay(1000);
//////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,1500);
//////		HAL_Delay(1000);
//////	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,1700);
//////		HAL_Delay(1000);

//}
////		
HAL_Delay(3000);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,1500);
	
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,2500);
	
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,1500);
	
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,1500);
			HAL_Delay(1000);
			
/*****************************************************************************          线程相关             *************************************/

//printf("1");
ALIGN(RT_ALIGN_SIZE)/* 定义线程控栈时要求RT_ALIGN_SIZE个字节对齐 */

static rt_uint8_t rt_led1_thread_stack[1024];/* 定义线程栈 */

/* 定义线程控制块 */
static struct rt_thread led1_thread;
//线程1：led指示
rt_thread_init(&led1_thread,                  /* 线程控制块 */    
            "led1",                       /* 线程名字 */          
            led1_thread_entry,            /* 线程入口函数 */      
            RT_NULL,                      /* 线程入口函数参数 */  
            &rt_led1_thread_stack[0],     /* 线程栈起始地址 */    
            sizeof(rt_led1_thread_stack), /* 线程栈大小 */        
            3,                            /* 线程的优先级 */      
            20);   

 rt_thread_startup(&led1_thread);




//线程2：mpu6050
static rt_uint8_t rt_mpu6050_thread_stack[1024];/* 定义线程栈 */
static struct rt_thread mpu6050_thread;

rt_thread_init(&mpu6050_thread,                  /* 线程控制块 */    
            "mpu6050",                       /* 线程名字 */          
            mpu6050_thread_entry,            /* 线程入口函数 */      
            RT_NULL,                      /* 线程入口函数参数 */  
            &rt_mpu6050_thread_stack[0],     /* 线程栈起始地址 */    
            sizeof(rt_mpu6050_thread_stack), /* 线程栈大小 */        
            2,                            /* 线程的优先级 */      
            20);   

 rt_thread_startup(&mpu6050_thread);


//线程3：给上位机发数据
static rt_uint8_t rt_SendData_thread_stack[1024];/* 定义线程栈 */
static struct rt_thread SendData_thread;

rt_thread_init(&SendData_thread,                  /* 线程控制块 */    
            "SendData",                       /* 线程名字 */          
            SendData_thread_entry,            /* 线程入口函数 */      
            RT_NULL,                      /* 线程入口函数参数 */  
            &rt_SendData_thread_stack[0],     /* 线程栈起始地址 */    
            sizeof(rt_SendData_thread_stack), /* 线程栈大小 */        
            3,                            /* 线程的优先级 */      
            20);   

 rt_thread_startup(&SendData_thread);


//线程4：深度计
static rt_uint8_t rt_MS5837_thread_stack[1024];/* 定义线程栈 */
static struct rt_thread MS5837_thread;

rt_thread_init(&MS5837_thread,                  /* 线程控制块 */    
            "MS5837",                       /* 线程名字 */          
            MS5837_thread_entry,            /* 线程入口函数 */      
            RT_NULL,                      /* 线程入口函数参数 */  
            &rt_MS5837_thread_stack[0],     /* 线程栈起始地址 */    
            sizeof(rt_MS5837_thread_stack), /* 线程栈大小 */        
            3,                            /* 线程的优先级 */      
            20);   

// rt_thread_startup(&MS5837_thread);

//线程5:接收数据
static rt_uint8_t rt_data_receive_thread_stack[1024];/* 定义线程栈 */
static struct rt_thread data_receive_thread;

rt_thread_init(&data_receive_thread,                  /* 线程控制块 */    
            "data_receive",                       /* 线程名字 */          
            data_receive_thread_entry,            /* 线程入口函数 */      
            RT_NULL,                      /* 线程入口函数参数 */  
            &rt_data_receive_thread_stack[0],     /* 线程栈起始地址 */    
            sizeof(rt_data_receive_thread_stack), /* 线程栈大小 */        
            1,                            /* 线程的优先级 */      
            5);   

 rt_thread_startup(&data_receive_thread);



//线程6:水下超声波
static rt_uint8_t rt_Ultrasound_thread_stack[1024];/* 定义线程栈 */
static struct rt_thread Ultrasound_thread;

rt_thread_init(&Ultrasound_thread,                  /* 线程控制块 */    
            "Ultrasound",                       /* 线程名字 */          
            Ultrasound_thread_entry,            /* 线程入口函数 */      
            RT_NULL,                      /* 线程入口函数参数 */  
            &rt_Ultrasound_thread_stack[0],     /* 线程栈起始地址 */    
            sizeof(rt_Ultrasound_thread_stack), /* 线程栈大小 */        
            3,                            /* 线程的优先级 */      
            20);   

 rt_thread_startup(&Ultrasound_thread);


//线程7：ph计
static rt_uint8_t rt_PH_thread_stack[1024];/* 定义线程栈 */
static struct rt_thread PH_thread;

rt_thread_init(&PH_thread,                  /* 线程控制块 */    
            "PH",                       /* 线程名字 */          
            PH_thread_entry,            /* 线程入口函数 */      
            RT_NULL,                      /* 线程入口函数参数 */  
            &rt_PH_thread_stack[0],     /* 线程栈起始地址 */    
            sizeof(rt_PH_thread_stack), /* 线程栈大小 */        
            3,                            /* 线程的优先级 */      
            20);   

 rt_thread_startup(&PH_thread);


//线程8：水质传感器
static rt_uint8_t rt_water_quality_thread_stack[1024];/* 定义线程栈 */
static struct rt_thread water_quality_thread;

rt_thread_init(&water_quality_thread,                  /* 线程控制块 */    
            "water_quality",                       /* 线程名字 */          
            water_quality_thread_entry,            /* 线程入口函数 */      
            RT_NULL,                      /* 线程入口函数参数 */  
            &rt_water_quality_thread_stack[0],     /* 线程栈起始地址 */    
            sizeof(rt_water_quality_thread_stack), /* 线程栈大小 */        
            3,                            /* 线程的优先级 */      
            20);   

 rt_thread_startup(&water_quality_thread);

//线程9：运动控制
static rt_uint8_t rt_motion_control_thread_stack[1024];/* 定义线程栈 */
static struct rt_thread motion_control_thread;

rt_thread_init(&motion_control_thread,                  /* 线程控制块 */    
            "motion_control",                       /* 线程名字 */          
            motion_control_thread_entry,            /* 线程入口函数 */      
            RT_NULL,                      /* 线程入口函数参数 */  
            &rt_motion_control_thread_stack[0],     /* 线程栈起始地址 */    
            sizeof(rt_motion_control_thread_stack), /* 线程栈大小 */        
            2,                            /* 线程的优先级 */      
            20);   

 rt_thread_startup(&motion_control_thread);

/*************************************************************************************************************************************************/

        




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//uint8_t b=1;
//printf("b=");
//HAL_UART_Transmit(&huart1,&b,sizeof(b),0xff);
//HAL_UART_Receive_IT(&huart1, rx_buf, 50);
  rt_thread_delay(500);   /* 延时500个tick */
//printf("1");




    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
