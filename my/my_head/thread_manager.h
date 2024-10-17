#include "main.h"
void led1_thread_entry(void* parameter);
void mpu6050_thread_entry(void* parameter);
void SendData_thread_entry(void* parameter);
void MS5837_thread_entry(void* parameter);
void data_receive_thread_entry(void* parameter);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Ultrasound_thread_entry(void *parameter);
void PH_thread_entry(void *parameter);
void water_quality_thread_entry(void *parameter);
void motion_control_thread_entry(void *parameter);
