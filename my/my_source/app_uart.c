#include "app_uart.h"
#include "board.h"
void test_thread_entry(void* parameter)
{    
    rt_uint8_t uart_rx_data;
    /* �򿪴��� */
    if (uart_open("uart2") != RT_EOK)
    {
        rt_kprintf("uart open error.\n");
         while (1)
         {
            rt_thread_delay(10);
         }
    }
    /* �����ַ�д */
    uart_putchar('2');
    uart_putchar('0');
    uart_putchar('1');
    uart_putchar('8');
    uart_putchar('\n');
    /* д�ַ��� */
    uart_putstring("Hello RT-Thread!\r\n");
    while (1)
    {   
        /* ������ */
        uart_rx_data = uart_getchar();
        /* ��λ */
        uart_rx_data = uart_rx_data + 1;
        /* ��� */
        uart_putchar(uart_rx_data);
    }            
}

int main(void)
{
    rt_thread_t tid; 
    /* ����test�߳� */
    tid = rt_thread_create("test",
                    test_thread_entry, 
                    RT_NULL,
                    1024, 
                    2, 
                    10);
    /* �����ɹ��������߳� */
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    return 0;
}