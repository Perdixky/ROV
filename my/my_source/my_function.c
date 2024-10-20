# include "stdio.h"
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "my_function.h"
#include "mpu6050.h"
#include "math.h"
#include <stdlib.h>



extern MPU6050DATATYPE mpu6050;
extern soundDATATYPE Ultrasound;
extern phDATATYPE ph;
extern MS5837_DATATYPE ms5837;
extern JY901B_DataType jy901b;
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}




float my_abs(float a)
{
    return a > 0 ? a : -a;
}















//假动态数组,用来快速锁定不同数据帧的大小
//以后需要加入新的通信帧，只需要
//1.在这里加入它的数据量，
//2.在add函数里面添加数据内容即可
//3. Send_Data_Task()里面添加相应的send函数即可
u8 DT_TX_Buffer_SIZE[] =
{
    0,    //0x00
    16 + 4 //0x01
    , 4+4 //0x02
    , 8+4  //0x03
	,8+4//0x04
	,4+6//0x05
};

u8 *p[sizeof(DT_TX_Buffer_SIZE)];
void DT_TX_P_Init()
{
    for (int i=0; i < sizeof(DT_TX_Buffer_SIZE); i++)
    {
        p[i] = (u8 *) malloc((int)DT_TX_Buffer_SIZE[i]); //定义一堆数组作为数据帧
    }
}



//通信程序
static u8 DT_RxBuffer[256], DT_data_cnt = 0;
void ANO_DT_LX_Data_Receive_Prepare(u8 data)
{
    static u8 _data_len = 0, _data_cnt = 0;
    static u8 rxstate = 0;

    //判断帧头是否满足匿名协议的0xAA
    if (rxstate == 0 && data == 0xAA)
    {
        rxstate = 1;
        DT_RxBuffer[0] = data;
    }


    //接收帧CMD字节
    else if (rxstate == 1)
    {
        rxstate = 2;
        DT_RxBuffer[2] = data;
    }
    //接收数据长度字节
    else if (rxstate == 2 && data < 250)
    {
        rxstate = 3;
        DT_RxBuffer[3] = data;
        _data_len = data;
        _data_cnt = 0;
    }
    //接收数据区
    else if (rxstate == 3 && _data_len > 0)
    {
        _data_len--;
        DT_RxBuffer[3 + _data_cnt++] = data;
        if (_data_len == 0)
            rxstate = 4;
    }

    //接收校验字节，表示一帧数据接收完毕，调用数据解析函数
    else if (rxstate == 4)
    {
        rxstate = 5;
        DT_RxBuffer[3 + _data_cnt] = data;
        DT_data_cnt = _data_cnt + 5;
        //ano_dt_data_ok = 1;
        ANO_DT_LX_Data_Receive_Anl(DT_RxBuffer, DT_data_cnt);
    }
    else
    {
        rxstate = 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数ANO_Data_Receive_Prepare自动调用
static void ANO_DT_LX_Data_Receive_Anl(u8 *data, u8 len)
{
    u8 check_sum1 = 0;
    //判断数据长度是否正确
    if (*(data + 3) != (len - 6))
        return;
    //根据收到的数据计算校验字节1和2
    for (u8 i = 0; i < len - 2; i++)
    {
        check_sum1 += *(data + i);

    }
    //计算出的校验字节和收到的校验字节做对比，完全一致代表本帧数据合法，不一致则跳出解析函数
    if ((check_sum1 != *(data + len - 2)))  //判断sum校验
        return;


    //=============================================================================
    //根据帧的CMD，也就是第3字节，进行对应数据的解析
    //PWM数据

    switch (*(data + 1))
    {
    case 0x20:
        ;
    }


    /*
    if (*(data + 2) == 0X20)
    {
        pwm_to_esc.pwm_m1 = *((u16 *)(data + 4));
        pwm_to_esc.pwm_m2 = *((u16 *)(data + 6));
        pwm_to_esc.pwm_m3 = *((u16 *)(data + 8));
        pwm_to_esc.pwm_m4 = *((u16 *)(data + 10));
        pwm_to_esc.pwm_m5 = *((u16 *)(data + 12));
        pwm_to_esc.pwm_m6 = *((u16 *)(data + 14));
        pwm_to_esc.pwm_m7 = *((u16 *)(data + 16));
        pwm_to_esc.pwm_m8 = *((u16 *)(data + 18));
    }
    //凌霄IMU发出的RGB灯光数据
    else if (*(data + 2) == 0X0f)
    {
        led.brightness[0] = *(data + 4);
        led.brightness[1] = *(data + 5);
        led.brightness[2] = *(data + 6);
        led.brightness[3] = *(data + 7);
    }
    //凌霄飞控当前的运行状态
    else if (*(data + 2) == 0X06)
    {
        fc_sta.fc_mode_sta = *(data + 4);
        fc_sta.unlock_sta = *(data + 5);
        fc_sta.cmd_fun.CID = *(data + 6);
        fc_sta.cmd_fun.CMD_0 = *(data + 7);
        fc_sta.cmd_fun.CMD_1 = *(data + 8);
    }
    //飞行速度
    else if (*(data + 2) == 0X07)
    {
        for(u8 i=0;i<6;i++)
        {
            fc_vel.byte_data[i] = *(data + 4 + i);
        }
    }
    //姿态角（需要在上位机凌霄IMU界面配置输出功能）
    else if (*(data + 2) == 0X03)
    {
        for(u8 i=0;i<7;i++)
        {
            fc_att.byte_data[i] = *(data + 4 + i);
        }
    }
    //姿态四元数
    else if (*(data + 2) == 0X03)
    {
        for(u8 i=0;i<9;i++)
        {
            fc_att_qua.byte_data[i] = *(data + 4 + i);
        }
    }
    //传感器数据
    else if (*(data + 2) == 0X01)
    {

        acc_x = *((s16 *)(data + 4));
        acc_y = *((s16 *)(data + 6));
        acc_z = *((s16 *)(data + 8));
        gyr_x = *((s16 *)(data + 10));
        gyr_y = *((s16 *)(data + 12));
        gyr_z = *((s16 *)(data + 14));
        state = *(data + 16);

    }
    //命令E0，具体命令格式及功能，参见匿名通信协议V7版
    else if (*(data + 2) == 0XE0)
    {
        //根据命令ID：(*(data + 4)) ，来执行不同的命令
        switch (*(data + 4))
        {
        case 0x01:
        {
        }
        break;
        case 0x02:
        {
        }
        break;
        case 0x10:
        {
        }
        break;
        case 0x11:
        {
        }
        break;
        default:
            break;
        }
        //收到命令后，需要返回对应的应答信息，也就是CK_Back函数
        dt.ck_send.ID = *(data + 4);
        dt.ck_send.SC = check_sum1;
        dt.ck_send.AC = check_sum2;
        CK_Back(SWJ_ADDR, &dt.ck_send);
    }
    //收到的是ck返回
    else if (*(data + 2) == 0X00)
    {
        //判断收到的CK信息和发送的CK信息是否相等
        if ((dt.ck_back.ID == *(data + 4)) && (dt.ck_back.SC == *(data + 5)) && (dt.ck_back.AC == *(data + 6)))
        {
            //校验成功
            dt.wait_ck = 0;
        }
    }
    //读取参数
    else if (*(data + 2) == 0XE1)
    {
        //获取需要读取的参数的id
        u16 _par = *(data + 4) + *(data + 5) * 256;
        dt.par_data.par_id = _par;
        dt.par_data.par_val = 0;
        //发送该参数
        PAR_Back(0xff, &dt.par_data);
    }
    //写入参数
    else if (*(data + 2) == 0xE2)
    {
        //目前凌霄开源MCU不涉及参数的写入，推荐大家直接使用源码方式调整自己定义的参数，故此处只返回对应的CK校验信息
        //      u16 _par = *(data+4)+*(data+5)*256;
        //      u32 _val = (s32)(((*(data+6))) + ((*(data+7))<<8) + ((*(data+8))<<16) + ((*(data+9))<<24));
        //
        dt.ck_send.ID = *(data + 4);
        dt.ck_send.SC = check_sum1;
        dt.ck_send.AC = check_sum2;
        CK_Back(0xff, &dt.ck_send);
        //赋值参数
        //Parameter_Set(_par,_val);
    }
    */
}

//===================================================================
//通信帧填充函数
//===================================================================
static void Add_Send_Data(u8 ID, u8 Tx_buffer[])
{
    s16 temp_data;
    s32 temp_data_32;

    u8 len, num = 0; //num用来指示位置
    //根据需要发送的帧ID，也就是frame_num，来填充数据，填充到send_buffer数组内
    int sum = 0;
    Tx_buffer[0] = 0xAA;
    sum += Tx_buffer[num++];
    Tx_buffer[1] = ID;
    sum += Tx_buffer[num++];

    switch (ID)
    {
    case 0x01:
        num++;
        for (len = 0; len < 16; len++)
        {
            Tx_buffer[num++] = mpu6050.byte[len];
            if ((len + 1) % 4 == 0)
                sum += mpu6050.byte[len];
//printf("sum=%d",sum);
        }
        break;

    case 0x02:
        num++;
        for (len = 0; len < 4; len++)
        {
            Tx_buffer[num++] = Ultrasound.byte[len];
            if ((len + 1) % 4 == 0)
                sum += Ultrasound.byte[len];
//printf("sum=%d",sum);
        }
        break;


    case 0x03:
        num++;
        for (len = 0; len < 8; len++)
        {
            Tx_buffer[num++] = ph.byte[len];
            if ((len + 1) % 4 == 0)
                sum += ph.byte[len];
//printf("sum=%d",sum);
        }
        break;
			
		    case 0x04:
        num++;
        for (len = 0; len < 8; len++)
        {
            Tx_buffer[num++] = ms5837.byte[len];
            if ((len + 1) % 4 == 0)
                sum += ms5837.byte[len];
//printf("sum=%d",sum);
        }
        break;
				 case 0x05://0x05就是jy901b的数据
        num++;
        for (len = 0; len < 5; len++)
        {
            Tx_buffer[num++] = jy901b.byte[len];
            if ((len + 1) % 2 == 0)
                sum += jy901b.byte[len];
//printf("sum=%d",sum);
        }
        break;

    }
    Tx_buffer[2] = len;
    sum += len;
    Tx_buffer[num] = sum;

}

//===================================================================
//单个通信帧发送函数
//===================================================================
void Send_Data(u8 ID)
{
//int i;
//for(i=0;i<256;i++)


    Add_Send_Data(ID, p[ID]);
   int a= HAL_UART_Transmit(&huart1, p[ID], DT_TX_Buffer_SIZE[ID], 100);
printf("1");


}
//通信帧发送任务
void Send_Data_Task()
{
    Send_Data(0x01);
    Send_Data(0x02);
    Send_Data(0x03);
    Send_Data(0x04);
		Send_Data(0x05);
//		double x[3]={((jy901b.byte[1]<<8)|jy901b.byte[0])/32768*180,((jy901b.byte[3]<<8)|jy901b.byte[2])/32768*180,((jy901b.byte[5]<<8)|jy901b.byte[4])/32768*180};
//		   int a= HAL_UART_Transmit(&huart1, x, 6, 100);


}





