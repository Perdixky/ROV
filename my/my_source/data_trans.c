#include "data_trance.h"
   extern uint8_t sum ;
	 extern uint8_t len, head, tail, id;
	 static u8 DT_RxBuffer[256];
	 extern ps2DATATYPE PS_2;
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
void SHIP_DT_LX_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;int i=1;

	//�ж�֡ͷ�Ƿ���������Э���0xAA
	if (rxstate == 0 && data == 0xAA)
	{
		rxstate = 1;
		head = data;
		sum+=data;
	}
	//�ж�id
	else if (rxstate == 1 && (data=0x00))
	{
		rxstate = 3;
		 id = data;
	  sum+=data;
	}

//���ø�Դ���ˣ�ֱ�ӵ�3
	//�������ݳ����ֽ�
	else if (rxstate == 3 && data < 250)//len
	{
		rxstate = 4;
		len = data;
		_data_len = data;
		_data_cnt = 0;
		sum+=data;
	}
	//����������
	else if (rxstate == 4 && _data_len > 0)//float*8
	{
		_data_len--;
		DT_RxBuffer[ _data_cnt++] = data;
	
		if(_data_cnt%4==0)
		{sum+=data;}
		
		if (_data_len == 0)
			rxstate = 5;
		
	}
	//����sum
	else if (rxstate == 5)
	{
		rxstate = 6;
		i = data;
	}
	//�ж�У���Ƿ�ͨ��
	else if (rxstate == 6)
	{
		rxstate = 0;
		if(sum==i)
		{ 
			for (int k = 0; k < len; k++)
                    {
                        PS_2.byte[k] = DT_RxBuffer[ k ];
                    }
		}
		else
		{
		printf("error");
		}
//		DT_RxBuffer[4 + _data_cnt] = data;
//		DT_data_cnt = _data_cnt + 5;
//		//ano_dt_data_ok = 1;
//		ANO_DT_LX_Data_Receive_Anl(DT_RxBuffer, DT_data_cnt);
	}
	else
	{
		rxstate = 0;
	}
}