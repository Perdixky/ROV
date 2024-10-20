#include "Joystick.h"
JoystickDataType joystick_data;

void phraseJoystickData(const unsigned char data, JoystickDataType *joystick_data)
{
    static unsigned char status = 0;
    if (data == 0xAA)  // 无论何时，只要接收到0xAA，就说明数据开始
    {
        status = 1;
    }
    if (status == 1)
    {
        joystick_data->left_x = data;
        status = 2;
    }
    else if (status == 2)
    {
        joystick_data->left_y = data;
        status = 3;
    }
    else if (status == 3)
    {
        joystick_data->right_x = data;
        status = 4;
    }
    else if (status == 4)
    {
        joystick_data->right_y = data;
        status = 5;
    }
    else if (status == 5)
    {
        joystick_data->A = data;
        status = 6;
    }
    else if (status == 6)
    {
        joystick_data->B = data;
        status = 7;
    }
    else if (status == 7)
    {
        joystick_data->X = data;
        status = 8;
    }
    else if (status == 8)
    {
        joystick_data->Y = data;
        status = 9;
    }
    else if (status == 9)
    {
        if (data == joystick_data->left_x + joystick_data->left_y + joystick_data->right_x + joystick_data->right_y
            + joystick_data->A + joystick_data->B + joystick_data->X + joystick_data->Y)  // 校验和
        {
            status = 0;
        }
        else
        {
            status = 0;
        }
    }
}