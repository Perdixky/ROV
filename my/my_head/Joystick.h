/**
 * @struct 手柄数据结构体
 *
 * @note left_x, left_y, right_x, right_y: 左右摇杆的x和y坐标，范围为-100~100
 * @note A, B, X, Y: A、B、X、Y按钮的状态，0表示未按下，1表示按下
 * 
 * @note 数据以以下格式发送：0xAA left_x left_y right_x right_y A B X Y SUM
 */
typedef struct
{
    int left_x;
    int left_y;
    int right_x;
    int right_y;
    int A;
    int B;
    int X;
    int Y;
} JoystickDataType;

/**
 * @brief 解析手柄数据
 * 
 * @param data 每次接收到的数据
 * @param joystick_data 手柄数据结构体指针（用于存储解析后的数据）
 */
void phraseJoystickData(const unsigned char data, JoystickDataType *joystick_data);

extern JoystickDataType joystick_data;