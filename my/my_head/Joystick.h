/**
 * @struct �ֱ����ݽṹ��
 *
 * @note left_x, left_y, right_x, right_y: ����ҡ�˵�x��y���꣬��ΧΪ-100~100
 * @note A, B, X, Y: A��B��X��Y��ť��״̬��0��ʾδ���£�1��ʾ����
 * 
 * @note ���������¸�ʽ���ͣ�0xAA left_x left_y right_x right_y A B X Y SUM
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
 * @brief �����ֱ�����
 * 
 * @param data ÿ�ν��յ�������
 * @param joystick_data �ֱ����ݽṹ��ָ�루���ڴ洢����������ݣ�
 */
void phraseJoystickData(const unsigned char data, JoystickDataType *joystick_data);

extern JoystickDataType joystick_data;