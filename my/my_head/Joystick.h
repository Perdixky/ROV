/**
 * @struct Joystick data structure
 *
 * @note left_x, left_y, right_x, right_y: the x and y coordinates of the left and right joysticks, the range is -100~100
 * @note A, B, X, Y: the status of the A, B, X, Y buttons, 0 is not pressed, 1 is pressed
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

JoystickDataType phraseJoystickData(unsigned char *data)
{
}