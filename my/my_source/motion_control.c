#include "motion_control.h"
#include "mpu6050.h"
#include "main.h"
#include "math.h"

//电机输出映射
//pwm_to_force[]=
//{0.1,0.15,0.15,0.17,0.19,0.21}




//船体常量
int l1 = 110, l3 = 220; //前后电机到重心距离,l1是前，l2是后
double boat_weight = 2000; //船体重量
double fx, fy, fz;
double f1, f2, f3;
double angle1, angle2, angle3 = 30, angle4; //因为方程个数少于未知数，令角度3为定值

double pwm_1 = 0, pwm_2 = 0, pwm_3 = 0;
extern float pitch_1, roll_1;
uint8_t rotate_flag;//FLAG用于标示是否使用船壳旋转，0标示不用，1标示用

extern  TIM_HandleTypeDef htim2;
extern  TIM_HandleTypeDef htim3;

//一些简化操作
void servo_left(int speed, int angle) //
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, angle); //每一度对应0.135个us
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
}
void servo_right(int speed, int angle)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angle);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}
void servo_back(int speed, int angle)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, angle);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
}





// 限幅函数
float limit(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}

//输出膨胀函数 把角度偏移量转换为电机输出
float out_expand_fun(float roll_out, float input_min, float input_max, float out_min, float out_max)
{

    float expand = (out_max - out_min) / (input_max - input_min);
    roll_out = (roll_out - input_min) * expand + out_min;
    return roll_out;
}
//pitch_1姿态控制    积分控制
float pitch_target = 0, roll_target = 0;
float pitch_ki = 0.01, pitch_kp = 1.0;
float roll_ki = 0.01, roll_kp = 0.2;
float roll_integral, roll_out; //roll_1方向的偏移量
float pitch_integral, pitch_out; //roll_1方向的偏移量
float f1_balancez, f2_balancez, f3_balancez, f1_upz, f2_upz, f3_upz, f1_z, f2_z, f3_z, fx_judge;

void balance_control()
{

    if ((pitch_1 > 5) || (pitch_1 < -5))
    {
//pitch_1的积分控制
        pitch_integral += pitch_ki * (pitch_1 - pitch_target);
        pitch_out = limit(pitch_integral, -0.5, 0.5) + pitch_kp * (pitch_1 - pitch_target);
        pitch_out = limit(pitch_out, -5, 5);
//    f3_balancez = -pitch_out;
        f3_balancez = out_expand_fun(pitch_out, -5, 5, -200, 200); //由于在后面，力臂大一些，所以输出少一些
    }
    else
    {
        f3_balancez = 0;


    }

    if ((roll_1 > 15) || (roll_1 < -15))
    {
//roll_1的积分控制
        roll_integral += roll_ki * (roll_1 - roll_target);
        roll_out = limit(roll_integral, -1, 1) + roll_kp * (roll_1 - roll_target);
        roll_out = limit(roll_out, -5, 5);
//    printf("pitch_int:%.1f    roll_int:%.1f\n", pitch_integral, roll_integral);
//把roll_1的偏移量膨胀到1250-1350/1650-1750之间输出，推力大致介于0.25kg-0.65kg

        f1_balancez = out_expand_fun(roll_out, -5, 5, -100, 100);;
        f2_balancez = out_expand_fun(-roll_out, -5, 5, -100, 100);;


    }
    else
    {
        f1_balancez = 0;
        f2_balancez = 0;

    }
}


float k3 = 1; //用于在平衡的电机输出比例上等比例扩大，以实现上浮
float f3_x = 0, f1_x = 0, f2_x = 0, fx_judje, fx_left_x, fx_qian; //多余的自由度变量，固定这个变量是因为后面舵机动一动好看
double a1, a2;
float k = 0.5;

float fx_abs, fy_abs, fz_abs, f_sum;
float fx_ratio, fy_ratio, fz_ratio;

//平动运动控制函数
#if old
void control_motion(double fx, double fy, double fz)
{
    /*老一代算法    */
//    balance_control();


    f1_upz = out_expand_fun(fz, 0, 2, -800 * k, 800 * k);
    f2_upz = out_expand_fun(fz, 0, 2, -800 * k, 800 * k);
//  f3_upz=1.5*out_expand_fun(fz,0,2,-450,450);


////前后控制
    fx_judge = out_expand_fun(fx, 0, 2, -1000 * k, 1000 * k);
    if (fx_judge > 250 * k)

    {
        f3_x = fx_judge - 250 * k;
    }
    else if ((fx_judge < -100 * k) && (fx_judge > -975 * k))
    {
        f3_x = fx_judge + 100 * k;

    }



    else
    {
        f3_x = 10;
    }
    fx_qian = out_expand_fun(fx, 0, 2, -200 * k, 200 * k);
    f1_x = out_expand_fun(fx, 0, 2, -200 * k, 200 * k);
    f2_x = f1_x;

//左右控制,先不测
    fx_left_x = out_expand_fun(fy, 0, 2, -350 * k, 350 * k);
//f1_x=fx_qian+fx_left_x;
//f2_x=fx_qian-fx_left_x;

//叠加
    f1_z = f1_balancez + f1_upz;
    f2_z = f2_balancez + f2_upz;
    f3_z = f3_balancez + f3_upz;

    a1 = atan((double)(f1_z / f1_x));
    a2 = atan((double)(f2_z / f2_x));
    angle1 = 2.35619 - atan((double)(f1_z / f1_x));
//      angle1 = atan((double)(f1_z / f1_x));
//      angle1 = 4.71-(atan((double)(f1_z / f1_x))+2.35);

//      if((fx>1)&&(fy>1))
//  {
//    f1 = 1510 + sqrt((double)(f1_x * f1_x + f1_z * f1_z))*(fx-1)/my_abs(fx-1)+fx_left_x;
//      f2 = 1510 + sqrt((double)(f2_x * f2_x + f2_z * f2_z))*(fx-1)/my_abs(fx-1)-fx_left_x;
//  }
//  else        if((fx>1)&&(fy<1))
//  {
//    f1 = 1510 + sqrt((double)(f1_x * f1_x + f1_z * f1_z))*(fx-1)/my_abs(fx-1)-fx_left_x;
//      f2 = 1510 + sqrt((double)(f2_x * f2_x + f2_z * f2_z))*(fx-1)/my_abs(fx-1)+fx_left_x;
//  }
//  else        if((fx<1)&&(fy>1))
//  {
//    f1 = 1510 - sqrt((double)(f1_x * f1_x + f1_z * f1_z))*(fx-1)/my_abs(fx-1)+fx_left_x;
//      f2 = 1510 - sqrt((double)(f2_x * f2_x + f2_z * f2_z))*(fx-1)/my_abs(fx-1)-fx_left_x;
//  }
//  else        if((fx<1)&&(fy<1))
//  {
//    f1 = 1510 - sqrt((double)(f1_x * f1_x + f1_z * f1_z))*(fx-1)/my_abs(fx-1)-fx_left_x;
//      f2 = 1510 - sqrt((double)(f2_x * f2_x + f2_z * f2_z))*(fx-1)/my_abs(fx-1)+fx_left_x;
//  }




//
    if (fy > 0)
    {
        f1 = 1510 + sqrt((double)(f1_x * f1_x + f1_z * f1_z)) * (fx - 1) / my_abs(fx - 1) + fx_left_x;
        f2 = 1510 + sqrt((double)(f2_x * f2_x + f2_z * f2_z)) * (fx - 1) / my_abs(fx - 1) - fx_left_x;
    }
    else
    {
        f1 = 1510 - sqrt((double)(f1_x * f1_x + f1_z * f1_z)) * (fx - 1) / my_abs(fx - 1) - fx_left_x;
        f2 = 1510 - sqrt((double)(f2_x * f2_x + f2_z * f2_z)) * (fx - 1) / my_abs(fx - 1) + fx_left_x;;
    }
//   f1 = 1510 + sqrt((double)(f1_x * f1_x + f1_z * f1_z))*(fx-1)/my_abs(fx-1);
    angle2 = 4.712389 - (2.35619 - atan((double)(f2_z / f2_x)));
//      angle2 = (atan((double)(f2_z / f2_x))+2.35);



//    f2 = 1510 + sqrt((double)(f2_x * f2_x + f2_z * f2_z))*(fx-1)/my_abs(fx-1);
    angle3 = -atan((double)(f3_z / f3_x));

    if (f3_x > 0)
    {
        f3 = 1500 + sqrt((double)(f3_x * f3_x + f3_z * f3_z));
    }
    else
    {
        f3 = 1500 - sqrt((double)(f3_x * f3_x + f3_z * f3_z));
    }
//    pwm_1 = out_expand_fun(angle1, -2.35, 2.35, 500, 2500);
//    pwm_2 = out_expand_fun(angle2, -2.35, 2.35, 500, 2500);
    pwm_1 = out_expand_fun(angle1, 0, 4.71, 500, 2500);
    pwm_2 = out_expand_fun(angle2, 0, 4.71, 500, 2500);
    pwm_3 = out_expand_fun(angle3, -1.6, 1.6, 500, 2500);
    servo_right(f2, pwm_2);
    servo_left(f1, pwm_1);
    printf("f1=%f,pwm1=%f,angle1=%f\n", f1, pwm_1 - 1500, angle1 / 3.14);

    servo_back(f3, pwm_3);


}
#else
void control_motion(double fx, double fy, double fz, double speed)
{

    /**  第四套算法，比例自动分配法  **/
    fx_abs = my_abs(fx);
    fy_abs = my_abs(fy);
    fz_abs = my_abs(fz);

    if (fx_abs < 0.02) //防止过渡状态混乱加的死区
        fx_abs = 0;
    f_sum = fx_abs + fy_abs + fz_abs;

    if ((f_sum) < 0.03)
    {
        fx_ratio = fy_ratio = fz_ratio = 0;
    }
    else
    {
        fx_ratio = fx / f_sum;
        fy_ratio = fy / f_sum;
        fz_ratio = fz / f_sum;
    }
    if (fx_ratio > 0) //x大于0就正推
    {
        f1 = 1500 + 500 * speed;
        f2 = 1500 + 500 * speed;
        f3 = 1500 + 200 * speed;
    }
    else
    {
        f1 = 1500 - 500 * speed;
        f2 = 1500 - 500 * speed;
        f3 = 1500 - 200 * speed;
    }


//if(fy_ratio>0)
//{
    f1 = f1 + 450 * fy_ratio;
    f2 = f2 - 450 * fy_ratio;
//}
//else
//{
//f1=f1+100*fy_ratio;
//f2=f2-100*fy_ratio;
//}


    angle1 = 2.35619 - atan((double)(fz_ratio / (fx_ratio + 0.01)));
    angle2 = 4.712389 - (2.35619 - atan((double)(fz_ratio / (0.01 + fx_ratio))));


//    angle3 = -atan((double)(fz_ratio / fx_ratio));
//angle3=angle1;

    pwm_1 = out_expand_fun(angle1, 0, 4.71, 500, 2500);
    pwm_2 = out_expand_fun(angle2, 0, 4.71, 500, 2500);
//    pwm_3 = out_expand_fun(angle3, -1.6, 1.6, 500, 2500);
//pwm_3=pwm_1;
    servo_right(f2, pwm_2);
    servo_left(f1-25, pwm_1);
    printf("f1=%f,pwm1=%f,angle1=%f\n", f1, pwm_1 - 1500, angle1 / 3.14);

    servo_back(f3, pwm_1);






}

#endif



















