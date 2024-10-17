/*
 * motor.h
 *
 *  Created on: Apr 4, 2023
 *      Author: 77454
 */

#ifndef MOTOR_MOTOR_H_
#define MOTOR_MOTOR_H_

#include "main.h"
#include "tim.h"
void servo_left(int speed,int angle) ;
void servo_right(int speed,int angle) ; 
void servo_back(int speed,int angle) ;
float limit(float value, float min, float max);
float out_expand_fun(float roll_out,float input_min,float input_max,float out_min,float out_max);
void balance_control(void);
#if old
void control_motion(double fx, double fy, double fz);
#else
void control_motion(double fx, double fy, double fz,double speed);
#endif



void MotorControl();

#endif /* MOTOR_MOTOR_H_ */

