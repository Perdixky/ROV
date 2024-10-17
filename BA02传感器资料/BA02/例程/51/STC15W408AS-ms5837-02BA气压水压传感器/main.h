#ifndef __MAIN_H_
#define __MAIN_H_
#include "STC15F2K60S2.h"
#include "intrins.h"

#ifndef u8
#define u8 unsigned char
#endif
#ifndef u16
#define u16 unsigned int
#endif
#ifndef u32
#define u32 unsigned long
#endif
#ifndef uchar
#define uchar unsigned char
#endif
#ifndef uint 
#define uint unsigned int
#endif
#ifndef ulong 
#define ulong unsigned long ;
#endif
void delayus(u8 i);
void delayms(u16 i);
//
void Send1(u8 i);
void MS583703BA_RESET(void);
void MS5837_init(void);
void MS583703BA_getTemperature(void);
void MS583703BA_getPressure(void);
void MS5837_IIC_Init(void);

#endif