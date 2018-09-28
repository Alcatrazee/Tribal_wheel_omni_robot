#ifndef COMMON_FCN__H__
#define COMMON_FCN__H__

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "exti.h"
#include "timer.h"
#include "math.h"
#include "usart2.h"
#include "pwm.h"
#include "usart3.h"
#include "ctrl.h"

// this file is used to store some structures and common functions

float my_abs(float num);
u8 strcmp_real(char str1[3],char str2[3]);
float rad2deg(float rad);
float deg2rad(float deg);
void Transformation_from_global2robot(float target[2],float result[2]);

typedef struct
{
	short   frame_X;						//����Xλ��	 ��λ������
	short 	frame_Y;						//����Yλ��	 ��λ������	
	float 	frame_Vx;
	float 	frame_Vy;
	float 	omega;
	float 	angle;							//����Ƕ�	 ��λ����								
	float   frame_ax;
	float   frame_ay;
	float   omega_wheel[3];
	float   angle_offset;
}RB_State;



































#endif



