#ifndef __LED_H
#define __LED_H
#include "sys.h"

//LED�˿ڶ���
#define LED0 PBout(9)	// DS0

void LED_Init(void);//��ʼ��		 		
void led_turn(void);
void led_on(void);
void led_off(void);
#endif
