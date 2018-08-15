#ifndef __USART3__H__
#define __USART3__H__
#include "sys.h"
#include "stdio.h"	
#include "stm32f4xx_conf.h"

void uart3_init(u32 bouderrate);
void USART3_Process(void);
void USART3_Clear_Buff(void);
void UART3_Put_Char(unsigned char DataToSend);

#endif
