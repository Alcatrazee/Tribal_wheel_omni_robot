#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define velocity_mode	0
#define position_mode	1
#define tracking_mode 2
#define releasing_mode 3

void UART1_Put_Char(unsigned char DataToSend);
void UART1_Put2_Char(u8 arr[2]);
void UART1_Put_String(char *p);
void UART1_Put_String_with_space(char *p);
void UART1_Put_Char_with_space(char DataToSend);
void UART1_Print_timestamp(void);
void uart_init(u32 bound);
void Process(void);
void Clear(void);

extern u8 action_mode;

#endif


