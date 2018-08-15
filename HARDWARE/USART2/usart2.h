#ifndef __USART2_H
#define __USART2_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

void uart2_init(u32 bound);
void UART2_Put_Char(unsigned char DataToSend);
void UART2_Put2_Char(u8 arr[2]);
void UART2_Send_Str(u8 *str);
void UART2_Clear(void);
void UART2_Send_IMUdat(u8 *dat);
void Data_Process(void);
void IMUdata_Serialization(void);

#endif


