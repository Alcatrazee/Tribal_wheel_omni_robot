#ifndef BROCASTER__H__
#define BROCASTER__H__

#include "includes.h"
#include "sys.h"

void Print_IMU_Data(void);
void Crack_float(float fp,char int_part[7],char dec_part[3]);
void Clear_arr(char int_part[7],char dec_part[3]);
void UART1_Print_timestamp(void);

#endif



