#include "brocaster.h"
#include "common_fcn.h"

extern RB_State State,Exp_State;

void Print_IMU_Data(void){
	char int_part[7]={0},dec_part[3]={0};
	char X_pos[6]={0},Y_pos[6]={0};
	
	sprintf(X_pos,"%d",State.frame_X);
	sprintf(Y_pos,"%d",State.frame_Y);
		
	UART1_Put_String_with_space("IMU");
	
	UART1_Put_String_with_space(X_pos);
	UART1_Put_String_with_space(Y_pos);
	
	Crack_float(State.angle,int_part,dec_part);
	UART1_Put_String_with_space(int_part);
	UART1_Put_String_with_space(dec_part);
	Clear_arr(int_part,dec_part);

	Crack_float(State.frame_Vx,int_part,dec_part);
	UART1_Put_String_with_space(int_part);
	UART1_Put_String_with_space(dec_part);
	Clear_arr(int_part,dec_part);
	
	Crack_float(State.frame_Vy,int_part,dec_part);
	UART1_Put_String_with_space(int_part);
	UART1_Put_String_with_space(dec_part);
	Clear_arr(int_part,dec_part);
	
	Crack_float(State.omega,int_part,dec_part);
	UART1_Put_String_with_space(int_part);
	UART1_Put_String_with_space(dec_part);
	Clear_arr(int_part,dec_part);
	
	UART1_Print_timestamp();
}

void Crack_float(float fp,char int_part[7],char dec_part[3]){
	int dec=0;
	int int_part_var=0;
	int_part_var = (int)fp;
	dec = my_abs((fp - int_part_var)*100);
	sprintf(int_part,"%d",int_part_var);
	sprintf(dec_part,"%d",dec);
}

void Clear_arr(char int_part[7],char dec_part[3]){
	u8 i;
	for(i=0;i<7;i++)
		int_part[i]=0;
	for(i=0;i<3;i++)
		dec_part[i]=0;
}


