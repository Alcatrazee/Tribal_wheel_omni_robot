#include "common_fcn.h"
#include "includes.h"

// global varaibles declaration

RB_State State={0,0,0,0,0,0,0,0};
RB_State Exp_State={0,0,0,0,0,0,0,0};

long steps1=0,steps2=0,steps3=0,steps4=0,steps_L=0,steps_R=0;

//common functions definition

float my_abs(float num){
	if(num<0)
			num=-num;
	return num;
}

float deg2rad(float deg){
	return deg*3.14159f/180;
}

float rad2deg(float rad){
	return rad*180/3.14159f;
}

u8 strcmp_real(char str1[3],char str2[3]){
	u16 sum1=0,sum2=0;
	
	sum1 = str1[0]+str1[1]+str1[2];
	sum2 = str2[0]+str2[1]+str2[2];
	if(sum1==sum2)
		return 0;
	else 
		return 1;
}

void Transformation_from_global2robot(float target[2],float result[2]){
	result[0] = cos(deg2rad(State.angle))*target[0]+sin(deg2rad(State.angle))*target[1]-State.frame_X;
	result[1] = -sin(deg2rad(State.angle))*target[0]+cos(deg2rad(State.angle))*target[1]-State.frame_Y;
}



