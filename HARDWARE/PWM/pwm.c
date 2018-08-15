#include "pwm.h"
#include "led.h"
#include "usart.h"
#include "includes.h"

//TIM14 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOF9复用为定时器14
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11;           //GPIOF9			for		servomotor 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=168-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1 ,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM1 , &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC2Init(TIM1 , &TIM_OCInitStructure);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);

	TIM_OC1PreloadConfig(TIM1 , TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM1 , TIM_OCPreload_Enable); 
 
  TIM_ARRPreloadConfig(TIM1 ,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM1 , ENABLE);  //使能TIM14				  
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5); //GPIOF9复用为定时器14
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM5);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;           //PWM
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PF9
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;           //DIR
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	  
	TIM_TimeBaseStructure.TIM_Prescaler=3-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=1400-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5 ,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM5 , &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 4OC1
	TIM_OC2Init(TIM5 , &TIM_OCInitStructure);
	TIM_OC3Init(TIM5 , &TIM_OCInitStructure);
	TIM_OC4Init(TIM5 , &TIM_OCInitStructure);
	
	TIM_CtrlPWMOutputs(TIM5,ENABLE);

	TIM_OC1PreloadConfig(TIM5 , TIM_OCPreload_Enable);  //使能TIM54在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM5 , TIM_OCPreload_Enable); 
	TIM_OC3PreloadConfig(TIM5 , TIM_OCPreload_Enable); 
	TIM_OC4PreloadConfig(TIM5 , TIM_OCPreload_Enable); 
 
  TIM_ARRPreloadConfig(TIM5 ,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM5 , ENABLE);  //使能TIM14			  
	
}  


// 对于电机3，CH1>CH2意味着正转
// 对于电机2，CH4>CH3意味着正转
// 对于电机1，CH1>CH2意味着正转

void motor2_pwm(u16 newPwm,u8 dir){
	if(dir==backward){
		TIM_SetCompare2(TIM5 ,newPwm);
		PCout(3)=0;
	}else if (dir==forward){
		TIM_SetCompare2(TIM5 ,1400-newPwm);
		PCout(3)=1;
	}
}

void motor4_pwm(u16 newPwm,u8 dir){
	if(dir==forward){
		TIM_SetCompare4(TIM5 ,newPwm);
		PAout(5)=0;
	}else if (dir==backward){
		TIM_SetCompare4(TIM5 ,1400-newPwm);
		PAout(5)=1;
	}
}

void motor3_pwm(u16 newPwm,u8 dir){
	if(dir==backward){
		TIM_SetCompare3(TIM5 ,1400-newPwm);
		PAout(4)=1;
	}else if (dir==forward){
		TIM_SetCompare3(TIM5 ,newPwm);
		PAout(4)=0;
	}
}

void motor1_pwm(u16 newPwm,u8 dir){
	if(dir==backward){
		TIM_SetCompare1(TIM5 ,newPwm);
		PCout(2)=0;
	}else if (dir==forward){
		TIM_SetCompare1(TIM5 ,1400-newPwm);
		PCout(2)=1;
	}
}

void Run_as_vol(float vol,u8 motornum){
	float v_temp;
	v_temp = vol;
	if(vol<0){
		v_temp = -v_temp;
	}
	switch(motornum){
		case motor1: 
					if(vol>=0)
						motor1_pwm(v_temp,forward);
					else if(vol<0)
						motor1_pwm(v_temp,backward);
		break;
		case motor2:
					if(vol>=0)
						motor2_pwm(v_temp,forward);
					else if(vol<0)
						motor2_pwm(v_temp,backward);
		break;
		case motor3:
					if(vol>=0)
						motor3_pwm(v_temp,forward);
					else if(vol<0)
						motor3_pwm(v_temp,backward);
		break;
	}
}

void stop(u8 motornum){
	switch(motornum){
		case motor1: 
			TIM5->CCR1=0;
		break;
		case motor2:
			TIM5->CCR2=0;
		break;
		case motor3:
			TIM5->CCR3=0;
		break;
		case motor4:
			TIM5->CCR4=0;
		break;
	}
}

void stop_all(void){
	stop(motor1);
	stop(motor2);
	stop(motor3);
	stop(motor4);
}

extern float dt_vol;

float motor_pid(float exp_vol,float vol,u8 motor_n){
	OS_ERR err;
	float out;
	float error_vol;
	double error_sum;
	static float error_sum1=0,error_sum2=0,error_sum3=0,former_err1=0,former_err2=0,former_err3=0;
	float A;
	float Kp,Ki,Kd;
	float Kp1 = 0.00095,Ki1 = 0.0000001,Kd1 = 0.000030000;
	float Kp2 = 0.00095,Ki2 = 0.0000001,Kd2 = 0.000030000;
	float Kp3 = 0.00095,Ki3 = 0.0000001,Kd3 = 0.000030000;
	float dt=0;
	//motor1 dt
	float time_now_1=0;
	static float time_last_1=0;
	//motor2 dt
	float time_now_2=0;
	static float time_last_2=0;
	//motor3 dt
	float time_now_3=0;
	static float time_last_3=0;
	
	if(exp_vol<100){
		error_sum1 = 0;
		error_sum2 = 0;
		error_sum3 = 0;
	}

	error_vol = exp_vol - vol;
	
	switch(motor_n){
		case 1: 
			time_now_1 = 5*(float)OSTimeGet(&err)/1000;
			dt = time_now_1 - time_last_1;
			time_last_1 = time_now_1;
			A = (error_vol-former_err1)/dt;	
			Kp = Kp1;	
			Ki = Ki1; 
			Kd = Kd1;	
			error_sum = error_sum1;	
			break;
		case 2: 
			time_now_2 = 5*(float)OSTimeGet(&err)/1000;
			dt = time_now_2 - time_last_2;
			time_last_2 = time_now_2;
			A = (error_vol-former_err2)/dt;	
			Kp = Kp2;	
			Ki = Ki2; 
			Kd = Kd2;	
			error_sum = error_sum2;	
			break;
		case 3: 
			time_now_3 = 5*(float)OSTimeGet(&err)/1000;
			dt = time_now_3 - time_last_3;
			time_last_3 = time_now_3;
			A = (error_vol-former_err3)/dt;	
			Kp = Kp3;	
			Ki = Ki3; 
			Kd = Kd3;	
			error_sum = error_sum3;	
		break;
	}
	
	error_sum+=error_vol;
	out = Kp*error_vol+Ki*error_sum+Kd*A;		//Kd需要除以一个时间dt   Kd*A/dt
	
	switch(motor_n){
		case 1: error_sum1 = error_sum; former_err1 = error_vol; break;
		case 2: error_sum2 = error_sum; former_err2 = error_vol; break;
		case 3: error_sum3 = error_sum; former_err3 = error_vol; break;
	}
	if(out>1400)
		out = 1400;
	else if(out<-1400)
		out = -1400;
	return out;
}



