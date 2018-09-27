#include "ctrl.h"
#include "common_fcn.h"

#define Axis_x 0
#define Axis_y 1

float Kp_x,Ki_x,Kd_x;																						//parameter needed to be made
short exp_x=0,exp_y=0;																					//position expected
float linear_vxy[3]={0};																				//linear velocity of the robot,calculated by the expected velocity,not the current velocity
u8 action_mode=position_mode;

extern RB_State State;
extern RB_State Exp_State;
extern long steps1,steps2,steps3,steps_L,steps_R;

#define max_pwm 1400
#define min_pwm -max_pwm
#define reduction 78
#define pulse_per_round 360
#define wheel_radius 29
#define pi 3.14159f
// formular: ppr*redu*velocity/(2*pi*radius)
void Move(float linear_v[4]){
		static float temp1,temp2,temp3;
		float temp1_1,temp2_2,temp3_3;
		float omega1,omega2,omega3;

		omega1 = pulse_per_round*reduction*linear_v[0]/(2*pi*wheel_radius);						//calculatae the pulse per second
		omega2 = pulse_per_round*reduction*linear_v[1]/(2*pi*wheel_radius);
		omega3 = pulse_per_round*reduction*linear_v[2]/(2*pi*wheel_radius);
		
		temp1_1 = motor_pid(omega1,State.omega_wheel[0],motor1);
		temp1+=temp1_1;
		if(omega1==0)
			temp1 = 0;
		if(temp1>max_pwm)
			temp1=max_pwm;
		else if (temp1<min_pwm)
			temp1=min_pwm;
		
		
		temp2_2 = motor_pid(omega2,State.omega_wheel[1],motor2);
		temp2+=temp2_2;
		if(omega2==0)
			temp2 = 0;
		if(temp2>max_pwm)
			temp2=max_pwm;
		else if (temp2<min_pwm)
			temp2=min_pwm;
		
		
		temp3_3 = motor_pid(omega3,State.omega_wheel[2],motor3);
		temp3+=temp3_3;
		if(omega3==0)
			temp3 = 0;
		if(temp3>max_pwm)
			temp3=max_pwm;
		else if (temp3<min_pwm)
			temp3=min_pwm;

		Run_as_vol(temp1,motor1);
		Run_as_vol(temp2,motor2);
		Run_as_vol(temp3,motor3);
}

//					motor2
//
//
//
// motor1             motor3
#define VX_VALUE           (0.5f)
#define VY_VALUE           (sqrt(3)/2.f)      
#define L_value            (12*0.01f)  
#define RADIUS_value       (1.0/12.5*0.01f)
#define max_output 650
void Speed_Moto_Control(float linear_xyz[3],float linear_v[4])
{ 
	 float k=1;
	 u8 i=0;
   linear_v[0] = (-VX_VALUE*linear_xyz[0] + VY_VALUE*linear_xyz[1] + L_value*linear_xyz[2]*2.f);
   linear_v[2] = (-VX_VALUE*linear_xyz[0] - VY_VALUE*linear_xyz[1] + L_value*linear_xyz[2]*2.f);
   linear_v[1] = (linear_xyz[0] + L_value*linear_xyz[2]*2.f);
	 // limit the max speed within [0,600]
	 if(my_abs(linear_v[0])>=max_output){
		 k=linear_v[0]/max_output;
	 }else if(my_abs(linear_v[1])>=max_output){
		 k=linear_v[1]/max_output;
	 }else if(my_abs(linear_v[2])>=max_output){
		 k=linear_v[2]/max_output;
	 }
	 if(k<0)																// use my_abs(k)
		 k=-k;
	 for(i=0;i<3;i++)
		linear_v[i]/=k;
}

#define max_out 800
float Angle_PID(float exp_angle){
	OS_ERR err;
	float 	Kp_a = 70.000,
					Ki_a = 00.0000000,
					Kd_a = 6.10000;
	float error_sum=0,error,d_error=0;
	static float error_f;
	float out;
	float dt = 0;		
	float time_now=0;
	static float time_last=0;
//get dt
	time_now = 5*(float)OSTimeGet(&err)/1000;
	dt = time_now - time_last;
	time_last = time_now;

	
	error = exp_angle - State.angle;
	error_sum+=error;
	
	d_error = error - error_f;
	
	out = Kp_a*error + Ki_a*error_sum + Kd_a*d_error/dt;
	
	
	if(out>=max_out)
		out = max_out;
	else if(out<-max_out)
		out = -max_out;
	
	error_f = error;
	
	
	return -out;
}

//	the whole algrithm can be rewrite,  1. get theta3 first,theta3 is the diretion of the movemont on the global frame,the we can do whatever we want,it might save some time,just might...[
//	output : mm
#define radius 38.0f
void Cult_pos(int steps_delta[2],float pos[2],float theta2){
	OS_ERR err;
	float distance = 0.0f;
	float delta_y = 0.0,delta_x = 0.0;
	static float former_x=0,former_y=0;
	float theta1;
	float WLX,WLY,WRX,WRY;
	float dis_L,dis_R;
	float combine_vector[2];															// distance of movement of robot's in it's own frame
	static float linear_former_vxy[2]={0,0};								// to store the former value of velocity, just used to calculate the accelartion
	static float pos_temp[2]={0,0};
	float v_x=0,v_y=0;
	
	float dt = 0;		
	float time_now=0;
	static float time_last=0;

//get dt
	time_now = 5*(float)OSTimeGet(&err)/1000;
	dt = time_now - time_last;
	time_last = time_now;
	
	dis_L = steps_delta[0]*0.1193805f;
	dis_R = steps_delta[1]*0.1193805f;
	
	WLX = -dis_L*0.70711f;
	WLY = dis_L*0.70711f;
	WRX = dis_R*0.70711f;
	WRY = dis_R*0.70711f;
	
	combine_vector[0] = WLX+WRX;
	combine_vector[1] = WLY+WRY;
	
	theta1 = atan(combine_vector[1]/combine_vector[0]);
	
	theta2 = theta2*3.14f/180;
	
	distance = sqrt(combine_vector[0]*combine_vector[0]+combine_vector[1]*combine_vector[1]);
	
	if(combine_vector[1]>=0&&combine_vector[0]<0){
		theta1+=3.1415926f;
	}else if(combine_vector[1]<0&&combine_vector[0]<0){
		theta1-=3.14159f;
	}else if(combine_vector[0]==0&&combine_vector[1]>=0){
		theta1 = 1.57;
	}else if(combine_vector[0]==0&&combine_vector[1]<0){
		theta1 = -1.57;
	}
	
	delta_x = distance*cos(theta1+theta2);
	delta_y = distance*sin(theta1+theta2);

//	test_x = cos(theta2)*dis_X+sin(theta2)*dis_Y;			// test code
//	test_y = -sin(pi/2+theta2)*dis_X+cos(pi/2+theta2)*dis_Y;		// test code
	
	State.frame_Vx=delta_x/dt;												//x axis
	State.frame_Vy=delta_y/dt;
	
//	linear_axyc[0]=(linear_vxyc[0]-linear_former_vxy[0])/dt;
//	linear_axyc[1]=(linear_vxyc[1]-linear_former_vxy[1])/dt;
//	
//	linear_former_vxy[0]=linear_vxyc[0];
//	linear_former_vxy[1]=linear_vxyc[1];
	
	pos_temp[0] = former_x + delta_x;
	pos_temp[1] = former_y + delta_y;
	
//	pos[0] = pos_temp[0]-(short)get_pos_offset_fourier(theta2,Axis_x);
//	pos[1] = pos_temp[1]-(short)get_pos_offset_fourier(theta2,Axis_y);
	pos[0] = pos_temp[0];
	pos[1] = pos_temp[1];
	former_x = pos_temp[0];
	former_y = pos_temp[1];
}


float get_pos_offset(float angle,u8 axis){
	float a=0,b=0,c=0,d=0,e=0,f=0;
	float val = 0;
	angle = angle/pi*180;
	if(angle>=0){
		switch(axis){
			case Axis_x:
				a = 56.54;
				b = 0.01738;
				c = -3.036;
				d = 104.2;
				e = 0.0001079;
				f = 3.088;
				break;
			case Axis_y:
				a = 85.36 ;
				b = 0.007144 ;
				c = -2.767 ;
				d = 34.07 ;
				e = 0.02094 ;
				f = 1.145 ;
				break;
		}
	}else{
		switch(axis){
			case Axis_x:
				a = 59.59;
				b = 0.01729;
				c = -3.098;
				d = 8.503;
				e = 0.005585;
				f = -3.455;
				break;
			case Axis_y:
				a = 72.02;
				b = 0.000265;
				c = -0.8633;
				d = 56.3;
				e = 0.01777;
				f = -4.498;
				break;
		}
	}
	val = a*sin(b*angle+c) + d*sin(e*angle+f);
	return val;
}

float get_pos_offset_fourier(float angle,u8 axis){
	float a0=0,a1=0,b1=0,w=0;
	float a2=0,b2=0;
	float val = 0;
	if(angle>=0){
		switch(axis){
			case Axis_x:
				a0 = 3.61;
				a1 = -4.223;
				b1 = -55.37;
				w = 1.006;
				break;
			case Axis_y:
       a0 =      -36.59;
       a1 =       15.95;
       b1 =      -55.22;
       a2 =       20.58;
       b2 =       14.86;
       w =      0.6319 ;
			break;
		}
	}else{
		switch(axis){
			case Axis_x:
				a0 = 6.949;
				a1 = -5.677;
				b1 = -58.42;
				w = 0.9988;
				break;
			case Axis_y:
				a0 = -55.86;
				a1 = 2.216 ;
				b1 = 1.934;
				a2 =  53.9 ;
				b2 = -12.25;
				w =  0.5115;

				break;
		}
	}
	switch(axis){
		case Axis_x: val = a0+a1*cos(w*angle) + b1*sin(w*angle);
		case Axis_y: val = a0+a1*cos(w*angle) + b1*sin(w*angle) + a2*cos(2*angle*w)+b2*sin(2*angle*w);
	}
	return val;
}

#define vol_max 450
#define vol_min -vol_max
#define error_sum_max    2000
#define error_sum_min    -error_sum_max
float PID_POS(u8 axis){
	/* with velocity loop
	float Kpx = 2.1,Kix = 0.00010000,Kdx = 0.008;
	float Kpy = 2.1,Kiy = 0.00010000,Kdy = 0.008;
	*/
	OS_ERR err;
	float Kpx = 2,Kix = 0.0,Kdx = 0.00001;
	float Kpy = 2,Kiy = 0.0,Kdy = 0.00001;
	float out = 0;
	float error_pos;
	float error_sum;
	static float error_sumx,error_sumy;
	static float former_errorx,former_errory;
	float A,abs_error=0;
	float Kp=0,Ki=0,Kd=0;
	float dt=0;
	//x axis dt
	float time_now_x=0;
	static float time_last_x=0;
	//y axis dt	
	float time_now_y=0;
	static float time_last_y=0;
		
	switch(axis){
		case Axis_x: 
			//get dt
			time_now_x = 5*(float)OSTimeGet(&err)/1000;
			dt = time_now_x - time_last_x;
			time_last_x = time_now_x;
			error_pos = (float)Exp_State.frame_X - (float)State.frame_X; 
			A = error_pos-former_errorx;	
			Kp = Kpx;	Ki = Kix; 
			Kd = Kdx;	
			error_sum = error_sumx;	
			break;
		case Axis_y: 
			//get dt
			time_now_y = 5*(float)OSTimeGet(&err)/1000;
			dt = time_now_y - time_last_y;
			time_last_y = time_now_y;
			error_pos = (float)Exp_State.frame_Y - (float)State.frame_Y; 
			A = error_pos-former_errory;	
			Kp = Kpy;	
			Ki = Kiy; 
			Kd = Kdy;	
			error_sum = error_sumy;	
			break;
	}
	error_sum+=error_pos;
	if(error_sum>error_sum_max){
		error_sum = error_sum_max;
	}else if(error_sum<error_sum_min){
		error_sum = error_sum_min;
	}
	
	abs_error=my_abs(error_pos);
	if(abs_error<=10){
		switch(axis){
			case Axis_x: error_sumx=0; Kpx = 0.5; Kdx = 0;	break;
			case Axis_y: error_sumy=0; Kpy = 0.5;	Kdy = 0;	break;
		}
		out=0;
	}
	
	out = Kp*error_pos-Ki*error_sum-Kd*A/dt;								
	
	switch(axis){
		case Axis_x: error_sumx = error_sum;break;
		case Axis_y: error_sumy = error_sum;break;
	}
	
	if(out>vol_max)
		out = vol_max;
	else if(out<vol_min)
		out = vol_min;
	
	return out;
}

#define errorv_sum_max 2000
#define errorv_sum_min -errorv_sum_max
#define max_fake_vol 20
#define min_fake_vol -max_fake_vol
float PID_V(u8 axis){
	float Kpvx = 0.012,Kivx = 0.0005,Kdvx = 0.08;
	float Kpvy = 0.012,Kivy = 0.0005,Kdvy = 0.08;
	float out = 0;
	float error_v=0;
	float former_err=0;
	static float former_err_x=0,former_err_y=0,former_former_err_x=0,former_former_err_y=0;
	float A=0;
	float Kp=0,Ki=0,Kd=0;
	switch(axis){
		case Axis_x: 	error_v = (float)Exp_State.frame_Vx - (float)State.frame_Vx;
									A = error_v-2*former_err_x+former_former_err_x;	
									Kp = Kpvx;	
									Ki = Kivx;  
									Kd = Kdvx;	
									former_former_err_x = former_err_x;
									former_err_x=error_v;
									break;
		case Axis_y: 	error_v = (float)Exp_State.frame_Vy - (float)State.frame_Vy; 
									A = error_v-2*former_err_y+former_former_err_y;	
									Kp = Kpvy;	
									Ki = Kivy; 
									Kd = Kdvy;	
									former_former_err_y = former_err_y; 
									former_err_y=error_v;
									break;
	}
	
	out = Kp*(error_v-former_err)+Ki*error_v+Kd*A;							
	
	if(out>max_fake_vol)
		out = max_fake_vol;
	else if(out<min_fake_vol)
		out = min_fake_vol;
	
	return out;
}

#define max_speed 500
#define min_speed -max_speed
u8 axis_tracker=0;
void Action(void){
	float linear_v[3] = {0,0,0};						//control output array
	
	//  here we change the velocity depend on the ctrl mode
	if(action_mode == position_mode)														//position mode:move to the point 
		{
			Position_mode();
		}else if(action_mode == velocity_mode){
			Velocity_mode();
		}
		linear_vxy[2]=Angle_PID(Exp_State.angle);											// angle lock
		
		// velocity limitation
		if(linear_vxy[1]>=max_speed)
			linear_vxy[1]=max_speed;
		else if(linear_vxy[1]<=min_speed)
			linear_vxy[1]=min_speed;
		
		if(linear_vxy[0]>=max_speed)
			linear_vxy[0]=max_speed;
		else if(linear_vxy[0]<=min_speed)
			linear_vxy[0]=min_speed;
		
//		if(linear_vxy[2]>=max_speed)
//			linear_vxy[2]=max_speed;
//		else if(linear_vxy[2]<=min_speed)
//			linear_vxy[2]=min_speed;
		
		// inverse kinemetic
		Speed_Moto_Control(linear_vxy,linear_v);
//		printf("%f\t%f\t%f\r\n",linear_v[0],linear_v[1],linear_v[2]);
		Move(linear_v);
}

void Calculate_State(void){
	OS_ERR err;
	float steps1_delta,steps2_delta,steps3_delta;
	int steps_EOD[2];
	float current_steps1,current_steps2,current_steps3;
	static float steps1_former=0,steps2_former=0,steps3_former=0,steps_L_former=0,steps_R_former=0;
	static float pos_EOD[2];			//[0] is X axis 
	float dt = 0;		
	float time_now=0;
	static float time_last=0;

	time_now = 5*(float)OSTimeGet(&err)/1000;
	dt = time_now - time_last;
	time_last = time_now;
//Calculation of each encoder
	steps_EOD[0] = steps_L - steps_L_former;
	steps_EOD[1] = steps_R - steps_R_former;
	
	Cult_pos(steps_EOD,pos_EOD,State.angle);
	
	steps_L_former = steps_L;
	steps_R_former = steps_R;
	
//  update states		
	State.frame_X=pos_EOD[0];
	State.frame_Y=pos_EOD[1];
	
//calculate the angular velocity of each wheel		
	current_steps1 = steps1;
	current_steps2 = steps2;
	current_steps3 = steps3;

	steps1_delta = current_steps1 - steps1_former;
	steps2_delta = current_steps2 - steps2_former;
	steps3_delta = current_steps3 - steps3_former;

	State.omega_wheel[0] = steps1_delta/dt;
	State.omega_wheel[1] = steps2_delta/dt;
	State.omega_wheel[2] = steps3_delta/dt;

	steps1_former = current_steps1;
	steps2_former = current_steps2;
	steps3_former = current_steps3;
	//printf("%f\t%f\t%ld\t%ld\r\n",pos_EOD[0],pos_EOD[1],steps_X,steps_Y);
}

void Position_mode(void){
	float tempx,tempy;
	float deg_angle;
	float rox,roy;
	
	deg_angle = deg2rad(State.angle);
	
	#ifdef USING_DOUBLE_LOOP_CTRL
		Exp_State.frame_Vy=PID_POS(Axis_y);
		Exp_State.frame_Vx=PID_POS(Axis_x);
		tempx = PID_V(Axis_x);
		tempy = PID_V(Axis_y);		
	#else
		tempx=PID_POS(Axis_x);
		tempy=PID_POS(Axis_y);
	#endif
	
	Exp_State.frame_Vx = State.frame_Vx;
	Exp_State.frame_Vy = State.frame_Vy;	
	
	rox = cos(deg_angle)*tempx+sin(deg_angle)*tempy;			//x axis
	roy = -sin(deg_angle)*tempx+cos(deg_angle)*tempy;			
	
	linear_vxy[0] = rox;
	linear_vxy[1] = roy;
}

void Velocity_mode(void){
	float tempx,tempy;
	float deg_angle;
	
	tempx = PID_V(Axis_x);
	tempy = PID_V(Axis_y);		
	
	deg_angle = deg2rad(State.angle);
	
	linear_vxy[0]+= cos(deg_angle)*tempx+sin(deg_angle)*tempy;			//x axis
	linear_vxy[1]+= -sin(deg_angle)*tempx+cos(deg_angle)*tempy;			
	
	Exp_State.frame_X = State.frame_X;
	Exp_State.frame_Y = State.frame_Y;
}

void P2P_algorithm(void){
	
}
