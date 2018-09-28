#ifndef CTRL__H__
#define CTRL__H__

#include "includes.h"

void Move(float linear_v[4]);
void Speed_Moto_Control(float linear_xyz[3],float linear_v[4]);
void Cult_pos(int steps_delta[2],float pos[2],float theta2);
float get_pos_offset(float angle,u8 axis);
float get_pos_offset_fourier(float angle,u8 axis);
float Angle_PID(float exp_angle);																//pid of angle ,to make sure the robot stay in the expected angle
float PID_POS(u8 axis);																					//pid of position
float PID_V(u8 axis);
void Action(void);
void Calculate_State(void);
void Position_mode(void);
void Velocity_mode(void);
void P2P_algorithm(void);
float Get_theta2(float target_coordinate[2]);
float Velocity_controller(float target_coordinate_in_robot_frame[2]);

#endif


