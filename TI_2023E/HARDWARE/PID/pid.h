#ifndef __PID_H
#define __PID_H
#include <stdio.h>

typedef struct
{
	float target_val;   //目标值
	float err;          //偏差值
	float err_last1;     //上一个偏差值
	float err_last2;     //上上一个偏差值
	float Kp,Ki,Kd;     //比例、积分、微分系数
	float integral;     //积分值
	float output_val;   //输出值
} PID_TypeDef;


void PID_Init(PID_TypeDef *port, float p, float i, float d);
float PID_Realize(PID_TypeDef *port, float target_val,float actual_val);//位置式
float pid_Realize(PID_TypeDef *port, float target_val,float actual_val);//增量式
float PID_X(float target_val,float actual_val);
float PID_Y(float target_val,float actual_val);
#endif
