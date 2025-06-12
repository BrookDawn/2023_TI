#include "pid.h"

extern float target_val_X;   //???
extern float err_X;          //???
extern float err_last1_X;     //??????
extern float err_last2_X;     //???????
extern float Kp_X,Ki_X,Kd_X;     //??????????
extern float integral_X;     //???
extern float output_val_X;   //???

extern float target_val_Y;   //???
extern float err_Y;          //???
extern float err_last1_Y;     //??????
extern float err_last2_Y;     //???????
extern float Kp_Y,Ki_Y,Kd_Y;     //??????????
extern float integral_Y;     //???
extern float output_val_Y;   //???


void PID_Init(PID_TypeDef *port, float p, float i, float d)
{
    port->target_val = 0.0;
    port->err = 0.0;
    port->err_last1 = 0.0;
    port->err_last2 = 0.0;
    port->output_val = 0.0;
    port->integral = 0.0;
    port->Kp = p;
    port->Ki = i;
    port->Kd = d;
}

float PID_Realize(PID_TypeDef *port, float target_val,float actual_val)
{
	port->target_val = target_val;
	port->err = port->target_val - actual_val;
	port->integral += port->err;
	port->output_val = port->Kp * port->err +
                           port->Ki * port->integral +
                           port->Kd * (port->err - port->err_last1);

	port->err_last1 = port->err;
	return port->output_val;
}

float pid_Realize(PID_TypeDef *port, float target_val,float actual_val)
{
	port->target_val = target_val;
	port->err = port->target_val - actual_val;
	port->integral += port->err;
	port->output_val = port->Kp * (port->err - port->err_last1) +
                           port->Ki * port->err +
                           port->Kd * (port->err - port->err_last1 + port->err_last2 - port->err_last1);

	port->err_last2 = port->err_last1;
        port->err_last1 = port->err;
	return port->output_val;
}


float PID_X(float target_val,float actual_val)
{
	target_val_X = target_val;
	err_X = target_val_X - actual_val;
	integral_X += err_X;
	output_val_X = Kp_X * (err_X - err_last1_X) +
                           Ki_X * err_X +
                           Kd_X * (err_X - err_last1_X + err_last2_X - err_last1_X);

	err_last2_X = err_last1_X;
        err_last1_X = err_X;
	return output_val_X;
}

float PID_Y(float target_val,float actual_val)
{
	target_val_Y = target_val;
	err_Y = target_val_Y - actual_val;
	integral_Y += err_Y;
	output_val_Y = Kp_Y * (err_Y - err_last1_Y) +
                           Ki_Y * err_Y +
                           Kd_Y * (err_Y - err_last1_Y + err_last2_Y - err_last1_Y);

	err_last2_Y = err_last1_Y;
        err_last1_Y = err_Y;
	return output_val_Y;
}






