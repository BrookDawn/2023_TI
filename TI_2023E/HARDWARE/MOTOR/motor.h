#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "pid.h"


#define AIN1 PFout(7)
#define AIN2 PFout(8)
#define BIN1 PFout(10)
#define BIN2 PFout(9)

#define CIN1 PGout(10)
#define CIN2 PGout(11)
#define DIN1 PGout(14)
#define DIN2 PGout(15)

 
void motorPidPositionSpeed(float Motor1SetSpeed,float Motor2SetSpeed,float Positiontarget);
void MotorF_Set(int Motor1,int Motor2);
void MotorB_Set(int Motor3,int Motor4);
void STOP(void);
void sevro_angle(float angleX,float angleY);
//void PWM_ServeAngle(int pwm1,int pwm2);

void servo_degress_points_to_move(float zero[2],int points[4][2], int step);

#endif

