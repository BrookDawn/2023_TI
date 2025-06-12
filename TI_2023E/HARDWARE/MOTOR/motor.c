#include "motor.h"

extern float taget[2];//目标坐标
extern int point_n;
extern int step_n;
extern int zero[2];
extern int KeyNum;  //标志位
extern int point_c;

void MotorF_Set(int Motor1,int Motor2)
{
  //根据正负设置方向
  if(Motor1<0)
	{
		AIN1 = 0;
	  AIN2 = 1;
	}
	else
	{
	 AIN1 = 1;
	 AIN2 = 0;
  }
  if(Motor2<0)
	{
		BIN1 = 0;
		BIN2 = 1;
	}
  else
	{
		BIN1 = 1;
		BIN2 = 0;
	}

//然后设置占空比
	if(Motor1<0)
	{
	  if(Motor1<-800) Motor1=-800;
		TIM_SetCompare2(TIM1,-Motor1);
	}
	else
	{
	  if(Motor1>800) Motor1=800;
		TIM_SetCompare2(TIM1,Motor1);
	}

	if(Motor2<0)
	{
	  if(Motor2<-800) Motor2=-800;
		TIM_SetCompare1(TIM1,-Motor2);
	}
	else
	{
	  if(Motor2>800) Motor2=800;
		TIM_SetCompare1(TIM1,Motor2);
	}

}



void MotorB_Set(int Motor3,int Motor4)
{
  //根据正负设置方向
  if(Motor3<0)
	{
		CIN1 = 0;
	  CIN2 = 1;
	}
	else
	{
	 CIN1 = 1;
	 CIN2 = 0;
  }
  if(Motor4<0)
	{
		DIN1 = 0;
		DIN2 = 1;
	}
  else
	{
		DIN1 = 1;
		DIN2 = 0;
	}

//然后设置占空比
	if(Motor3<0)
	{
	  if(Motor3<-950) Motor3=-950;
		TIM_SetCompare4(TIM1,-Motor3);
	}
	else
	{
	  if(Motor3>950) Motor3=950;
		TIM_SetCompare4(TIM1,Motor3);
	}

	if(Motor4<0)
	{
	  if(Motor4<-950) Motor4=-950;
		TIM_SetCompare3(TIM1,-Motor4);
	}
	else
	{
	  if(Motor4>950) Motor4=950;
		TIM_SetCompare3(TIM1,Motor4);
	}
}


void STOP(void)
{
    CIN1 = 0;
    CIN2 = 0;
    DIN1 = 0;
    DIN2 = 0;
}

void sevro_angle(float angleX,float angleY)
{
    TIM_SetCompare1(TIM9,angleX / 180*20+1500);
    TIM_SetCompare2(TIM9,angleY / 180*20+1500);
}


void servo_degress_points_to_move(float zero[2],int points[4][2], int step)
{
    if (point_n == 4)
	{ // Last point, so calculate transition values with the first point.
        taget[0] = (points[0][0] - points[3][0]) / step * step_n + points[3][0];
        taget[1] = (points[0][1] - points[3][1]) / step * step_n + points[3][1];
    }
	else if(point_n == 0)
	{
		taget[0] = (points[point_n][0] - zero[0]) / step * step_n + zero[0];
		taget[1] = (points[point_n][1] - zero[1]) / step * step_n + zero[1];
	}
    else
		{
        taget[0] = (points[point_n][0] - points[point_n-1][0]) / step * step_n + points[point_n-1][0];
        taget[1] = (points[point_n][1] - points[point_n-1][1]) / step * step_n + points[point_n-1][1];
    }
//    step_n++;
//    if (step_n > step)
//    {
//        step_n = 0;
//        point_n++;
//        if (point_n > 4)
//        {
//            point_n = 1;
//        }
//    }
//    if(point_n == 0)
//		{
//        taget[0] = (points[point_n][0] - zero[0]) / step * step_n + zero[0];
//        taget[1] = (points[point_n][1] - zero[1]) / step * step_n + zero[1];
//    }
//		else if(point_n < 4)
//		{
//        taget[0] = (points[point_n][0] - points[point_n-1][0]) / step * step_n + points[point_n-1][0];
//        taget[1] = (points[point_n][1] - points[point_n-1][1]) / step * step_n + points[point_n-1][1];
//		}
//    else
//		{
//        taget[0] = (points[0][0] - points[3][0]) / step * step_n + points[3][0];
//        taget[1] = (points[0][1] - points[3][1]) / step * step_n + points[3][1];
//    }
    step_n++;
    if (step_n > step)
    {
        step_n = 0;
        point_n++;
        point_c++;
        if (point_n > 4)
        {
            point_n = 1;
        }
    }
}

//extern int point_p;
//extern int step_p;
//extern int last_points_num;
//extern float ro, pi;

//void servo_pwm_points_to_move(int points[4][2], int num_points, int step_tot)
//{
//	last_points_num = num_points - 1;

//    if (point_p == last_points_num) { // Last point, so calculate transition values with the first point.
//        ro = (points[0][0] - points[last_points_num][0]) / step_tot * step_p + points[last_points_num][0];
//        pi = (points[0][1] - points[last_points_num][1]) / step_tot * step_p + points[last_points_num][1];
//    }
//    else {
//        ro = (points[point_p + 1][0] - points[point_p][0]) / step_tot * step_p + points[point_p][0];
//        pi = (points[point_p + 1][1] - points[point_p][1]) / step_tot * step_p + points[point_p][1];
//    }

//		PWM_ServeAngle(ro, pi);
//    step_p++;
//    if (step_p > step_tot)
//    {
//        step_p = 0;
//        point_p++;
//        if (point_p > last_points_num)
//        {
//            point_p = 0;
//        }
//    }
//}

