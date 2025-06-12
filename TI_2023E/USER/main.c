#include "sys.h"
#include "delay.h"
#include "led.h"
#include "timer.h"
#include "oled.h"
#include "pwm.h"
#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "Serial.h"
#include "pid.h"
#include <string.h>
#include <math.h>

extern void TIM1_PWM_Init(u32 arr,u32 psc);
extern void TIM9_PWM_Init(u32 arr,u32 psc);
extern uint8_t KEY_Scan(uint8_t mode);
/*当前的中心位置 （78.5，55.5）    
		Kp_X = 7;
		Kp_Y = 7;
		Ki_X = 0.3;
		Ki_Y = 0.3;
		Kd_X = 0.9;
		Kd_Y = 0.9;
		
		中心PWM(1551,1605);
		左上();
		左下(1666,1500);
		*/
//QQVGA  -> 7,0.3,0.9;
//QVGA   ->

#define zero_x 78.5//中心坐标
#define zero_y 55.5

uint8_t KEY_value,mode;
char OledString[10];
int KeyNum = 1;  //标志位
float Angle;  //舵机角度
int openmv_data[30];  //串口数据
int Red_Coordinates[2];  //红点坐标
int Rectangle_Coordinates[8][2];  //矩形坐标
int temp_Rectangle_Coordinates[4][2] = {0};//转存矩阵
int Default_Coordinates[4][2] = {{45,21},{112,22},{110,88},{44,87}};//默认铅笔黑框
int Ang_Default_Coordinates[4][2] = {{1640,1717},{1430,1717},{1440,1500},{1666,1500}};
int Data_integration_coordinate[4][2];

//uint32_t buff[20];  //OLED缓存
//PID_TypeDef pid_x;  //x轴PID
//PID_TypeDef pid_y;  //y轴PID
float Ang_x = 197;  //x轴角度
float Ang_y = 995;  //y轴角度
float zero_Ang[2] = {197,995};
float Ang_xPid,Ang_yPid;
float taget[2];//目标坐标
int point_n = 0; 
int step_n = 0;
float zero[2] = {zero_x,zero_y};//中心点坐标

float target_val_X = 0;  
float err_X = 0;          
float err_last1_X = 0;     
float err_last2_X = 0;     
float Kp_X,Ki_X,Kd_X = 0;   
float integral_X = 0;     
float output_val_X = 0;  

float target_val_Y = 0;   
float err_Y = 0;        
float err_last1_Y = 0;    
float err_last2_Y = 0;     
float Kp_Y,Ki_Y,Kd_Y = 0;    
float integral_Y = 0;     
float output_val_Y = 0;   

int point_c = 0; 
//int step_p;
//int last_points_num;
//float ro, pi;


int main(void)
{ 
    Kp_X = 9;
		Kp_Y = 9;
		Ki_X = 0.3;
		Ki_Y = 0.3;
		Kd_X = 0.5;
		Kd_Y = 0.5;
    
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
		delay_init(168);                   //初始化延时函数
		LED_Init();				           //初始化LED端口
		KEY_Init();
		OLED_Init();
		TIM1_PWM_Init(20000-1,168-1);  /*PE13*/
		TIM9_PWM_Init(20000-1,168-1);   /*PE5*/
		sevro_angle(220,995);
//		sevro_angle(180,1000);
		delay_ms(1000);

		TIM3_Int_Init(1000-1,1680-1);	//定时器时钟84M，分频系数8400，84000000/8400/50=200hz,   5ms
//		Control_Init();
//		Encoder_Init_TIM4();
//		Encoder_Init_TIM2();
		Serial_Init();
		Serial2_Init();
		Serial3_Init();

//    PID_Init(&pid_x,10,0,0); //x轴PID
//	  PID_Init(&pid_y,10,0,0); //y轴PID
//    Target_Position_1 = 200;
//    Target_Position_2 = 200;
	
	while(1)
	{
		/*模式扫描*/
		KEY_value = KEY_Scan(0);
		if(KEY_value == 1){mode = 1;}
		if(KEY_value == 2){mode = 2;}        
		if(KEY_value == 3){mode = 3;}	
		if(KEY_value == 4){mode = 4;}
		if(KEY_value == 5){mode = 5;}
				
		
		sprintf(OledString, "X:%d",Red_Coordinates[0]); 
		OLED_ShowString(1,1,OledString);
		
		sprintf(OledString, "Y:%d",Red_Coordinates[1]); 
		OLED_ShowString(2,1,OledString);

		sprintf(OledString, "rect_x:%d",(int)taget[0]); 
		OLED_ShowString(3,1,OledString);
		
		sprintf(OledString, "rect_y:%d",(int)taget[1]); 
		OLED_ShowString(4,1,OledString);
        
//		sprintf(OledString, "rect_x:%.4d",(int)Ang_x); 
//		OLED_ShowString(3,1,OledString);
//		
//		sprintf(OledString, "rect_y:%.4d",(int)Ang_y); 
//		OLED_ShowString(4,1,OledString);
	}
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)                     //5ms一次中断
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
        
		/*红点复位*/
		if(mode == 1)
		{
//				Ang_xPid = PID_X(zero[0],Red_Coordinates[0]);
//				Ang_yPid = PID_Y(zero[1],Red_Coordinates[1]);
//				Ang_x -= Ang_xPid;
//				Ang_y -= Ang_yPid;
//				sevro_angle(Ang_x,Ang_y);
		}
		/*铅笔框*/
		else if(mode == 2)
		{				
//			servo_degress_points_to_move(zero_Ang,Ang_Default_Coordinates,300);
//		  sevro_angle(taget[0],taget[1]);
//			PWM_ServeAngle(1430,1707);
            Kp_X = 8;
            Kp_Y = 8;
            Ki_X = 0.9;
            Ki_Y = 0.9;
            Kd_X = 0.1;
            Kd_Y = 0.1;
			if(point_c < 6)
				servo_degress_points_to_move(zero,Default_Coordinates,200);
				Ang_xPid = PID_X(taget[0],Red_Coordinates[0]);
				Ang_yPid = PID_Y(taget[1],Red_Coordinates[1]);	
				Ang_x -= Ang_xPid;
				Ang_y -= Ang_yPid;
				sevro_angle(Ang_x,Ang_y);	
			
		}
		else if(mode == 4)
		{
            Kp_X = 10;
            Kp_Y = 10;
            Ki_X = 0.3;
            Ki_Y = 0.3;
            Kd_X = 0.1;
            Kd_Y = 0.1;
			if(point_c < 6)				
				servo_degress_points_to_move(zero,Rectangle_Coordinates,250);
				Ang_xPid = PID_X(taget[0],Red_Coordinates[0]);
				Ang_yPid = PID_Y(taget[1],Red_Coordinates[1]);	
				Ang_x -= Ang_xPid;
				Ang_y -= Ang_yPid;
				sevro_angle(Ang_x,Ang_y);	
		}
		else if(mode == 3)
		{
            Kp_X = 10;
            Kp_Y = 10;
            Ki_X = 0.28;
            Ki_Y = 0.28;
            Kd_X = 0.05;
            Kd_Y = 0.05;
			if(point_c < 6)				
				servo_degress_points_to_move(zero,Data_integration_coordinate,300);
				Ang_xPid = PID_X(taget[0],Red_Coordinates[0]);
				Ang_yPid = PID_Y(taget[1],Red_Coordinates[1]);	
				Ang_x -= Ang_xPid;
				Ang_y -= Ang_yPid;
				sevro_angle(Ang_x,Ang_y);	
		}
		
		else
		{
            Kp_X = 9;
            Kp_Y = 9;
            Ki_X = 0.3;
            Ki_Y = 0.3;
            Kd_X = 0.5;
            Kd_Y = 0.5;
			
				Ang_xPid = PID_X(zero[0],Red_Coordinates[0]);
				Ang_yPid = PID_Y(zero[1],Red_Coordinates[1]);
				Ang_x -= Ang_xPid;
				Ang_y -= Ang_yPid;
				sevro_angle(Ang_x,Ang_y);
		}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}

