#include "timer.h"
#include "led.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"


extern uint32_t KeyNum;  //标志位
extern float Angle;  //舵机角度
extern int openmv_data[20];  //串口数据
extern int Red_Coordinates[2];  //红点坐标
extern int Rectangle_Coordinates[4][2];  //矩形坐标
extern int Default_Coordinates[4][2];//默认铅笔黑框
extern int PWM_Default_Coordinates[4][2];
//uint32_t buff[20];  //OLED缓存
extern PID_TypeDef pid_x;  //x轴PID
extern PID_TypeDef pid_y;  //y轴PID
extern float Ang_x;  //x轴角度
extern float Ang_y;  //y轴角度
extern float Ang_xPid,Ang_yPid;



//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
    TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}



