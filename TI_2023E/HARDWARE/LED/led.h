#ifndef __LED_H
#define __LED_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//LED�˿ڶ���
#define LED0 PCout(13)	// DS0

/*����ķ�ʽ��ͨ��ֱ�Ӳ����⺯����ʽ��ȡIO*/
#define KEY1 		GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_1) //PF1
#define KEY2 		GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_2)	//PF2 
#define KEY3 		GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_3) //PF3
#define KEY4 	  GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_4)	//PF4
#define KEY5 	  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)	//PA15



void LED_Init(void);//��ʼ��	
void KEY_Init(void);
uint8_t KEY_Scan(uint8_t mode);
#endif
