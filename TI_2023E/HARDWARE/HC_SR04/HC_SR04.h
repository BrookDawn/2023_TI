#ifndef HC_SR04_H
#define HC_SR04_H
#include "sys.h"

#define HC_SR04 PAout(3) 
#define ECHO_Reci PAin(2) 

float Senor_Using(void);
void TIM5_CH1_Cap_Init(u32 arr,u16 psc);
void SR04_GPIO_Init(void);

int TCRT5000_Dist(void);

#endif

