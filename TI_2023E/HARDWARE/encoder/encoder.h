#ifndef __ENCODER_H
#define __ENCODER_H

#include "sys.h"
void Encoder_Init_TIM4(void);
void Encoder_Init_TIM2(void);
float read_encoder1(void);
float read_encoder2(void);
int read_encoderchaoche1(void);
#endif
