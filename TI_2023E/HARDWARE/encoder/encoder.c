#include "encoder.h"



void Encoder_Init_TIM4(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;    

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;          
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  
    GPIO_Init(GPIOB, &GPIO_InitStructure);                          

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);           
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);           

    TIM_TimeBaseStructure.TIM_Period = 65536-1; 	                      
    TIM_TimeBaseStructure.TIM_Prescaler=1-1;                        
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);                  

    TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            
    TIM_ICInitStructure.TIM_ICFilter =0xF;                            
    TIM_ICInit(TIM4,&TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            
    TIM_ICInitStructure.TIM_ICFilter=0xF;                             
    TIM_ICInit(TIM4,&TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising );
                                                  
    TIM_Cmd(TIM4,ENABLE);   
}

void Encoder_Init_TIM2(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;    

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;          
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  
    GPIO_Init(GPIOA, &GPIO_InitStructure);                          

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);           
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);           

    TIM_TimeBaseStructure.TIM_Period = 65536-1; 	                      
    TIM_TimeBaseStructure.TIM_Prescaler=1-1;                        
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);                  

    TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            
    TIM_ICInitStructure.TIM_ICFilter =0xF;                            
    TIM_ICInit(TIM2,&TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            
    TIM_ICInitStructure.TIM_ICFilter=0xF;                             
    TIM_ICInit(TIM2,&TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising );
                                                  
    TIM_Cmd(TIM2,ENABLE);   
}


//float read_encoder1(void)
//{
//	
//    Encoder1Count = -(short)TIM_GetCounter(TIM4);
//    TIM_SetCounter(TIM4, 0);	
//    Speed1 = (float)Encoder1Count*200/20/13/4;
//	
//	return Speed1;
//}

//float read_encoder2(void)
//{
//   
//    Encoder2Count = (short)TIM_GetCounter(TIM2);
//    TIM_SetCounter(TIM2, 0);
//    Speed2 = (float)Encoder2Count*200/20/13/4;

//    return Speed2;
//}



