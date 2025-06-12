#include "stm32f4xx.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "serial.h"
#include "oled.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"


extern int openmv_data[20];  //¥Æø⁄ ˝æ›
extern int Red_Coordinates[2];  //∫Ïµ„◊¯±Í
extern int Rectangle_Coordinates[8][2];  //æÿ–Œ◊¯±Í
extern int Default_Coordinates[4][2];//ƒ¨»œ«¶± ∫⁄øÚ
extern int PWM_Default_Coordinates[4][2];
extern int Data_integration_coordinate[4][2];


void Serial_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);// πƒ‹USART1 ±÷”
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); // πƒ‹GPIOA ±÷”
	
	//¥Æø⁄1∂‘”¶“˝Ω≈∏¥”√”≥…‰
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9∏¥”√Œ™USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10∏¥”√Œ™USART1
	

	//USART1∂Àø⁄≈‰÷√
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9”ÎGPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//∏¥”√π¶ƒ‹
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//ÀŸ∂»50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Õ∆ÕÏ∏¥”√ ‰≥ˆ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //…œ¿≠
	GPIO_Init(GPIOA,&GPIO_InitStructure); //≥ı ºªØPA9£¨PA10
	

	//USART1 ≥ı ºªØ…Ë÷√
	USART_InitStructure.USART_BaudRate = 115200;//≤®Ãÿ¬ …Ë÷√
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//◊÷≥§Œ™8Œª ˝æ›∏Ò Ω
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//“ª∏ˆÕ£÷πŒª
	USART_InitStructure.USART_Parity = USART_Parity_No;//Œﬁ∆Ê≈º–£—ÈŒª
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Œﬁ”≤º˛ ˝æ›¡˜øÿ÷∆
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	// ’∑¢ƒ£ Ω
    USART_Init(USART1, &USART_InitStructure); //≥ı ºªØ¥Æø⁄1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
	
}

void Serial3_Init(void)     //¥Æø⁄»˝≈‰÷√
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);// πƒ‹USART3 ±÷”
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); // πƒ‹GPIOB ±÷”
	
	//¥Æø⁄3∂‘”¶“˝Ω≈∏¥”√”≥…‰
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10∏¥”√Œ™USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11∏¥”√Œ™USART3
	

	//USART3∂Àø⁄≈‰÷√
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10”ÎGPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//∏¥”√π¶ƒ‹
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//ÀŸ∂»50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Õ∆ÕÏ∏¥”√ ‰≥ˆ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //…œ¿≠
	GPIO_Init(GPIOB,&GPIO_InitStructure); //≥ı ºªØPB10£¨PB11
	

	//USART3 ≥ı ºªØ…Ë÷√
	USART_InitStructure.USART_BaudRate = 115200;//≤®Ãÿ¬ …Ë÷√
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//◊÷≥§Œ™8Œª ˝æ›∏Ò Ω
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//“ª∏ˆÕ£÷πŒª
	USART_InitStructure.USART_Parity = USART_Parity_No;//Œﬁ∆Ê≈º–£—ÈŒª
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Œﬁ”≤º˛ ˝æ›¡˜øÿ÷∆
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	// ’∑¢ƒ£ Ω
    USART_Init(USART3, &USART_InitStructure); //≥ı ºªØ¥Æø⁄3
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART3, ENABLE);
	
}


void Serial2_Init(void)     //¥Æø⁄∂˛≈‰÷√
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);// πƒ‹USART2 ±÷”
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); // πƒ‹GPIOA ±÷”
	
	//¥Æø⁄2∂‘”¶“˝Ω≈∏¥”√”≥…‰
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2∏¥”√Œ™USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3∏¥”√Œ™USART2
	

	//USART2∂Àø⁄≈‰÷√
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2”ÎGPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//∏¥”√π¶ƒ‹
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//ÀŸ∂»50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Õ∆ÕÏ∏¥”√ ‰≥ˆ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //…œ¿≠
	GPIO_Init(GPIOA,&GPIO_InitStructure); //≥ı ºªØPA2£¨PA3
	

	//USART2 ≥ı ºªØ…Ë÷√
	USART_InitStructure.USART_BaudRate = 9600;//≤®Ãÿ¬ …Ë÷√
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//◊÷≥§Œ™8Œª ˝æ›∏Ò Ω
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//“ª∏ˆÕ£÷πŒª
	USART_InitStructure.USART_Parity = USART_Parity_No;//Œﬁ∆Ê≈º–£—ÈŒª
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Œﬁ”≤º˛ ˝æ›¡˜øÿ÷∆
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	// ’∑¢ƒ£ Ω
    USART_Init(USART2, &USART_InitStructure); //≥ı ºªØ¥Æø⁄2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
	
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial2_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}


void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

//int fputc(int ch, FILE *f)
//{
//	Serial_SendByte(ch);
//	return ch;
//}

//void Serial_Printf(char *format, ...)
//{
//	char String[100];
//	va_list arg;
//	va_start(arg, format);
//	vsprintf(String, format, arg);
//	va_end(arg);
//	Serial_SendString(String);
//}
//uint8_t Seria1_GetRxFlag(void)
//{
//	if (Seria1_RxFlag == 1)
//	{
//		Seria1_RxFlag = 0;
//		return 1;
//	}
//	return 0;
//}

//uint8_t Serial_GetRxFlag(void)
//{
//	if (Serial_RxFlag == 1)
//	{
//		Serial_RxFlag = 0;
//		return 1;
//	}
//	return 0;
//}

//void Serial_SendPacket(void)
//{
//	Serial_SendByte(0xFF);
//	Serial_SendArray(Serial_TxPacket, 4);
//	Serial_SendByte(0xFE);
//}


//USART1 »´æ÷÷–∂œ∑˛ŒÒ∫Ø ˝
void USART1_IRQHandler(void)
{
	 static int i=0;
	 
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{
        openmv_data[i++] = USART_ReceiveData(USART1);
		if(openmv_data[0]!=0xa3 ) i=0;
		if((i==2)&&(openmv_data[1]!=0xb3)) i=0;
		if(i==19)
		{
			if( openmv_data[18] == 0xc3)
			{
                //Âõõ‰Ωç‰∏∫‰∏Ä‰∏™ÂùêÊ†á   ->   //ÂçÅÂÖ≠ËøõÂà∂ËΩ¨ÂçÅËøõÂà∂
				Rectangle_Coordinates[0][0] = (int)openmv_data[3]; 
				Rectangle_Coordinates[0][1] = (int)openmv_data[2];
                
				Rectangle_Coordinates[1][0] = (int)openmv_data[5];
				Rectangle_Coordinates[1][1] = (int)openmv_data[4];	
                
                //ÂùêÊ†á‰∫å
				Rectangle_Coordinates[2][0] = (int)openmv_data[7];
				Rectangle_Coordinates[2][1] = (int)openmv_data[6];
                
				Rectangle_Coordinates[3][0] = (int)openmv_data[9];
				Rectangle_Coordinates[3][1] = (int)openmv_data[8];
                
                Rectangle_Coordinates[4][0] = (int)openmv_data[11];
				Rectangle_Coordinates[4][1] = (int)openmv_data[10];
                
				Rectangle_Coordinates[5][0] = (int)openmv_data[13];
				Rectangle_Coordinates[5][1] = (int)openmv_data[12];	
                
				Rectangle_Coordinates[6][0] = (int)openmv_data[15];
				Rectangle_Coordinates[6][1] = (int)openmv_data[14];
                
				Rectangle_Coordinates[7][0] = (int)openmv_data[17];
				Rectangle_Coordinates[7][1] = (int)openmv_data[16];
                
                //Êï∞ÊçÆÊï¥Âêà
                Data_integration_coordinate[0][0] = Rectangle_Coordinates[0][0] * 256 + Rectangle_Coordinates[0][1];
                Data_integration_coordinate[0][1] = Rectangle_Coordinates[1][0] * 256 + Rectangle_Coordinates[1][1];
                
                Data_integration_coordinate[1][0] = Rectangle_Coordinates[2][0] * 256 + Rectangle_Coordinates[2][1];
                Data_integration_coordinate[1][1] = Rectangle_Coordinates[3][0] * 256 + Rectangle_Coordinates[3][1];
                
                Data_integration_coordinate[2][0] = Rectangle_Coordinates[4][0] * 256 + Rectangle_Coordinates[4][1];
                Data_integration_coordinate[2][1] = Rectangle_Coordinates[5][0] * 256 + Rectangle_Coordinates[5][1];
                
                Data_integration_coordinate[3][0] = Rectangle_Coordinates[6][0] * 256 + Rectangle_Coordinates[6][1];
                Data_integration_coordinate[3][1] = Rectangle_Coordinates[7][0] * 256 + Rectangle_Coordinates[7][1];
                
			}
			i=0;
			USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		}
	}	
}



/*¿∂—¿*/
//void USART2_IRQHandler(void)
//{
//   if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
//   {
//    blue_tooth_data = USART_ReceiveData(USART2);
//	   
//    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//   }

//}



//void USART2_IRQHandler(void)
//{
//   if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
//   {
//	    hc_05_data = USART_ReceiveData(USART2);
//	   
//      if(hc_05_data == 1) motorPidSetSpeed(1,1);
//	    
//		  USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	 }

//}
