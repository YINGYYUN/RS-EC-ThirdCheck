#include "stm32f10x.h"                  // Device header
#include "LED.h"

/*引脚配置*/
#define LED1         GPIO_Pin_0    //PA0

void LED_Init(void)
{
	//配置RCC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	//配置GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //推挽输出模式
	GPIO_InitStructure.GPIO_Pin = LED1;          //多个端口之间相或
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = LED;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = LED;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, LED1);    //多个端口之间相或
//	GPIO_SetBits(GPIOB, LED);
//	GPIO_SetBits(GPIOC, LED);
	
//	LED_GPIO[1] = LED_GPIOA;  LED_Pin[1] = LED1;
//	LED_GPIO[2] = LED_GPIOA;  LED_Pin[2] = LED2;
//	LED_GPIO[] = LED_GPIO;  LED_Pin[] = LED;        //多个LED复制此处
}


void LED_OFF(void)
{
	GPIO_SetBits(GPIOA,LED1);
}

void LED_ON(void)
{
	GPIO_ResetBits(GPIOA,LED1);
}

uint8_t LED_Mode = LED_OFFMode;

void LED_SetMode(uint8_t Mode)
{
	if (Mode == LED_SlowFlashMode || Mode == LED_FastFlashMode)LED_Mode = Mode;
}

void LED_Tick(void)
{
	static uint16_t LED_Count = 0;
	switch(LED_Mode)
	{
		case LED_OFFMode:
			LED_OFF();
		
			break;
		
		case LED_ONMode:
			LED_ON();
		
			break;
		
		case LED_SlowFlashMode:
			LED_Count ++;
			LED_Count %= 1000;
			if (LED_Count < 100)	{LED_ON();}
			else 				{LED_OFF();}
			
			break;
		
		case LED_FastFlashMode:
			LED_Count ++;
			LED_Count %= 100;
			if (LED_Count < 50)	{LED_ON();}
			else 				{LED_OFF();}
		
			break;
		
		default:
			//留一手
		
			break;		
	}
}
