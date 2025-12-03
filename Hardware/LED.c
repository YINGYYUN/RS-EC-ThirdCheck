#include "stm32f10x.h"                  // Device header
#include "LED.h"

typedef struct {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
}LED_Pindef;

const LED_Pindef LED_Pinlist[LED_NUM] = {
	{GPIOA, LED_PIN_1},
	{GPIOA, LED_PIN_2},
	{GPIOA, LED_PIN_3}
};

void LED_Init(void)
{
	//配置RCC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	//配置GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //推挽输出模式
	GPIO_InitStructure.GPIO_Pin = LED_ALL_PINS;          //多个端口之间相或
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = LED;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = LED;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, LED_ALL_PINS);    //多个端口之间相或
//	GPIO_SetBits(GPIOB, LED);
//	GPIO_SetBits(GPIOC, LED);
	
//	LED_GPIO[1] = LED_GPIOA;  LED_Pin[1] = LED1;
//	LED_GPIO[2] = LED_GPIOA;  LED_Pin[2] = LED2;
//	LED_GPIO[] = LED_GPIO;  LED_Pin[] = LED;        //多个LED复制此处
}

void LED_OFF_SET(uint8_t Num)
{
	if(Num >= LED_NUM)return ;
	GPIO_SetBits(LED_Pinlist[Num - 1].GPIOx, LED_Pinlist[Num - 1].GPIO_Pin);
}

void LED_ON_SET(uint8_t Num)
{
	if(Num > LED_NUM)return ;
	GPIO_ResetBits(LED_Pinlist[Num - 1].GPIOx, LED_Pinlist[Num - 1].GPIO_Pin);
}

void LED_OFF_ALL(void)
{
	GPIO_SetBits(GPIOA, LED_ALL_PINS);
}

void LED_ON_ALL(void)
{
	GPIO_ResetBits(GPIOA, LED_ALL_PINS);
}
