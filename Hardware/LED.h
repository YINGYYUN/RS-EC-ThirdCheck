#ifndef __LED_H
#define __LED_H

extern uint8_t LED_Mode;

/*引脚配置*/
#define LED_NUM				 4

#define LED_PIN_1	        GPIO_Pin_8      //蓝LED===PA8
#define LED_PIN_2	        GPIO_Pin_11     //绿LED===PA11
#define LED_PIN_3	        GPIO_Pin_12     //红LED===PA12
#define LED_PIN_4			GPIO_Pin_13		//PC13
#define LED_ALL_PINS 			(LED_PIN_1 | LED_PIN_2 | LED_PIN_3 | LED_PIN_4)

void LED_Init(void);
void LED_OFF_SET(uint8_t Num);
void LED_ON_SET(uint8_t Num);
void LED_A_OFF_ALL(void);
void LED_A_ON_ALL(void);

#endif
