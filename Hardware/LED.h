#ifndef __LED_H
#define __LED_H

extern uint8_t LED_Mode;

/*模式定义*/
#define LED_OFFMode              0
#define LED_ONMode               1
#define LED_SlowFlashMode        2
#define LED_FastFlashMode        3
//#define LED_DotFlashMode         4

void LED_Init(void);
void LED_SetMode(uint8_t Mode);
void LED_Tick(void);

#endif
