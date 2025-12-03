#ifndef __HCSR04_H
#define __HCSR04_H
#include "stm32f10x.h"                  // Device header


void HCSR04_Init(void);
uint16_t HCSR04_GetValue(void); // non-blocking: may start a new measurement and return last value

// Non-blocking tick function: call from a 1ms tick (e.g. TIM1_UP_IRQHandler)
void HCSR04_Tick(void);

// Optional: start a measurement explicitly
void HCSR04_StartMeasure(void);

// Check if a measurement result is ready
uint8_t HCSR04_IsReady(void);


#endif 
