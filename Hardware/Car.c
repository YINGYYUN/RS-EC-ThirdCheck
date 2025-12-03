#include "stm32f10x.h"                  // Device header
#include "Motor.h"

#include "Car.h"

void Go_Ahead(void)
{
	Motor_SetPWM1(95);
	Motor_SetPWM2(95);
}

void Go_Back(void)
{
	Motor_SetPWM1(-95);
	Motor_SetPWM2(-95);
}

void Self_Left(void)
{
	Motor_SetPWM1(-95);
	Motor_SetPWM2(95);
}

void Self_Right(void)
{
	Motor_SetPWM1(95);
	Motor_SetPWM2(-95);
}

void Car_Stop(void)
{
	Motor_SetPWM1(0);
	Motor_SetPWM2(0);
}
