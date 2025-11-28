#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Timer.h"
#include "OLED.h"
//#include "LED.h"
#include "Serial.h"
#include "MPU6050.h"
//#include "Servo.h"
#include <math.h>

int16_t AX, AY, AZ, GX, GY, GZ;

//定时中断重叠标志位
uint8_t TimerErrorFlag;

//定时中断执行时长
uint16_t TimerCount;

float AngleAcc;			//加速度计算的俯仰角
float AngleGyro;		//陀螺仪积分的俯仰角
float Angle;			//融合后的俯仰角

float RollAcc;    		// 加速度计计算的横滚角
float RollGyro;   		// 陀螺仪积分的横滚角
float Roll;       		// 融合后的横滚角

float Yaw = 0;			//偏航角

int main(void)
{
	OLED_Init();
//	LED_Init();
	Serial_Init();
	MPU6050_Init();
//	Servo_Init();
	
	Timer_Init();
	
//	LED_SetMode(LED_OFFMode);
	

	
	while(1)
	{
		
		
		OLED_Printf(0, 0, OLED_8X16, "%+06d", AX);
		OLED_Printf(0, 16, OLED_8X16, "%+06d", AY);
		OLED_Printf(0, 32, OLED_8X16, "%+06d", AZ);
		OLED_Printf(64, 0, OLED_8X16, "%+06d", GX);
		OLED_Printf(64, 16, OLED_8X16, "%+06d", GY);
		OLED_Printf(64, 32, OLED_8X16, "%+06d", GZ);
		
		OLED_Printf(0, 48, OLED_8X16, "Flag:%1d", TimerErrorFlag);
		OLED_Printf(64, 48, OLED_8X16, "C:%05d", TimerCount);
		
		OLED_Update();
		
//		BlueSerial_Printf("[plot,%d,%d,%d]", GX, GY, GZ);
		Serial_Printf("%f,%f,%f\r\n", RollAcc, RollGyro,Roll);
	}
	
}

//1ms的定时中断
void TIM1_UP_IRQHandler(void)
{
	//检查标志位
	if (TIM_GetITStatus(TIM1,TIM_IT_Update) == SET )
	{
		//清除标志位
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
		//保证数据的及时读取
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
//		LED_Tick();
		
		//校准零飘
		GX += 55;
		GY += 18;
		GZ += 10;
		
		// 俯仰角计算
		 AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;  			// 俯仰角（绕Y轴）
        AngleGyro = Angle + GY / 32768.0 * 2000 * 0.001;  		// 陀螺仪积分（2000是量程，0.001是1ms采样间隔）
        Angle = 0.001 * AngleAcc + (1 - 0.001) * AngleGyro;  	// 互补滤波
		
		 // 横滚角计算
        RollAcc = atan2(AY, AZ) / 3.14159 * 180;  				// 横滚角（绕X轴）
        RollGyro = Roll + GX / 32768.0 * 2000 * 0.001;  		// 陀螺仪X轴积分
        Roll = 0.001 * RollAcc + (1 - 0.001) * RollGyro;  		// 相同互补滤波算法
		
		 // 偏航角：仅陀螺仪积分（无加速度计校准，会漂移）
        Yaw += GZ / 32768.0 * 2000 * 0.001;  // 仅积分，无校准
		
//		//俯仰角
//		AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;
//		
//		//角速度积分
//		AngleGyro = Angle + GY / 32768.0 * 2000 * 0.001;
//		
//		float Alpha  = 0.001;
//		Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;
		
		if (TIM_GetITStatus(TIM1,TIM_IT_Update) == SET )
		{
			//中断重叠标志位
			TimerErrorFlag = 1;
			//清除标志位
			TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
		}
		TimerCount = TIM_GetCounter(TIM1);
	}
}
