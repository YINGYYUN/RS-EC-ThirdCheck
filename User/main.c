#include "stm32f10x.h"                  // Device header

#include "Delay.h"
#include "Timer.h"

#include "OLED.h"
//#include "LED.h"
#include "MPU6050.h"
#include "Servo.h"
#include "Motor.h"
#include "Serial.h"

#include "TCS34725.h"//颜色传感器
#include "TCS34725_IIC.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>




/* ==================== [START] MPU6050相关变量定义 [START] ==================== */
//解算开关标志位
uint8_t MPU6050_Resolving_ENABLE = 0;

//MPU6050读取接收
//int16_t AX, AY, AZ, GX, GY, GZ;
int16_t GZ;

uint8_t MPU6050_ENABLE = 0;

//float RollAcc;    		// 加速度计计算的横滚角
//float RollGyro;   		// 陀螺仪积分的横滚角
//float Roll;       		// 融合后的横滚角

float Yaw = 0;			//偏航角

//float PitchAcc;			//加速度计算的俯仰角
//float PitchGyro;		//陀螺仪积分的俯仰角
//float Pitch;			//融合后的俯仰角	
/* ==================== [END] MPU6050相关变量定义 [END] ==================== */




/* ==================== [START] 蓝牙虚拟按键相关变量定义 [START] ==================== */
uint8_t Key_Event = 0;

#define UP			1
#define DOWN		2
#define LEFT		3
#define RIGHT		4
#define STOP		5
/* ==================== [END] 蓝牙虚拟按键相关变量定义 [END] ==================== */




//舵机角度
float S_Angle = 90.0f;


int main(void)
{
	OLED_Init();
//	LED_Init();
	Serial_Init();
	MPU6050_Init();
	Servo_Init();
	Motor_Init();
	
	Timer_Init();
	
//	LED_SetMode(LED_OFFMode);
	
	Serial_RxFlag = 0;
	
	//颜色识别模块TCS34725
	RGB rgb;					//结构体
	TCS34725_GPIO_Init();		//颜色传感器GPIO初始化
	TCS34725_Init();			//颜色传感器初始化
	integrationTime(33);		//积分时间

	
	while(1)
	{
		/* =================== [START] 蓝牙收发与处理模块 [START]==================== */		
		if (Serial_RxFlag == 1)
		{			
			//字符串分割
			char * Tag = strtok(Serial_RxPacket, ",");
			
			//按键解析
			if (strcmp(Tag, "key") == 0)
			{
				//NULL表示为后续分割
				char * Name = strtok(NULL, ",");
				//如果没有新的子串，函数会返回空指针
				char * Action = strtok(NULL, ",");
				
				if (strcmp(Action, "up") == 0)
				{
					Key_Event = 0;
				}
				else if (strcmp(Name, "UP") == 0 && strcmp(Action, "down") == 0)
				{
					Key_Event = UP;
				}
				else if (strcmp(Name, "DOWN") == 0 && strcmp(Action, "down") == 0)
				{
					Key_Event = DOWN;
				}
				else if (strcmp(Name, "LEFT") == 0 && strcmp(Action, "down") == 0)
				{
					Key_Event = LEFT;
				}
				else if (strcmp(Name, "RIGHT") == 0 && strcmp(Action, "down") == 0)
				{
					Key_Event = RIGHT;
				}
				else if (strcmp(Name, "STOP") == 0 && strcmp(Action, "down") == 0)
				{
					Key_Event = STOP;
				}
			}
			
			//滑杆解析
			else 
			if (strcmp(Tag, "slider") == 0)
			{
				char * Name = strtok(NULL, ",");
				char * Value = strtok(NULL, ",");
				
//				if (strcmp(Name, "KP") == 0)
//				{
//					float FloatValue = atof(Value);					
//					KP = FloatValue;
//				}
//				else
//				if (strcmp(Name, "KI") == 0)
//				{
//					float FloatValue = atof(Value);				
//					KI = FloatValue;
//				}
//				else 
//				if (strcmp(Name, "KD") == 0)
//				{
//					float FloatValue = atof(Value);				
//					KD = FloatValue;
//				}
				if (strcmp(Name, "S_Angle") == 0)
				{
					int IntValue = atoi(Value);				
					S_Angle = IntValue;
					
					Servo_SetAngle(S_Angle);
				}
			}
			Serial_RxFlag = 0;
		}
		/* =================== [END] 蓝牙收发与处理模块 [END]==================== */
		
		
		MPU6050_Resolving_ENABLE = 1;
		
		/* =================== [START] 手动控制模块 [START]==================== */		
		switch(Key_Event)
		{
			case 0:
				Motor_SetPWM1(0);
				Motor_SetPWM2(0);
				break;
			
			case UP:
				Motor_SetPWM1(90);
				Motor_SetPWM2(90);
				break;
			
			case DOWN:
				Motor_SetPWM1(-90);
				Motor_SetPWM2(-90);
				break;
			
			case LEFT:
				Motor_SetPWM1(-90);
				Motor_SetPWM2(90);
				break;
			
			case RIGHT:
				Motor_SetPWM1(90);
				Motor_SetPWM2(-90);
				break;
			
			case STOP:
				Motor_SetPWM1(0);
				Motor_SetPWM2(0);
				break;
			
			default:
				
				break;		
		}
		/* =================== [END] 手动控制模块 [END]==================== */		
		
		rgb=TCS34725_Get_RGBData();
		RGB888=TCS34725_GetRGB888(rgb);//将原始数据转化为RGB888格式
		RGB565=TCS34725_GetRGB565(rgb);//将原始数据转化为RGB565格式
		Dis_Temp();//转化为可读颜色数据
		Serial_Printf("[display,0,0,Yaw]");
		Serial_Printf("[display,0,20,%+02.3f  ]", Yaw);
		Serial_Printf("[display,0,40,R     G     B]");
		Serial_Printf("[display,0,60,%3d   %3d   %3d   ]", R_Dat, G_Dat, B_Dat);
		
		
//		Serial_Printf("%f,%f,%f\r\n", Roll, Yaw,Pitch);
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
		
//		LED_Tick();
		
		if(MPU6050_Resolving_ENABLE)//启用MPU6050
		{
			MPU6050_GetGZ(&GZ);
			
			//校准零飘
//			GX += 55;
//			GY += 18;
			GZ += 10;
		
//			// 横滚角计算
//			RollAcc = atan2(AY, AZ) / 3.14159 * 180;  				// 横滚角（绕X轴）
//			RollGyro = Roll + GX / 32768.0 * 2000 * 0.001;  		// 陀螺仪X轴积分
//			Roll = 0.001 * RollAcc + (1 - 0.001) * RollGyro;  		// 相同互补滤波算法
			
			// 偏航角：仅陀螺仪积分（无加速度计校准，会漂移）
			Yaw += GZ / 32768.0 * 2000 * 0.001;  // 仅积分，无校准
			
//			// 俯仰角计算
//			PitchAcc = -atan2(AX, AZ) / 3.14159 * 180;  			// 俯仰角（绕Y轴）
//			PitchGyro = Pitch + GY / 32768.0 * 2000 * 0.001;  		// 陀螺仪积分（2000是量程，0.001是1ms采样间隔）
//			Pitch = 0.001 * PitchAcc + (1 - 0.001) * PitchGyro;  	// 互补滤波
			
		}

	}
}
