#include "stm32f10x.h"                  // Device header

#include "Delay.h"
#include "Timer.h"

#include "OLED.h"
#include "LED.h"
#include "MPU6050.h"//陀螺仪模块
#include "Servo.h"
#include "Motor.h"
#include "Serial.h"
#include "TCS34725.h"//颜色识别模块
#include "TCS34725_IIC.h"
#include "HCSR04.h"//超声波模块
#include "Car.h"

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
#define LEFT_90		5
#define RIGHT_90	6
#define STOP		7
/* ==================== [END] 蓝牙虚拟按键相关变量定义 [END] ==================== */




/* =================== [START] LED响应颜色识别模块 [START]==================== */
#define OTHERS		0
#define RED			1
#define GREEN		2
#define BLUE		3
#define BLACK		4

uint8_t Color_Flag = OTHERS;
/* =================== [END] LED响应颜色识别模块 [END]==================== */




RGB rgb;					//颜色结构体

uint16_t HCSR04_Distance=0;  //超声波测到的距离

float Car_Tar_Yaw = 0.0f;
uint8_t Car_Turn_ENABLE = 0;
uint8_t Car_Turn_Count = 0;
uint8_t Cur_Flag, Pre_Flag;

int main(void)
{
	OLED_Init();
	LED_Init();
	Serial_Init();
	MPU6050_Init();
	Motor_Init();
	HCSR04_Init();
	
	LED_OFF_ALL();
	
	Serial_RxFlag = 0;
	
	//颜色识别模块TCS34725
	TCS34725_GPIO_Init();		//颜色传感器GPIO初始化
	TCS34725_Init();			//颜色传感器初始化
	integrationTime(33);		//积分时间

	Timer_Init();
	
	//舵机角度
	uint8_t Servo_Angle = 90;
	Servo_SetAngle(Servo_Angle);
	
	
	
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
				else if (strcmp(Name, "LEFT_90") == 0 && strcmp(Action, "down") == 0)
				{
					Key_Event = LEFT_90;
					Car_Tar_Yaw = Yaw + 90.0f;
					Car_Turn_ENABLE = 1;
				}
				else if (strcmp(Name, "RIGHT_90") == 0 && strcmp(Action, "down") == 0)
				{
					Key_Event = RIGHT_90;
					Car_Tar_Yaw = Yaw - 90.0f;
					Car_Turn_ENABLE = 1;
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
					Servo_Angle = 180 - IntValue;
					
					Servo_SetAngle(Servo_Angle);
				}
			}
			Serial_RxFlag = 0;
		}
		/* =================== [END] 蓝牙收发与处理模块 [END]==================== */
		
		
		MPU6050_Resolving_ENABLE = 1;
		
		/* =================== [START] 控制模块 [START]==================== */
		switch(Key_Event)
		{
			case 0:
				Car_Stop();
				Cur_Flag = F_Car_Stop;
				break;
			
			case UP:
				Go_Ahead();
				Cur_Flag = F_Go_Ahead;
				break;
			
			case DOWN:
				Go_Back();
				Cur_Flag = F_Go_Back;
				break;
			
			case LEFT:
				Self_Left();
				Cur_Flag = F_Self_Left;
				break;
			
			case RIGHT:
				Self_Right();
				Cur_Flag = F_Self_Right;
				break;
			
//			case LEFT_90:
//				Self_Right();
//				Car_Tar_Yaw = Yaw + 90.0f;
//				Car_Turn_ENABLE = 1;

//				break;
//			
//			case RIGHT_90:
//				Self_Right();
//				Car_Tar_Yaw = Yaw - 90.0f;
//				Car_Turn_ENABLE = 1;
//				break;
			
			case STOP:
				Car_Stop();
				Cur_Flag = F_Car_Stop;
				break;
			
			default:
				
				break;		
		}
		if(Car_Turn_ENABLE){
			if (Car_Turn_Count >= 2)
			{
				Car_Turn_Count = 0;
				Car_Stop();
				Car_Turn_ENABLE = 0;
			}
			else if(Yaw - Car_Tar_Yaw > 1.0f)
			{
				Self_Right();
				Cur_Flag = F_Self_Right;
				if (Pre_Flag != Cur_Flag)Car_Turn_Count ++;
			}
			else if(Car_Tar_Yaw - Yaw > 1.0f)
			{
				Self_Left();
				Cur_Flag = F_Self_Left;
				if (Pre_Flag != Cur_Flag)Car_Turn_Count ++;
			}
			else
			{
				Car_Turn_Count = 0;
				Car_Stop();
				Car_Turn_ENABLE = 0;
			}
		}
		if (Pre_Flag != Cur_Flag)Pre_Flag = Cur_Flag;
		/* =================== [END] 控制模块 [END]==================== */		
		

		Serial_Printf("[display,0,0,Yaw]");
		Serial_Printf("[display,0,20,%+02.3f  ]", Yaw);
		Serial_Printf("[display,0,40,R     G     B]");
		Serial_Printf("[display,0,60,%3d   %3d   %3d   ]", R_Dat, G_Dat, B_Dat);
		Serial_Printf("[display,0,80,S_Angle]");
		Serial_Printf("[display,0,100,%d  ]", Servo_Angle);
		
		HCSR04_Distance = HCSR04_GetValue();
		
		Serial_Printf("[display,0,120,HCSR04]");
		Serial_Printf("[display,0,140,%d  ]", HCSR04_Distance);
//		Serial_Printf("%f,%f,%f\r\n", Roll, Yaw,Pitch);
		
		/* =================== [START] LED响应颜色识别模块 [START]==================== */
		if (0)//红
		{
			Color_Flag = RED;
			LED_OFF_ALL();
			LED_ON_SET(3);
		}
		else if(0)//绿
		{
			Color_Flag = GREEN;
			LED_OFF_ALL();
			LED_ON_SET(2);			
		}
		else if(0)//蓝
		{
			Color_Flag = BLUE;
			LED_OFF_ALL();
			LED_ON_SET(1);			
		}
		else if(0)//黑
		{
			Color_Flag = BLACK;
			LED_ON_ALL();
		}
		else //其他
		{
			Color_Flag = OTHERS;
			LED_OFF_ALL();
		}		
		/* =================== [END] LED响应颜色识别模块 [END]==================== */
	}
}

uint16_t TimeTick;

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
		TimeTick ++;

		//超声波模块HCSR04进程
		HCSR04_Tick();
		

		if(TimeTick >= 100)
		{
			TimeTick = 0;
			//颜色识别模块TCS34725
			rgb=TCS34725_Get_RGBData();
			RGB888=TCS34725_GetRGB888(rgb);//将原始数据转化为RGB888格式
			RGB565=TCS34725_GetRGB565(rgb);//将原始数据转化为RGB565格式
			Dis_Temp();//转化为可读颜色数据
		}
		
		/* =================== [START] MPU6050解算模块 [START]==================== */
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
			if (GZ <= -2 || 2 <= GZ){Yaw += GZ / 32768.0 * 2000 * 0.001;}

//			// 俯仰角计算
//			PitchAcc = -atan2(AX, AZ) / 3.14159 * 180;  			// 俯仰角（绕Y轴）
//			PitchGyro = Pitch + GY / 32768.0 * 2000 * 0.001;  		// 陀螺仪积分（2000是量程，0.001是1ms采样间隔）
//			Pitch = 0.001 * PitchAcc + (1 - 0.001) * PitchGyro;  	// 互补滤波
		}
		/* =================== [END] MPU6050解算模块 [END]==================== */
		
	}
}
