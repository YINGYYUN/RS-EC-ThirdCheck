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




/* ==================== [START] (全模式)MPU6050姿态解算相关变量定义 [START] ==================== */
//MPU6050原始数据读取接收
//int16_t AX, AY, AZ, GX, GY, GZ;
int16_t GZ;

//float RollAcc;    		// 加速度计计算的横滚角
//float RollGyro;   		// 陀螺仪积分的横滚角
//float Roll;       		// 融合后的横滚角

float Yaw = 0;				//偏航角

//float PitchAcc;			//加速度计算的俯仰角
//float PitchGyro;			//陀螺仪积分的俯仰角
//float Pitch;				//融合后的俯仰角	
/* ==================== [END] (全模式)MPU6050姿态解算相关变量定义 [END] ==================== */




/* ==================== [START] (全模式)运动状态控制相关变量定义 [START] ==================== */
//小车运行状态标志位
uint8_t Car_Movtion_Event = 0;
//(自动模式)小车运行状态标志位
uint8_t Car_Movtion_Event_History [40] = {0};
uint8_t Car_Movtion_Count = 0;
//uint8_t Car_Turn_Event_History [40] = {0};
#define R_Priority			0
#define L_Priority			1
uint8_t Car_Turn_Mode = R_Priority;
uint8_t Corridor_Flag = 0;
uint8_t Corridor_Count = 0;

//uint8_t Car_Turn_Schedule_History [30] = {0};
//uint8_t Car_Turn_Count = 0;
#define STOP				0
#define UP					1
#define DOWN				2
#define LEFT				3
#define RIGHT				4
#define LEFT_90				5
#define RIGHT_90			6
#define AROUND				7


//(自动模式)定时直行请求挂起标志位
uint8_t Car_StraightRun_Falg = 0;

//定角度转向请求挂起标志位
uint8_t Car_Turn_ENABLE = 0;
//转向目标角度存储
float Car_Tar_Yaw = 0.0f;
//旋转超时计时
uint16_t Car_Turn_TimeTick = 0;

//定时运动计时和标志位
uint16_t Car_Movtion_Delay_TimeTick = 0;
uint8_t Car_Movtion_Delay_Flag = 0;
/* ==================== [END] (全模式)运动状态控制相关变量定义 [END] ==================== */




/* =================== [START] (全模式)颜色识别模块 [START]==================== */
RGB rgb;					//颜色结构体

#define OTHERS		0
#define RED			1
#define GREEN		2
#define BLUE		3
#define BLACK		4

//颜色分类判定
uint8_t Color_Flag = OTHERS;
/* =================== [END] (全模式)颜色识别模块 [END]==================== */




/* =================== [START] (全模式)舵机旋转模块 [START]==================== */
//舵机当前角度（每次上电后自动旋转至该角度）
uint8_t Cur_Servo_Angle = 90;

//舵机目标角度
//uint8_t Tar_Servo_Angle = 90;

//舵机运行时基
uint16_t Servo_TimeTick = 0;

//舵机旋转标志位
//01234代表旋转本身的不同阶段，防止频繁发出旋转指令
uint8_t Servo_State = 0;

//舵机旋转测距标志位
//0		不运行
//1		请求挂起
//2		测距完成，等待复位
uint8_t Servo_Turn_Flag = 0;
/* =================== [END] (全模式)舵机旋转模块 [END]==================== */




//超声波测到的距离
//[0]左
//[1]前
//[2]右
uint8_t HCSR04_Distance[3] = {0, 0, 0};
//准备给测量滤波的输入
//[0]无效值
//[1]第一次
//[2]第二次
//[3]第三次
uint16_t HCSR04_History[4] = {0, 0, 0, 0};
uint16_t HCSR04_Sample_TimeTick = 0;
uint8_t HCSR04_Sample_Count = 0;
uint8_t HCSR04_Sample_State = 0;
uint8_t HCSR04_Sample_ENABLE = 0;
uint8_t HCSR04_Target = 0;

uint16_t Force_ENABLE = 0;

int main(void)
{
	OLED_Init();
	LED_Init();
	Serial_Init();
	MPU6050_Init();
	Motor_Init();
	HCSR04_Init();
	
	LED_A_OFF_ALL();
	
	Serial_RxFlag = 0;
	
	//颜色识别模块TCS34725
	TCS34725_GPIO_Init();		//颜色传感器GPIO初始化
	TCS34725_Init();			//颜色传感器初始化
	integrationTime(33);		//积分时间

	Timer_Init();
	
	Servo_SetAngle(Cur_Servo_Angle);
	
	
	
	
	/* =================== [START] (全模式)菜单初始化模块 [START] =================== */	
	char Mode_Menu[][15] = {"Manual Mode  ", "Auto Mode    "};
	#define Flag_Manual_Mode			0
	#define Flag_Auto_Mode				1
	
	//功能状态
	uint8_t FUNCTION_State = Flag_Manual_Mode;
	
	Serial_Printf("[display-clear]");
	Serial_Printf("[display,0,0,FUNCTION State]");
	Serial_Printf("[display,0,20,%s]", Mode_Menu[FUNCTION_State]);
	Serial_Printf("[display,0,40,Yaw]");
	Serial_Printf("[display,0,80, R   G   B]");
//	Serial_Printf("[display,0,120,S_Angle]");
	Serial_Printf("[display,0,160,HCSR04]");
	Serial_Printf("[display,0,200,     |     ]");
	/* =================== [END] (全模式)菜单初始化模块 [END] =================== */	
	
	
	
	
	while(1)
	{
//		//时间间隔测试保留
//		if (Car_Movtion_Delay_TimeTick == 0 && Car_Movtion_Delay_Flag == 1)
//		{
//			Car_Stop();
//			Car_Movtion_Event = STOP;
//			Car_Turn_ENABLE = 0;
//		}
		
		
		
		
		/* =================== [START] (全模式)蓝牙收发与处理模块 [START]==================== */		
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
				
				//手动/自动模式切换
				if (strcmp(Name, "MODE") == 0 && strcmp(Action, "down") == 0)
				{				
					//撤回可能存在的直行请求
					Car_StraightRun_Falg = 0;
					
					//撤回可能存在的定角度转向请求
					Car_Turn_TimeTick = 0;
					Car_Turn_ENABLE = 0;
					Car_Stop();
					
					Car_Movtion_Event = STOP;
					
					//模式标志位更改
					FUNCTION_State = 1 - FUNCTION_State;
					Serial_Printf("[display,0,20,%s]", Mode_Menu[FUNCTION_State]);
					
					//自动模式初始
					if(FUNCTION_State == Flag_Auto_Mode)
					{
						//舵机先旋转测距
						HCSR04_Distance[0] = 0;
						HCSR04_Distance[1] = 0;
						HCSR04_Distance[2] = 0;
						
						Servo_Turn_Flag = 1;
						Servo_State = 0;
						Cur_Servo_Angle = 90;
						Servo_SetAngle(Cur_Servo_Angle);
						Servo_TimeTick = 5580;				
					}
					else
					{
						Servo_Turn_Flag = 0;
					}
				}
				
				//手动控制按键状态解析及响应
				if (FUNCTION_State == Flag_Manual_Mode)
				{
					if (strcmp(Action, "up") == 0)
					{
						//只有手动运动控制按键存在松开判定						
						Car_Stop();
						Car_Movtion_Event = STOP;
						Car_Turn_ENABLE = 0;
					}
					else 
					if (strcmp(Name, "UP") == 0 && strcmp(Action, "down") == 0)
					{
						Go_Ahead_SET(95);
						Car_Movtion_Event = UP;		
						
//						//时间间隔测试保留
//						Car_Movtion_Delay_Flag = 1;
//						Car_Movtion_Delay_TimeTick = 1400;
						
					}
					else if (strcmp(Name, "DOWN") == 0 && strcmp(Action, "down") == 0)
					{
						Go_Back_SET(95);
						Car_Movtion_Event = DOWN;				
					}
					else if (strcmp(Name, "LEFT") == 0 && strcmp(Action, "down") == 0)
					{
						Self_Left();
						Car_Movtion_Event = LEFT;					
					}
					else if (strcmp(Name, "RIGHT") == 0 && strcmp(Action, "down") == 0)
					{
						Self_Right();
						Car_Movtion_Event = RIGHT;						
					}
					else if (strcmp(Name, "L_90") == 0 && strcmp(Action, "down") == 0)
					{					
						//发送定角度转向请求
						Car_Tar_Yaw = Yaw + 90.0f;
						Car_Turn_ENABLE = 1;
						Car_Turn_TimeTick = 4000;
						Car_Movtion_Event = LEFT_90;
					}
					else if (strcmp(Name, "R_90") == 0 && strcmp(Action, "down") == 0)
					{						
						//发送定角度转向请求
						Car_Tar_Yaw = Yaw - 90.0f;
						Car_Turn_ENABLE = 1;
						Car_Turn_TimeTick = 4000;
						Car_Movtion_Event = RIGHT_90;
					}
					else if (strcmp(Name, "STOP") == 0 && strcmp(Action, "down") == 0)
					{						
						Car_Stop();
						Car_Turn_ENABLE = 0;
						
						Car_Movtion_Event = STOP;
					}
					else if (strcmp(Name, "Servo") == 0 && strcmp(Action, "down") == 0)
					{
						Servo_Turn_Flag = 1;
						Servo_State = 1;
						Force_ENABLE = 1;
						Servo_TimeTick = 4580;
						HCSR04_Sample_ENABLE=0; 
						HCSR04_Sample_State=0;
					}
				}
			}
			
			//滑杆解析
			else 
			if (strcmp(Tag, "slider") == 0)
			{				
				char * Name = strtok(NULL, ",");
				char * Value = strtok(NULL, ",");
				
				if (FUNCTION_State == Flag_Manual_Mode && strcmp(Name, "S_Angle") == 0)
				{
					int IntValue = atoi(Value);				
					Cur_Servo_Angle = IntValue;
					
					Servo_SetAngle(Cur_Servo_Angle);
					
					//舵机测距回传标示
					//前
					if (Cur_Servo_Angle == 90)
					{
						Serial_Printf("[display,0,200,     |     ]");
					}
					//左
					else if (Cur_Servo_Angle > 90)
					{
						Serial_Printf("[display,0,200, |         ]");
					}
					//右
					else 
					{
						Serial_Printf("[display,0,200,         | ]");
					}			
				}
			}
			
			//摇杆解析不启用，代码部分删除
			
			Serial_RxFlag = 0;
		}
		/* =================== [END] (全模式)蓝牙收发与处理模块 [END]==================== */
		
		
		
		
		/* =================== [START] (全模式)小车定角度旋转响应模块 [START]==================== */
		//接收到定角度旋转请求
		if(Car_Turn_ENABLE)
		{
			//超时强制叫停
			if (Car_Turn_TimeTick == 0)
			{
				Car_Stop();
				Car_Turn_ENABLE = 0;
				Car_Movtion_Event = STOP;
				
				//自动模式下尝试强制直行
				if (FUNCTION_State == Flag_Auto_Mode)
				{
					Car_StraightRun_Falg = 1;
					Car_Movtion_Delay_TimeTick = 1400;
					Car_Movtion_Event = UP;		
				}
			}
			//旋转
			else
			{
				float yaw_diff = Car_Tar_Yaw - Yaw;
				float abs_diff = fabs(yaw_diff);
				
				//基于与目标差值的二级旋转速度
				uint8_t turn_pwm = (abs_diff > 7.0f) ? 99 : 85;
				
				 if(yaw_diff > 0.5f)  // 偏左>0.5°，需左转
				{
					Self_Left_SET(turn_pwm);
				}
				else if(yaw_diff < -0.5f) // 偏右>0.5°，需右转
				{
					Self_Right_SET(turn_pwm);
				}
				else
				{
					//完成定角度旋转
					Car_Stop();
					Car_Turn_ENABLE = 0;
					Car_Turn_TimeTick = 0;
					
					//自动模式下尝试强制直行
					if (FUNCTION_State == Flag_Auto_Mode)
					{
						Car_StraightRun_Falg = 1;
						Car_Movtion_Delay_TimeTick = 1400;
						Car_Movtion_Event = UP;		
					}
				}
			}
		}
		/* =================== [END] (全模式)小车定角度旋转响应模块 [END]==================== */		
		

		
		
		/* =================== [START] (全模式)传感器数据自动回传模块 [START]==================== */
//		Serial_Printf("[display,0,60,%d  ]",(int)GZ);
		Serial_Printf("[display,0,60,%+02.3f  ]", Yaw);
		Serial_Printf("[display,0,100,%03d %03d %03d]", R_Dat, G_Dat, B_Dat);
//		Serial_Printf("[display,0,140,%d ]", Cur_Servo_Angle);
		if (FUNCTION_State == Flag_Manual_Mode)
		{
			Serial_Printf("[display,0,180,%03d,%03d,%03d  ]", HCSR04_Distance[0], HCSR04_Distance[1], HCSR04_Distance[2]);
		}
		/* =================== [END] (全模式)传感器数据自动回传模块 [END]==================== */
		
		
		
		
		/* =================== [START] (全模式)LED自动响应颜色识别模块 [START]==================== */
		//根据RGB判定
		if (0)//红
		{
			Color_Flag = RED;
			LED_A_OFF_ALL();
			LED_ON_SET(3);
		}
		else if(0)//绿
		{
			Color_Flag = GREEN;
			LED_A_OFF_ALL();
			LED_ON_SET(2);
		}
		else if(0)//蓝
		{
			Color_Flag = BLUE;
			LED_A_OFF_ALL();
			LED_ON_SET(1);
		}
		else if(0)//黑
		{
			Color_Flag = BLACK;
			LED_A_ON_ALL();
		}
		else //其他
		{
			Color_Flag = OTHERS;
			LED_A_OFF_ALL();
		}		
		/* =================== [END] (全模式)LED自动响应颜色识别模块 [END]==================== */
		
		
		
		
		//手动模式
		if (FUNCTION_State == Flag_Manual_Mode && Force_ENABLE == 0)
		{
			
			
			
						
			/* =================== [START] (手动模式)舵机测距数据分配模块 [START]==================== */
			if (Cur_Servo_Angle == 90)
			{
				HCSR04_Distance[1] = HCSR04_GetValue();
			}
			else if (Cur_Servo_Angle > 90)
			{
				HCSR04_Distance[0] = HCSR04_GetValue();
			}
			else 
			{
				HCSR04_Distance[2] = HCSR04_GetValue();
			}			
			/* =================== [START] (手动模式)舵机测距数据分配模块 [START]==================== */
			
			
			
						
		}
		
		//自动模式
		if (FUNCTION_State == Flag_Auto_Mode || Force_ENABLE)
		{
			//优先级：定角度旋转>旋转测距>直行
			//定角度旋转屏蔽测距
			//遇墙抢断优先（）
			
			
			
			
			/* =================== [START] (自动模式)自动测距模块 [START]==================== */
			//HCSR04_Distance有三个元素，分别对应	左[0]	中[1]	右[2]	
			
			//舵机和小车无旋转
			if (Servo_Turn_Flag == 0 && Car_Turn_ENABLE == 0)
			{
				HCSR04_Distance[1] = HCSR04_GetValue();
				Serial_Printf("[display,0,180,%03d,%03d,%03d  ]", HCSR04_Distance[0], HCSR04_Distance[1], HCSR04_Distance[2]);
				//到墙
				if (Car_Movtion_Delay_TimeTick <= 1200 && HCSR04_Distance[1] <= 7 && HCSR04_Distance[1] != 0)
				{
					Car_StraightRun_Falg = 0;
					Car_Stop();
					Car_Movtion_Event = STOP;
					Servo_Turn_Flag = 1;
					Servo_TimeTick = 4580;
					Servo_State = 0;			
				}
			}
			//运行舵机左前右测距请求
			if(Servo_Turn_Flag == 1)
			{
				//模式切换后存在额外的一次旋转等待，这里的代码段是没有体现的
				if	(4520 < Servo_TimeTick && Servo_TimeTick <= 4580 && Servo_State == 0)
				{
					if (HCSR04_Sample_ENABLE == 0) 
					{
						HCSR04_Sample_ENABLE = 1;
						HCSR04_Sample_State = 0;
						HCSR04_Target = 1;
					}
					Servo_State = 1;
				}	
				else if (3520 < Servo_TimeTick && Servo_TimeTick <= 4520 && Servo_State == 1)
				{
					Servo_SetAngle(180);
					Servo_State = 2;
				}
				else if (3320 < Servo_TimeTick && Servo_TimeTick <= 3520 && Servo_State == 2)
				{
					if (HCSR04_Sample_ENABLE == 0) 
					{
						HCSR04_Sample_ENABLE = 1;
						HCSR04_Sample_State = 0;
						HCSR04_Target = 0;
					}
					Servo_State = 3;
				}
				else if (1320 < Servo_TimeTick && Servo_TimeTick <= 3320 && Servo_State == 3)
				{					
					Servo_SetAngle(0);
					Servo_State = 4;
				}
				else if (1120 < Servo_TimeTick && Servo_TimeTick <= 1320 && Servo_State == 4)
				{		
					if (HCSR04_Sample_ENABLE == 0) 
					{
						HCSR04_Sample_ENABLE = 1;
						HCSR04_Sample_State = 0;
						HCSR04_Target = 2;
					}
					Servo_State = 5;
				}
				else if (120 < Servo_TimeTick && Servo_TimeTick <= 1120 && Servo_State == 5)
				{					
					Servo_SetAngle(90);
					Servo_State = 6;
				}
				else if (0 < Servo_TimeTick && Servo_TimeTick <= 120 && Servo_State == 6)
				{					
					HCSR04_StartMeasure();
					Servo_State = 7;
				}
				else if (Servo_TimeTick == 0 && Servo_State == 7)
				{
					//等待重置状态
					Serial_Printf("[display,0,180,%03d,%03d,%03d  ]", HCSR04_Distance[0], HCSR04_Distance[1], HCSR04_Distance[2]);
					Servo_Turn_Flag = 2;
					Force_ENABLE = 0;
				}
			}
			/* =================== [END] (自动模式)自动测距模块 [END]==================== */
			
			
			
		}
		if (FUNCTION_State == Flag_Auto_Mode)
		{
			
			
			
			
			/* =================== [START] (自动模式)寻路决策模块 [START]==================== */		
			//测距完成，发出对应动作请求
			if (Servo_Turn_Flag == 2)		
			{
				//一层：判断
				if(Car_Turn_Mode == R_Priority)
				{
					//右
					if (HCSR04_Distance[2] >= 18)
					{		
						Car_Movtion_Event = RIGHT_90;
//						Car_Turn_Event_History [Car_Turn_Count] = RIGHT_90;
//						Car_Turn_Count ++;			
						Car_Movtion_Event_History [Car_Movtion_Count] = RIGHT_90;
						Car_Movtion_Count ++;
					}
					//前
					else if (HCSR04_Distance[1] >= 10)
					{								
						Car_Movtion_Event = UP;
						Car_Movtion_Event_History [Car_Movtion_Count] = UP;
						Car_Movtion_Count ++;
					}
					//左
					else if (HCSR04_Distance[0] >= 18)
					{										
						Car_Movtion_Event = LEFT_90;
//						Car_Turn_Event_History [Car_Turn_Count] = LEFT_90;
//						Car_Turn_Count ++;
						Car_Movtion_Event_History [Car_Movtion_Count] = LEFT_90;
						Car_Movtion_Count ++;
					}
					//后
					else 
					{										
						Car_Movtion_Event = AROUND;
						Car_Movtion_Event_History [Car_Movtion_Count] = AROUND;
						Car_Movtion_Count ++;
					}
				}
				else if(Car_Turn_Mode == L_Priority)
				{
					//左
					if (HCSR04_Distance[0] >= 18)
					{										
						Car_Movtion_Event = LEFT_90;
//						Car_Turn_Event_History [Car_Turn_Count] = LEFT_90;
//						Car_Turn_Count ++;
						Car_Movtion_Event_History [Car_Movtion_Count] = LEFT_90;
						Car_Movtion_Count ++;
					}
					//前
					else if (HCSR04_Distance[1] >= 10)
					{								
						Car_Movtion_Event = UP;
						Car_Movtion_Event_History [Car_Movtion_Count] = UP;
						Car_Movtion_Count ++;
					}
					//右					
					else if (HCSR04_Distance[2] >= 18)
					{		
						Car_Movtion_Event = RIGHT_90;
//						Car_Turn_Event_History [Car_Turn_Count] = RIGHT_90;
//						Car_Turn_Count ++;
						Car_Movtion_Event_History [Car_Movtion_Count] = RIGHT_90;
						Car_Movtion_Count ++;
					}
					//后
					else 
					{										
						Car_Movtion_Event = AROUND;
						Car_Movtion_Event_History [Car_Movtion_Count] = AROUND;
						Car_Movtion_Count ++;
					}
				}
				
				if(HCSR04_Distance[1] >= 50 && Corridor_Flag == 0)
				{					
					Corridor_Flag = 1;
				}
				if(Corridor_Flag == 1 && Car_Movtion_Event != UP)
				{
					Corridor_Count ++;
				}				
				if(Corridor_Count == 5)
				{
					Car_Turn_Mode = L_Priority;
				}
//				//介入方向控制（标记式）
//				if (Car_Turn_Schedule_History[Car_Turn_Count] != 0 &&
//					Car_Turn_Schedule_History[Car_Turn_Count] != Car_Turn_Event_History [Car_Turn_Count])
//				{
//					Car_Movtion_Event = Car_Turn_Schedule_History[Car_Turn_Count];
//				}
				
				//二层：执行
				switch(Car_Movtion_Event)
				{
					case UP:
					{
						Car_StraightRun_Falg = 1;
						Car_Movtion_Delay_TimeTick = 1400;

						break;
					}
					
					case AROUND:
					{
						//发送定角度转向请求
						Car_Tar_Yaw = Yaw - 180.0f;
						Car_Turn_ENABLE = 1;
						Car_Turn_TimeTick =  7000;
						
						break;
					}						
					
					case RIGHT_90:
					{
						//发送定角度转向请求
						Car_Tar_Yaw = Yaw - 90.0f;
						Car_Turn_ENABLE = 1;
						Car_Turn_TimeTick = 2200;
															
						break;
					}
					
					case LEFT_90:
					{
						//发送定角度转向请求
						Car_Tar_Yaw = Yaw + 90.0f;
						Car_Turn_ENABLE = 1;
						Car_Turn_TimeTick = 2200;
						
						break;
					}
				}				
				Servo_Turn_Flag = 0;
			}
			/* =================== [END] (自动模式)寻路决策模块 [END]==================== */
			
			
			
			
			/* =================== [START] (自动模式)直行请求响应模块 [START]==================== */
			//在小车定角度旋转结束的前提下
			if (Car_Turn_ENABLE == 0 && Car_StraightRun_Falg == 1)
			{
				Go_Ahead_SET(95);
				Car_Movtion_Event = UP;
				Car_StraightRun_Falg = 2;
			}
			else if (Car_StraightRun_Falg == 2)
			{
				//直行超时
				if (Car_Movtion_Delay_TimeTick == 0)
				{
					//停车
					Car_Stop();
					Car_Movtion_Event = STOP;
					Car_StraightRun_Falg = 0;
					
					//开始舵机旋转测距
					Servo_Turn_Flag = 1;
					Servo_TimeTick = 4580;
					Servo_State = 0;	
				}
			}			
			/* ===================  [END] (自动模式)直行请求响应模块 [END]==================== */
			
			
			
			
		}
		
		
		
		
		/* =================== [START] ()超声波数据采样模块 [START]==================== */
		if (HCSR04_Sample_ENABLE)
		{
			if (HCSR04_Sample_State == 0)
			{
				//空数据
				HCSR04_History[0] = HCSR04_GetValue();
				HCSR04_StartMeasure();
				HCSR04_Sample_Count = 0;
				HCSR04_History[1]=HCSR04_History[2]=HCSR04_History[3]=0;
				HCSR04_Sample_TimeTick = 62;
				
				HCSR04_Sample_State = 1;
			}
			else if (HCSR04_Sample_State == 1 && HCSR04_Sample_TimeTick == 0)
			{
				HCSR04_History[1] = HCSR04_GetValue();
				HCSR04_StartMeasure();
				if (HCSR04_History[1]) {HCSR04_Sample_Count ++;}
				HCSR04_Sample_TimeTick = 62;
				
				HCSR04_Sample_State = 2;
			}
			else if (HCSR04_Sample_State == 2 && HCSR04_Sample_TimeTick == 0)
			{
				HCSR04_History[2] = HCSR04_GetValue();
				HCSR04_StartMeasure();
				if (HCSR04_History[2]) {HCSR04_Sample_Count ++;}
				HCSR04_Sample_TimeTick = 62;
				
				HCSR04_Sample_State = 3;
			}
			else if (HCSR04_Sample_State == 3 && HCSR04_Sample_TimeTick == 0)
			{
				HCSR04_History[3] = HCSR04_GetValue();
				if (HCSR04_History[3]) {HCSR04_Sample_Count ++;}								
				if (HCSR04_Sample_Count)
				{
					HCSR04_Distance[HCSR04_Target] = (HCSR04_History[1] + HCSR04_History[2] + HCSR04_History[3]) / 1.0 / HCSR04_Sample_Count;
				}
				
				HCSR04_Sample_State	= 0;	
				HCSR04_Sample_ENABLE = 0;			
			}
		}
	
		/* =================== [END] ()超声波数据采样模块 [END]==================== */
		
		
		
		
	}//while(1)
}//int main(void)

uint16_t TimeTick;
//uint8_t Servo_Test_State = 0;

//1ms的定时中断
void TIM1_UP_IRQHandler(void)
{
	//检查标志位
	if (TIM_GetITStatus(TIM1,TIM_IT_Update) == SET )
	{
		//清除标志位
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
		//保证数据的及时读取		
		MPU6050_GetGZ(&GZ);
		
		//各个计时
		TimeTick ++;
		if (Car_Turn_TimeTick > 0) Car_Turn_TimeTick --;
		if (Servo_TimeTick > 0)Servo_TimeTick --;
		if (Car_StraightRun_Falg == 2 && Car_Movtion_Delay_TimeTick > 0)Car_Movtion_Delay_TimeTick --;
		if (HCSR04_Sample_TimeTick > 0 ) HCSR04_Sample_TimeTick --;
//		if (Car_Movtion_Delay_TimeTick > 0)Car_Movtion_Delay_TimeTick --;
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
		
		
		
		
		/* =================== [START] (全模式)MPU6050解算模块 [START]==================== */			
			//校准零飘
//			GX += 55;
//			GY += 18;
			GZ += 8;
					
			if(-1 < GZ && GZ < 2){GZ = 0;}
					
//			// 横滚角计算
//			RollAcc = atan2(AY, AZ) / 3.14159 * 180;  				// 横滚角（绕X轴）
//			RollGyro = Roll + GX / 32768.0 * 2000 * 0.001;  		// 陀螺仪X轴积分
//			Roll = 0.001 * RollAcc + (1 - 0.001) * RollGyro;  		// 相同互补滤波算法
			
			// 偏航角：仅陀螺仪积分（无加速度计校准，会漂移）
//			if (GZ <= -2 || 2 <= GZ){Yaw += GZ / 32768.0 * 2000 * 0.001;}
			Yaw += GZ / 32768.0 * 2050 * 0.001;
			
//			// 俯仰角计算
//			PitchAcc = -atan2(AX, AZ) / 3.14159 * 180;  			// 俯仰角（绕Y轴）
//			PitchGyro = Pitch + GY / 32768.0 * 2000 * 0.001;  		// 陀螺仪积分（2000是量程，0.001是1ms采样间隔）
//			Pitch = 0.001 * PitchAcc + (1 - 0.001) * PitchGyro;  	// 互补滤波
		/* =================== [END] (全模式)MPU6050解算模块 [END]==================== */
		
			
			
			
	}
}
