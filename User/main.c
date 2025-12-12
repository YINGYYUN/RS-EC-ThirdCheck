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




/* ==================== [START] MPU6050姿态解算相关变量定义 [START] ==================== */
//解算开关标志位
uint8_t MPU6050_Resolving_ENABLE = 0;

//MPU6050原始数据读取接收
//int16_t AX, AY, AZ, GX, GY, GZ;
int16_t GZ;

uint8_t MPU6050_ENABLE = 0;

//float RollAcc;    		// 加速度计计算的横滚角
//float RollGyro;   		// 陀螺仪积分的横滚角
//float Roll;       		// 融合后的横滚角

float Yaw = 0;				//偏航角

//float PitchAcc;			//加速度计算的俯仰角
//float PitchGyro;			//陀螺仪积分的俯仰角
//float Pitch;				//融合后的俯仰角	
/* ==================== [END] MPU6050姿态解算相关变量定义 [END] ==================== */




/* ==================== [START] 运动状态控制相关变量定义 [START] ==================== */
uint8_t Car_Movtion_Event = 0;

#define STOP		0
#define UP			1
#define DOWN		2
#define LEFT		3
#define RIGHT		4
#define LEFT_90		5
#define RIGHT_90	6
#define AROUND		7

//定角度转向请求挂起标志位
uint8_t Car_Turn_ENABLE = 0;
//转向目标角度存储
float Car_Tar_Yaw = 0.0f;
//运动状态标记
uint8_t Cur_Flag, Pre_Flag;

uint16_t Car_Movtion_Delay_TimeTick = 0;
//旋转超时计时
uint16_t Car_Turn_TimeTick = 0;
/* ==================== [END] 运动状态控制相关变量定义 [END] ==================== */




/* =================== [START] 颜色识别模块 [START]==================== */
RGB rgb;					//颜色结构体

#define OTHERS		0
#define RED			1
#define GREEN		2
#define BLUE		3
#define BLACK		4

//颜色分类判定
uint8_t Color_Flag = OTHERS;
/* =================== [END] 颜色识别模块 [END]==================== */




/* =================== [START] 舵机旋转模块 [START]==================== */
//舵机当前角度（每次上电后自动旋转至该角度）
uint8_t Cur_Servo_Angle = 90;
//舵机目标角度
uint8_t Tar_Servo_Angle = 90;
uint16_t Servo_Need_Wait_Time = 0;
uint8_t Servo_State = 0;
uint8_t Servo_Turn_Flag = 0;
/* =================== [END] 舵机旋转模块 [END]==================== */




//舵机旋转等待
uint16_t Servo_Delay_TimeTick = 0;
//超声波测到的距离
uint8_t HCSR04_Distance[3] = {0, 0, 0};  

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
	
	
	
	
	/* =================== [START] 菜单初始化模块 [START] =================== */	
	char Mode_Menu[][15] = {"Manual Mode  ", "Auto Mode    "};
	#define Flag_Manual_Mode			0
	#define Flag_Auto_Mode				1
	
	//功能状态
	uint8_t FUNCTION_State = Flag_Manual_Mode;
	
	Serial_Printf("[display-clear]");
	Serial_Printf("[display,0,0,FUNCTION State]");
	Serial_Printf("[display,0,20,%s]", Mode_Menu[FUNCTION_State]);
	Serial_Printf("[display,0,40,Yaw]");
	Serial_Printf("[display,0,80,R     G     B]");
	Serial_Printf("[display,0,120,S_Angle]");
	Serial_Printf("[display,0,160,HCSR04]");
/* =================== [END] 菜单初始化模块 [END] =================== */	
	
	
	
	
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
				
				if (strcmp(Name, "MODE") == 0 && strcmp(Action, "down") == 0)
				{
					//手动/自动模式切换
					
					//撤回可能存在的定角度转向请求
					Car_Turn_ENABLE = 0;
					Car_Stop();
				
					Car_Movtion_Event = STOP;
					
					Servo_Turn_Flag = 0;
					Servo_State = 0;
					Servo_Need_Wait_Time = 0;
					Cur_Servo_Angle = 90;
					Servo_SetAngle(Cur_Servo_Angle);
					
					FUNCTION_State = 1 - FUNCTION_State;
					Serial_Printf("[display,0,20,%s]", Mode_Menu[FUNCTION_State]);
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
					else if (strcmp(Name, "UP") == 0 && strcmp(Action, "down") == 0)
					{
						Go_Ahead();
						Car_Movtion_Event = UP;					
					}
					else if (strcmp(Name, "DOWN") == 0 && strcmp(Action, "down") == 0)
					{
						Go_Back();
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
						Car_Turn_TimeTick = 2200;
						Car_Movtion_Event = LEFT_90;
					}
					else if (strcmp(Name, "R_90") == 0 && strcmp(Action, "down") == 0)
					{						
						//发送定角度转向请求
						Car_Tar_Yaw = Yaw - 90.0f;
						Car_Turn_ENABLE = 1;
						Car_Turn_TimeTick = 2200;
						Car_Movtion_Event = RIGHT_90;
					}
					else if (strcmp(Name, "STOP") == 0 && strcmp(Action, "down") == 0)
					{						
						Car_Stop();
						Car_Turn_ENABLE = 0;
						
						Car_Movtion_Event = STOP;
					}			
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
				if (FUNCTION_State == Flag_Manual_Mode && strcmp(Name, "S_Angle") == 0)
				{
					int IntValue = atoi(Value);				
					Cur_Servo_Angle = IntValue;
					
					Servo_SetAngle(Cur_Servo_Angle);
				}
			}
			
			//摇杆解析不启用，代码部分删除
			
			Serial_RxFlag = 0;
		}
		/* =================== [END] 蓝牙收发与处理模块 [END]==================== */
		
		
		
		
		/* =================== [START] 小车定角度旋转控制模块 [START]==================== */
		//转向请求运作模块
		if(Car_Turn_ENABLE)
		{
			//超时强制叫停
			if (Car_Turn_TimeTick == 0)
			{
				Car_Stop();
				Car_Turn_ENABLE = 0;
				Car_Movtion_Event = STOP;
			}
			else
			{
				float yaw_diff = Car_Tar_Yaw - Yaw;
				float abs_diff = fabs(yaw_diff);
				
				//基于目标差值的二级旋转速度
				uint8_t turn_pwm = (abs_diff > 7.0f) ? 95 : 60;
				
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
					Car_Stop();
					Car_Turn_ENABLE = 0;
					Car_Movtion_Event = STOP;
					Car_Turn_TimeTick = 0;
				}
			}
		}
		if (Pre_Flag != Cur_Flag)Pre_Flag = Cur_Flag;
		/* =================== [END] 小车定角度旋转控制模块 [END]==================== */		
		

		
		
		/* =================== [START] 数据自动回传模块 [START]==================== */
		Serial_Printf("[display,0,60,%+02.3f  ]", Yaw);
		Serial_Printf("[display,0,100,%3d   %3d   %3d   ]", R_Dat, G_Dat, B_Dat);
		Serial_Printf("[display,0,140,%d  ]", Cur_Servo_Angle);	
		Serial_Printf("[display,0,180,%d, %d, %d  ]", HCSR04_Distance[0], HCSR04_Distance[1], HCSR04_Distance[2]);
		/* =================== [END] 数据自动回传模块 [END]==================== */
		
		
		
		
		/* =================== [START] LED自动响应颜色识别模块 [START]==================== */
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
		/* =================== [END] LED自动响应颜色识别模块 [END]==================== */
		
		
		
		if (FUNCTION_State == Flag_Auto_Mode)
		{
			/* =================== [START] 测距模块 [START]==================== */
			//HCSR04_Distance有三个元素，分别对应	左[0]	中[1]	右[2]	
			HCSR04_Distance[1] = HCSR04_GetValue();
			//数据目前是测试性质的
			
			//判定是否开始左右测距
			if (Servo_Turn_Flag == 0 && Car_Turn_ENABLE == 0 && HCSR04_Distance[1] <= 14)
			{
				Car_Stop();
				Car_Movtion_Event = STOP;
				Servo_Turn_Flag = 1;
				Servo_Need_Wait_Time = 3000;
				Servo_State = 0;				
			}
			//运行舵机左右测距请求
			if(Servo_Turn_Flag)
			{
				if (2000 < Servo_Need_Wait_Time && Servo_Need_Wait_Time <= 3000 && Servo_State == 0)
				{
					Servo_SetAngle(180);				
					Servo_State = 1;
				}
				else if (1000 < Servo_Need_Wait_Time && Servo_Need_Wait_Time <= 2000 && Servo_State == 1)
				{
					HCSR04_Distance[0] = HCSR04_GetValue();
					Servo_SetAngle(0);
					Servo_State = 2;
				}
				else if (0 < Servo_Need_Wait_Time && Servo_Need_Wait_Time <= 1000 && Servo_State == 2)
				{
					HCSR04_Distance[2] = HCSR04_GetValue();
					Servo_SetAngle(90);
					Servo_State = 3;
				}
				else if (Servo_Need_Wait_Time == 0 && Servo_State == 3)
				{
					Servo_Turn_Flag = 2;
				}
			}
			/* =================== [END] 测距模块 [END]==================== */
			
			
			
			
			/* =================== [START] 寻路模块 [START]==================== */
			if (Servo_Turn_Flag == 0)
			{
				Go_Ahead();
				Car_Movtion_Event = UP;
			}
			else if (Servo_Turn_Flag == 2)		
			{
				if (HCSR04_Distance[0] >= 14)
				{
					//发送定角度转向请求
					Car_Tar_Yaw = Yaw + 90.0f;
					Car_Turn_ENABLE = 1;
					Car_Turn_TimeTick = 2200;
					Car_Movtion_Event = LEFT_90;
				}
				else if (HCSR04_Distance[2] >= 14)
				{
					//发送定角度转向请求
					Car_Tar_Yaw = Yaw - 90.0f;
					Car_Turn_ENABLE = 1;
					Car_Turn_TimeTick = 2200;
					Car_Movtion_Event = RIGHT_90;
				}
				else 
				{
					//发送定角度转向请求
					Car_Tar_Yaw = Yaw - 180.0f;
					Car_Turn_ENABLE = 1;
					Car_Turn_TimeTick =  4000;
					Car_Movtion_Event = AROUND;
				}
				Servo_Turn_Flag = 0;
			}
			/* =================== [END] 寻路模块 [END]==================== */
		}
		
		
		
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
		
//		LED_Tick();
		TimeTick ++;

//		if (2000 < Servo_Need_Wait_Time && Servo_Need_Wait_Time <= 3000 && Servo_Test_State == 0)
//		{
//			Servo_SetAngle(0);          // 首次进入该区间，执行一次角度设置
//			Servo_Test_State = 1;       // 切换状态，避免重复执行
//		}
//		else if (1000 < Servo_Need_Wait_Time && Servo_Need_Wait_Time <= 2000 && Servo_Test_State == 1)
//		{
//			Servo_SetAngle(180);        // 首次进入该区间，执行一次角度设置
//			Servo_Test_State = 2;       // 切换状态
//		}
//		else if (0 < Servo_Need_Wait_Time && Servo_Need_Wait_Time <= 1000 && Servo_Test_State == 2)
//		{
//			Servo_SetAngle(90);         // 首次进入该区间，执行一次角度设置
//			Servo_Test_State = 3;       
//		}
//		else if (Servo_Need_Wait_Time == 0 && Servo_Test_State == 3)
//		{
//			Servo_Need_Wait_Time = 3000;// 重置为初始值，开始下一轮循环
//			Servo_Test_State = 0;       // 重置状态
//		}
		
		
		if (Car_Turn_TimeTick > 0) Car_Turn_TimeTick --;
		if (Servo_Need_Wait_Time > 0)Servo_Need_Wait_Time --;
		if (Car_Movtion_Delay_TimeTick > 0)Car_Movtion_Delay_TimeTick --;
		if (Servo_Delay_TimeTick > 0)Servo_Delay_TimeTick --;
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
			MPU6050_GetGZ(&GZ);
			
			//校准零飘
//			GX += 55;
//			GY += 18;
			GZ += 10;
			if(-2 <= GZ && GZ <= 2){GZ = 0;}
		
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
		/* =================== [END] MPU6050解算模块 [END]==================== */
		
	}
}
