#include "stm32f10x.h"                  // Device header

#include "Delay.h"
#include "Timer.h"

#include "OLED.h"
#include "LED.h"
#include "MPU6050.h"    // 陀螺仪模块
#include "Servo.h"
#include "Motor.h"
#include "Serial.h"
#include "TCS34725.h"   // 颜色识别模块
#include "TCS34725_IIC.h"
#include "HCSR04.h"     // 超声波模块
#include "Car.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ==================== [START] MPU6050姿态解算变量 [START] ==================== */
volatile int16_t GZ;          // 陀螺仪Z轴原始数据（ISR更新）
volatile float   Yaw = 0.0f;  // 偏航角（ISR积分更新）
/* ==================== [END] MPU6050姿态解算变量 [END] ==================== */


/* ==================== [START] 运动状态控制变量 [START] ==================== */
// 小车运行状态标志位
uint8_t Car_Movtion_Event = 0;
// (自动模式)历史动作事件记录（环形缓冲）
uint8_t Car_Movtion_Event_History[60] = {0};
uint8_t Car_Movtion_Count = 0;

#define STOP        0
#define UP          1
#define DOWN        2
#define LEFT        3
#define RIGHT       4
#define LEFT_90     5
#define RIGHT_90    6
#define AROUND      7

// 转向策略
#define Right_Priority  0
#define Left_Priority   1
uint8_t Car_Turn_Strategy = Right_Priority;

// 走廊检测
uint8_t Corridor_Flag  = 0;
uint8_t Corridor_Count = 0;

// (自动模式)定时直行请求挂起标志位
volatile uint8_t  Car_StraightRun_Falg = 0;

// 定角度转向请求挂起标志位
volatile uint8_t  Car_Turn_ENABLE = 0;
// 转向目标角度存储
float             Car_Tar_Yaw = 0.0f;
// 旋转超时计时
volatile uint16_t Car_Turn_TimeTick = 0;

// 定时运动计时
volatile uint16_t Car_Movtion_Delay_TimeTick = 0;
uint8_t           Car_Movtion_Delay_Flag = 0;
/* ==================== [END] 运动状态控制变量 [END] ==================== */


/* =================== [START] 颜色识别模块 [START] ==================== */
RGB rgb; // 颜色结构体

#define OTHERS  0
#define RED     1
#define GREEN   2
#define BLUE    3
#define BLACK   4

uint8_t Color_Flag = OTHERS;
/* =================== [END] 颜色识别模块 [END] ==================== */


/* =================== [START] 舵机旋转/扫描状态机 [START] ==================== */
// 舵机当前角度（每次上电后自动旋转至该角度）
uint8_t Cur_Servo_Angle = 90;

// 舵机扫描标志位：0 不运行；1 扫描进行中；2 扫描完成等待决策
volatile uint8_t Servo_Turn_Flag = 0;

// 舵机扫描状态枚举（事件驱动，不依赖窄时间窗）
typedef enum {
    SERVO_SCAN_IDLE = 0,
    SCAN_FRONT_START,
    SCAN_FRONT_WAIT,
    TURN_LEFT,
    SCAN_LEFT_START,
    SCAN_LEFT_WAIT,
    TURN_RIGHT,
    SCAN_RIGHT_START,
    SCAN_RIGHT_WAIT,
    RETURN_CENTER,
    START_FINAL_MEASURE,
    SCAN_DONE
} ServoScanState;

uint8_t Servo_State = SERVO_SCAN_IDLE;

// 舵机动作/稳定等待计时（ms），由TIM1 ISR每1ms递减
volatile uint16_t Servo_Settle_Tick = 0;

// 等效等待时间常量（ms），对应你第二版的窗口语义
// 若希望“切到自动模式立刻开始前向采样”，可将 PRE_DELAY_MS 改为 0。
#define PRE_DELAY_MS            1000  // 等效原 5580->4580 的预等待
#define FRONT_TO_LEFT_MS          60  // 等效原 4580->4520
#define LEFT_SETTLE_MS          1000  // 等效原 4520->3520
#define LEFT_TO_RIGHT_MS         200  // 等效原 3520->3320
#define RIGHT_SETTLE_MS         2000  // 等效原 3320->1320
#define RIGHT_TO_CENTER_MS       200  // 等效原 1320->1120
#define CENTER_SETTLE_MS        1000  // 等效原 1120->120
#define FINAL_MEASURE_MS         120  // 等效原 120->0
/* =================== [END] 舵机旋转/扫描状态机 [END] ==================== */


/* =================== [START] 超声波模块 [START] ==================== */
// 超声波三向距离 [0]左 [1]前 [2]右（16位避免8位截断）
uint16_t HCSR04_Distance[3] = {0, 0, 0};

// 采样滤波输入
uint16_t        HCSR04_History[4]     = {0, 0, 0, 0};
volatile uint16_t HCSR04_Sample_TimeTick = 0;
uint8_t         HCSR04_Sample_Count   = 0;
volatile uint8_t HCSR04_Sample_State  = 0;
volatile uint8_t HCSR04_Sample_ENABLE = 0;
uint8_t         HCSR04_Target         = 0;
/* =================== [END] 超声波模块 [END] ==================== */


// 手动模式强制舵机扫描使能
volatile uint8_t Force_ENABLE = 0;

// 全局时基
volatile uint16_t TimeTick;


/* =================== [START] 辅助函数 [START] ==================== */
static inline void record_motion_event(uint8_t ev) {
    Car_Movtion_Event_History[Car_Movtion_Count % 60] = ev;
    Car_Movtion_Count++;
}
/* =================== [END] 辅助函数 [END] ==================== */


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

    // 颜色识别模块TCS34725
    TCS34725_GPIO_Init();   // 颜色传感器GPIO初始化
    TCS34725_Init();        // 颜色传感器初始化
    integrationTime(33);    // 积分时间

    Timer_Init();

    Servo_SetAngle(Cur_Servo_Angle);

    /* =================== [START] 菜单初始化模块 [START] =================== */
    char Mode_Menu[][15] = {"Manual Mode  ", "Auto Mode    "};
    #define Flag_Manual_Mode    0
    #define Flag_Auto_Mode      1

    // 功能状态
    uint8_t FUNCTION_State = Flag_Manual_Mode;

    Serial_Printf("[display-clear]");
    Serial_Printf("[display,0,0,FUNCTION State]");
    Serial_Printf("[display,0,20,%s]", Mode_Menu[FUNCTION_State]);
    Serial_Printf("[display,0,40,Yaw]");
    Serial_Printf("[display,0,80, R   G   B]");
    Serial_Printf("[display,0,160,HCSR04]");
    Serial_Printf("[display,0,200,     |     ]");
    /* =================== [END] 菜单初始化模块 [END] =================== */

    while (1)
    {
        /* =================== [START] 蓝牙收发与处理模块 [START] ==================== */
        if (Serial_RxFlag == 1)
        {
            char * Tag = strtok(Serial_RxPacket, ",");

            // 按键解析
            if (Tag && strcmp(Tag, "key") == 0)
            {
                char * Name   = strtok(NULL, ",");
                char * Action = strtok(NULL, ",");

                // 手动/自动模式切换
                if (Name && Action && strcmp(Name, "MODE") == 0 && strcmp(Action, "down") == 0)
                {
                    // 撤回可能存在的直行/转向请求
                    Car_StraightRun_Falg = 0;
                    Car_Turn_TimeTick    = 0;
                    Car_Turn_ENABLE      = 0;
                    Car_Stop();
                    Car_Movtion_Event    = STOP;

                    // 模式标志位更改
                    FUNCTION_State = 1 - FUNCTION_State;
                    Serial_Printf("[display,0,20,%s]", Mode_Menu[FUNCTION_State]);

                    if (FUNCTION_State == Flag_Auto_Mode)
                    {
                        // 清零距离与采样状态，避免残留
                        HCSR04_Distance[0] = HCSR04_Distance[1] = HCSR04_Distance[2] = 0;
                        HCSR04_Sample_ENABLE = 0;
                        HCSR04_Sample_State  = 0;
                        HCSR04_Sample_Count  = 0;
                        HCSR04_History[0] = HCSR04_History[1] = HCSR04_History[2] = HCSR04_History[3] = 0;

                        // 舵机扫描状态机：立即启动（保留原窗口等待语义）
                        Servo_Turn_Flag   = 1;
                        Servo_State       = SCAN_FRONT_START;
                        Servo_Settle_Tick = PRE_DELAY_MS;   // 预等待，等效原 5580->4580
                        Cur_Servo_Angle   = 90;
                        Servo_SetAngle(90);

                        // 走廊状态复位；Corridor_Count 不清零以便累计后切策略
                        Corridor_Flag     = 0;
                    }
                    else
                    {
                        Servo_Turn_Flag   = 0;
                        Servo_State       = SERVO_SCAN_IDLE;
                        Servo_Settle_Tick = 0;
                    }
                }

                // 手动控制
                if (FUNCTION_State == Flag_Manual_Mode && Name && Action)
                {
                    if (strcmp(Action, "up") == 0)
                    {
                        Car_Stop();
                        Car_Movtion_Event = STOP;
                        Car_Turn_ENABLE   = 0;
                    }
                    else if (strcmp(Name, "UP") == 0 && strcmp(Action, "down") == 0)
                    {
                        Go_Ahead_SET(95);
                        Car_Movtion_Event = UP;
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
                        Car_Tar_Yaw      = Yaw + 90.0f;
                        Car_Turn_ENABLE  = 1;
                        Car_Turn_TimeTick= 4000;
                        Car_Movtion_Event= LEFT_90;
                    }
                    else if (strcmp(Name, "R_90") == 0 && strcmp(Action, "down") == 0)
                    {
                        Car_Tar_Yaw      = Yaw - 90.0f;
                        Car_Turn_ENABLE  = 1;
                        Car_Turn_TimeTick= 4000;
                        Car_Movtion_Event= RIGHT_90;
                    }
                    else if (strcmp(Name, "STOP") == 0 && strcmp(Action, "down") == 0)
                    {
                        Car_Stop();
                        Car_Turn_ENABLE   = 0;
                        Car_Movtion_Event = STOP;
                    }
                    else if (strcmp(Name, "Servo") == 0 && strcmp(Action, "down") == 0)
                    {
                        // 手动强制一轮扫描
                        Servo_Turn_Flag   = 1;
                        Servo_State       = SCAN_FRONT_START;
                        Force_ENABLE      = 1;
                        Servo_Settle_Tick = PRE_DELAY_MS;  // 保留预等待语义（可改为0）
                        HCSR04_Sample_ENABLE = 0;
                        HCSR04_Sample_State  = 0;
                        Cur_Servo_Angle   = 90;
                        Servo_SetAngle(90);
                    }
                }
            }
            // 滑杆解析
            else if (Tag && strcmp(Tag, "slider") == 0)
            {
                char * Name  = strtok(NULL, ",");
                char * Value = strtok(NULL, ",");

                if (FUNCTION_State == Flag_Manual_Mode && Name && strcmp(Name, "S_Angle") == 0 && Value)
                {
                    int IntValue = atoi(Value);
                    Cur_Servo_Angle = (uint8_t)IntValue;
                    Servo_SetAngle(Cur_Servo_Angle);

                    // 舵机测距回传标示
                    if (Cur_Servo_Angle == 90)
                    {
                        Serial_Printf("[display,0,200,     |     ]");
                    }
                    else if (Cur_Servo_Angle > 90)
                    {
                        Serial_Printf("[display,0,200, |         ]");
                    }
                    else
                    {
                        Serial_Printf("[display,0,200,         | ]");
                    }
                }
            }

            Serial_RxFlag = 0;
        }
        /* =================== [END] 蓝牙收发与处理模块 [END] ==================== */


        /* =================== [START] 定角度旋转响应模块 [START] ==================== */
        if (Car_Turn_ENABLE)
        {
            if (Car_Turn_TimeTick == 0)
            {
                Car_Stop();
                Car_Turn_ENABLE   = 0;
                Car_Movtion_Event = STOP;

                if (FUNCTION_State == Flag_Auto_Mode)
                {
                    Car_StraightRun_Falg       = 1;
                    Car_Movtion_Delay_TimeTick = 1400;
                    Car_Movtion_Event          = UP;
                }
            }
            else
            {
                float yaw_diff = Car_Tar_Yaw - Yaw;
                float abs_diff = fabsf(yaw_diff);
                uint8_t turn_pwm = (abs_diff > 7.0f) ? 99 : 85;

                if (yaw_diff > 0.5f)
                {
                    Self_Left_SET(turn_pwm);
                }
                else if (yaw_diff < -0.5f)
                {
                    Self_Right_SET(turn_pwm);
                }
                else
                {
                    Car_Stop();
                    Car_Turn_ENABLE   = 0;
                    Car_Turn_TimeTick = 0;

                    if (FUNCTION_State == Flag_Auto_Mode)
                    {
                        Car_StraightRun_Falg       = 1;
                        Car_Movtion_Delay_TimeTick = 1400;
                        Car_Movtion_Event          = UP;
                    }
                }
            }
        }
        /* =================== [END] 定角度旋转响应模块 [END] ==================== */


        /* =================== [START] 传感器数据自动回传模块 [START] ==================== */
        Serial_Printf("[display,0,60,%+02.3f  ]", Yaw);
        Serial_Printf("[display,0,100,%03d %03d %03d]", R_Dat, G_Dat, B_Dat);
        Serial_Printf("[display,0,140,%d ]", Corridor_Count);
        if (FUNCTION_State == Flag_Manual_Mode)
        {
            Serial_Printf("[display,0,180,%04d,%04d,%04d  ]",
                          (int)HCSR04_Distance[0], (int)HCSR04_Distance[1], (int)HCSR04_Distance[2]);
        }
        /* =================== [END] 传感器数据自动回传模块 [END] ==================== */


        /* =================== [START] LED自动响应颜色识别模块 [START] ==================== */
        if (0) // 红（占位）
        {
            Color_Flag = RED;
            LED_A_OFF_ALL();
            LED_ON_SET(3);
        }
        else if (0) // 绿
        {
            Color_Flag = GREEN;
            LED_A_OFF_ALL();
            LED_ON_SET(2);
        }
        else if (0) // 蓝
        {
            Color_Flag = BLUE;
            LED_A_OFF_ALL();
            LED_ON_SET(1);
        }
        else if (0) // 黑
        {
            Color_Flag = BLACK;
            LED_A_ON_ALL();
        }
        else // 其他
        {
            Color_Flag = OTHERS;
            LED_A_OFF_ALL();
        }
        /* =================== [END] LED自动响应颜色识别模块 [END] ==================== */


        /* =================== [START] 手动模式测距分配 [START] ==================== */
        if (FUNCTION_State == Flag_Manual_Mode && Force_ENABLE == 0)
        {
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
        }
        /* =================== [END] 手动模式测距分配 [END] ==================== */


        /* =================== [START] 自动模式/强制扫描：事件驱动舵机扫描 [START] ==================== */
        if ((FUNCTION_State == Flag_Auto_Mode || Force_ENABLE) && Car_Turn_ENABLE == 0)
        {
            if (Servo_Turn_Flag == 1)
            {
                switch (Servo_State)
                {
                    case SCAN_FRONT_START:
                        if (Servo_Settle_Tick == 0)
                        {
                            if (Cur_Servo_Angle != 90) { Cur_Servo_Angle = 90; Servo_SetAngle(90); }
                            if (HCSR04_Sample_ENABLE == 0)
                            {
                                HCSR04_Sample_ENABLE = 1;
                                HCSR04_Sample_State  = 0;
                                HCSR04_Target        = 1; // 前
                                HCSR04_StartMeasure();
                                Servo_State          = SCAN_FRONT_WAIT;
                            }
                        }
                        break;

                    case SCAN_FRONT_WAIT:
                        if (HCSR04_Sample_ENABLE == 0 && HCSR04_Sample_State == 0)
                        {
                            Servo_Settle_Tick = FRONT_TO_LEFT_MS;
                            Servo_State       = TURN_LEFT;
                        }
                        break;

                    case TURN_LEFT:
                        if (Servo_Settle_Tick == 0)
                        {
                            Servo_SetAngle(180);
                            Cur_Servo_Angle   = 180;
                            Servo_Settle_Tick = LEFT_SETTLE_MS;
                            Servo_State       = SCAN_LEFT_START;
                        }
                        break;

                    case SCAN_LEFT_START:
                        if (Servo_Settle_Tick == 0 && HCSR04_Sample_ENABLE == 0)
                        {
                            HCSR04_Sample_ENABLE = 1;
                            HCSR04_Sample_State  = 0;
                            HCSR04_Target        = 0; // 左
                            HCSR04_StartMeasure();
                            Servo_State          = SCAN_LEFT_WAIT;
                        }
                        break;

                    case SCAN_LEFT_WAIT:
                        if (HCSR04_Sample_ENABLE == 0 && HCSR04_Sample_State == 0)
                        {
                            Servo_Settle_Tick = LEFT_TO_RIGHT_MS;
                            Servo_State       = TURN_RIGHT;
                        }
                        break;

                    case TURN_RIGHT:
                        if (Servo_Settle_Tick == 0)
                        {
                            Servo_SetAngle(0);
                            Cur_Servo_Angle   = 0;
                            Servo_Settle_Tick = RIGHT_SETTLE_MS;
                            Servo_State       = SCAN_RIGHT_START;
                        }
                        break;

                    case SCAN_RIGHT_START:
                        if (Servo_Settle_Tick == 0 && HCSR04_Sample_ENABLE == 0)
                        {
                            HCSR04_Sample_ENABLE = 1;
                            HCSR04_Sample_State  = 0;
                            HCSR04_Target        = 2; // 右
                            HCSR04_StartMeasure();
                            Servo_State          = SCAN_RIGHT_WAIT;
                        }
                        break;

                    case SCAN_RIGHT_WAIT:
                        if (HCSR04_Sample_ENABLE == 0 && HCSR04_Sample_State == 0)
                        {
                            Servo_Settle_Tick = RIGHT_TO_CENTER_MS;
                            Servo_State       = RETURN_CENTER;
                        }
                        break;

                    case RETURN_CENTER:
                        if (Servo_Settle_Tick == 0)
                        {
                            Servo_SetAngle(90);
                            Cur_Servo_Angle   = 90;
                            Servo_Settle_Tick = CENTER_SETTLE_MS;
                            Servo_State       = START_FINAL_MEASURE;
                        }
                        break;

                    case START_FINAL_MEASURE:
                        if (Servo_Settle_Tick == 0)
                        {
                            HCSR04_StartMeasure();            // 最后一次触发测量
                            Servo_Settle_Tick = FINAL_MEASURE_MS;
                            Servo_State       = SCAN_DONE;
                        }
                        break;

                    case SCAN_DONE:
                        if (Servo_Settle_Tick == 0)
                        {
                            Serial_Printf("[display,0,180,%04d,%04d,%04d  ]",
                                          (int)HCSR04_Distance[0], (int)HCSR04_Distance[1], (int)HCSR04_Distance[2]);
                            Servo_Turn_Flag = 2;       // 扫描完成，进入寻路决策
                            Servo_State     = SERVO_SCAN_IDLE;
                            Force_ENABLE    = 0;
                        }
                        break;

                    default:
                        break;
                }
            }
            else if (Servo_Turn_Flag == 0)
            {
                // 无扫描时，正前方实时测距用于抢断
                HCSR04_Distance[1] = HCSR04_GetValue();
                Serial_Printf("[display,0,180,%04d,%04d,%04d  ]",
                              (int)HCSR04_Distance[0], (int)HCSR04_Distance[1], (int)HCSR04_Distance[2]);

                // 到墙抢断
                if (Car_Movtion_Delay_TimeTick <= 1200 &&
                    HCSR04_Distance[1] <= 7 && HCSR04_Distance[1] != 0)
                {
                    Car_StraightRun_Falg = 0;
                    Car_Stop();
                    Car_Movtion_Event  = STOP;

                    Servo_Turn_Flag    = 1;
                    Servo_State        = SCAN_FRONT_START;
                    Servo_Settle_Tick  = PRE_DELAY_MS;
                }
            }
        }
        /* =================== [END] 自动模式/强制扫描：事件驱动舵机扫描 [END] ==================== */


        /* =================== [START] 自动模式寻路与直行响应 [START] ==================== */
        if (FUNCTION_State == Flag_Auto_Mode)
        {
            // 寻路决策：在扫描完成后进行
            if (Servo_Turn_Flag == 2)
            {
                if (Car_Turn_Strategy == Right_Priority)
                {
                    if (HCSR04_Distance[2] >= 18) { Car_Movtion_Event = RIGHT_90; record_motion_event(RIGHT_90); }
                    else if (HCSR04_Distance[1] >= 10) { Car_Movtion_Event = UP; record_motion_event(UP); }
                    else if (HCSR04_Distance[0] >= 18) { Car_Movtion_Event = LEFT_90; record_motion_event(LEFT_90); }
                    else { Car_Movtion_Event = AROUND; record_motion_event(AROUND); }
                }
                else // Left_Priority
                {
                    if (HCSR04_Distance[0] >= 18) { Car_Movtion_Event = LEFT_90; record_motion_event(LEFT_90); }
                    else if (HCSR04_Distance[1] >= 10) { Car_Movtion_Event = UP; record_motion_event(UP); }
                    else if (HCSR04_Distance[2] >= 18) { Car_Movtion_Event = RIGHT_90; record_motion_event(RIGHT_90); }
                    else { Car_Movtion_Event = AROUND; record_motion_event(AROUND); }
                }

                // 走廊检测与策略切换
                if (Corridor_Flag == 0 && HCSR04_Distance[1] >= 30) { Corridor_Flag = 1; Corridor_Count++; }
                else if (Corridor_Flag == 1 && Car_Movtion_Event != UP) { Corridor_Flag = 0; }
                if (Corridor_Count >= 5 && Car_Turn_Strategy == Right_Priority) { Car_Turn_Strategy = Left_Priority; }

                // 执行动作
                switch (Car_Movtion_Event)
                {
                    case UP:
                        Car_StraightRun_Falg       = 1;
                        Car_Movtion_Delay_TimeTick = 1400;
                        break;
                    case AROUND:
                        Car_Tar_Yaw      = Yaw - 180.0f;
                        Car_Turn_ENABLE  = 1;
                        Car_Turn_TimeTick= 7000;
                        break;
                    case RIGHT_90:
                        Car_Tar_Yaw      = Yaw - 90.0f;
                        Car_Turn_ENABLE  = 1;
                        Car_Turn_TimeTick= 2200;
                        break;
                    case LEFT_90:
                        Car_Tar_Yaw      = Yaw + 90.0f;
                        Car_Turn_ENABLE  = 1;
                        Car_Turn_TimeTick= 2200;
                        break;
                    default:
                        break;
                }

                Servo_Turn_Flag = 0; // 决策已下发
            }

            // 直行请求响应
            if (Car_Turn_ENABLE == 0 && Car_StraightRun_Falg == 1)
            {
                Go_Ahead_SET(95);
                Car_Movtion_Event   = UP;
                Car_StraightRun_Falg= 2;
            }
            else if (Car_StraightRun_Falg == 2)
            {
                if (Car_Movtion_Delay_TimeTick == 0)
                {
                    Car_Stop();
                    Car_Movtion_Event   = STOP;
                    Car_StraightRun_Falg= 0;

                    // 开始下一轮舵机旋转测距
                    Servo_Turn_Flag   = 1;
                    Servo_State       = SCAN_FRONT_START;
                    Servo_Settle_Tick = PRE_DELAY_MS;
                }
            }
        }
        /* =================== [END] 自动模式寻路与直行响应 [END] ==================== */


        /* =================== [START] 超声波采样状态机 [START] ==================== */
        if (HCSR04_Sample_ENABLE)
        {
            if (HCSR04_Sample_State == 0)
            {
                HCSR04_History[0] = HCSR04_GetValue();
                HCSR04_StartMeasure();
                HCSR04_Sample_Count = 0;
                HCSR04_History[1] = HCSR04_History[2] = HCSR04_History[3] = 0;
                HCSR04_Sample_TimeTick = 62;
                HCSR04_Sample_State = 1;
            }
            else if (HCSR04_Sample_State == 1 && HCSR04_Sample_TimeTick == 0)
            {
                HCSR04_History[1] = HCSR04_GetValue();
                HCSR04_StartMeasure();
                if (HCSR04_History[1]) { HCSR04_Sample_Count++; }
                HCSR04_Sample_TimeTick = 62;
                HCSR04_Sample_State = 2;
            }
            else if (HCSR04_Sample_State == 2 && HCSR04_Sample_TimeTick == 0)
            {
                HCSR04_History[2] = HCSR04_GetValue();
                HCSR04_StartMeasure();
                if (HCSR04_History[2]) { HCSR04_Sample_Count++; }
                HCSR04_Sample_TimeTick = 62;
                HCSR04_Sample_State = 3;
            }
            else if (HCSR04_Sample_State == 3 && HCSR04_Sample_TimeTick == 0)
            {
                HCSR04_History[3] = HCSR04_GetValue();
                if (HCSR04_History[3]) { HCSR04_Sample_Count++; }
                if (HCSR04_Sample_Count)
                {
                    HCSR04_Distance[HCSR04_Target] =
                        (HCSR04_History[1] + HCSR04_History[2] + HCSR04_History[3]) / HCSR04_Sample_Count;
                }
                HCSR04_Sample_State  = 0;
                HCSR04_Sample_ENABLE = 0;
            }
        }
        /* =================== [END] 超声波采样状态机 [END] ==================== */
    }
}


/* =================== [START] TIM1 更新中断（1ms） [START] ==================== */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        // 陀螺仪读取
        MPU6050_GetGZ(&GZ);

        // 计时递减
        TimeTick++;
        if (Car_Turn_TimeTick > 0)            Car_Turn_TimeTick--;
        if (Servo_Settle_Tick > 0)            Servo_Settle_Tick--;
        if (Car_StraightRun_Falg == 2 && Car_Movtion_Delay_TimeTick > 0) Car_Movtion_Delay_TimeTick--;
        if (HCSR04_Sample_TimeTick > 0)       HCSR04_Sample_TimeTick--;

        // 超声波内部tick
        HCSR04_Tick();

        // 颜色识别定时更新（100ms）
        if (TimeTick >= 100)
        {
            TimeTick = 0;
            rgb = TCS34725_Get_RGBData();
            RGB888 = TCS34725_GetRGB888(rgb);
            RGB565 = TCS34725_GetRGB565(rgb);
            Dis_Temp();
        }

        /* 姿态解算（仅Z轴积分，存在漂移）：保留你原来的简易零漂与积分 */
        GZ += 8;
        if (-1 < GZ && GZ < 2) { GZ = 0; }
        Yaw += (float)GZ / 32768.0f * 2050.0f * 0.001f;
    }
}
/* =================== [END] TIM1 更新中断（1ms） [END] ==================== */
