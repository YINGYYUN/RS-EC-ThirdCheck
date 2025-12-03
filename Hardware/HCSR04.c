#include "stm32f10x.h"    // STM32F10x 设备头文件
#include "HCSR04.h"       // HC-SR04 模块驱动头文件

uint16_t Time;            // 定时器计数变量，用于记录 Echo 为高电平的时间周期数

// Non-blocking state machine for HC-SR04
typedef enum { HCSR_IDLE = 0, HCSR_TRIG_HIGH, HCSR_WAITING, HCSR_DONE } HCSR_State_t;
static volatile HCSR_State_t hcsr_state = HCSR_IDLE;
static volatile uint16_t hcsr_ms = 0;        // millisecond counter for state transitions
static volatile uint16_t hcsr_last_distance = 0; // last measured distance (cm)

/**
 * @brief  初始化 TIM4 定时器，用于计时 Echo 引脚高电平持续时间
 * @note   定时器时钟来源为内置时钟，计数周期：ARR=7199，PSC=0 可得每次更新中断间隔 0.0001s
 */
void TIM4_Init(void)
{
    // 使能 TIM4 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // 配置为内部时钟模式
    TIM_InternalClockConfig(TIM4);

    // 定时器基础参数配置
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;          // 不分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;      // 向上计数模式
    TIM_TimeBaseInitStructure.TIM_Period = 7199;                         // 自动重装载寄存器 (ARR)，对应更新事件间隔 0.0001s
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;                         // 预分频器 (PSC)，不分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                 // 仅在高级定时器中有效，普通定时器置 0
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    // 清除更新中断标志
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    // 使能更新中断
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    // 中断优先级配置
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                      // 定时器 4 中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                      // 使能中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;            // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                   // 响应优先级
    NVIC_Init(&NVIC_InitStructure);

    // 启动定时器
    TIM_Cmd(TIM4, ENABLE);
}

/**
 * @brief  TIM4 中断服务函数，每次更新中断调用一次
 * @note   在 Echo 引脚为高电平时累加 Time 计数
 */
void TIM4_IRQHandler(void)
{
    // 判断是否为更新中断
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        // 读取 PA5（Echo）引脚电平，高电平时计时变量加 1
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 1)
        {
            Time++;
        }
        // 清除中断标志
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

/**
 * @brief  初始化 HC-SR04 模块所用的 GPIO：Trig 输出，Echo 下拉输入
 */
void HCSR04_Init(void)
{
    // 使能 GPIOA 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;

    // Trig (PA4) 推挽输出，用于产生触发脉冲
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Echo (PA5) 下拉输入，用于接收回波信号
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;  // 下拉输入
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 初始将 Trig 拉低
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);

    // ensure timer disabled initially
    TIM_Cmd(TIM4, DISABLE);
    hcsr_state = HCSR_IDLE;
}

/**
 * @brief Start a non-blocking measurement (explicit)
 */
void HCSR04_StartMeasure(void)
{
    if (hcsr_state != HCSR_IDLE) return; // already measuring

    // 发起触发：置高并进入 TRIG_HIGH 状态
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
    hcsr_ms = 0;
    hcsr_state = HCSR_TRIG_HIGH;
}

/**
 * @brief  获取一次超声波测距结果（单位：cm）
 * @return 距离值，范围 ~2cm–400cm
 */
uint16_t HCSR04_GetValue(void)
{
    // Non-blocking behavior:
    // - If idle: start a new measurement and return last distance
    // - If done: return latest measured distance and reset to idle
    // - Otherwise: return last measured distance (measurement in progress)
    if (hcsr_state == HCSR_IDLE)
    {
        HCSR04_StartMeasure();
        return hcsr_last_distance;
    }

    if (hcsr_state == HCSR_DONE)
    {
        uint16_t d = hcsr_last_distance;
        hcsr_state = HCSR_IDLE;
        return d;
    }

    return hcsr_last_distance;
}

/**
 * @brief Called from a 1ms tick (e.g. TIM1 1ms IRQ). Advances state machine.
 */
void HCSR04_Tick(void)
{
    switch (hcsr_state)
    {
        case HCSR_IDLE:
            break;

        case HCSR_TRIG_HIGH:
            // Keep trigger high for 1 ms (sufficient and non-blocking in our 1ms tick)
            hcsr_ms++;
            if (hcsr_ms >= 1)
            {
                // end trigger pulse
                GPIO_ResetBits(GPIOA, GPIO_Pin_4);

                // reset echo timing variable
                Time = 0;
                // start counting echo using TIM4
                TIM4_Init();
                hcsr_ms = 0;
                hcsr_state = HCSR_WAITING;
            }
            break;

        case HCSR_WAITING:
            hcsr_ms++;
            // wait up to 60ms for measurement to complete (HC-SR04 min spacing is ~60ms)
            if (hcsr_ms >= 60)
            {
                // stop timer and compute distance
                TIM_Cmd(TIM4, DISABLE);
                if (Time > 235) Time = 0; // >4m -> clamp

                // Time * 0.0001s is echo high time
                hcsr_last_distance = (uint16_t)(((Time * 0.0001f) * 34000) / 2);

                hcsr_state = HCSR_DONE;
            }
            break;

        case HCSR_DONE:
            // wait until caller reads value (GetValue will move to IDLE)
            break;
    }
}

uint8_t HCSR04_IsReady(void)
{
    return (hcsr_state == HCSR_DONE) ? 1 : 0;
}
