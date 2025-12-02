#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
	//电机PWM
	
	// 1. 使能时钟（TIM2和GPIOA）
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// 2. 配置PA0（TIM2_CH1）和PA1（TIM2_CH2）为复用推挽输出
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽（PWM输出）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  // 同时配置PA0和PA1
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 3. 配置TIM2内部时钟
	TIM_InternalClockConfig(TIM2);
	
	// 4. 配置时基单元（决定PWM频率）
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 200 -1;		// ARR=199
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;		// PSC=719
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	// 5. 配置CH1（PA0，一号电机PWMA）
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);  // 初始化默认值
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  // PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  // 高电平有效
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 使能输出
	TIM_OCInitStructure.TIM_Pulse = 0;		// CCR初始值0（占空比0%）
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  // 应用到CH1
	
	// 6. 配置CH2（PA1，二号电机PWMB）
	TIM_OCInitStructure.TIM_Pulse = 0;		// 二号电机初始占空比0%
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  // 应用到CH2
	
	// 7. 使能TIM2
	TIM_Cmd(TIM2, ENABLE);
	
	//舵机PWM
	
	// 1. 使能时钟（TIM3属于APB1总线，GPIOA属于APB2总线）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // 2. 配置PA6(TIM3_CH1)为复用推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 显式赋值，避免复用隐患
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. 配置TIM3内部时钟源
    TIM_InternalClockConfig(TIM3);
    
    // 4. 配置TIM3时基单元（舵机PWM频率=50Hz，周期20ms）
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 显式赋值
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;            // ARR=19999
    TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;            // PSC=719
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    
    // 5. 配置TIM3_CH1（PA6：舵机PWM）
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;                         // 初始值1500（90°）
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    
    // 6. 使能TIM3定时器
    TIM_Cmd(TIM3, ENABLE);
}

// 设置一号电机PWM占空比（PA0，TIM2_CH1）
void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM2, Compare);  // 范围0~199（对应0%~99.5%）
}

// 设置二号电机PWM占空比（PA1，TIM2_CH2）
void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM2, Compare);  // 范围0~199（对应0%~99.5%）
}

// 设置舵机PWM CCR值（PA6，TIM3_CH1）
void PWM_SetCompare3(uint16_t Compare)
{
	TIM_SetCompare1(TIM3, Compare);  // 范围0~19999（对应舵机0.5ms~2.5ms）
}
