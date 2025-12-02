#include "stm32f10x.h"                  // Device header
#include "PWM.h"
/**
  * 函    数：舵机设置角度
  * 参    数：Angle 要设置的舵机角度，范围：0~180
  * 返 回 值：无
  * 说明：将角度线性映射到 0.5ms~2.5ms（500~2500） 的比较值，
  *      假定定时器 tick 为 1us（即 PSC=72-1, timerclk=72MHz）。
  */
void Servo_SetAngle(float Angle)
{
    // 边界保护
    if (Angle < 0.0f)
        Angle = 0.0f;
    if (Angle > 180.0f)
        Angle = 180.0f;

    // 计算比较值，Angle=0 -> 500(0.5ms)，Angle=180 -> 2500(2.5ms)
    // 加 0.5f 做四舍五入
    uint16_t compare = (uint16_t)(Angle / 180.0f * 2000.0f + 500.0f + 0.5f);
    PWM_SetCompare3(compare);
}
