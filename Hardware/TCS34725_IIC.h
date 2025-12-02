#ifndef __TCS34725_IIC_H_
#define __TCS34725_IIC_H_


/**********************************
包含头文件
**********************************/
#include "stm32f10x.h"

/**********************************
PIN口定义
**********************************/
#define IIC_SDA_PIN   				GPIO_Pin_0  //PB11-SDA							
#define IIC_SCL_PIN    				GPIO_Pin_1	//PB10-SCL

#define  IIC_SDA_H   			GPIO_SetBits(GPIOB,IIC_SDA_PIN)
#define  IIC_SDA_L   			GPIO_ResetBits(GPIOB,IIC_SDA_PIN)

#define  IIC_SCL_H   			GPIO_SetBits(GPIOB,IIC_SCL_PIN)
#define  IIC_SCL_L   			GPIO_ResetBits(GPIOB,IIC_SCL_PIN)

#define  IIC_SDA_Read   GPIO_ReadInputDataBit(GPIOB, IIC_SDA_PIN)



void TCS34725_GPIO_Init(void);
void SDA_Pin_IN(void);//SDA输入
void SDA_Pin_Output(void);
	

void TCS34725_IIC_Init(void);//IIC总线初始化函数
void TCS34725_IIC_start(void); //通讯起始信号
void TCS34725_IIC_stop(void);//通讯停止信号
uint8_t TCS34725_IIC_Get_ack(void);//接收应答信号函数
void TCS34725_IIC_ACK(void);//发送应答信号函数
void TCS34725_IIC_NACK(void);//非应答
void TCS34725_IIC_write_byte(uint8_t Data);//向IIC总线写入一个字节的数据
uint8_t TCS34725_IIC_read_byte(void);//从IIC总线读取一个字节的数据函数

#endif


