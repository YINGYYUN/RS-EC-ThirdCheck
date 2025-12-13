#ifndef __CAR_H
#define __CAR_H

void Go_Ahead(void);
void Go_Back(void);
void Go_Ahead_SET(uint8_t PWM);
void Go_Back_SET(uint8_t PWM);
void Self_Left(void);
void Self_Right(void);
void Self_Left_SET(uint8_t PWM);
void Self_Right_SET(uint8_t PWM);
void Car_Stop(void);

#define F_Go_Ahead			0
#define F_Go_Back			1
#define F_Self_Left			2
#define F_Self_Right		3
#define F_Car_Stop			4

#endif
