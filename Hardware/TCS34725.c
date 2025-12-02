#include "stm32f10x.h"                  // Device header
#include "TCS34725.h"


uint32_t RGB888=0;;////将原始数据转化为RGB888格式
uint16_t RGB565=0;////将原始数据转化为RGB565格式
uint8_t R_Dat=0,G_Dat=0,B_Dat=0;
uint8_t  Dis_Buf1[8];//R
uint8_t  Dis_Buf2[8];//G
uint8_t  Dis_Buf3[8];//B


float INTEGRATION_CYCLES_MIN = 1.0f;
float INTEGRATION_CYCLES_MAX = 256.0f;
float INTEGRATION_TIME_MS_MIN= 2.4f;
float INTEGRATION_TIME_MS_MAX =2.4 * 256.0f;


uint8_t C_Dat[8]={0};


//读指定地址数据
uint8_t TCS34725_ReadWord(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
	TCS34725_IIC_start();
	TCS34725_IIC_write_byte(TCS34725_ADDRESS);
	TCS34725_IIC_Get_ack();				//接收应答	
	TCS34725_IIC_write_byte(TCS34725_CMD_BIT|ReadAddr);
	TCS34725_IIC_Get_ack();				//接收应答	
	
	TCS34725_IIC_start();
	TCS34725_IIC_write_byte(TCS34725_ADDRESS+1);
	TCS34725_IIC_Get_ack();				//接收应答	
	while(NumByteToRead)  
  {
		*pBuffer = TCS34725_IIC_read_byte();
		if(NumByteToRead == 1)
    {
				/* Disable Acknowledgement */
				TCS34725_IIC_NACK();

				/* Send STOP Condition */
				TCS34725_IIC_stop();


		}
		else
		{
			TCS34725_IIC_ACK();
		}
		 pBuffer++; 
		 NumByteToRead--; 
	}
	return 0;
}

uint8_t TCS34725_WriteByte(uint8_t addr,uint8_t data) 
{
		TCS34725_IIC_start();
		TCS34725_IIC_write_byte(TCS34725_ADDRESS);
		TCS34725_IIC_Get_ack();				//接收应答	
		TCS34725_IIC_write_byte(TCS34725_CMD_BIT|addr);
		TCS34725_IIC_Get_ack();				//接收应答	
		
		TCS34725_IIC_write_byte(data);
		TCS34725_IIC_Get_ack();				//接收应答		
		return 0;
}

//初始化
uint8_t TCS34725_Init(void)  
{

		uint8_t ID = 0;
		TCS34725_ReadWord(&ID, TCS34725_ID, 1);
		if(ID != 0x44 && ID != 0x4D)
		{
				return 1;
		}
		TCS34725_WriteByte(TCS34725_CMD_Read_Byte,TCS34725_ENABLE_AIEN|TCS34725_ENABLE_AEN|TCS34725_ENABLE_PON);
		TCS34725_WriteByte(TCS34725_PERS,TCS34725_PERS_NONE);
		TCS34725_WriteByte(TCS34725_CONTROL,TCS34725_GAIN_60X);
		return 0;
}

void integrationTime(float ms) // 2.4 - 614.4 ms
{
		uint8_t data;
		if (ms < INTEGRATION_TIME_MS_MIN) ms = INTEGRATION_TIME_MS_MIN;
		if (ms > INTEGRATION_TIME_MS_MAX) ms = INTEGRATION_TIME_MS_MAX;
		 data = (uint8_t)(256.f - ms / INTEGRATION_TIME_MS_MIN);
		TCS34725_WriteByte(TCS34725_ATIME,data);	

}

RGB TCS34725_Get_RGBData(void)
{
	  RGB temp;
		uint8_t STATUS_AINT=0;

		while(STATUS_AINT!=(TCS34725_STATUS_AINT|TCS34725_STATUS_AVALID))
		{
				TCS34725_ReadWord(&STATUS_AINT,TCS34725_STATUS,1);
		}

		TCS34725_ReadWord(C_Dat,TCS34725_CDATAL | TCS34725_CMD_Read_Word,8);

	
		temp.C = (uint16_t)C_Dat[1]<<8|C_Dat[0];
		temp.R = (uint16_t)C_Dat[3]<<8|C_Dat[2];
		temp.G = (uint16_t)C_Dat[5]<<8|C_Dat[4];
		temp.B = (uint16_t)C_Dat[7]<<8|C_Dat[6];

		return temp;
}
/******************************************************************************
function:   将原始RGB值转换为RGB888格式
parameter	:
     rgb    : RGB数值
******************************************************************************/

uint32_t TCS34725_GetRGB888(RGB rgb)
{
    float i=1;
    //Limit data range
    if(rgb.R >= rgb.G && rgb.R >= rgb.B){ 
        i = rgb.R / 255 + 1;
    }
    else if(rgb.G >= rgb.R && rgb.G >= rgb.B){ 
        i = rgb.G / 255 + 1;
    }
    else if(rgb.B >=  rgb.G && rgb.B >= rgb.R){ 
        i = rgb.B / 255 + 1;  
    }
    if(i!=0)
    {
        rgb.R = (rgb.R) / i;
        rgb.G = (rgb.G) / i;
        rgb.B = (rgb.B) / i;
    }
    //Amplify data differences
    /*Please don't try to make the data negative, 
        unless you don't change the data type*/
    if(rgb.R > 30)
        rgb.R = rgb.R - 30;
    if(rgb.G > 30)
        rgb.G = rgb.G - 30;
    if(rgb.B > 30)
        rgb.B = rgb.B - 30;
    rgb.R = rgb.R * 255 / 225;
    rgb.G = rgb.G * 255 / 225;
    rgb.B = rgb.B * 255 / 225;
    
    if(rgb.R>255)
           rgb.R = 255; 
    if(rgb.G>255)
           rgb.G = 255; 
    if(rgb.B>255)
           rgb.B = 255; 
    return (rgb.R << 16) | (rgb.G << 8) | (rgb.B);
}

/******************************************************************************
function:   将原始RGB值转换为RGB565格式
parameter	:
     rgb    : RGB数值
******************************************************************************/
uint16_t TCS34725_GetRGB565(RGB rgb)
{
    float i=1;
    //Limit data range
    if(rgb.R >= rgb.G && rgb.R >= rgb.B){ 
        i = rgb.R / 255 + 1;
    }
    else if(rgb.G >= rgb.R && rgb.G >= rgb.B){ 
        i = rgb.G / 255 + 1;
    }
    else if(rgb.B >=  rgb.G && rgb.B >= rgb.R){ 
        i = rgb.B / 255 + 1;  
    }
    if(i!=0){
        rgb.R = (rgb.R) / i;
        rgb.G = (rgb.G) / i;
        rgb.B = (rgb.B) / i;
    }
    if(rgb.R > 30)
        rgb.R = rgb.R - 30;
    if(rgb.G > 30)
        rgb.G = rgb.G - 30;
    if(rgb.B > 30)
        rgb.B = rgb.B - 30;
    rgb.R = rgb.R * 255 / 225;
    rgb.G = rgb.G * 255 / 225;
    rgb.B = rgb.B * 255 / 225;
    
    if(rgb.R>255)
           rgb.R = 255; 
    if(rgb.G>255)
           rgb.G = 255; 
    if(rgb.B>255)
           rgb.B = 255; 
    return (rgb.R << 11) | (rgb.G << 5) | (rgb.B);
}

//距离数据填到Dis_Buf中
//*P_data：存放数组
//Location：在数组中存放开始位数 如上面uint8_t  Dis_Buf1[8]={"R:   "};  填入数据是从Dis_Buf1[2]开始
//temp_data：需要转换的数据
//Length：数据位数长度 如999是3位数  此处为3
void OLED_printf( uint8_t *P_data,uint8_t Location,uint16_t temp_data,uint8_t Length)
{
		uint8_t i;
		uint32_t Remainder=0;
		uint32_t Divisor=0;
	
		Remainder=1;
		Divisor=1;
    for(i=0;i<Length;i++)
		{	
				Remainder=Remainder*10;

		}			
    for(i=0;i<Length-1;i++)
		{	

				Divisor=Divisor*10;
		} 
    for(i=0;i<Length;i++)
		{			

				P_data[Location+i] =temp_data%Remainder/Divisor+0x30;
				Remainder=Remainder/10;
			  Divisor=Remainder/10;
		}

}

void Dis_Temp(void)//转化为可读颜色数据
{
		R_Dat=(RGB888>>16);
		G_Dat=(RGB888>>8) & 0xff;
		B_Dat=(RGB888) & 0xff;
	
		OLED_printf(Dis_Buf1,0,R_Dat,3);
		OLED_printf(Dis_Buf2,0,G_Dat,3);
		OLED_printf(Dis_Buf3,0,B_Dat,3);

}

