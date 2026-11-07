#include "stm32f4xx.h"
#include "OLED_Font.h"
#include "Delay.h"
#include <math.h>

#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))

void OLED_I2C_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	Delay_us(5);
	OLED_W_SDA(0);
	Delay_us(5);
	OLED_W_SCL(0);
}

void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	Delay_us(5);
	OLED_W_SDA(1);
	Delay_us(5);
}

void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(Byte & (0x80 >> i));
		Delay_us(5);
		OLED_W_SCL(1);
		Delay_us(5);
		OLED_W_SCL(0);
		Delay_us(5);
	}
	OLED_W_SCL(1);	
	Delay_us(5);
	OLED_W_SCL(0);
	Delay_us(5);
}

void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);
	OLED_I2C_SendByte(0x00);
	OLED_I2C_SendByte(Command); 
	OLED_I2C_Stop();
}

void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);
	OLED_I2C_SendByte(0x40);
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}

void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
	OLED_WriteCommand(0x00 | (X & 0x0F));
}

void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
	}
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

void OLED_Init(void)
{
	uint32_t i, j;
	
	for (i = 0; i < 1000; i++)
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_I2C_Init();
	
	OLED_WriteCommand(0xAE);
	
	OLED_WriteCommand(0xD5);
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);
	
	OLED_WriteCommand(0xA1);
	
	OLED_WriteCommand(0xC8);

	OLED_WriteCommand(0xDA);
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);

	OLED_WriteCommand(0xA6);

	OLED_WriteCommand(0x8D);
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);
		
	OLED_Clear();
}

void OLED_ClearLine(uint8_t Line)
{
    uint8_t page = (Line - 1) * 2;
    uint8_t x;
    OLED_SetCursor(page, 0);
    for(x = 0; x < 128; x++)
    {
        OLED_WriteData(0x00);
    }
    OLED_SetCursor(page + 1, 0);
    for(x = 0; x < 128; x++)
    {
        OLED_WriteData(0x00);
    }
}

/**
 * @brief  OLED显示浮点数（适配当前驱动：参数为行、列，无size参数，内部调用原生OLED函数）
 * @param Line 显示行号 1~4
 * @param Column 显示起始列号 1~16
 * @param dat 待显示浮点数值
 * @param decimal 小数保留位数
 */

 
#include <math.h>
/**
 * @brief  显示浮点频率，限制总字符长度，防止乱码
 * @param Line 行号 0~7
 * @param Column 起始列
 * @param dat 频率数值
 * @param decimal 小数位数 1即可
 */
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float dat, uint8_t decimal)
{
    uint32_t int_num;
    uint32_t dec_num;
    uint8_t pos = 0;

    // 负数屏蔽，频率不会负
    if(dat < 0) dat = 0;

    int_num = (uint32_t)dat;
    // 整数最多4位，适配0~9999Hz
    if(int_num >= 1000)
    {
        OLED_ShowNum(Line, Column + pos, int_num, 4);
        pos += 4;
    }
    else if(int_num >= 100)
    {
        OLED_ShowNum(Line, Column + pos, int_num, 3);
        pos += 3;
    }
    else if(int_num >= 10)
    {
        OLED_ShowNum(Line, Column + pos, int_num, 2);
        pos += 2;
    }
    else
    {
        OLED_ShowNum(Line, Column + pos, int_num, 1);
        pos += 1;
    }

    OLED_ShowChar(Line, Column + pos, '.');
    pos++;

    dec_num = (uint32_t)((dat - int_num) * pow(10, decimal) + 0.5f);
    OLED_ShowNum(Line, Column + pos, dec_num, decimal);
}
/**
 * @brief 显示电压，格式 x.xxxV，保留3位小数
 */
void OLED_ShowVoltage(uint8_t Line, uint8_t Column, float voltage)
{
    if (voltage < 0) voltage = 0;
    if (voltage > 3.3) voltage = 3.3;
    uint8_t int_part = (uint8_t)voltage;
    uint16_t frac = (uint16_t)((voltage - int_part) * 1000 + 0.5f);
    OLED_ShowChar(Line, Column, int_part + '0');
    OLED_ShowChar(Line, Column + 1, '.');
    OLED_ShowNum(Line, Column + 2, frac, 3);
    OLED_ShowChar(Line, Column + 5, 'V');
}

/**
 * @brief  显示峰峰值（单位mV），格式 "XXXX mV"，4位整数 + 单位
 *         3.3V满量程对应 0~3300mV，用mV显示直观，误差50mV清晰可见
 * @param  Line    行号 1~4
 * @param  Column  起始列 1~16
 * @param  vpp_mv  峰峰值电压，单位mV
 */
void OLED_ShowVpp_mV(uint8_t Line, uint8_t Column, float vpp_mv)
{
    uint16_t val;
    uint8_t  pos = 0;

    if (vpp_mv < 0.0f) vpp_mv = 0.0f;
    if (vpp_mv > 3300.0f) vpp_mv = 3300.0f;

    // 四舍五入取整，mV级别不需要小数
    val = (uint16_t)(vpp_mv + 0.5f);

    // 4位数字，前面补空格（不补0，更美观）
    if (val >= 1000)
    {
        OLED_ShowNum(Line, Column + pos, val, 4);
        pos += 4;
    }
    else if (val >= 100)
    {
        OLED_ShowChar(Line, Column + pos, ' ');
        OLED_ShowNum(Line, Column + pos + 1, val, 3);
        pos += 4;
    }
    else if (val >= 10)
    {
        OLED_ShowChar(Line, Column + pos, ' ');
        OLED_ShowChar(Line, Column + pos + 1, ' ');
        OLED_ShowNum(Line, Column + pos + 2, val, 2);
        pos += 4;
    }
    else
    {
        OLED_ShowChar(Line, Column + pos, ' ');
        OLED_ShowChar(Line, Column + pos + 1, ' ');
        OLED_ShowChar(Line, Column + pos + 2, ' ');
        OLED_ShowNum(Line, Column + pos + 3, val, 1);
        pos += 4;
    }

    // 空格 + 单位mV
    OLED_ShowChar(Line, Column + pos, ' ');
    OLED_ShowChar(Line, Column + pos + 1, 'm');
    OLED_ShowChar(Line, Column + pos + 2, 'V');
}

/**
 * @brief  显示频率（单位Hz），格式 "XXXXX Hz"，右对齐 + 单位
 *         0~99999Hz 适用；如果 <1000Hz 显示整数，>=1000 也整数
 *         可按需扩展 kHz 版本
 * @param  Line     行号 1~4
 * @param  Column   起始列 1~16
 * @param  freq_hz  频率值，单位Hz（0=没信号，会显示 -- Hz）
 */
void OLED_ShowFreq_Hz(uint8_t Line, uint8_t Column, float freq_hz)
{
    uint32_t val;
    uint8_t  pos = 0;
    uint8_t  digit;

    // 没捕获到信号时用 "-- Hz" 占位，避免显示 000 让人误解
    if (freq_hz < 0.5f)
    {
        OLED_ShowChar(Line, Column + 0, ' ');
        OLED_ShowChar(Line, Column + 1, ' ');
        OLED_ShowChar(Line, Column + 2, ' ');
        OLED_ShowChar(Line, Column + 3, '-');
        OLED_ShowChar(Line, Column + 4, '-');
        OLED_ShowChar(Line, Column + 5, ' ');
        OLED_ShowChar(Line, Column + 6, 'H');
        OLED_ShowChar(Line, Column + 7, 'z');
        return;
    }

    // 限制上限（按Fs/2≈200KHz以内可正确估计，给个宽限到99999）
    if (freq_hz > 99999.0f) freq_hz = 99999.0f;

    val = (uint32_t)(freq_hz + 0.5f);   // 四舍五入到整数Hz

    // 数位数决定补空格（右对齐，5位数字）
    if      (val >= 10000) digit = 5;
    else if (val >= 1000)  digit = 4;
    else if (val >= 100)   digit = 3;
    else if (val >= 10)    digit = 2;
    else                   digit = 1;

    // 补前导空格
    for (uint8_t i = 0; i < 5 - digit; i++)
    {
        OLED_ShowChar(Line, Column + pos, ' ');
        pos++;
    }
    // 显示数字
    OLED_ShowNum(Line, Column + pos, val, digit);
    pos += digit;

    // 空格 + 单位 Hz
    OLED_ShowChar(Line, Column + pos, ' ');
    OLED_ShowChar(Line, Column + pos + 1, 'H');
    OLED_ShowChar(Line, Column + pos + 2, 'z');
}
