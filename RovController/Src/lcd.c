/*
 * lcd.c
 *
 *  Created on: 13.02.2019
 *      Author: Kurat
 */

#include "lcd.h"
#include "main.h"

#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_gpio.h"
#define MIN(a,b) (((a)<(b))?(a):(b))

// us max value = 0xFFFF (65535)
void us_delay(uint32_t us)
{
	LL_TIM_SetCounter(TIM3, MIN(65535, us));
	LL_TIM_EnableCounter(TIM3);

	// timer zlicza w dó³ wiec czekamy az zliczy
	while(LL_TIM_GetCounter(TIM3) > 0)
	{
	}

	LL_TIM_DisableCounter(TIM3);
}



static const uint32_t writeTimeConstant = 10;
static uint8_t mode_8_4_I2C = 1;


static uint8_t DisplayControl = 0x0F;
static uint8_t FunctionSet = 0x38;

//***** Functions definitions *****//
//Private functions
//1) Enable EN pulse
static void LCD1602_EnablePulse(void)
{
	LL_GPIO_SetOutputPin(LCD_E_GPIO_Port, LCD_E_Pin);
	LCD1602_TIM_MicorSecDelay(writeTimeConstant);
	LL_GPIO_ResetOutputPin(LCD_E_GPIO_Port, LCD_E_Pin);
	LCD1602_TIM_MicorSecDelay(60);
}
//2) RS control
static void LCD1602_RS(bool state)
{
	if(state) LL_GPIO_SetOutputPin(LCD_RS_GPIO_Port, LCD_RS_Pin);
	else LL_GPIO_ResetOutputPin(LCD_RS_GPIO_Port, LCD_RS_Pin);
}

//3) Write Parallel interface
void write_pin(GPIO_TypeDef* port, uint32_t pin, uint8_t value)
{
	value != 0 ? LL_GPIO_SetOutputPin(port, pin): LL_GPIO_ResetOutputPin(port, pin);
}



static void LCD1602_write(uint8_t byte)
{
	uint8_t LSB_nibble = byte&0xF, MSB_nibble = (byte>>4)&0xF;

	if(mode_8_4_I2C == 1)		//8bits mode
	{
		//write data to output pins
		//LSB data

		// blad

		//Write the Enable pulse
		LCD1602_EnablePulse();
	}
	else if(mode_8_4_I2C == 2)	//4 bits mode
	{
		//write data to output pins
		//MSB data
		write_pin(LCD_D4_GPIO_Port, LCD_D4_Pin, (GPIO_PinState)(MSB_nibble&0x1));
		write_pin(LCD_D5_GPIO_Port, LCD_D5_Pin, (GPIO_PinState)(MSB_nibble&0x2));
		write_pin(LCD_D6_GPIO_Port, LCD_D6_Pin, (GPIO_PinState)(MSB_nibble&0x4));
		write_pin(LCD_D7_GPIO_Port, LCD_D7_Pin, (GPIO_PinState)(MSB_nibble&0x8));
		//Write the Enable pulse
		LCD1602_EnablePulse();

		//LSB data
		write_pin(LCD_D4_GPIO_Port, LCD_D4_Pin, (GPIO_PinState)(LSB_nibble&0x1));
		write_pin(LCD_D5_GPIO_Port, LCD_D5_Pin, (GPIO_PinState)(LSB_nibble&0x2));
		write_pin(LCD_D6_GPIO_Port, LCD_D6_Pin, (GPIO_PinState)(LSB_nibble&0x4));
		write_pin(LCD_D7_GPIO_Port, LCD_D7_Pin, (GPIO_PinState)(LSB_nibble&0x8));
		//Write the Enable pulse
		LCD1602_EnablePulse();
	}
}
//4) Microsecond delay functions

//5) Write command
static void LCD1602_writeCommand(uint8_t command)
{
	//Set RS to 0
	LCD1602_RS(false);
	//Call low level write parallel function
	LCD1602_write(command);
}
//6) Write 8 bits data
static void LCD1602_writeData(uint8_t data)
{
	//Set RS to 1
	LCD1602_RS(true);
	//Call low level write parallel function
	LCD1602_write(data);
}
//7) Write 4 bits command, *FOR 4 BITS MODE ONLY*
static void LCD1602_write4bitCommand(uint8_t nibble)
{
	uint8_t LSB_nibble = nibble&0xF;
	//Set RS to 0
	LCD1602_RS(false);
	//LSB data
	write_pin(LCD_D4_GPIO_Port, LCD_D4_Pin, (GPIO_PinState)(LSB_nibble&0x1));
	write_pin(LCD_D5_GPIO_Port, LCD_D5_Pin, (GPIO_PinState)(LSB_nibble&0x2));
	write_pin(LCD_D6_GPIO_Port, LCD_D6_Pin, (GPIO_PinState)(LSB_nibble&0x4));
	write_pin(LCD_D7_GPIO_Port, LCD_D7_Pin, (GPIO_PinState)(LSB_nibble&0x8));
	//Write the Enable pulse
	LCD1602_EnablePulse();
}

//Public functions
//1) LCD begin 8 bits function
void LCD1602_Begin8BIT()
{

	//Initialise microsecond timer
	LCD1602_TIM_Config();
	//Set the mode to 8 bits
	mode_8_4_I2C = 1;
	//Function set variable to 8 bits mode
	FunctionSet = 0x38;

	//Initialise LCD
	//1. Wait at least 15ms
	LL_mDelay(20);
	//2. Attentions sequence
	LCD1602_writeCommand(0x30);
	LL_mDelay(5);
	LCD1602_writeCommand(0x30);
	LL_mDelay(1);
	LCD1602_writeCommand(0x30);
	LL_mDelay(1);
	//3. Function set; Enable 2 lines, Data length to 8 bits
	LCD1602_writeCommand(LCD_FUNCTIONSET | LCD_FUNCTION_N | LCD_FUNCTION_DL);
	//4. Display control (Display ON, Cursor ON, blink cursor)
	LCD1602_writeCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C | LCD_DISPLAY_D);
	//5. Clear LCD and return home
	LCD1602_writeCommand(LCD_CLEARDISPLAY);
	LL_mDelay(2);
}
//2) LCD begin 4 bits function
void LCD1602_Begin4BIT()
{
	//Set GPIO Ports and Pins data

	//Initialise microsecond timer
	LCD1602_TIM_Config();
	//Set the mode to 4 bits
	mode_8_4_I2C = 2;
	//Function set variable to 4 bits mode
	FunctionSet = 0x28;

	//Initialise LCD
	//1. Wait at least 15ms
	LL_mDelay(20);
	//2. Attentions sequence
	LCD1602_write4bitCommand(0x3);
	LL_mDelay(5);
	LCD1602_write4bitCommand(0x3);
	LL_mDelay(1);
	LCD1602_write4bitCommand(0x3);
	LL_mDelay(1);
	LCD1602_write4bitCommand(0x2);  //4 bit mode
	LL_mDelay(1);
	//3. Display control (Display ON, Cursor ON, blink cursor)
	LCD1602_writeCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C | LCD_DISPLAY_D);
	//4. Clear LCD and return home
	LCD1602_writeCommand(LCD_CLEARDISPLAY);
	LL_mDelay(3);
	//4. Function set; Enable 2 lines, Data length to 8 bits
	LCD1602_writeCommand(LCD_FUNCTIONSET | LCD_FUNCTION_N);
	LL_mDelay(3);
}
//3) LCD print string
void LCD1602_print(char string[])
{
	for(uint8_t i=0;  i< 16 && string[i]!=NULL; i++)
	{
		LCD1602_writeData((uint8_t)string[i]);
	}
}
//4) set cursor position
void LCD1602_setCursor(uint8_t row, uint8_t col)
{
	uint8_t maskData;
	maskData = (col-1)&0x0F;
	if(row==1)
	{
		maskData |= (0x80);
		LCD1602_writeCommand(maskData);
	}
	else
	{
		maskData |= (0xc0);
		LCD1602_writeCommand(maskData);
	}
}
void LCD1602_1stLine(void)
{
	LCD1602_setCursor(1,1);
}
void LCD1602_2ndLine(void)
{
	LCD1602_setCursor(2,1);
}
//5) Enable two lines
void LCD1602_TwoLines(void)
{
	FunctionSet |= (0x08);
	LCD1602_writeCommand(FunctionSet);
}
void LCD1602_OneLine(void)
{
	FunctionSet &= ~(0x08);
	LCD1602_writeCommand(FunctionSet);
}
//6) Cursor ON/OFF
void LCD1602_noCursor(void)
{
	DisplayControl &= ~(0x02);
	LCD1602_writeCommand(DisplayControl);
}
void LCD1602_cursor(void)
{
	DisplayControl |= (0x02);
	LCD1602_writeCommand(DisplayControl);
}
//7) Clear display
void LCD1602_clear(void)
{
	LCD1602_writeCommand(LCD_CLEARDISPLAY);
	LL_mDelay(3);
}
//8) Blinking cursor
void LCD1602_noBlink(void)
{
	DisplayControl &= ~(0x01);
	LCD1602_writeCommand(DisplayControl);
}
void LCD1602_blink(void)
{
	DisplayControl |= 0x01;
	LCD1602_writeCommand(DisplayControl);
}
//9) Display ON/OFF
void LCD1602_noDisplay(void)
{
	DisplayControl &= ~(0x04);
	LCD1602_writeCommand(DisplayControl);
}
void LCD1602_display(void)
{
	DisplayControl |= (0x04);
	LCD1602_writeCommand(DisplayControl);
}
//10) Shift Display, right or left
void LCD1602_shiftToRight(uint8_t num)
{
	for(uint8_t i=0; i<num;i++)
	{
		LCD1602_writeCommand(0x1c);
	}
}
void LCD1602_shiftToLeft(uint8_t num)
{
	for(uint8_t i=0; i<num;i++)
	{
		LCD1602_writeCommand(0x18);
	}
}

//********** Print numbers to LCD **********//
//1. Integer
void LCD1602_PrintInt(int number)
{
	char numStr[16];
	sprintf(numStr,"%d", number);
	LCD1602_print(numStr);
}
//2. Float
void LCD1602_PrintFloat(float number, int decimalPoints)
{
	char numStr[16];
	sprintf(numStr,"%.*f",decimalPoints, number);
	LCD1602_print(numStr);
}
