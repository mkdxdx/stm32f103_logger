#include "stm32f10x.h"
#include "hd4478.h"
#include "iox.h"

void Display_Print(char * string, uint8_t x, uint8_t y) {
	Display_SetXY(x,y);
	Display_PutS(string);
}

void Display_PutS(char *data) {
	do {
		Display_Data(*data);
	} while (*++data);
}

void Display_SetCGADDR(uint8_t cgaddr) {
	Display_Cmd(LCD_CMD_CGADDR + cgaddr);
}

void Display_CreateChar(uint8_t cnum, uint8_t * gfx) {
	uint8_t i;
	for (i = 0; i<8; i++) {
		Display_SetCGADDR(cnum * LCD_CGNUM + i);
		Display_Data(*gfx++);
	}
}

void Display_SetXY(uint8_t x, uint8_t y) {
	uint8_t addr = x + (y * 0x40);
	Display_Cmd(LCD_CMD_DDADDR | addr);
}

void Display_Init(void) {
	Display_Cmd(0x33);	// set 8 bit data line first
	Display_Cmd(0x32);	//
	Display_Cmd(0x28);	// set 4 bit data line next
	Display_Cmd(0x0E);	// set display mode, display ON, underscore curson ON, square cursor OFF

	Display_Cmd(0x01);	// clear sreen
	delay_ms(5);

	Display_Cmd(0x06);	// set shift, address increment ON, screen shift OFF
	Display_Cmd(0x80);	// set address ddram 0

}


void Display_Write4(uint8_t hi, uint8_t lo) {
	IOXWrite(hi | 1<<D_Strobe);
	IOXWrite(hi);
	IOXWrite(lo | 1<<D_Strobe);
	IOXWrite(lo);
}


void Display_Cmd(uint8_t data) {
	uint8_t hi = (data & 0xF0);
	uint8_t lo = (((data & 0x0F) << 4) & 0xF0);
	Display_Write4(hi,lo);
}

void Display_Data(uint8_t data) {
	uint8_t hi = ((data & 0xF0) | 1<<D_Data);
	uint8_t lo = ((((data & 0x0F) << 4) & 0xF0) | 1<<D_Data);
	Display_Write4(hi,lo);
}
