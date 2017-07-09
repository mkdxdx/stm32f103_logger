#define D_Data		0
#define D_Read		1
#define D_Strobe	2

#define LCD_CMD_CLR		0x01	// clear screen
#define LCD_CMD_DRRST	0x02	// reset DDRAM counter to 0, reset shifts

#define LCD_CMD_SHMOD	0x04	// set shift,
#define LCD_SHINC		1<<1	// shift command increment bit set 1
#define LCD_SHSCR		1<<0	// shift command shift screen bit set 1

#define LCD_CMD_DISPLAY	0x08	// function set, turn screen OFF/ON, undescore cursor OFF/ON, square cursor OFF/ON
#define LCD_DISPL_EN	1<<2	// enable display
#define LCD_UNDCUR_EN	1<<1	// underscore cursor enable
#define LCD_SQCUR_EN	1<<0	// square cursor enable

#define LCD_CMD_SHSET	0x10	// set screen/cursor shift, shift right/left
#define LCD_SHCUR_EN	1<<3	// cursor shift enable
#define LCD_SHLEFT_EN	1<<2	// shift left enable

#define LCD_CMD_FUNC	0x20	// data line 4/8 bit, screen 1/2 lines, font 5x8/5x10
#define LCD_DL_8BIT		1<<4	// data line 8 bit
#define LCD_LC_2LINE	1<<3	// 2 line screen
#define LCD_FS_LARGE	1<<2	// font 5x10

#define LCD_CMD_CGADDR	0x40	// set CGRAM address
#define LCD_CMD_DDADDR	0x80	// set DDRAM address

#define LCD_CGNUM		0x08


void Display_SetXY(uint8_t x, uint8_t y);
void Display_SetCGADDR(uint8_t cgaddr);
void Display_CreateChar(uint8_t cnum, uint8_t * gfx);
void Display_Print(char * string, uint8_t x, uint8_t y);
void Display_Init(void);
void Display_PutS(char *data);
