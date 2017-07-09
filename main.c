#include "stm32f10x.h"
//#include "sdlib.h"
#include "comm.h"
#include "ff.h"
#include "diskio.h"
#include "delay.h"
#include "rtccmd.h"
#include "stdlib.h"
#include "string.h"
#include "hd4478.h"
#include "shiftin.h"


#define TIMx			TIM4
#define TIMx_RCC		RCC_APB1Periph_TIM4
#define TIMx_IRQn		TIM4_IRQn

#define VOL_NUM			0

#define KEYCOD_UP			1<<5
#define KEYCOD_DN			1<<7
#define KEYCOD_LT			1<<4
#define KEYCOD_RT			1<<6

#define DIG_GPIO			GPIOB
#define DIG_GPIO_RCC		RCC_APB2Periph_GPIOB
#define DIG_GPIO_CH1		GPIO_Pin_0
#define DIG_GPIO_CH2		GPIO_Pin_1
#define DIG_GPIO_CH3		GPIO_Pin_5	// since PB3 is used as JTAG and i dont want to disable it yet
#define DIG_GPIO_CH4		GPIO_Pin_4

#define AN_GPIO				GPIOA
#define AN_GPIO_RCC			RCC_APB2Periph_GPIOA
#define AN_GPIO_CH1			GPIO_Pin_0
#define AN_GPIO_CH2			GPIO_Pin_1
#define AN_GPIO_CH3			GPIO_Pin_2
#define AN_GPIO_CH4			GPIO_Pin_3

#define AN_ADCx				ADC1
#define AN_ADCx_RCC			RCC_APB2Periph_ADC1
#define AN_CH1				ADC_Channel_0
#define AN_CH2				ADC_Channel_1
#define AN_CH3				ADC_Channel_2
#define AN_CH4				ADC_Channel_3

#define C_UPDN					0
#define	C_DSBL					2
#define C_ENBL					1
#define NUM_DIGITAL_CHANNELS 	4
#define NUM_ANALOG_CHANNELS 	4

typedef enum E_KEY {
	K_NONE = 0,
	K_UP,
	K_DOWN,
	K_LEFT,
	K_RIGHT,
	K_UPR,
	K_DOWNR,
	K_LEFTR,
	K_RIGHTR
} E_KEY;

#define CHKEY(r,k)		((r & k) == 0)

enum E_LOGMODE {
	LM_SINGLE = 0,
	LM_HOURLY,
	LM_DAILY,
	LM_WEEKLY
};


typedef enum E_AC_MULTIPLIER {
	ACM_DISABLED = 0,
	ACM_RAW,
	ACM_1x,
	ACM_10x,
	ACM_100x,
} E_AC_MULTIPLIER;

typedef struct {
	uint8_t 	LOG_ENABLED;
	uint16_t 	LOG_DELAY;
	uint16_t 	LOG_TIMER;
	uint8_t 	Digital;
	uint16_t	Analog[NUM_ANALOG_CHANNELS];
	E_AC_MULTIPLIER AMultiplier[NUM_ANALOG_CHANNELS];
	int16_t 	Temp1;
	int16_t 	Temp2;
	char 		Sep;
	enum E_LOGMODE LOG_MODE;
	void (*current)(E_KEY);
} LOGGER_T;

FRESULT LogAddLine(char * string);
void SetRTC (void);
void ITInit(void);
void TIMInit(void);
void GPIOInit(void);
void ADCInit(void);
void DMAInit(void);

void DisplayDiskStatus(FRESULT fr);
void DateTimeStr(RTC_DateTimeTypeDef* rtcs, char * ts, char * ds);
void ZeroPad(char *str);
void LoggerMain(uint8_t key);
void LoggerSetDelay(uint8_t key) ;
void LoggerSetMode(uint8_t key);
E_KEY GetPressedKey(void);
void LoggerSetTime(E_KEY key);
void LoggerReadChannels(void);
void LoggerDigitalSettings(E_KEY key);
void LoggerAnalogSettings(E_KEY key);
void sftoa(float fv, char *s);


RTC_DateTimeTypeDef rtc;
const char s_muld[] = "DSBL";
const char s_mulr[] = "RAW";
const char s_mul1x[] = "1x";
const char s_mul10x[] = "10x";
const char s_mul100x[] = "100x";
const char logname[] = "log.txt";
const uint8_t symb_updown[] = {0x04, 0x0E, 0x1F, 0x00, 0x00, 0x1F, 0x0E, 0x04};
const uint8_t symb_enabled[] = {0x1F, 0x1F, 0x1F, 0x1F, 0x11, 0x11, 0x11, 0x1F};
const uint8_t symb_disabled[] = {0x1F, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F};


const char FR_S_OK[] = "OK   ";
const char FR_S_DISK_ERR[] = "IOERR";
const char FR_S_INT_ERR[] = "INERR";
const char FR_S_NOT_READY[] = "NORDY";
const char FR_S_NO_FILE[] = "NOFIL";
const char FR_S_NO_PATH[] = "NOPTH";
const char FR_S_INVALID_NAME[] = "NAERR";
const char FR_S_DENIED[] = "ADENY";
const char FR_S_EXIST[] = "EXIST";
const char FR_S_INVALID_OBJECT[] = "INOBJ";
const char FR_S_WRITE_PROTECTED[] = "WRPRT";
const char FR_S_INVALID_DRIVE[] = "IDNUM";
const char FR_S_NOT_ENABLED[] = "NOWKA";
const char FR_S_NO_FILESYSTEM[] = "NOFS ";
const char FR_S_MKFS_ABORTED[] = "MFABT";
const char FR_S_TIMEOUT[] = "TOUT ";
const char FR_S_LOCKED[] = "LOCK ";
const char FR_S_NOT_ENOUGH_CORE[] = "NOCOR";
const char FR_S_TOO_MANY_OPEN_FILES[] = "ONOVF";
const char FR_S_INVALID_PARAMETER[] = "NOPAR";
const uint8_t curpos_clkset[] = {0x00, 0x03, 0x06, 0x40, 0x43, 0x46};



char datestr[14];
char timestr[9];
uint8_t tempb[512];
char wrstr[120];


char strval[8];
char strval2[8];
RTC_DateTimeTypeDef rtc;
FRESULT g_ferr;
LOGGER_T LOGGER;

volatile uint32_t ms;

void TIM4_IRQHandler(void) {
	if (TIM_GetITStatus(TIMx, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIMx,TIM_IT_Update);
		disk_timerproc();
		if (ms) ms--;
	}
}

void main(void)
{
	LoggerInit();
    while(1) {
    	LoggerLoop();
    	if (LOGGER.LOG_ENABLED == 0) {
    		Delay_MSec(100);
    	} else {
    		Delay_MSec(900);
    	}

    }
}

void Delay_Sec(uint16_t sec) {
	ms = 1000*sec;
	while (ms) {};
}

void Delay_MSec(uint16_t msec) {
	ms = msec;
	while (ms) {};
}



void LoggerInit(void) {
	uint8_t i;

	GPIOInit();
	SH_GPIOInit();
	Delay_Init(72);
	USARTInit(115200);
	TIMInit();
	ITInit();
	DMAInit();
	ADCInit();
	SetRTC();
	I2CInit();

	IOXWrite(0x00);
	delay_ms(20);
	Display_Init();
	Display_CreateChar(C_UPDN,symb_updown);
	Display_CreateChar(C_ENBL,symb_enabled);
	Display_CreateChar(C_DSBL,symb_disabled);

	LOGGER.LOG_ENABLED = 0;
	LOGGER.LOG_MODE = LM_SINGLE;
	LOGGER.LOG_DELAY = 1;
	LOGGER.LOG_TIMER = LOGGER.LOG_DELAY;
	LOGGER.current = &LoggerMain;
	LOGGER.Digital = 0;
	for (i = 0; i<NUM_ANALOG_CHANNELS; i++) {
		LOGGER.Analog[i] = 0;
		LOGGER.AMultiplier[i] = ACM_1x;
	}
	LOGGER.Sep = ',';
	LOGGER.Temp1 = 0;
	LOGGER.Temp2 = 0;

	g_ferr = FR_DISK_ERR;

	DSTATUS ds = disk_initialize(VOL_NUM);
}


void DisplayDiskStatus(FRESULT fr) {
	switch (fr) {
		case FR_OK: Display_Print(FR_S_OK,11,1); break;
		case FR_DISK_ERR: Display_Print(FR_S_DISK_ERR,11,1); break;
		case FR_INT_ERR: Display_Print(FR_S_INT_ERR,11,1); break;
		case FR_NOT_READY: Display_Print(FR_S_NOT_READY,11,1); break;
		case FR_NO_FILE: Display_Print(FR_S_NO_FILE,11,1); break;
		case FR_NO_PATH: Display_Print(FR_S_NO_PATH,11,1); break;
		case FR_INVALID_NAME: Display_Print(FR_S_INVALID_NAME,11,1); break;
		case FR_DENIED: Display_Print(FR_S_DENIED,11,1); break;
		case FR_EXIST: Display_Print(FR_S_EXIST,11,1); break;
		case FR_INVALID_OBJECT: Display_Print(FR_S_INVALID_OBJECT,11,1); break;
		case FR_WRITE_PROTECTED: Display_Print(FR_S_WRITE_PROTECTED,11,1); break;
		case FR_INVALID_DRIVE: Display_Print(FR_S_INVALID_DRIVE,11,1); break;
		case FR_NOT_ENABLED: Display_Print(FR_S_NOT_ENABLED,11,1); break;
		case FR_NO_FILESYSTEM: Display_Print(FR_S_NO_FILESYSTEM,11,1); break;
		case FR_MKFS_ABORTED: Display_Print(FR_S_MKFS_ABORTED,11,1); break;
		case FR_TIMEOUT: Display_Print(FR_S_TIMEOUT,11,1); break;
		case FR_LOCKED: Display_Print(FR_S_LOCKED,11,1); break;
		case FR_NOT_ENOUGH_CORE: Display_Print(FR_S_NOT_ENOUGH_CORE,11,1); break;
		case FR_TOO_MANY_OPEN_FILES: Display_Print(FR_S_TOO_MANY_OPEN_FILES,11,1); break;
		case FR_INVALID_PARAMETER: Display_Print(FR_S_INVALID_PARAMETER,11,1); break;
		default: break;
	}
}

E_KEY GetPressedKey(void) {
	static uint8_t pk;
	uint8_t key = SH_Read();
	E_KEY kv = K_NONE;

	if (key != pk) {
		if CHKEY(key,KEYCOD_UP) {
			kv = K_UP;
		}

		if CHKEY(key,KEYCOD_DN) {
			kv = K_DOWN;
		}

		if CHKEY(key,KEYCOD_LT) {
			kv = K_LEFT;
		}

		if CHKEY(key,KEYCOD_RT) {
			kv = K_RIGHT;
		}
	} else {
		if CHKEY(key,KEYCOD_UP) {
			kv = K_UPR;
		}

		if CHKEY(key,KEYCOD_DN) {
			kv = K_DOWNR;
		}

		if CHKEY(key,KEYCOD_LT) {
			kv = K_LEFTR;
		}

		if CHKEY(key,KEYCOD_RT) {
			kv = K_RIGHTR;
		}
	}

	pk = key;
	return kv;

}

void LoggerMain(E_KEY key) {
	DisplayTime(0);
	DisplayDiskStatus(g_ferr);
	if (LOGGER.LOG_ENABLED != 0) {
		Display_Print("ENABL",11,0);
	} else {
		Display_Print("DSBL ",11,0);
	}

	if ((key == K_RIGHT)){
		LOGGER.current = &LoggerSetDelay;
		LOGGER.LOG_ENABLED = 0;
	}
	if ((key == K_LEFT)) {
		LOGGER.LOG_ENABLED ^= 1<<0;
	}
}

void LoggerSetDelay(E_KEY key) {
	Display_Print("DELAY:",0,0);

	if ((key == K_UP) | (key == K_UPR)) {
		LOGGER.LOG_DELAY++;
	} else if ((key == K_DOWN) | (key == K_DOWNR)) {
		LOGGER.LOG_DELAY--;
	}

	itoa(LOGGER.LOG_DELAY,strval,10);
	Display_PutS(strval);
	Display_PutS(" sec    ");

	PrintCtrls();


	if ((key == K_RIGHT)) {
		LOGGER.current = &LoggerSetMode;
	}
	if ((key == K_LEFT)) {
		LOGGER.current = &LoggerMain;
	}
}

void PrintCtrls(void) {
	Display_Print("<",0,1);
	Display_Print(">",15,1);
	Display_Print((char*){C_UPDN, 0},15,0);
}

void LoggerSetMode(E_KEY key) {
	Display_Print("MODE:",0,0);

	if ((key == K_UP) | (key == K_UPR)) {
		LOGGER.LOG_MODE++;
		if (LOGGER.LOG_MODE>LM_WEEKLY) {
			LOGGER.LOG_MODE = LM_SINGLE;
		}
	}

	switch (LOGGER.LOG_MODE) {
		case (LM_SINGLE): Display_PutS("Single"); break;
		case (LM_HOURLY): Display_PutS("Hourly"); break;
		case (LM_DAILY): Display_PutS("Daily "); break;
		case (LM_WEEKLY): Display_PutS("Weekly"); break;
		default: break;
	}

	PrintCtrls();

	if ((key == K_RIGHT)) {
		LOGGER.current = &LoggerSetTime;
	}
	if ((key == K_LEFT)) {
		LOGGER.current = &LoggerSetDelay;
	}
}

void LoggerSetTime(E_KEY key) {
	static int8_t cursor;
	uint16_t rtca[6];

	DisplayTime(1);

	rtca[2] = rtc.RTC_Seconds;
	rtca[1] = rtc.RTC_Minutes;
	rtca[0] = rtc.RTC_Hours;
	rtca[3] = rtc.RTC_Date;
	rtca[4] = rtc.RTC_Month;
	rtca[5] = rtc.RTC_Year;


	if ((key == K_UP) | (key == K_UPR)) {
		rtca[cursor]++;
	}
	if ((key == K_DOWN) | (key == K_DOWNR)) {
		rtca[cursor]--;
	}

	if ((key == K_RIGHT)) {
		cursor++;
		if (cursor>5) {
			cursor = 0;
			LOGGER.current = &LoggerDigitalSettings;
		}
	}
	if ((key == K_LEFT)) {
		cursor--;
		if (cursor<0) {
			cursor = 0;
			LOGGER.current = &LoggerSetMode;
		}
	}

	Display_Cmd(LCD_CMD_DDADDR | 0);
	Display_Data(' ');
	Display_Cmd(LCD_CMD_DDADDR | 0x40);
	Display_Data(' ');
	Display_Cmd(LCD_CMD_DDADDR | curpos_clkset[cursor]);
	Display_Data(0);


	rtca[2] = (rtca[2] > 23) ? 0 : rtca[2];
	rtca[1] = (rtca[1] > 59) ? 0 : rtca[1];
	rtca[0] = (rtca[0] > 59) ? 0 : rtca[0];
	rtca[3] = (rtca[3] > 31) ? 0 : rtca[3];
	rtca[4] = (rtca[4] > 12) ? 0 : rtca[4];

	rtc.RTC_Seconds = (uint8_t)rtca[2];
	rtc.RTC_Minutes = (uint8_t)rtca[1];
	rtc.RTC_Hours = (uint8_t)rtca[0];
	rtc.RTC_Date = (uint8_t)rtca[3];
	rtc.RTC_Month = (uint8_t)rtca[4];
	rtc.RTC_Year = rtca[5];

	RTC_SetCounter(RTC_GetRTC_Counter(&rtc));
}


void LoggerReadChannels(void) {
	LOGGER.Digital = 0;
	LOGGER.Digital |= (GPIO_ReadInputDataBit(DIG_GPIO, DIG_GPIO_CH1) == SET) ? (1<<0): 0;
	LOGGER.Digital |= (GPIO_ReadInputDataBit(DIG_GPIO, DIG_GPIO_CH2) == SET) ? (1<<1): 0;
	LOGGER.Digital |= (GPIO_ReadInputDataBit(DIG_GPIO, DIG_GPIO_CH3) == SET) ? (1<<2): 0;
	LOGGER.Digital |= (GPIO_ReadInputDataBit(DIG_GPIO, DIG_GPIO_CH4) == SET) ? (1<<3): 0;
}

void LoggerDigitalSettings(E_KEY key) {
	uint8_t i;
	Display_Print("DGTL:",0,0);

	Display_SetXY(5,0);
	for (i = 0; i<NUM_DIGITAL_CHANNELS; i++) {
		Display_Data((uint8_t)'1'+i);
		if ((LOGGER.Digital & (1<<i)) == 0) {
			Display_Data('L');
		} else {
			Display_Data('H');
		}
	}

	PrintCtrls();

	if ((key == K_RIGHT)) {
		LOGGER.current = &LoggerAnalogSettings;
	}
	if ((key == K_LEFT)) {
		LOGGER.current = &LoggerSetTime;
	}
}

void LoggerAnalogSettings(E_KEY key) {
	static uint8_t index = 0;
	uint8_t i;
	E_AC_MULTIPLIER mx;
	float fv = 0;
	char *sp;
	uint16_t tv;

	if (index>=NUM_ANALOG_CHANNELS) {
		index = 0;
	}

	Display_Print("ANLG:",0,0);

	Display_SetXY(5,0);
	for (i = 0; i<NUM_ANALOG_CHANNELS; i++) {
		if (index == i) {
			Display_Data(C_UPDN);
		} else {
			Display_Data(' ');
		}

		if (LOGGER.AMultiplier[i] == ACM_DISABLED) {
			Display_Data(C_DSBL);
		} else {
			Display_Data(C_ENBL);
		}
	}

	Display_SetXY(1,1);
	for (i=0; i<14; i++) {
		Display_Data(' ');
	}
	Display_SetXY(1,1);
	mx = LOGGER.AMultiplier[index];
	tv = LOGGER.Analog[index];
	switch (mx) {
	case ACM_DISABLED:
		sp = s_muld;
		strval[0] = 'x';
		strval[1] = '\0';
		break;
	case ACM_RAW:
		itoa(tv, strval,10);
		sp = s_mulr;
		break;
	case ACM_1x:
		sp = s_mul1x;
		fv = 3.3 * ((float)(tv)/(float)0xFFF);
		sftoa(fv,strval);
		break;
	case ACM_10x:
		sp = s_mul10x;
		fv = 3.3 * ((float)(tv)/(float)0xFFF)*10;
		sftoa(fv,strval);
		break;
	case ACM_100x:
		sp = s_mul100x;
		fv = 3.3 * ((float)(tv)/(float)0xFFF)*100;
		sftoa(fv,strval);
		break;
	default: sp = s_muld; break;
	}

	Display_PutS(sp);
	Display_Data('|');
	Display_PutS(strval);


	PrintCtrls();

	if ((key == K_RIGHT)) {
		index++;
		if (index >= NUM_ANALOG_CHANNELS) {
			index = 0;
			LOGGER.current = &LoggerMain;
		}
	}
	if ((key == K_LEFT)) {
		index--;
		if (index >= NUM_ANALOG_CHANNELS) {
			index = 0;
			LOGGER.current = &LoggerDigitalSettings;
		}
	}

	if ((key == K_UP)) {
		LOGGER.AMultiplier[index]++;
		if (LOGGER.AMultiplier[index]>ACM_100x) {
			LOGGER.AMultiplier[index] = ACM_DISABLED;
		}
	}

	if ((key == K_DOWN)) {
		LOGGER.AMultiplier[index]--;
		if (LOGGER.AMultiplier[index]>ACM_100x) {
			LOGGER.AMultiplier[index] = ACM_100x;
		}
	}
}


void sftoa(float fv, char *s) {
	char sb[5];
	uint16_t t;
	itoa(fv,s,10);
	strcat(s,".");
	t = (fv - (uint16_t)fv)*100;
	itoa(t,sb,10);
	strcat(s,sb);
}

void LoggerLoop(void) {
	static void(*pv)(E_KEY);

	RTC_GetDateTime(RTC_GetCounter(), &rtc);
	DateTimeStr(&rtc,timestr,datestr);
	LoggerReadChannels();

	LOGGER.current(GetPressedKey());

	if (LOGGER.current != pv) {
		Display_Cmd(0x00);
		Display_Cmd(0x01);	// clear sreen
		delay_ms(5);
	}

	if (LOGGER.LOG_ENABLED != 0) {
		if (LOGGER.LOG_TIMER<=0) {
			LOGGER.LOG_TIMER = LOGGER.LOG_DELAY;
			LoggerWriter();
		}
		LOGGER.LOG_TIMER--;
	}
	pv = LOGGER.current;
}


void LoggerWriter(void) {
	FRESULT f_err;
	uint8_t i;
	float fv = 0;
	E_AC_MULTIPLIER mx;
	uint16_t tv;
	char separator[2];
	separator[0] = LOGGER.Sep;
	separator[1] = '\0';

	memset(wrstr,0,sizeof(wrstr));

	strcpy(wrstr,timestr);
	strcat(wrstr,separator);
	strcat(wrstr,datestr);
	strcat(wrstr,separator);

	for (i = 0; i<NUM_DIGITAL_CHANNELS; i++) {
		if ((LOGGER.Digital & (1<<i)) == 0) {
			strcat(wrstr,"L");
		} else {
			strcat(wrstr,"H");
		}
		strcat(wrstr,separator);
	}


	for (i = 0; i<NUM_ANALOG_CHANNELS; i++) {
		mx = LOGGER.AMultiplier[i];
		tv = LOGGER.Analog[i];

		switch (mx) {
		case ACM_DISABLED:
			strval[0] = 'x';
			strval[1] = '\0';
			break;
		case ACM_RAW:
			itoa(tv, strval,10);
			break;
		case ACM_1x:
			fv = 3.3 * ((float)(tv)/(float)0xFFF);
			sftoa(fv,strval);
			break;
		case ACM_10x:
			fv = 3.3 * ((float)(tv)/(float)0xFFF)*10;
			sftoa(fv,strval);
			break;
		case ACM_100x:
			fv = 3.3 * ((float)(tv)/(float)0xFFF)*100;
			sftoa(fv,strval);
			break;
		default: break;
		}
		strcat(wrstr,strval);
		strcat(wrstr,separator);
	}
	strcat(wrstr,"\r\n");


	Display_Print("x",15,1);
	g_ferr = LogAddLine(wrstr);
	Display_Print(" ",15,1);
}

void ZeroPad(char *str) {
	if (strlen(str) <= 1) {
		str[1] = str[0];
		str[0] = '0';
		str[2] = '\0';
	}
}

void DateTimeStr(RTC_DateTimeTypeDef* rtcs, char * ts, char * ds) {

	itoa(rtcs->RTC_Hours,strval,10);
	ZeroPad(strval);
	strcpy(ts,strval);
	strcat(ts,":");

	itoa(rtcs->RTC_Minutes,strval,10);
	ZeroPad(strval);
	strcat(ts,strval);
	strcat(ts,":");

	itoa(rtcs->RTC_Seconds,strval,10);
	ZeroPad(strval);
	strcat(ts,strval);

	itoa(rtcs->RTC_Date,strval,10);
	ZeroPad(strval);
	strcpy(ds,strval);
	strcat(ds,".");

	itoa(rtcs->RTC_Month,strval,10);
	ZeroPad(strval);
	strcat(ds,strval);
	strcat(ds,".");

	itoa(rtcs->RTC_Year,strval,10);
	ZeroPad(strval);
	strcat(ds,strval);
}


void DisplayTime(uint8_t sh) {
	// display time
	Display_Print(timestr,sh,0);
	// display date
	Display_Print(datestr,sh,1);
}

void SetRTC (void) {
	if (RTC_Init() == 1) {
		rtc.RTC_Date = 2;
		rtc.RTC_Month = 7;
		rtc.RTC_Year = 2017;
		rtc.RTC_Hours = 17;
		rtc.RTC_Minutes = 37;
		rtc.RTC_Seconds = 00;
		delay_ms(500);
		RTC_SetCounter(RTC_GetRTC_Counter(&rtc));
		return 1;
	} else {
		return 0;
	}
}

FRESULT LogAddLine(char * string) {
	uint16_t cnt;

	FRESULT f_err;
	FATFS fsobj;
	FIL fobj;

	f_err = f_mount(&fsobj,"",1);
	if (f_err == FR_OK) {
		f_err = f_open(&fobj, logname, (FA_OPEN_ALWAYS | FA_WRITE));
		if (f_err == FR_OK) {
			f_err = f_lseek(&fobj, f_size(&fobj));
			if (f_err == FR_OK) {
				strcpy(tempb,string);
				f_err = f_write(&fobj,tempb,strlen(string),&cnt);
			}
			f_close(&fobj);
		}
		f_mount(NULL,"",1);
	}
	return f_err;
}



void ITInit(void) {
	NVIC_InitTypeDef nitd;
	nitd.NVIC_IRQChannel = TIMx_IRQn;
	nitd.NVIC_IRQChannelCmd = ENABLE;
	nitd.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Init(&nitd);

	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);

	__enable_irq();
}

void TIMInit(void) {
	RCC_APB1PeriphClockCmd(TIMx_RCC,ENABLE);

	TIM_TimeBaseInitTypeDef tbi;
	tbi.TIM_Prescaler = 7200;
	tbi.TIM_CounterMode = TIM_CounterMode_Up;
	tbi.TIM_ClockDivision = 0;
	tbi.TIM_Period = 10;
	TIM_TimeBaseInit(TIMx, &tbi);

	TIM_Cmd(TIMx, ENABLE);
}

void GPIOInit(void) {
	GPIO_InitTypeDef port;
	RCC_APB2PeriphClockCmd((GPIO_RCC | DIG_GPIO_RCC | AN_GPIO_RCC),ENABLE);

	GPIO_StructInit(&port);
	port.GPIO_Pin = PIN_RX;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	port.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(UGPIO,&port);

	port.GPIO_Pin = PIN_TX;
	port.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(UGPIO, &port);

	GPIO_StructInit(&port);
	port.GPIO_Pin = (DIG_GPIO_CH1 | DIG_GPIO_CH2 | DIG_GPIO_CH3 | DIG_GPIO_CH4);
	port.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DIG_GPIO, &port);

	GPIO_StructInit(&port);
	port.GPIO_Pin = (AN_GPIO_CH1 | AN_GPIO_CH2 | AN_GPIO_CH3 | AN_GPIO_CH4);
	port.GPIO_Mode = GPIO_Mode_AIN;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AN_GPIO, &port);

}

void DMAInit(void) {
	DMA_InitTypeDef dis;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	dis.DMA_BufferSize = NUM_ANALOG_CHANNELS;
	dis.DMA_DIR = DMA_DIR_PeripheralSRC;
	dis.DMA_M2M = DMA_M2M_Disable;
	dis.DMA_MemoryBaseAddr = (uint32_t)(&LOGGER.Analog[0]);
	dis.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dis.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dis.DMA_Mode = DMA_Mode_Circular;
	dis.DMA_PeripheralBaseAddr = (uint32_t)(&ADC1->DR);
	dis.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dis.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dis.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel1, &dis);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

void ADCInit(void) {
	ADC_InitTypeDef ais;

	RCC_APB2PeriphClockCmd(AN_ADCx_RCC, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	ais.ADC_ContinuousConvMode = ENABLE;
	ais.ADC_DataAlign = ADC_DataAlign_Right;
	ais.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ais.ADC_Mode = ADC_Mode_Independent;
	ais.ADC_NbrOfChannel = 4;
	ais.ADC_ScanConvMode = ENABLE;
	ADC_Init(AN_ADCx, &ais);

	ADC_RegularChannelConfig(AN_ADCx, AN_CH1, 1, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(AN_ADCx, AN_CH2, 2, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(AN_ADCx, AN_CH3, 3, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(AN_ADCx, AN_CH4, 4, ADC_SampleTime_7Cycles5);
	ADC_Cmd(AN_ADCx, ENABLE);
	ADC_DMACmd(AN_ADCx, ENABLE);
	ADC_ResetCalibration(AN_ADCx);

	while (ADC_GetResetCalibrationStatus(AN_ADCx)) {};
	ADC_StartCalibration(AN_ADCx);

	while (ADC_GetCalibrationStatus(AN_ADCx));
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);


}



/*---------------------------------------------------------*/
/* User provided RTC function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called back     */
/* from FatFs module.                                      */

#if !FF_FS_NORTC && !FF_FS_READONLY
DWORD get_fattime (void)
{
	uint32_t RTC_Counter = 0;
	RTC_DateTimeTypeDef rtc;


	RTC_Counter = RTC_GetCounter();
	RTC_GetDateTime(RTC_Counter, &rtc);


	/* Pack date and time into a DWORD variable */
	return	  ((DWORD)(rtc.RTC_Year - 1980) << 25)
			| ((DWORD)rtc.RTC_Month << 21)
			| ((DWORD)rtc.RTC_Date << 16)
			| ((DWORD)rtc.RTC_Hours << 11)
			| ((DWORD)rtc.RTC_Minutes << 5)
			| ((DWORD)rtc.RTC_Seconds >> 1);
}
#endif

