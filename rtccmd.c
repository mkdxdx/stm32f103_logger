#include "stm32f10x.h"
#include "rtccmd.h"

uint8_t RTC_Init(void) {
	// pwr and backup domain clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	// allow backup access
	PWR_BackupAccessCmd(ENABLE);

	if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)
	{
		// reset backup
		RCC_BackupResetCmd(ENABLE);
		RCC_BackupResetCmd(DISABLE);

		// turn 32khz osc on
		RCC_LSEConfig(RCC_LSE_ON);
		while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) {}
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

		// set timer prescaler to count seconds
		RTC_SetPrescaler(0x7FFF);

		// rtc enable
		RCC_RTCCLKCmd(ENABLE);

		RTC_WaitForSynchro();

		return 1;
	}
	return 0;
}


void RTC_GetDateTime(uint32_t RTC_Counter,RTC_DateTimeTypeDef* RTC_DateTimeStruct) {
    unsigned long time;
    unsigned long t1, a, b, c, d, e, m;
    int year = 0;
    int mon = 0;
    int wday = 0;
    int mday = 0;
    int hour = 0;
    int min = 0;
    int sec = 0;
    uint64_t jd = 0;;
    uint64_t jdn = 0;

    jd = ((RTC_Counter+43200)/(86400>>1)) + (2440587<<1) + 1;
    jdn = jd>>1;

    time = RTC_Counter;
    t1 = time/60;
    sec = time - t1*60;

    time = t1;
    t1 = time/60;
    min = time - t1*60;

    time = t1;
    t1 = time/24;
    hour = time - t1*24;

    wday = jdn%7;

    a = jdn + 32044;
    b = (4*a+3)/146097;
    c = a - (146097*b)/4;
    d = (4*c+3)/1461;
    e = c - (1461*d)/4;
    m = (5*e+2)/153;
    mday = e - (153*m+2)/5 + 1;
    mon = m + 3 - 12*(m/10);
    year = 100*b + d - 4800 + (m/10);

    RTC_DateTimeStruct->RTC_Year = year;
    RTC_DateTimeStruct->RTC_Month = mon;
    RTC_DateTimeStruct->RTC_Date = mday;
    RTC_DateTimeStruct->RTC_Hours = hour;
    RTC_DateTimeStruct->RTC_Minutes = min;
    RTC_DateTimeStruct->RTC_Seconds = sec;
    RTC_DateTimeStruct->RTC_Wday = wday;
}

// Convert Date to Counter
uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct) {
    uint8_t a;
    uint16_t y;
    uint8_t m;
    uint32_t JDN;

    a=(14-RTC_DateTimeStruct->RTC_Month)/12;
    y=RTC_DateTimeStruct->RTC_Year+4800-a;
    m=RTC_DateTimeStruct->RTC_Month+(12*a)-3;

    JDN=RTC_DateTimeStruct->RTC_Date;
    JDN+=(153*m+2)/5;
    JDN+=365*y;
    JDN+=y/4;
    JDN+=-y/100;
    JDN+=y/400;
    JDN = JDN -32045;
    JDN = JDN - JULIAN_DATE_BASE;
    JDN*=86400;
    JDN+=(RTC_DateTimeStruct->RTC_Hours*3600);
    JDN+=(RTC_DateTimeStruct->RTC_Minutes*60);
    JDN+=(RTC_DateTimeStruct->RTC_Seconds);

    return JDN;
}
