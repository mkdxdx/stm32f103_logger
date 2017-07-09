#define SH_GPIO			GPIOB
#define SH_GPIO_RCC		RCC_APB2Periph_GPIOB
#define SH_GPIO_DAT		GPIO_Pin_9
#define SH_GPIO_CLK		GPIO_Pin_7
#define SH_GPIO_LD		GPIO_Pin_8
#define SH_GPIO_RCC_CMD	RCC_APB2PeriphClockCmd

#define LD_HIGH()		GPIO_WriteBit(SH_GPIO, SH_GPIO_LD, SET)
#define LD_LOW()		GPIO_WriteBit(SH_GPIO, SH_GPIO_LD, RESET)

#define CLK_HIGH()		GPIO_WriteBit(SH_GPIO, SH_GPIO_CLK, SET)
#define CLK_LOW()		GPIO_WriteBit(SH_GPIO, SH_GPIO_CLK, RESET)

#define DAT_READ()		GPIO_ReadInputDataBit(SH_GPIO, SH_GPIO_DAT)

#define SH_DELAY()		delay_us(100)

void SH_GPIOInit(void);
uint8_t SH_Read(void);
