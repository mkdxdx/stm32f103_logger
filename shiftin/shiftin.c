#include "stm32f10x.h"
#include "shiftin.h"
#include "delay.h"

void SH_GPIOInit(void) {
	GPIO_InitTypeDef port;
	
	SH_GPIO_RCC_CMD(SH_GPIO_RCC,ENABLE);
	

	GPIO_StructInit(&port);
	port.GPIO_Pin = (SH_GPIO_CLK | SH_GPIO_LD);
	port.GPIO_Mode = GPIO_Mode_Out_PP;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SH_GPIO, &port);
	
	GPIO_StructInit(&port);
	port.GPIO_Pin = (SH_GPIO_DAT);
	port.GPIO_Mode = GPIO_Mode_IPU;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SH_GPIO, &port);

}

uint8_t SH_Read(void) {


	CLK_HIGH();

	uint8_t data = 0;
	uint8_t i;
	
	LD_HIGH();
	LD_LOW();
	LD_HIGH();

	
	for (i = 0; i<8; i++) {
		data<<=1;
		
		CLK_HIGH();
		CLK_LOW();
		
		if (DAT_READ() == SET) {
			data |= 1<<0;
		}
	}
	
	return data;
}
