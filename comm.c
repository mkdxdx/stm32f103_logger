#include "stm32f10x.h"
#include "comm.h"

void USARTInit(uint32_t baud) {
	RCC_APB2PeriphClockCmd(USART_RCC,ENABLE);

	USART_InitTypeDef uis;

	USART_StructInit(&uis);
	uis.USART_BaudRate = baud;
	uis.USART_Mode = (USART_Mode_Rx | USART_Mode_Tx);
	uis.USART_WordLength = USART_WordLength_8b;
	uis.USART_StopBits = USART_StopBits_1;
	uis.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USARTx,&uis);
	USART_Cmd(USARTx,ENABLE);
}

void USend(char data) {
	USART_SendData(USARTx,data);
	while(!USART_GetFlagStatus(USARTx,USART_FLAG_TC));
}

void USendStr(char *data) {
	while (*data!='\0') {
		USend(*data++);
	}
}
