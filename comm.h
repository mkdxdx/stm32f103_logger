#define USARTx			USART1
#define PIN_RX			GPIO_Pin_10
#define PIN_TX			GPIO_Pin_9
#define UGPIO			GPIOA
#define GPIO_RCC		RCC_APB2Periph_GPIOA
#define USART_RCC		RCC_APB2Periph_USART1
#define AFIO_RCC		RCC_APB2Periph_AFIO


void USARTInit(uint32_t baud);
void USend(char data);
void USendStr(char *data);
