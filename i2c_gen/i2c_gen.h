// generic i2c header file
// this header file gives me free ~500b on multi-deviced i2c bus

#define I2C_RCC		RCC_APB1Periph_I2C2
#define I2Cx		I2C2
#define I2C_SDA		GPIO_Pin_11
#define I2C_SCL		GPIO_Pin_10
#define I2Cx_Port	GPIOB
#define I2C_Spd		100000
#define I2C_GPIO_RCC	RCC_APB2Periph_GPIOB

#define I2C_Stop()      I2C_GenerateSTOP(I2Cx,ENABLE)

void I2CInit(void);
void I2C_Start();
void I2C_TXInit(uint8_t address);
void I2C_RXInit(uint8_t address);
void I2C_WriteByte(uint8_t byte);
uint8_t I2C_ReadByte(void);
