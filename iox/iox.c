#include "stm32f10x.h"
#include "iox.h"
#include "i2c_gen.h"

void IOXWrite(uint8_t data) {
	I2C_Start();
	I2C_TXInit(IOX_Addr);
	I2C_WriteByte(data);
	I2C_Stop();
}

uint8_t IOXRead(void) {
	I2C_Start();
	I2C_RXInit(IOX_Addr);
	uint8_t data = I2C_ReadByte();
	I2C_Stop();
	return data;
}
