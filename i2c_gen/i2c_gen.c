#include "stm32f10x.h"
#include "i2c_gen.h"

void I2CInit(void) {
	I2C_InitTypeDef	iis;
	GPIO_InitTypeDef port;

	RCC_APB2PeriphClockCmd(I2C_GPIO_RCC ,ENABLE);
	RCC_APB1PeriphClockCmd(I2C_RCC, ENABLE);

	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_AF_OD;
	port.GPIO_Pin = (I2C_SDA | I2C_SCL);
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2Cx_Port, &port);

	I2C_StructInit(&iis);
	iis.I2C_Ack = I2C_Ack_Disable;
	iis.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	iis.I2C_Mode = I2C_Mode_I2C;
	iis.I2C_DutyCycle = I2C_DutyCycle_2;
	iis.I2C_OwnAddress1 = 0x1A;
	iis.I2C_ClockSpeed = I2C_Spd;
	I2C_Init(I2Cx, &iis);
	I2C_Cmd(I2Cx, ENABLE);
}

void I2C_Start() {
  uint16_t w = (uint16_t)-1;
  I2C_GenerateSTART(I2Cx,ENABLE);
  while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT));
}

void I2C_TXInit(uint8_t address) {
   uint16_t w = (uint16_t)-1;
   I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);
   while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
}
void I2C_RXInit(uint8_t address) {
  uint16_t w = (uint16_t)-1;
  I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
}

void I2C_WriteByte(uint8_t byte) {
  I2C_SendData(I2Cx, byte);
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t I2C_ReadByte(void) {
  uint16_t w = (uint16_t)-1;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
  uint8_t data = I2C_ReceiveData(I2Cx);
  return data;
}
