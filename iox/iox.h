#define IOX_Addr	(0x27<<1) // address must be preshifted

uint8_t IOXRead(void);
void IOXWrite(uint8_t data);
