#include "EERTOS_f3.h"
#include "stm32f30x_spi.h"

void spi_init(void);
u8 spi_sendByte(u8 byteToSend);
void spi_writeData(u8 address, u8 dataToWrite);
u8 spi_readData(u8 address);
void spi_readData_buf(u8 address, u8 *data, u8 len);
