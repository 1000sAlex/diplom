#include "EERTOS_f3.h"
#include "stm32f30x_i2c.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"
/*
Битовые смещения для настройки разных регистров
Чтобы не использовать в коде магические числа, и код был читаемым
Например:
I2C->CR2 |= 1<<I2C_OFFSET_CR2_NBYTES;	// Здесь понятно, что передаётся 1 байт
I2C->CR2 |= 1<<16;						// Здесь возникает путаница, то ли 1 байт, то ли 16
*/
#define I2C_OFFSET_TIMINGR_SCLL		0
#define I2C_OFFSET_TIMINGR_SCLH		8
#define I2C_OFFSET_TIMINGR_SDADEL	16
#define I2C_OFFSET_TIMINGR_SCLDEL	20
#define I2C_OFFSET_TIMINGR_PRESC	28
#define I2C_OFFSET_CR2_NBYTES		16

u8 i2c_data_get_buf[14];

void i2c_init(void);
u8 i2c_write_buf(u8 Adress, u8 Register, u8 *Data, u8 Size);
u8 i2c_read_buf(u8 Adress, u8 Register, u8 *Data, u8 Size);
void i2c_read_buf_int(u8 Adress, u8 Register,u8 Size);
void i2c_get_buf_int(u8 *Data, u8 len);
void i2c_write(u8 Adress, u8 Register, u8 data);
u8 i2c_read(u8 Adress, u8 Register);
