#include "EERTOS_f3.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"

#define BAUDRATE2 115200
#define USART2_BRR_REG (F_CPU / BAUDRATE2)

#define BAUDRATE1 256000
#define USART1_BRR_REG (F_CPU / BAUDRATE1)

#define Uart_newLine() Uart1_write('\n')
#define Uart_Tab() Uart1_write('\t') 

void Uart1_init(void);
uint16_t Uart1_available(void);//количество прин€тых байт
uint8_t Uart1_overflow_flag(void);//флаг переполнени€
uint8_t Uart1_get_char(void);
void Uart1_write(uint8_t data);
void Uart1_string(char *s);


void Uart2_init(void);
uint16_t Uart2_available(void);//количество прин€тых байт
uint8_t Uart2_overflow_flag(void);//флаг переполнени€
uint8_t Uart2_get_char(void);
void Uart2_write(uint8_t data);
void Uart2_string(char *s);

char * utoa_builtin_div(u32 value, char *buffer);

#define NUMBER_UINT
#define NUMBER_INT
#define NUMBER_FLOAT
void Uart_IntWrite(s32 value);
void Uart2_IntWrite(s32 value);
void Uart_UintWrite(u32 value);
void Uart2_UintWrite(u32 value);
void Uart_FloatWrite(float value, u32 n);
