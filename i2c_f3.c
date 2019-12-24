#include "i2c_f3.h"
#include "mpu6050.h"

typedef enum _I2C_Direction {I2C_Transmitter=0, I2C_Receiver=1} I2C_Direction;

void i2c_init(void)
    {
    RCC->AHBENR  |= RCC_AHBPeriph_GPIOB;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;		// Включаю тактирование I2C
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW;
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C1_FMP|
		     SYSCFG_CFGR1_I2C_PB6_FMP|
		     SYSCFG_CFGR1_I2C_PB7_FMP;
    // Настройка ног PA9, PA10
    GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;		// Режим альтернативной функции
    GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;		// Открытый коллектор
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;	// Максимальная скорость
	// Выбор альтернативной функции
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_4);	// I2C1_SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);	// I2C1_SDA

    I2C1->CR1 &= ~I2C_CR1_PE;		// Отключаю I2C
    while (I2C1->CR1 & I2C_CR1_PE) {};	// Жду пока отключится

    // Частота тактирования модуля I2C = 72 МГц
    // Частота шины I2C = 1000 kHz
    // Настраиваю тайминги
    I2C1->TIMINGR |= \
	(3  << I2C_OFFSET_TIMINGR_PRESC)|\
	(36 << I2C_OFFSET_TIMINGR_SCLL)|\
	(12  << I2C_OFFSET_TIMINGR_SCLH)|\
	(8  << I2C_OFFSET_TIMINGR_SCLDEL)|\
	(0  << I2C_OFFSET_TIMINGR_SDADEL);

    I2C1->CR1 |= I2C_CR1_PE;			// Включаю I2C
    while ((I2C1->CR1 & I2C_CR1_PE)==0);	// Жду пока включится

    NVIC_InitTypeDef NVIC_InitStructure;
	  /* Configure the Priority Group to 2 bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //конфигурируем количество групп и подгрупп прерываний
	  /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn; //прерывание по uart2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //задаем приоритет в группе
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //задаем приоритет в подгруппе
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //разрешаем прерывание
    NVIC_Init(&NVIC_InitStructure); //инициализируем
    }


/*
Это служебная функция, использовать её не нужно.
Устанавливает направление данных - приём или передача.
Задаёт объём пересылаемых данных.
Задаёт адрес ведомого устройства.
Выдаёт старт на шину.
Параметры:
Direction - направление (0-отправка, 1-приём)
Adress - адрес ведомого устройства
Size - размер данных (от 1 до 255 байт)
*/
void I2C_Start_Direction_Adress_Size (I2C_Direction Direction, u8 Adress, u8 Size)
    {
	//I2C1->CR2 &= ~I2C_CR2_AUTOEND;				// Выдавать стоп вручную
	//I2C1->CR2 &= ~I2C_CR2_RELOAD;				// Не использовать режим перезагрузки
	if (Direction) I2C1->CR2 |= I2C_CR2_RD_WRN;	// Режим приёма
	else I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= Size<<I2C_OFFSET_CR2_NBYTES;	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= Adress;			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
    }


/*
Это служебная функция, использовать её не нужно.
Выдаёт стоп на шину.
Очищает флаги.
Проверяет наличие ошибок, очищает флаги ошибок.
*/
void i2c_stop(void)
    {
	I2C1->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
	while (I2C1->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
	// Очищаю флаги - необходимо для дальнейшей работы шины
	I2C1->ICR |= I2C_ICR_STOPCF|I2C_ICR_NACKCF;// STOP флаг // NACK флаг
	// Если есть ошибки на шине - очищаю флаги
	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C1->ICR |= I2C_ICR_ARLOCF;
		I2C1->ICR |= I2C_ICR_BERRCF;
	}
    }

void i2c_read_buf_int(u8 Adress, u8 Register, u8 Size)
{
    // Старт
    I2C_Start_Direction_Adress_Size (I2C_Transmitter, Adress, 1);
    // Сейчас либо I2C запросит первый байт для отправки,
    // Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
    // Если взлетит NACK-флаг, отправку прекращаем.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = Register;	// Отправляю адрес регистра
	}
    // Повторный старт
    //I2C_Start_Direction_Adress_Size (I2C_Receiver, Adress, Size);
    I2C1->CR1 |= I2C_CR1_RXIE | I2C_CR1_TCIE;
    I2C1->CR2 |= I2C_CR2_RD_WRN;
    I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
    I2C1->CR2 |= Size<<I2C_OFFSET_CR2_NBYTES;	// Установить размер данных
    I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
    I2C1->CR2 |= Adress;			// Установить адрес ведомого устройства
    I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
    while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
    // Принимаем байты до тех пор, пока не взлетит включаем прерывание на прием

}


inline void i2c_get_buf_int(u8 *Data, u8 len)
    {
    for(u32 i = 0;i<len;i++)
	{
        *Data++ = i2c_data_get_buf[i];
	}
    }



void I2C1_EV_IRQHandler(void)
    {
    static u8 Count = 0;// Счётчик успешно принятых байт
    //GPIOE->BSRR = GPIO_BSRR_BS_6;
    if (I2C1->ISR & I2C_ISR_RXNE)//если прилетел 1 байт
	{
	*(i2c_data_get_buf+Count++) = I2C1->RXDR;
	}
    if (I2C1->ISR & I2C_ISR_TC)//если приняли все
	{
	//GPIOE->BSRR = GPIO_BSRR_BS_6;
	Count = 0;
	I2C1->CR1 &=~ (I2C_CR1_RXIE | I2C_CR1_TCIE);
	i2c_stop();
	SetTask(MPU_get_data);
	//GPIOE->BSRR = GPIO_BSRR_BR_6;
	}

    }


/*
Выполняет транзакцию записи Size байт в регистр Register по адресу Adress.
Параметры:
Adress - адрес ведомого устройства
Register - регистр, в который хотим передать данные
Data - указывает откуда брать данные для передачи
Size - сколько байт хотим передать (от 1 до 254)
Возвращает:
1 - если данные успешно переданы
0 - если произошла ошибка
*/
u8 i2c_write_buf(u8 Adress, u8 Register, u8 *Data, u8 Size)
    {
    u8 Count=0;	// Счётчик успешно переданных байт
    // Старт
    I2C_Start_Direction_Adress_Size (I2C_Transmitter, Adress, 1+Size);
    // Сейчас либо I2C запросит первый байт для отправки,
    // Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
    // Если взлетит NACK-флаг, отправку прекращаем.
    while ((((I2C1->ISR & I2C_ISR_TXIS)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY)) {};
    if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=Register;	// Отправляю адрес регистра
    // Отправляем байты до тех пор, пока не взлетит TC-флаг.
    // Если взлетит NACK-флаг, отправку прекращаем.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=*(Data+Count++);	// Отправляю данные
	}
    i2c_stop();
    if (Count == Size)
	{return 1;}
    else{return 0;}
    }

void i2c_write(u8 Adress, u8 Register, u8 data)
    {
    // Старт
    I2C_Start_Direction_Adress_Size (I2C_Transmitter, Adress, 2);
    while ((((I2C1->ISR & I2C_ISR_TXIS)==0)// Сейчас либо I2C запросит первый байт для отправки,
	&& ((I2C1->ISR & I2C_ISR_NACKF)==0))// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	&& (I2C1->ISR & I2C_ISR_BUSY));// Если взлетит NACK-флаг, отправку прекращаем.
    if (I2C1->ISR & I2C_ISR_TXIS){I2C1->TXDR=Register;}	// Отправляю адрес регистра
	// Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR= data ;	// Отправляю данные
	}
    i2c_stop();
    }

void i2c_write_1byte(u8 data)
    {
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
    I2C1->CR2 &= ~I2C_CR2_NBYTES;			// Очистить размер данных
    I2C1->CR2 |= 1<<I2C_OFFSET_CR2_NBYTES;	// Установить размер данных
    I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
    I2C1->CR2 |= data;		// Установить адрес ведомого устройства
    }


/*
Выполняет транзакцию чтения Size байт из регистра Register по адресу Adress.
Параметры:
Adress - адрес ведомого устройства
Register - регистр, из которого хотим принять данные
Data - указывает куда складывать принятые данные
Size - сколько байт хотим принять (от 1 до 255)
Возвращает:
1 - если данные успешно приняты
0 - если произошла ошибка
*/
u8 i2c_read_buf(u8 Adress, u8 Register, u8 *Data, u8 Size)
{
    u8 Count=0;	// Счётчик успешно принятых байт
    // Старт
    I2C_Start_Direction_Adress_Size (I2C_Transmitter, Adress, 1);
    // Сейчас либо I2C запросит первый байт для отправки,
    // Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
    // Если взлетит NACK-флаг, отправку прекращаем.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = Register;	// Отправляю адрес регистра
	}
    // Повторный старт
    I2C_Start_Direction_Adress_Size (I2C_Receiver, Adress, Size);
    // Принимаем байты до тех пор, пока не взлетит TC-флаг.
    // Если взлетит NACK-флаг, приём прекращаем.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_RXNE) *(Data+Count++) = I2C1->RXDR;	// Принимаю данные
	}
    i2c_stop();
    if (Count == Size) return 1; return 0;
}


u8 i2c_read(u8 Adress, u8 Register)
{
    volatile static u8 data;
    // Старт
    I2C_Start_Direction_Adress_Size (I2C_Transmitter, Adress, 1);
    // Сейчас либо I2C запросит первый байт для отправки,
    // Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
    // Если взлетит NACK-флаг, отправку прекращаем.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = Register;	// Отправляю адрес регистра
	}
    // Повторный старт
    I2C_Start_Direction_Adress_Size (I2C_Receiver, Adress, 1);
    // Принимаем байты до тех пор, пока не взлетит TC-флаг.
    // Если взлетит NACK-флаг, приём прекращаем.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_RXNE) {data = (I2C1->RXDR);}	// Принимаю данные
	}
    i2c_stop();
    return data;
}
