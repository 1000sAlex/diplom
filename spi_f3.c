#include "spi_f3.h"


#define SPI2_DR_8bit         *(__IO uint8_t*)&(SPI2->DR)

void spi_init(void)
{
//Включаем тактирование порта B
	RCC->AHBENR |=  RCC_AHBENR_GPIOBEN;
	//распиновка
	//PB13 - SCL  -  Alternative func. push-pull - OUT
	//PB14 - MISO -  Alternative func. push-pull - OUT  - IN
	//PB15 - MOSI -  Alternative func.  push-pull - OUT
	//PB12 - CS   -  GPIO - soft - OUT

	//13, 14, 15 вывод - альтернативная функция,
	GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
        GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13 |  GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR13 |  GPIO_PUPDR_PUPDR15);

        //назначаем выводам необходимые альтернативные ф-ции
	GPIOB->AFR[1] |= (0x05<<5*4);
	GPIOB->AFR[1] |= (0x05<<6*4);
	GPIOB->AFR[1] |= (0x05<<7*4);

	// 12 вывод - выход с подтяжкой к питанию
	GPIOB->MODER |= GPIO_MODER_MODER12_0;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR12_0;

	//конфигурируем SPI2
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;     //Enable oscil SPI2
	SPI2->CR1 |= SPI_CR1_BR_0|SPI_CR1_BR_2; //Baud rate = Fpclk/64
	SPI2->CR1 |= SPI_CR1_CPOL;              //Polarity cls signal CPOL = 0;
	SPI2->CR1 |= SPI_CR1_CPHA;              //Phase cls signal    CPHA = 0;
	SPI2->CR1 &= ~SPI_CR1_LSBFIRST;         //MSB will be first
	SPI2->CR1 |= SPI_CR1_SSM;               //Program mode NSS
	SPI2->CR1 |= SPI_CR1_SSI;               //анналогично состоянию, когда NSS 1
	SPI2->CR1 |= SPI_CR1_MSTR;              //Mode Master
	SPI2->CR1 |= SPI_CR1_SPE;               //Enable SPI2

	SPI2->CR2 |= SPI_CR2_FRXTH;            //RXNE 8 bit
	SPI2->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;  		//Data_Size
	GPIOB->BSRR = GPIO_BSRR_BS_12;
}

inline u8 spi_sendByte(u8 byteToSend)
{
    while ((SPI2->SR & SPI_I2S_FLAG_TXE) == RESET){};
    SPI_SendData8(SPI2, byteToSend);
    while ((SPI2->SR &  SPI_I2S_FLAG_RXNE) == RESET){};
    return (u8)SPI2_DR_8bit;
}

void spi_writeData(u8 address, u8 dataToWrite)
{
    SPI2->CR1 |= SPI_CR1_BR_0|SPI_CR1_BR_2; //Baud rate = Fpclk/64
    GPIOB->BSRR = GPIO_BSRR_BR_12;
    spi_sendByte(address & 0x7F);
    spi_sendByte(dataToWrite);
    GPIOB->BSRR = GPIO_BSRR_BS_12;
}

u8 spi_readData(u8 address)
{
    u8 temp = 0;
    SPI2->CR1 |= SPI_CR1_BR_0|SPI_CR1_BR_2; //Baud rate = Fpclk/64
    GPIOB->BSRR = GPIO_BSRR_BR_12;
    spi_sendByte(address | 0x80);
    temp = spi_sendByte(0x00);
    GPIOB->BSRR = GPIO_BSRR_BS_12;
    return temp;
}

void spi_readData_buf(u8 address, u8 *data, u8 len)
{
    SPI2->CR1 &=~ SPI_CR1_BR; //Baud rate = Fpclk/2
    GPIOB->BSRR = GPIO_BSRR_BR_12;
    spi_sendByte(address | 0x80);
    for (u8 i = 0;i<len;i++)
	{
	*data++ = spi_sendByte(0x00);
	}
    GPIOB->BSRR = GPIO_BSRR_BS_12;
}
