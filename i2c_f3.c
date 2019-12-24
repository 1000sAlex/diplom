#include "i2c_f3.h"
#include "mpu6050.h"

typedef enum _I2C_Direction {I2C_Transmitter=0, I2C_Receiver=1} I2C_Direction;

void i2c_init(void)
    {
    RCC->AHBENR  |= RCC_AHBPeriph_GPIOB;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;		// ������� ������������ I2C
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW;
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C1_FMP|
		     SYSCFG_CFGR1_I2C_PB6_FMP|
		     SYSCFG_CFGR1_I2C_PB7_FMP;
    // ��������� ��� PA9, PA10
    GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;		// ����� �������������� �������
    GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;		// �������� ���������
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;	// ������������ ��������
	// ����� �������������� �������
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_4);	// I2C1_SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);	// I2C1_SDA

    I2C1->CR1 &= ~I2C_CR1_PE;		// �������� I2C
    while (I2C1->CR1 & I2C_CR1_PE) {};	// ��� ���� ����������

    // ������� ������������ ������ I2C = 72 ���
    // ������� ���� I2C = 1000 kHz
    // ���������� ��������
    I2C1->TIMINGR |= \
	(3  << I2C_OFFSET_TIMINGR_PRESC)|\
	(36 << I2C_OFFSET_TIMINGR_SCLL)|\
	(12  << I2C_OFFSET_TIMINGR_SCLH)|\
	(8  << I2C_OFFSET_TIMINGR_SCLDEL)|\
	(0  << I2C_OFFSET_TIMINGR_SDADEL);

    I2C1->CR1 |= I2C_CR1_PE;			// ������� I2C
    while ((I2C1->CR1 & I2C_CR1_PE)==0);	// ��� ���� ���������

    NVIC_InitTypeDef NVIC_InitStructure;
	  /* Configure the Priority Group to 2 bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //������������� ���������� ����� � �������� ����������
	  /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn; //���������� �� uart2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //������ ��������� � ������
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //������ ��������� � ���������
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //��������� ����������
    NVIC_Init(&NVIC_InitStructure); //��������������
    }


/*
��� ��������� �������, ������������ � �� �����.
������������� ����������� ������ - ���� ��� ��������.
����� ����� ������������ ������.
����� ����� �������� ����������.
����� ����� �� ����.
���������:
Direction - ����������� (0-��������, 1-����)
Adress - ����� �������� ����������
Size - ������ ������ (�� 1 �� 255 ����)
*/
void I2C_Start_Direction_Adress_Size (I2C_Direction Direction, u8 Adress, u8 Size)
    {
	//I2C1->CR2 &= ~I2C_CR2_AUTOEND;				// �������� ���� �������
	//I2C1->CR2 &= ~I2C_CR2_RELOAD;				// �� ������������ ����� ������������
	if (Direction) I2C1->CR2 |= I2C_CR2_RD_WRN;	// ����� �����
	else I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// ����� ��������
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// �������� ������ ������
	I2C1->CR2 |= Size<<I2C_OFFSET_CR2_NBYTES;	// ���������� ������ ������
	I2C1->CR2 &= ~I2C_CR2_SADD;	// �������� ����� �������� ����������
	I2C1->CR2 |= Adress;			// ���������� ����� �������� ����������
	I2C1->CR2 |= I2C_CR2_START;					// ������ ����� �� ����
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// ������� ������ ������
    }


/*
��� ��������� �������, ������������ � �� �����.
����� ���� �� ����.
������� �����.
��������� ������� ������, ������� ����� ������.
*/
void i2c_stop(void)
    {
	I2C1->CR2 |= I2C_CR2_STOP;				// ������ ���� �� ����
	while (I2C1->ISR & I2C_ISR_BUSY) {};		// ������� ������ �����
	// ������ ����� - ���������� ��� ���������� ������ ����
	I2C1->ICR |= I2C_ICR_STOPCF|I2C_ICR_NACKCF;// STOP ���� // NACK ����
	// ���� ���� ������ �� ���� - ������ �����
	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C1->ICR |= I2C_ICR_ARLOCF;
		I2C1->ICR |= I2C_ICR_BERRCF;
	}
    }

void i2c_read_buf_int(u8 Adress, u8 Register, u8 Size)
{
    // �����
    I2C_Start_Direction_Adress_Size (I2C_Transmitter, Adress, 1);
    // ������ ���� I2C �������� ������ ���� ��� ��������,
    // ���� ������� NACK-����, ��������� � ���, ��� ���������� �� ��������.
    // ���� ������� NACK-����, �������� ����������.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = Register;	// ��������� ����� ��������
	}
    // ��������� �����
    //I2C_Start_Direction_Adress_Size (I2C_Receiver, Adress, Size);
    I2C1->CR1 |= I2C_CR1_RXIE | I2C_CR1_TCIE;
    I2C1->CR2 |= I2C_CR2_RD_WRN;
    I2C1->CR2 &= ~I2C_CR2_NBYTES;				// �������� ������ ������
    I2C1->CR2 |= Size<<I2C_OFFSET_CR2_NBYTES;	// ���������� ������ ������
    I2C1->CR2 &= ~I2C_CR2_SADD;	// �������� ����� �������� ����������
    I2C1->CR2 |= Adress;			// ���������� ����� �������� ����������
    I2C1->CR2 |= I2C_CR2_START;					// ������ ����� �� ����
    while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// ������� ������ ������
    // ��������� ����� �� ��� ���, ���� �� ������� �������� ���������� �� �����

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
    static u8 Count = 0;// ������� ������� �������� ����
    //GPIOE->BSRR = GPIO_BSRR_BS_6;
    if (I2C1->ISR & I2C_ISR_RXNE)//���� �������� 1 ����
	{
	*(i2c_data_get_buf+Count++) = I2C1->RXDR;
	}
    if (I2C1->ISR & I2C_ISR_TC)//���� ������� ���
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
��������� ���������� ������ Size ���� � ������� Register �� ������ Adress.
���������:
Adress - ����� �������� ����������
Register - �������, � ������� ����� �������� ������
Data - ��������� ������ ����� ������ ��� ��������
Size - ������� ���� ����� �������� (�� 1 �� 254)
����������:
1 - ���� ������ ������� ��������
0 - ���� ��������� ������
*/
u8 i2c_write_buf(u8 Adress, u8 Register, u8 *Data, u8 Size)
    {
    u8 Count=0;	// ������� ������� ���������� ����
    // �����
    I2C_Start_Direction_Adress_Size (I2C_Transmitter, Adress, 1+Size);
    // ������ ���� I2C �������� ������ ���� ��� ��������,
    // ���� ������� NACK-����, ��������� � ���, ��� ���������� �� ��������.
    // ���� ������� NACK-����, �������� ����������.
    while ((((I2C1->ISR & I2C_ISR_TXIS)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY)) {};
    if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=Register;	// ��������� ����� ��������
    // ���������� ����� �� ��� ���, ���� �� ������� TC-����.
    // ���� ������� NACK-����, �������� ����������.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=*(Data+Count++);	// ��������� ������
	}
    i2c_stop();
    if (Count == Size)
	{return 1;}
    else{return 0;}
    }

void i2c_write(u8 Adress, u8 Register, u8 data)
    {
    // �����
    I2C_Start_Direction_Adress_Size (I2C_Transmitter, Adress, 2);
    while ((((I2C1->ISR & I2C_ISR_TXIS)==0)// ������ ���� I2C �������� ������ ���� ��� ��������,
	&& ((I2C1->ISR & I2C_ISR_NACKF)==0))// ���� ������� NACK-����, ��������� � ���, ��� ���������� �� ��������.
	&& (I2C1->ISR & I2C_ISR_BUSY));// ���� ������� NACK-����, �������� ����������.
    if (I2C1->ISR & I2C_ISR_TXIS){I2C1->TXDR=Register;}	// ��������� ����� ��������
	// ���������� ����� �� ��� ���, ���� �� ������� TC-����.
	// ���� ������� NACK-����, �������� ����������.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR= data ;	// ��������� ������
	}
    i2c_stop();
    }

void i2c_write_1byte(u8 data)
    {
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// ����� ��������
    I2C1->CR2 &= ~I2C_CR2_NBYTES;			// �������� ������ ������
    I2C1->CR2 |= 1<<I2C_OFFSET_CR2_NBYTES;	// ���������� ������ ������
    I2C1->CR2 &= ~I2C_CR2_SADD;	// �������� ����� �������� ����������
    I2C1->CR2 |= data;		// ���������� ����� �������� ����������
    }


/*
��������� ���������� ������ Size ���� �� �������� Register �� ������ Adress.
���������:
Adress - ����� �������� ����������
Register - �������, �� �������� ����� ������� ������
Data - ��������� ���� ���������� �������� ������
Size - ������� ���� ����� ������� (�� 1 �� 255)
����������:
1 - ���� ������ ������� �������
0 - ���� ��������� ������
*/
u8 i2c_read_buf(u8 Adress, u8 Register, u8 *Data, u8 Size)
{
    u8 Count=0;	// ������� ������� �������� ����
    // �����
    I2C_Start_Direction_Adress_Size (I2C_Transmitter, Adress, 1);
    // ������ ���� I2C �������� ������ ���� ��� ��������,
    // ���� ������� NACK-����, ��������� � ���, ��� ���������� �� ��������.
    // ���� ������� NACK-����, �������� ����������.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = Register;	// ��������� ����� ��������
	}
    // ��������� �����
    I2C_Start_Direction_Adress_Size (I2C_Receiver, Adress, Size);
    // ��������� ����� �� ��� ���, ���� �� ������� TC-����.
    // ���� ������� NACK-����, ���� ����������.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_RXNE) *(Data+Count++) = I2C1->RXDR;	// �������� ������
	}
    i2c_stop();
    if (Count == Size) return 1; return 0;
}


u8 i2c_read(u8 Adress, u8 Register)
{
    volatile static u8 data;
    // �����
    I2C_Start_Direction_Adress_Size (I2C_Transmitter, Adress, 1);
    // ������ ���� I2C �������� ������ ���� ��� ��������,
    // ���� ������� NACK-����, ��������� � ���, ��� ���������� �� ��������.
    // ���� ������� NACK-����, �������� ����������.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = Register;	// ��������� ����� ��������
	}
    // ��������� �����
    I2C_Start_Direction_Adress_Size (I2C_Receiver, Adress, 1);
    // ��������� ����� �� ��� ���, ���� �� ������� TC-����.
    // ���� ������� NACK-����, ���� ����������.
    while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
	if (I2C1->ISR & I2C_ISR_RXNE) {data = (I2C1->RXDR);}	// �������� ������
	}
    i2c_stop();
    return data;
}
