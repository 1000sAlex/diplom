#include "uart.h"
#include "parsing.h"

// ����� �� �������� USART1
#define TX_BUFFER_SIZE_USART1 64 //������ ������
volatile uint8_t   tx_buffer_Uart1[TX_BUFFER_SIZE_USART1];
volatile uint16_t  tx_wr_index_Uart1 = 0, //������ ������ ������ (���� ������ ������)
                   tx_rd_index_Uart1 = 0, //������ ������ ������ (������ ������ ������)
                   tx_counter_Uart1 = 0; //���������� ������ � ������


// ����� �� ����� usart2
#define RX_BUFFER_SIZE_USART2 64 // ������ ������
volatile uint8_t    rx_buffer_uart2[RX_BUFFER_SIZE_USART2];
volatile uint16_t   rx_wr_index_uart2 = 0, //������ ������ ������ (���� ������ ������)
                    rx_rd_index_uart2 = 0, //������ ������ ������ (������ ������ ������)
                    rx_counter_uart2 = 0; //���������� ������ � ������
volatile uint8_t    rx_buffer_overflow_uart2 = 0; //���������� � ������������ ������
// ����� �� �������� usart2
#define TX_BUFFER_SIZE_USART2 164 //������ ������
volatile uint8_t   tx_buffer_uart2[TX_BUFFER_SIZE_USART2];
volatile uint16_t  tx_wr_index_uart2 = 0, //������ ������ ������ (���� ������ ������)
                   tx_rd_index_uart2 = 0, //������ ������ ������ (������ ������ ������)
                   tx_counter_uart2 = 0; //���������� ������ � ������



void buff_clean(char *s)
    {
    while (*s != 0)
        *s++ = 0;
    }


void Uart2_init(void)//�� ������
    {
    RCC->AHBENR |= (RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN);
    RCC->APB1ENR |= (RCC_APB1Periph_USART2);

    NVIC_InitTypeDef NVIC_InitStructure;
          /* Configure the Priority Group to 2 bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //������������� ���������� ����� � �������� ����������
          /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //���������� �� uart2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //������ ��������� � ������
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //������ ��������� � ���������
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //��������� ����������
    NVIC_Init(&NVIC_InitStructure); //��������������
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

//    USART2->CR3 |= USART_CR3_CTSE;//�������� ������� CTS
//    USART2->CR3 |= USART_CR3_RTSE;//�������� ������� RTS �� ��������� �����

    GPIOB->MODER |= (GPIO_MODER_MODER3_1|GPIO_MODER_MODER4_1);//alternative mode
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR3|GPIO_OSPEEDER_OSPEEDR4);//High speed
    GPIO_PinAFConfig(GPIOB,3,7);//��������� ���� ����� �� ��� ����
    GPIO_PinAFConfig(GPIOB,4,7);

    USART2->BRR= USART2_BRR_REG/2; //BaudRate *2 ���� ���� ��� ���� ����������� ���������
    USART2->CR1 |= USART_CR1_UE; //��������� ������ USART2
    USART2->CR1 |= USART_CR1_TE; //�������� ����������
    USART2->CR1 |= USART_CR1_RE; //�������� ��������
    }


inline uint16_t Uart2_available(void)//���������� �������� ����
    {
    return rx_counter_uart2;
    }

inline uint8_t Uart2_overflow_flag(void)//���� ������������
    {
    return rx_buffer_overflow_uart2;
    }

inline uint8_t Uart2_get_char(void) //����� ������
{
    uint8_t data; //���������� ��� ������
//    while (rx_counter_uart2==0);  //���� ������ ���, ����

    data = rx_buffer_uart2[rx_rd_index_uart2++]; //����� ������ �� ������
    if (rx_rd_index_uart2 == RX_BUFFER_SIZE_USART2) rx_rd_index_uart2 = 0; //���� �� �����

    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); //��������� ����������
    --rx_counter_uart2; //����� ��� �� �������� �������� ����������
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������� ����������
    return data;
}

void Uart2_write(uint8_t data) //����� ������
{
//    while (tx_counter_uart2 == TX_BUFFER_SIZE_USART2); //���� ����� ����������, ����
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE); //��������� ����������, ����� ��� �� ������ ������ ����������
    if (tx_counter_uart2 || (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)) //���� � ������ ��� ���-�� ���� ��� ���� � ������ ������ ���-�� ��� ����������
	{
	tx_buffer_uart2[tx_wr_index_uart2++] = data; //�� ������ ������ � �����
	if (tx_wr_index_uart2 == TX_BUFFER_SIZE_USART2) tx_wr_index_uart2 = 0; //���� �� �����
	++tx_counter_uart2; //����������� ������� ���������� ������ � ������
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //��������� ����������
	}
    else //���� UART ��������
	{
	USART_SendData(USART2,data); //�������� ������ ��� ����������
	}
}

void Uart2_string(char *s)
{
  while (*s != 0)
      Uart2_write(*s++);
}

void USART2_IRQHandler(void)
{
  u8 temp_data;
  if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET) //���������� �� ������ ������
  {
      if ((USART2->ISR & (USART_FLAG_NE|USART_FLAG_FE|USART_FLAG_PE|USART_FLAG_ORE)) == 0)//��������� ��� �� ������
	  {
	  temp_data = USART_ReceiveData(USART2);//��������� ������
	  rx_buffer_uart2[rx_wr_index_uart2++] = (uint8_t)(temp_data);//��������� ������ � �����, ������������� ����� ������

          if (rx_wr_index_uart2 == RX_BUFFER_SIZE_USART2) rx_wr_index_uart2 = 0;//���� �� �����
          if (++rx_counter_uart2 == RX_BUFFER_SIZE_USART2)//������������ ������
          	{
                rx_counter_uart2 = 0;//�������� ������� (������� ��� ������)
                rx_buffer_overflow_uart2 = 1;//�������� � ������������
                }
          if (temp_data == ';')//�����
              {
              SetTimerTask(Uart_pars,5);
              }
          }
	  else USART_ReceiveData(USART2);//������ ����� ����� ���������� ������, � �� ������ ���������� ����� ����
  }

  if(USART_GetITStatus(USART2, USART_IT_ORE) == SET) //���������� �� ������������ ������
  {
      USART_ReceiveData(USART2); //� ������ ����� ����� ���������� ������������ ������, �� �� ������ ���������� ���� ���� ���������� ������� �� �������� ������.
  }

  if(USART_GetITStatus(USART2, USART_IT_TXE) == SET) //���������� �� ��������
  {
  	if (tx_counter_uart2)//���� ���� ��� ��������
        {
        --tx_counter_uart2;// ��������� ���������� �� ���������� ������
        USART_SendData(USART2,tx_buffer_uart2[tx_rd_index_uart2++]);//�������� ������ ������������� ����� ������
        if (tx_rd_index_uart2 == TX_BUFFER_SIZE_USART2) tx_rd_index_uart2 = 0;//���� �� �����
        }
        else//���� ������ ��������, ��������� ���������� �� ��������
        {
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
  }
}


void Uart1_init(void)//�� ������
    {
    RCC->AHBENR |= (RCC_AHBENR_GPIOCEN);
    RCC->APB2ENR |= (RCC_APB2Periph_USART1);

    NVIC_InitTypeDef NVIC_InitStructure;
          /* Configure the Priority Group to 2 bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //������������� ���������� ����� � �������� ����������
          /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //���������� �� Uart1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //������ ��������� � ������
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //������ ��������� � ���������
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //��������� ����������
    NVIC_Init(&NVIC_InitStructure); //��������������
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

//    USART1->CR3 |= USART_CR3_CTSE;//�������� ������� CTS
//    USART1->CR3 |= USART_CR3_RTSE;//�������� ������� RTS �� ��������� �����

    GPIOC->MODER |= (GPIO_MODER_MODER4_1);//alternative mode
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4);//High speed
    GPIO_PinAFConfig(GPIOC,4,7);

    USART1->BRR= USART1_BRR_REG; //BaudRate  ���� ���� ��� ���� ����������� ���������
    USART1->CR1 |= USART_CR1_UE; //��������� ������ USART1
    USART1->CR1 |= USART_CR1_TE; //�������� ����������
    }

void Uart1_write(uint8_t data) //����� ������
{
    while (tx_counter_Uart1 == TX_BUFFER_SIZE_USART1); //���� ����� ����������, ����
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE); //��������� ����������, ����� ��� �� ������ ������ ����������
    if (tx_counter_Uart1 || (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)) //���� � ������ ��� ���-�� ���� ��� ���� � ������ ������ ���-�� ��� ����������
	{
	tx_buffer_Uart1[tx_wr_index_Uart1++] = data; //�� ������ ������ � �����
	if (tx_wr_index_Uart1 == TX_BUFFER_SIZE_USART1) tx_wr_index_Uart1 = 0; //���� �� �����
	++tx_counter_Uart1; //����������� ������� ���������� ������ � ������
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //��������� ����������
	}
    else //���� UART ��������
	{
	USART_SendData(USART1,data); //�������� ������ ��� ����������
	}
}

void Uart1_string(char *s)
{
  while (*s != 0)
      Uart1_write(*s++);
}

void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_TXE) == SET) //���������� �� ��������
  {
  	if (tx_counter_Uart1)//���� ���� ��� ��������
        {
        --tx_counter_Uart1;// ��������� ���������� �� ���������� ������
        USART_SendData(USART1,tx_buffer_Uart1[tx_rd_index_Uart1++]);//�������� ������ ������������� ����� ������
        if (tx_rd_index_Uart1 == TX_BUFFER_SIZE_USART1) tx_rd_index_Uart1 = 0;//���� �� �����
        }
        else//���� ������ ��������, ��������� ���������� �� ��������
        {
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
  }
}


char * utoa_builtin_div(u32 value, char *buffer)
{
   buffer += 11;
// 11 ���� ���������� ��� ����������� ������������� 32-� �������� �����
// �  ������������ ����
   *--buffer = 0;
   do
   {
      *--buffer = value % 10 + '0';
      value /= 10;
   }
   while (value != 0);
   return buffer;
}

#ifdef NUMBER_FLOAT
//�������� �������, ����� �� � .h
static u32 Uart_pow10(u32 n);
#endif


//*****************************************************************************
//����� �������������� �����
#ifdef NUMBER_INT
void Uart_IntWrite(s32 value)
{
	s32 i;

	if (value < 0)
	{
	    Uart1_write('-');
		value = -value;
	}

	i = 1;
	while ((value / i) > 9)
	{
		i *= 10;
	}

	Uart1_write(value/i + '0');	/* Display at least one symbol */
	i /= 10;

	while (i > 0)
	{
		Uart1_write('0' + ((value % (i*10)) / i));
		i /= 10;
	}
}

void Uart2_IntWrite(s32 value)
{
	s32 i;

	if (value < 0)
	{
	    Uart2_write('-');
		value = -value;
	}

	i = 1;
	while ((value / i) > 9)
	{
		i *= 10;
	}

	Uart2_write(value/i + '0');	/* Display at least one symbol */
	i /= 10;

	while (i > 0)
	{
		Uart2_write('0' + ((value % (i*10)) / i));
		i /= 10;
	}
}
#endif
//*****************************************************************************



//*****************************************************************************
//����� �������������� �����
#ifdef NUMBER_UINT
void Uart_UintWrite(u32 value)
{
	u32 i;
	i = 1;
	while ((value / i) > 9)
	{
		i *= 10;
	}

	Uart1_write(value/i + '0');	/* Display at least one symbol */
	i /= 10;

	while (i > 0)
	{
		Uart1_write('0' + ((value % (i*10)) / i));
		i /= 10;
	}
}

void Uart2_UintWrite(u32 value)
{
	u32 i;
	i = 1;
	while ((value / i) > 9)
	{
		i *= 10;
	}

	Uart2_write(value/i + '0');	/* Display at least one symbol */
	i /= 10;

	while (i > 0)
	{
		Uart2_write('0' + ((value % (i*10)) / i));
		i /= 10;
	}
}
#endif
//*****************************************************************************


//*****************************************************************************
//����� ����� � ������
#ifdef NUMBER_FLOAT


//	\brief	Display "n" right digits of "value".
void Uart_ntos(u32 value, u32 n)
{
    if (n > 0u)
	{
	u32 i = Uart_pow10(n - 1u);
	while (i > 0u)	// Display at least one symbol
	    {
	    Uart1_write('0' + ((value/i) % 10u));
	    i /= 10u;
	    }
	}
}

void Uart_FloatWrite(float value, u32 n)
{
	if (value < 0.0)
	{
		Uart1_write('-');
		value = -value;
	}
	Uart_IntWrite((s32)value); // ����� ����� �����

	if (n > 0u)
	{
		Uart1_write('.'); // �����
		Uart_ntos((u32)(value * (float)Uart_pow10(n)), n); // ����� ������� �����
	}
}

//	\brief	Returns 10^n value.
static u32 Uart_pow10(u32 n)
{
u32 retval = 1u;
while (n > 0u)
    {
    retval *= 10u;
    n--;
    }
return retval;
}
#endif
