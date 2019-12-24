#include "uart.h"
#include "parsing.h"

// Буфер на передачу USART1
#define TX_BUFFER_SIZE_USART1 64 //размер буфера
volatile uint8_t   tx_buffer_Uart1[TX_BUFFER_SIZE_USART1];
volatile uint16_t  tx_wr_index_Uart1 = 0, //индекс хвоста буфера (куда писать данные)
                   tx_rd_index_Uart1 = 0, //индекс начала буфера (откуда читать данные)
                   tx_counter_Uart1 = 0; //количество данных в буфере


// Буфер на прием usart2
#define RX_BUFFER_SIZE_USART2 64 // размер буфера
volatile uint8_t    rx_buffer_uart2[RX_BUFFER_SIZE_USART2];
volatile uint16_t   rx_wr_index_uart2 = 0, //индекс хвоста буфера (куда писать данные)
                    rx_rd_index_uart2 = 0, //индекс начала буфера (откуда читать данные)
                    rx_counter_uart2 = 0; //количество данных в буфере
volatile uint8_t    rx_buffer_overflow_uart2 = 0; //информация о переполнении буфера
// Буфер на передачу usart2
#define TX_BUFFER_SIZE_USART2 164 //размер буфера
volatile uint8_t   tx_buffer_uart2[TX_BUFFER_SIZE_USART2];
volatile uint16_t  tx_wr_index_uart2 = 0, //индекс хвоста буфера (куда писать данные)
                   tx_rd_index_uart2 = 0, //индекс начала буфера (откуда читать данные)
                   tx_counter_uart2 = 0; //количество данных в буфере



void buff_clean(char *s)
    {
    while (*s != 0)
        *s++ = 0;
    }


void Uart2_init(void)//на блютус
    {
    RCC->AHBENR |= (RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN);
    RCC->APB1ENR |= (RCC_APB1Periph_USART2);

    NVIC_InitTypeDef NVIC_InitStructure;
          /* Configure the Priority Group to 2 bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //конфигурируем количество групп и подгрупп прерываний
          /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //прерывание по uart2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //задаем приоритет в группе
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //задаем приоритет в подгруппе
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //разрешаем прерывание
    NVIC_Init(&NVIC_InitStructure); //инициализируем
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

//    USART2->CR3 |= USART_CR3_CTSE;//включаем хардвар CTS
//    USART2->CR3 |= USART_CR3_RTSE;//включаем хардвар RTS до включение уарта

    GPIOB->MODER |= (GPIO_MODER_MODER3_1|GPIO_MODER_MODER4_1);//alternative mode
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR3|GPIO_OSPEEDER_OSPEEDR4);//High speed
    GPIO_PinAFConfig(GPIOB,3,7);//настройки мапа уарта на эти ноги
    GPIO_PinAFConfig(GPIOB,4,7);

    USART2->BRR= USART2_BRR_REG/2; //BaudRate *2 изза того что шина тактируется медленнее
    USART2->CR1 |= USART_CR1_UE; //Разрешаем работу USART2
    USART2->CR1 |= USART_CR1_TE; //Включаем передатчик
    USART2->CR1 |= USART_CR1_RE; //Включаем приемник
    }


inline uint16_t Uart2_available(void)//количество принятых байт
    {
    return rx_counter_uart2;
    }

inline uint8_t Uart2_overflow_flag(void)//флаг переполнения
    {
    return rx_buffer_overflow_uart2;
    }

inline uint8_t Uart2_get_char(void) //прием данных
{
    uint8_t data; //переменная для данных
//    while (rx_counter_uart2==0);  //если данных нет, ждем

    data = rx_buffer_uart2[rx_rd_index_uart2++]; //берем данные из буфера
    if (rx_rd_index_uart2 == RX_BUFFER_SIZE_USART2) rx_rd_index_uart2 = 0; //идем по кругу

    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); //запрещаем прерывание
    --rx_counter_uart2; //чтобы оно не помешало изменить переменную
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//разрешаем прерывание
    return data;
}

void Uart2_write(uint8_t data) //вывод данных
{
//    while (tx_counter_uart2 == TX_BUFFER_SIZE_USART2); //если буфер переполнен, ждем
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE); //запрещаем прерывание, чтобы оно не мешало менять переменную
    if (tx_counter_uart2 || (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)) //если в буфере уже что-то есть или если в данный момент что-то уже передается
	{
	tx_buffer_uart2[tx_wr_index_uart2++] = data; //то кладем данные в буфер
	if (tx_wr_index_uart2 == TX_BUFFER_SIZE_USART2) tx_wr_index_uart2 = 0; //идем по кругу
	++tx_counter_uart2; //увеличиваем счетчик количества данных в буфере
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //разрешаем прерывание
	}
    else //если UART свободен
	{
	USART_SendData(USART2,data); //передаем данные без прерывания
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
  if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET) //прерывание по приему данных
  {
      if ((USART2->ISR & (USART_FLAG_NE|USART_FLAG_FE|USART_FLAG_PE|USART_FLAG_ORE)) == 0)//проверяем нет ли ошибок
	  {
	  temp_data = USART_ReceiveData(USART2);//считываем данные
	  rx_buffer_uart2[rx_wr_index_uart2++] = (uint8_t)(temp_data);//считываем данные в буфер, инкрементируя хвост буфера

          if (rx_wr_index_uart2 == RX_BUFFER_SIZE_USART2) rx_wr_index_uart2 = 0;//идем по кругу
          if (++rx_counter_uart2 == RX_BUFFER_SIZE_USART2)//переполнение буфера
          	{
                rx_counter_uart2 = 0;//начинаем сначала (удаляем все данные)
                rx_buffer_overflow_uart2 = 1;//сообщаем о переполнении
                }
          if (temp_data == ';')//конец
              {
              SetTimerTask(Uart_pars,5);
              }
          }
	  else USART_ReceiveData(USART2);//вообще здесь нужен обработчик ошибок, а мы просто пропускаем битый байт
  }

  if(USART_GetITStatus(USART2, USART_IT_ORE) == SET) //прерывание по переполнению буфера
  {
      USART_ReceiveData(USART2); //в идеале пишем здесь обработчик переполнения буфера, но мы просто сбрасываем этот флаг прерывания чтением из регистра данных.
  }

  if(USART_GetITStatus(USART2, USART_IT_TXE) == SET) //прерывание по передачи
  {
  	if (tx_counter_uart2)//если есть что передать
        {
        --tx_counter_uart2;// уменьшаем количество не переданных данных
        USART_SendData(USART2,tx_buffer_uart2[tx_rd_index_uart2++]);//передаем данные инкрементируя хвост буфера
        if (tx_rd_index_uart2 == TX_BUFFER_SIZE_USART2) tx_rd_index_uart2 = 0;//идем по кругу
        }
        else//если нечего передать, запрещаем прерывание по передачи
        {
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
  }
}


void Uart1_init(void)//на блютус
    {
    RCC->AHBENR |= (RCC_AHBENR_GPIOCEN);
    RCC->APB2ENR |= (RCC_APB2Periph_USART1);

    NVIC_InitTypeDef NVIC_InitStructure;
          /* Configure the Priority Group to 2 bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //конфигурируем количество групп и подгрупп прерываний
          /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //прерывание по Uart1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //задаем приоритет в группе
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //задаем приоритет в подгруппе
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //разрешаем прерывание
    NVIC_Init(&NVIC_InitStructure); //инициализируем
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

//    USART1->CR3 |= USART_CR3_CTSE;//включаем хардвар CTS
//    USART1->CR3 |= USART_CR3_RTSE;//включаем хардвар RTS до включение уарта

    GPIOC->MODER |= (GPIO_MODER_MODER4_1);//alternative mode
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4);//High speed
    GPIO_PinAFConfig(GPIOC,4,7);

    USART1->BRR= USART1_BRR_REG; //BaudRate  изза того что шина тактируется медленнее
    USART1->CR1 |= USART_CR1_UE; //Разрешаем работу USART1
    USART1->CR1 |= USART_CR1_TE; //Включаем передатчик
    }

void Uart1_write(uint8_t data) //вывод данных
{
    while (tx_counter_Uart1 == TX_BUFFER_SIZE_USART1); //если буфер переполнен, ждем
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE); //запрещаем прерывание, чтобы оно не мешало менять переменную
    if (tx_counter_Uart1 || (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)) //если в буфере уже что-то есть или если в данный момент что-то уже передается
	{
	tx_buffer_Uart1[tx_wr_index_Uart1++] = data; //то кладем данные в буфер
	if (tx_wr_index_Uart1 == TX_BUFFER_SIZE_USART1) tx_wr_index_Uart1 = 0; //идем по кругу
	++tx_counter_Uart1; //увеличиваем счетчик количества данных в буфере
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //разрешаем прерывание
	}
    else //если UART свободен
	{
	USART_SendData(USART1,data); //передаем данные без прерывания
	}
}

void Uart1_string(char *s)
{
  while (*s != 0)
      Uart1_write(*s++);
}

void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_TXE) == SET) //прерывание по передачи
  {
  	if (tx_counter_Uart1)//если есть что передать
        {
        --tx_counter_Uart1;// уменьшаем количество не переданных данных
        USART_SendData(USART1,tx_buffer_Uart1[tx_rd_index_Uart1++]);//передаем данные инкрементируя хвост буфера
        if (tx_rd_index_Uart1 == TX_BUFFER_SIZE_USART1) tx_rd_index_Uart1 = 0;//идем по кругу
        }
        else//если нечего передать, запрещаем прерывание по передачи
        {
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
  }
}


char * utoa_builtin_div(u32 value, char *buffer)
{
   buffer += 11;
// 11 байт достаточно для десятичного представления 32-х байтного числа
// и  завершающего нуля
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
//описание функции, чтобы не в .h
static u32 Uart_pow10(u32 n);
#endif


//*****************************************************************************
//вывод целочисленного числа
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
//вывод целочисленного числа
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
//вывод числа с точкой
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
	Uart_IntWrite((s32)value); // Вывод целой части

	if (n > 0u)
	{
		Uart1_write('.'); // Точка
		Uart_ntos((u32)(value * (float)Uart_pow10(n)), n); // Вывод дробной части
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
