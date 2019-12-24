#include "servo.h"


void servo_init(void)
    {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->AHBENR |=  RCC_AHBENR_GPIOAEN;

    GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);//alternative mode
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1);//High speed
    GPIO_PinAFConfig(GPIOA,0,1);
    GPIO_PinAFConfig(GPIOA,1,1);

    TIM2->PSC =  SERVO_PRESCALER;//делитель
    TIM2->ARR =  SERVO_RESOLUTION;//значение перезагрузки
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;//настроим на выход канал 1
    TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;//прямой шим
    TIM2->CR1 &=~ TIM_CR1_CMS;//выравнивание по фронту, Fast PWM
    TIM2->CR1 |=  TIM_CR1_CEN;//включаем счётчик
    } 


//PA0 - программный инверсный шим
void servo_pos1(u32 dat)
    {
    if(dat > SERVO_RESOLUTION_0_TO_180){dat = SERVO_RESOLUTION_0_TO_180;}//проверка на выход за границы
    TIM2->CCR1 = SERVO_OFSET + SERVO_VAL * dat;
    }

//PA1 - программный инверсный шим
void servo_pos2(u32 dat)
    {
    if(dat > 1700){dat = 1700;}//проверка на выход за границы
    if(dat < 700){dat = 700;}//проверка на выход за границы
    TIM2->CCR2 = SERVO_OFSET + SERVO_VAL * dat;
    }
