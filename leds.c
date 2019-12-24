#include "leds.h"

void Init_pins(void)
    {
    RCC->AHBENR |= (RCC_AHBENR_GPIOEEN);
    //светодиоды на плате
    GPIOE->MODER |= (GPIO_MODER_MODER8_0|
		     GPIO_MODER_MODER9_0|
		    GPIO_MODER_MODER10_0|
		    GPIO_MODER_MODER11_0|
		    GPIO_MODER_MODER12_0|
		    GPIO_MODER_MODER13_0|
		    GPIO_MODER_MODER14_0|
		    GPIO_MODER_MODER15_0);//out mode
    }

void Leds(void)
    {
    static u32 i = 0;
    SetTimerTask(Leds,50);
    i++;
    switch (i)
	{
	case 1:
	    GPIOE->BSRR = GPIO_BSRR_BR_15;
	    GPIOE->BSRR = GPIO_BSRR_BS_8;
//	    i++;
	    break;
	case 2:
	    GPIOE->BSRR = GPIO_BSRR_BR_8;
	    GPIOE->BSRR = GPIO_BSRR_BS_9;
//	    i++;
	    break;
	case 3:
	    GPIOE->BSRR = GPIO_BSRR_BR_9;
	    GPIOE->BSRR = GPIO_BSRR_BS_10;
//	    i++;
	    break;
	case 4:
	    GPIOE->BSRR = GPIO_BSRR_BR_10;
	    GPIOE->BSRR = GPIO_BSRR_BS_11;
//	    i++;
	    break;
	case 5:
	    GPIOE->BSRR = GPIO_BSRR_BR_11;
	    GPIOE->BSRR = GPIO_BSRR_BS_12;
//	    i++;
	    break;
	case 6:
	    GPIOE->BSRR = GPIO_BSRR_BR_12;
	    GPIOE->BSRR = GPIO_BSRR_BS_13;
//	    i++;
	    break;
	case 7:
	    GPIOE->BSRR = GPIO_BSRR_BR_13;
	    GPIOE->BSRR = GPIO_BSRR_BS_14;
//	    i++;
	    break;
	case 8:
	    GPIOE->BSRR = GPIO_BSRR_BR_14;
	    GPIOE->BSRR = GPIO_BSRR_BS_15;
	    i = 0;
	    break;
	}
    }
