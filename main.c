#include "EERTOS_f3.h"
#include "i2c_f3.h"
#include "mpu6050.h"
#include "leds.h"
#include "uart.h"
#include "spi_f3.h"
#include "servo.h"
#include "parsing.h"
#include "robo.h"

//SysTick Interrupt
void SysTick_Handler(void)
    {TimerService();}


extern volatile u32 error_flag;
void Hello(void)
    {
    if (BitIsSet(error_flag,31))
	{
	Uart2_string("HELLO");
	Uart2_write('\r');
	Uart2_write('\n');
	SetTimerTask(Hello,1000);
	}
    else
	{
	SetTask(Servo_goto);
	}
    }

int main(void)
{
    InitRTOS();
    RunRTOS();
    i2c_init();
    //spi_init();
    Init_pins();
    servo_init();
    Uart1_init();
    Uart2_init();
    SetTask(Leds);
    SetTask(Hello);
    MPU_init();
    while(1)
	{
	TaskManager();// Âûחמג הטסןועקונא
	}
    }
