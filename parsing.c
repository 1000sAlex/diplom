#include "parsing.h"
#include "stdlib.h"
#include "robo.h"


#define MAX_VALS_FROM_UART 10
volatile u32 vals_form_uart[MAX_VALS_FROM_UART];

void Uart_pars(void)
    {
    char local_buf[32];
    char *second_val_pos[MAX_VALS_FROM_UART - 1];
    u8 n_val = 0;
    u8 i = 0;
    while(Uart2_available() != 0)
	{
	local_buf[i] = Uart2_get_char();
	if (((local_buf[i] < '0')||(local_buf[i] > '9'))&&(local_buf[i] != '-'))
	    {
	    second_val_pos[n_val] = &local_buf[i];//находим разделитель
	    n_val++;
	    }
	i++;
	}
    vals_form_uart[0] = atoi(local_buf);//находим первое число
    for(u8 i = 1;i<n_val;i++)
	{
	vals_form_uart[i] = atoi(second_val_pos[i-1]);//находим второе число
	}
    Uart2_string("Angle1 = ");
    Uart2_IntWrite(vals_form_uart[0]);
    Uart2_string(" Angle2 = ");
    Uart2_IntWrite(vals_form_uart[1]);
    Uart2_string(" MAX_speed = ");
    Uart2_IntWrite(vals_form_uart[2]);
    Uart2_string(" deg/sec");
    Uart2_write('\r');
    Uart2_write('\n');
    SetTask(Servo_work);
    }
