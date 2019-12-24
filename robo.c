#include "robo.h"
#include "parsing.h"
#include "mpu6050.h"

extern volatile u32 vals_form_uart[];


#define ERROR_ANGLE1_TOO_HIGH 0
#define ERROR_ANGLE1_TOO_LOW 1
#define ERROR_ANGLE2_TOO_HIGH 2
#define ERROR_ANGLE2_TOO_LOW 3


volatile u32 error_flag = (1<<31);
volatile s16 angle1_real = 0;
volatile s16 angle2_real = 0;
volatile s16 angle1_last = 0;
volatile s16 angle2_last = 0;
volatile s16 angle1 = 0;
volatile s16 angle2 = 0;
void Servo_work(void)
    {
    s16 t_angle1 = vals_form_uart[0];
    s16 t_angle2 = vals_form_uart[1];
    error_flag &=~(1<<31);
    //угол 1
    if (t_angle1 > 90)//выход за физические границы
	{error_flag |= (1<<ERROR_ANGLE1_TOO_HIGH);t_angle1 = 90;}
    else
	{error_flag &=~(1<<ERROR_ANGLE1_TOO_HIGH);}

    if (t_angle1 < 0)//выход за физические границы
	{error_flag |= (1<<ERROR_ANGLE1_TOO_LOW);t_angle1 = 0;}
    else
	{error_flag &=~(1<<ERROR_ANGLE1_TOO_LOW);}

    //угол 2
    if (t_angle2 > 170)//выход за физические границы
	{error_flag |= (1<<ERROR_ANGLE2_TOO_HIGH);t_angle2 = 170;}
    else
	{error_flag &=~(1<<ERROR_ANGLE2_TOO_HIGH);}
    if (t_angle2 < 70)//выход за физические границы
	{error_flag |= (1<<ERROR_ANGLE2_TOO_LOW);t_angle2 = 70;}
    else
	{error_flag &=~(1<<ERROR_ANGLE2_TOO_LOW);}

    if (error_flag == 0)//если нет ошибок
	{
	angle1 = t_angle1*10;
	angle2 = t_angle2*10;
	//Servo_flag = 0;
	Uart2_string("New angle start");
	Uart2_string("\r\n");
	Uart2_string("\r\n");
	SetTask(Servo_goto);
	}
    else
	{
	Uart2_string("Angle erorr: ");
	if (BitIsSet(error_flag,ERROR_ANGLE1_TOO_HIGH))
	    {Uart2_string("ERROR_ANGLE_1_TOO_HIGH ");}
	if (BitIsSet(error_flag,ERROR_ANGLE1_TOO_LOW))
	    {Uart2_string("ERROR_ANGLE_1_TOO_LOW ");}
	if (BitIsSet(error_flag,ERROR_ANGLE2_TOO_HIGH))
	    {Uart2_string("ERROR_ANGLE_2_TOO_HIGH ");}
	if (BitIsSet(error_flag,ERROR_ANGLE2_TOO_LOW))
	    {Uart2_string("ERROR_ANGLE_2_TOO_LOW ");}
	Uart2_string("\r\n");
	}
    }

#define ANGLE1_READY 0
#define ANGLE2_READY 1
volatile u32 Servo_flag = 0;
#define UPD_TIME_MS 50

s16 count = 0;
void Servo_goto(void)
    {
    s16 speed = vals_form_uart[2];
    s16 angle_d = (speed)/(1000/UPD_TIME_MS);
    SetTimerTask(Servo_goto,UPD_TIME_MS);
    //угол 1
    if (angle1_last != angle1)
	{
	if ((angle1 < (angle1_real + angle_d))&&
	    (angle1 > (angle1_real - angle_d)))
	    {
	    angle1_last = angle1;
	    angle1_real = angle1;
	    Uart2_string("Angle 1 Finish");
	    Uart2_string("\r\n");
	    Servo_flag |= (1<<ANGLE1_READY);
	    count = 0;
	    }
	else
	    {
	    Servo_flag &=~(1<<ANGLE1_READY);
	    if (angle1 > angle1_last + angle_d)
		{angle1_real = angle1_real + angle_d;}
	    if (angle1 < angle1_last - angle_d)
		{angle1_real = angle1_real - angle_d;}
	    }
	}
    //угол 2
    if (angle2_last != angle2)
	{
	if ((angle2 < (angle2_real + angle_d))&&
	    (angle2 > (angle2_real - angle_d)))
	    {
	    angle2_last = angle2;
	    angle2_real = angle2;
	    Uart2_string("Angle 2 Finish");
	    Uart2_string("\r\n");
	    Servo_flag |= (1<<ANGLE2_READY);
	    count = 0;
	    }
	else
	    {
	    Servo_flag &=~(1<<ANGLE2_READY);
	    if (angle2 > angle2_last + angle_d)
		{angle2_real = angle2_real + angle_d;}
	    if (angle2 < angle2_last - angle_d)
		{angle2_real = angle2_real - angle_d;}
	    }
	}


    if ((BitIsSet(Servo_flag,ANGLE1_READY))&&
	(BitIsSet(Servo_flag,ANGLE2_READY)))
	{
	SetTask(Servo_mpu);
	}
    else
	{
	servo_pos1(angle1_real*2);
	servo_pos2(angle2_real);
	}
    }


s16 abs_my(s16 a, s16 b)
    {
    if ((a - b)>0)
	{return a - b;}
    else
	{return b - a;}
    }


extern volatile float roll;
extern volatile float pitch;
void Servo_mpu(void)
    {
    s16 sum = (900 + angle1_real) + (100 - angle2_real);
    s16 int_roll = (s16)(pitch * 10);
    Uart2_string("summ = ");
    Uart2_IntWrite(sum/10);
    Uart2_string("MPU = ");
    Uart2_IntWrite(int_roll);
    Uart2_string("\r\n");

    if (abs_my(sum,int_roll) > 30)
	{
	if (sum > int_roll)
	    {
	    count = count - 2;
	    servo_pos2(angle2_real + count);
	    servo_pos1(angle1_real*2 - count);
	    }
	else
	    {
	    count = count + 2;
	    servo_pos2(angle2_real + count);
	    servo_pos1(angle1_real*2 - count);
	    }
	}
    }






