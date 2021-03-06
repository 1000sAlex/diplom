#include "EERTOS_f3.h"
#include "stm32f30x_gpio.h"
//микросекунды
#define SERVO_US_180 2200
#define SERVO_US_0 500

#define SERVO_RESOLUTION_0_TO_180 1800


#define SERVO_FREQUENCY 50

#define SERVO_RESOLUTION 65535
#define SERVO_PRESCALER (F_CPU/(SERVO_RESOLUTION*SERVO_FREQUENCY))

#define SERVO_OFSET (SERVO_US_0*(F_CPU/1000000))/SERVO_PRESCALER

#define SERVO_MIN SERVO_OFSET
#define SERVO_MAX ((SERVO_US_180*(F_CPU/1000000))/SERVO_PRESCALER)
#define SERVO_VAL (SERVO_MAX - SERVO_MIN) / SERVO_RESOLUTION_0_TO_180
void servo_init(void);
void servo_pos1(u32 pos);
void servo_pos2(u32 pos);
