#ifndef EERTOS_H
#define EERTOS_H


#include "stm32f30x.h"
#include "stm32f30x_rcc.h"

#define BitIsSet(reg, bit)	((reg & (1<<(bit))) != 0)
#define BitIsClear(reg, bit)	((reg & (1<<(bit))) == 0)

//������� �����
#define F_CPU 		72000000UL
//������� ������ ������� �����������
#define F_RTOS		1000
#define TimerTick	F_CPU/F_RTOS-1


//���������� �����, ����� ��������� ���� ����������
#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

volatile u32 __count_tic;

//������� �� ������������� �������
#define DWT_start() SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;DWT_CYCCNT = 0;DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; // �������� �������
//������� ����� ������������� ������� � �������� ����� ����� ������� ��� ��������� ����������
#define DWT_finish() __count_tic = DWT_CYCCNT //��� ���� ����� �������� ~ 10 ������


//������������ ���������� �����
#define	TaskQueueSize		12

//������������ ���������� ��������
#define MainTimerQueueSize	15



extern void RunRTOS (void);

extern void InitRTOS(void);
extern void Idle(void);

typedef void (*TPTR)(void);

extern void SetTask(TPTR TS);
extern void SetTimerTask(TPTR TS, u32 NewTime);
extern void StopTask(TPTR TS);

extern void TaskManager(void);
extern void TimerService(void);
//void delay_ms (u32 time);


#endif
