#include <EERTOS_f3.h>


// ������� �����, ��������.
// ��� ������ - ��������� �� �������
volatile static TPTR	TaskQueue[TaskQueueSize+1];// ������� ����������
volatile static struct 	
	{
	TPTR GoToTask; 	// ��������� ��������
	u32 Time;	// �������� � ��
	}
	MainTimer[MainTimerQueueSize+1];// ������� ��������



//RTOS ������ ���������� �������
inline void RunRTOS (void)
{
SysTick->LOAD = TimerTick;
SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk);
__enable_irq();
}


/*inline void delay_ms (u32 time)
    {
    for (volatile u32 i =0;i<time;i++)
	{
	for (volatile u32 j = 0;j < 7200;j++);
	}
    }*/

// RTOS ����������. ������� ��������
inline void InitRTOS(void)
{
u32 index;
for(index=0;index!=TaskQueueSize+1;index++)	// �� ��� ������� ���������� Idle
	{
	TaskQueue[index] = Idle;
	}

for(index=0;index!=MainTimerQueueSize+1;index++) // �������� ��� �������.
	{
	MainTimer[index].GoToTask = Idle;
	MainTimer[index].Time = 0;
	}
}


//������ ��������� - ������� ����. 
inline void  Idle(void)
{
}



// ������� ��������� ������ � �������. ������������ �������� - ��������� �� �������
// ���������� �������� - ��� ������.
void SetTask(TPTR TS)
{
u32 index = 0;

__disable_irq();
while(TaskQueue[index] != Idle)// ����������� ������� ����� �� ������� ��������� ������
	{		     // � ��������� Idle - ����� �������.
	index++;
	if (index == TaskQueueSize+1)// ���� ������� ����������� �� ������� �� ������ ��������
		{
		__enable_irq();	// ���� �� �� � ����������, �� ��������� ����������
		return;		        // ������ ������� ���������� ��� ������ - ������� �����������. ���� �����.
		}
	}
												// ���� ����� ��������� �����, ��
TaskQueue[index] = TS;	// ���������� � ������� ������
__enable_irq(); // � �������� ���������� ���� �� � ����������� ����������.
}



// ������� ��������� ������. ������������ �������� - ��������� �� �������
// ���������� �������� - ��� ������.
void StopTask(TPTR TS)
{
u32 index = 0;

__disable_irq();
for(index = 0;index < TaskQueueSize+1;index++)// ����������� ������� �����
	{		     // � ��������� Idle - ����� �������.
	if (TaskQueue[index] == TS)// ���� ����� ������, �� ������ �� �� ����� Idle
		{
		for(u32 i = index;i!=TaskQueueSize;i++)// �������� ��� �������
			{
			TaskQueue[i]=TaskQueue[i+1];
			}
		TaskQueue[TaskQueueSize]= Idle; // � ��������� ������ ������ �������
		index = TaskQueueSize+1;
		}
	}												// ���� ����� ��������� �����, ��
// ������� ������ �� ������� ��������
for(index = 0;index != MainTimerQueueSize+1;++index)//����������� ������� ��������
	{
	if(MainTimer[index].GoToTask == TS)	// ���� ��� ���� ������ � ����� �������
		{
		for(u32 i = index;i!=TaskQueueSize;i++)// �������� ��� �������
		    {
		    MainTimer[i].GoToTask = MainTimer[i+1].GoToTask;// ��������� ���� �������� ������
		    MainTimer[i].Time = MainTimer[i+1].Time;// � ���� �������� �������
		    }
		__enable_irq(); // ��������� ���������� ���� �� ���� ���������.
		return;	// �������. ������ ��� ��� �������� ��������. ���� �����
		}
	}
__enable_irq(); // � �������� ���������� ���� �� � ����������� ����������.
}




//������� ��������� ������ �� �������. ������������ ��������� - ��������� �� �������, 
// ����� �������� � ����� ���������� �������. ��������� ��� ������.
void SetTimerTask(TPTR TS, u32 NewTime)
{
u32 index = 0;

__disable_irq();
for(index=0;index!=MainTimerQueueSize+1;++index)//����������� ������� ��������
	{
	if(MainTimer[index].GoToTask == TS)	// ���� ��� ���� ������ � ����� �������
		{
		MainTimer[index].Time = NewTime;// �������������� �� ��������
		__enable_irq(); // ��������� ���������� ���� �� ���� ���������.
		return;										// �������. ������ ��� ��� �������� ��������. ���� �����
		}
	}
	
for(index=0;index!=MainTimerQueueSize+1;++index)// ���� �� ������� ������� ������, �� ���� ����� ������
	{
	if (MainTimer[index].GoToTask == Idle)		
		{
		MainTimer[index].GoToTask = TS;	// ��������� ���� �������� ������
		MainTimer[index].Time = NewTime;// � ���� �������� �������
		__enable_irq(); // ��������� ����������
		return;	// �����.
		}
		
	}												// ��� ����� ������� return c ����� ������ - ��� ��������� ��������
}


/*=================================================================================
��������� ����� ��. �������� �� ������� ������ � ���������� �� ����������.
*/
inline void TaskManager(void)
{
u32 index=0;
TPTR GoToTask = Idle;	// �������������� ����������

__disable_irq(); 	// ��������� ����������!!!
GoToTask = TaskQueue[0];// ������� ������ �������� �� �������

if (GoToTask==Idle) 	// ���� ��� �����
	{
	__enable_irq();// ��������� ����������
	(Idle)(); 	// ��������� �� ��������� ������� �����
	}
else
	{
	for(index=0;index!=TaskQueueSize;index++)// � ��������� ������ �������� ��� �������
		{
		TaskQueue[index]=TaskQueue[index+1];
		}

	TaskQueue[TaskQueueSize]= Idle; // � ��������� ������ ������ �������

	__enable_irq();		// ��������� ����������
	(GoToTask)();			// ��������� � ������
	}
}


/*
������ �������� ����. ������ ���������� �� ���������� ��� � 1��. ���� ����� ����� ����������� � ����������� �� ������
*/
inline void TimerService(void)
{
u32 index;

for(index=0;index!=MainTimerQueueSize+1;index++)// ����������� ������� ��������
	{
	if(MainTimer[index].GoToTask == Idle) continue;	// ���� ����� �������� - ������� ��������� ��������
	if(MainTimer[index].Time !=0)			// ���� ������ �� ��������, �� ������� ��� ���.
		{					// To Do: ��������� �� ������, ��� ����� !=1 ��� !=0.
		MainTimer[index].Time--;	        // ��������� ����� � ������ ���� �� �����.
		}
	else
		{
		SetTask(MainTimer[index].GoToTask);	// ��������� �� ����? ������ � ������� ������
		MainTimer[index].GoToTask = Idle;	// � � ������ ����� �������
		}
	}
}




