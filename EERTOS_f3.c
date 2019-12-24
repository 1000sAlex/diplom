#include <EERTOS_f3.h>


// Очереди задач, таймеров.
// Тип данных - указатель на функцию
volatile static TPTR	TaskQueue[TaskQueueSize+1];// очередь указателей
volatile static struct 	
	{
	TPTR GoToTask; 	// Указатель перехода
	u32 Time;	// Выдержка в мс
	}
	MainTimer[MainTimerQueueSize+1];// Очередь таймеров



//RTOS Запуск системного таймера
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

// RTOS Подготовка. Очистка очередей
inline void InitRTOS(void)
{
u32 index;
for(index=0;index!=TaskQueueSize+1;index++)	// Во все позиции записываем Idle
	{
	TaskQueue[index] = Idle;
	}

for(index=0;index!=MainTimerQueueSize+1;index++) // Обнуляем все таймеры.
	{
	MainTimer[index].GoToTask = Idle;
	MainTimer[index].Time = 0;
	}
}


//Пустая процедура - простой ядра. 
inline void  Idle(void)
{
}



// Функция установки задачи в очередь. Передаваемый параметр - указатель на функцию
// Отдаваемое значение - код ошибки.
void SetTask(TPTR TS)
{
u32 index = 0;

__disable_irq();
while(TaskQueue[index] != Idle)// Прочесываем очередь задач на предмет свободной ячейки
	{		     // с значением Idle - конец очереди.
	index++;
	if (index == TaskQueueSize+1)// Если очередь переполнена то выходим не солоно хлебавши
		{
		__enable_irq();	// Если мы не в прерывании, то разрешаем прерывания
		return;		        // Раньше функция возвращала код ошибки - очередь переполнена. Пока убрал.
		}
	}
												// Если нашли свободное место, то
TaskQueue[index] = TS;	// Записываем в очередь задачу
__enable_irq(); // И включаем прерывания если не в обработчике прерывания.
}



// Функция остановки задачи. Передаваемый параметр - указатель на функцию
// Отдаваемое значение - код ошибки.
void StopTask(TPTR TS)
{
u32 index = 0;

__disable_irq();
for(index = 0;index < TaskQueueSize+1;index++)// Прочесываем очередь задач
	{		     // с значением Idle - конец очереди.
	if (TaskQueue[index] == TS)// Если нашли задачу, то ставим на ее место Idle
		{
		for(u32 i = index;i!=TaskQueueSize;i++)// Сдвигаем всю очередь
			{
			TaskQueue[i]=TaskQueue[i+1];
			}
		TaskQueue[TaskQueueSize]= Idle; // В последнюю запись пихаем затычку
		index = TaskQueueSize+1;
		}
	}												// Если нашли свободное место, то
// Удаляем задачу из очереди таймеров
for(index = 0;index != MainTimerQueueSize+1;++index)//Прочесываем очередь таймеров
	{
	if(MainTimer[index].GoToTask == TS)	// Если уже есть запись с таким адресом
		{
		for(u32 i = index;i!=TaskQueueSize;i++)// Сдвигаем всю очередь
		    {
		    MainTimer[i].GoToTask = MainTimer[i+1].GoToTask;// Заполняем поле перехода задачи
		    MainTimer[i].Time = MainTimer[i+1].Time;// И поле выдержки времени
		    }
		__enable_irq(); // Разрешаем прерывания если не были запрещены.
		return;	// Выходим. Раньше был код успешной операции. Пока убрал
		}
	}
__enable_irq(); // И включаем прерывания если не в обработчике прерывания.
}




//Функция установки задачи по таймеру. Передаваемые параметры - указатель на функцию, 
// Время выдержки в тиках системного таймера. Возвращет код ошибки.
void SetTimerTask(TPTR TS, u32 NewTime)
{
u32 index = 0;

__disable_irq();
for(index=0;index!=MainTimerQueueSize+1;++index)//Прочесываем очередь таймеров
	{
	if(MainTimer[index].GoToTask == TS)	// Если уже есть запись с таким адресом
		{
		MainTimer[index].Time = NewTime;// Перезаписываем ей выдержку
		__enable_irq(); // Разрешаем прерывания если не были запрещены.
		return;										// Выходим. Раньше был код успешной операции. Пока убрал
		}
	}
	
for(index=0;index!=MainTimerQueueSize+1;++index)// Если не находим похожий таймер, то ищем любой пустой
	{
	if (MainTimer[index].GoToTask == Idle)		
		{
		MainTimer[index].GoToTask = TS;	// Заполняем поле перехода задачи
		MainTimer[index].Time = NewTime;// И поле выдержки времени
		__enable_irq(); // Разрешаем прерывания
		return;	// Выход.
		}
		
	}												// тут можно сделать return c кодом ошибки - нет свободных таймеров
}


/*=================================================================================
Диспетчер задач ОС. Выбирает из очереди задачи и отправляет на выполнение.
*/
inline void TaskManager(void)
{
u32 index=0;
TPTR GoToTask = Idle;	// Инициализируем переменные

__disable_irq(); 	// Запрещаем прерывания!!!
GoToTask = TaskQueue[0];// Хватаем первое значение из очереди

if (GoToTask==Idle) 	// Если там пусто
	{
	__enable_irq();// Разрешаем прерывания
	(Idle)(); 	// Переходим на обработку пустого цикла
	}
else
	{
	for(index=0;index!=TaskQueueSize;index++)// В противном случае сдвигаем всю очередь
		{
		TaskQueue[index]=TaskQueue[index+1];
		}

	TaskQueue[TaskQueueSize]= Idle; // В последнюю запись пихаем затычку

	__enable_irq();		// Разрешаем прерывания
	(GoToTask)();			// Переходим к задаче
	}
}


/*
Служба таймеров ядра. Должна вызываться из прерывания раз в 1мс. Хотя время можно варьировать в зависимости от задачи
*/
inline void TimerService(void)
{
u32 index;

for(index=0;index!=MainTimerQueueSize+1;index++)// Прочесываем очередь таймеров
	{
	if(MainTimer[index].GoToTask == Idle) continue;	// Если нашли пустышку - щелкаем следующую итерацию
	if(MainTimer[index].Time !=0)			// Если таймер не выщелкал, то щелкаем еще раз.
		{					// To Do: Вычислить по тактам, что лучше !=1 или !=0.
		MainTimer[index].Time--;	        // Уменьшаем число в ячейке если не конец.
		}
	else
		{
		SetTask(MainTimer[index].GoToTask);	// Дощелкали до нуля? Пихаем в очередь задачу
		MainTimer[index].GoToTask = Idle;	// А в ячейку пишем затычку
		}
	}
}




