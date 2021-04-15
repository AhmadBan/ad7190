/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "..\inc\MOhmMeterEvent.h"
#include "..\inc\MOhmMeterState.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
MOhmMeter_t me;
osMailQId qid_EventQueue; // mail queue id
osMailQDef(EventQueue, MAILQUEUE_EVENTS, EVENT_TYPE);


/* USER CODE END Variables */
osThreadId eventDispatcherHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void startEventDispatcherTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  	  qid_EventQueue = osMailCreate(osMailQ(EventQueue), NULL); // create mail queue
  	if (!qid_EventQueue)
  	{
  		while (1)
  		{

  		} // Mail Queue object not created, handle failure
  	}

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of eventDispatcher */
  osThreadDef(eventDispatcher, startEventDispatcherTask, osPriorityNormal, 0, 128);
  eventDispatcherHandle = osThreadCreate(osThread(eventDispatcher), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_startEventDispatcherTask */
/**
 * @brief  Function implementing the eventDispatcher thread.
 * @param  argument: Not used
 * @retval None
 */
osStatus status;
/* USER CODE END Header_startEventDispatcherTask */
void startEventDispatcherTask(void const * argument)
{
  /* USER CODE BEGIN startEventDispatcherTask */
	/* Infinite loop */
	Event_t *rptr,*mptr = 0;

	osEvent evt;
	mOhmMeter_costructor(&me);
	mptr = osMailAlloc(qid_EventQueue,osWaitForever);                     // Allocate memory for the message
	mptr->sig =1;
	status=osMailPut(qid_EventQueue, mptr);
	mptr = osMailAlloc(qid_EventQueue,osWaitForever);                     // Allocate memory for the message
	mptr->sig =2;
	status=osMailPut(qid_EventQueue, mptr);

	Base_init((StateContext_t *)&me, (Event_t const *)0);

	while (1)
	{

		evt = osMailGet(qid_EventQueue, osWaitForever); // wait for mail
		if (evt.status == osEventMail)
		{
			rptr = evt.value.p;
			if (rptr)
			{
				Base_dispatch((StateContext_t *)&me, (Event_t const *)rptr); // process data
				osMailFree(qid_EventQueue, rptr);
			}
		}
	}
  /* USER CODE END startEventDispatcherTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
