/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
	FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
	31 architectures and receives 77500 downloads a year. It is professionally
	developed, strictly quality controlled, robust, supported, and free to use in
	commercial products without any requirement to expose your proprietary source
	code.

	This simple FreeRTOS demo does not make use of any IO ports, so will execute on
	any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
	locations that may require tailoring to, for example, include a manufacturer
	specific header file.

	This is a starter project, so only a subset of the RTOS features are
	demonstrated.  Ample source comments are provided, along with web links to
	relevant pages on the http://www.FreeRTOS.org site.

	Here is a description of the project's functionality:

	The main() Function:
	main() creates the tasks and software timers described in this section, before
	starting the scheduler.

	The Queue Send Task:
	The queue send task is implemented by the prvQueueSendTask() function.
	The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
	periodically send the number 100 on a queue.  The period is set to 200ms.  See
	the comments in the function for more details.
	http://www.freertos.org/vtaskdelayuntil.html
	http://www.freertos.org/a00117.html

	The Queue Receive Task:
	The queue receive task is implemented by the prvQueueReceiveTask() function.
	The task uses the FreeRTOS xQueueReceive() API function to receive values from
	a queue.  The values received are those sent by the queue send task.  The queue
	receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
	receives the value 100.  Therefore, as values are sent to the queue every 200ms,
	the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
	http://www.freertos.org/a00118.html

	An example software timer:
	A software timer is created with an auto reloading period of 1000ms.  The
	timer's callback function increments the ulCountOfTimerCallbackExecutions
	variable each time it is called.  Therefore the value of
	ulCountOfTimerCallbackExecutions will count seconds.
	http://www.freertos.org/RTOS-software-timer.html

	The FreeRTOS RTOS tick hook (or callback) function:
	The tick hook function executes in the context of the FreeRTOS tick interrupt.
	The function 'gives' a semaphore every 500th time it executes.  The semaphore
	is used to synchronise with the event semaphore task, which is described next.

	The event semaphore task:
	The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
	wait for the semaphore that is given by the RTOS tick hook function.  The task
	increments the ulCountOfReceivedSemaphores variable each time the semaphore is
	received.  As the semaphore is given every 500ms (assuming a tick frequency of
	1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

	The idle hook (or callback) function:
	The idle hook function queries the amount of free FreeRTOS heap space available.
	See vApplicationIdleHook().

	The malloc failed and stack overflow hook (or callback) functions:
	These two hook functions are provided as examples, but do not contain any
	functionality.
*/

/* Standard includes. */
#include <stdint.h>

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/* Priorities at which the tasks are created.  The event semaphore task is
given the maximum priority of ( configMAX_PRIORITIES - 1 ) to ensure it runs as
soon as the semaphore is given. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY	( configMAX_PRIORITIES - 1 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainQUEUE_SEND_PERIOD_MS			( 200 / portTICK_RATE_MS )

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainSOFTWARE_TIMER_PERIOD_MS		( 1000 / portTICK_RATE_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

/*-----------------------------------------------------------*/

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void initializeProgram( void );


static void taskCheckModule1( void *pvParameters );
static void taskCheckModule2( void *pvParameters );
static void taskCheckModule3( void *pvParameters );
static void taskCheckDistances( void *pvParameters );
static void processDistance( uint32_t distance, int sensor);


static xQueueHandle queueSensor1 = NULL;
static xQueueHandle queueSensor2 = NULL;
static xQueueHandle queueSensor3 = NULL;


static xSemaphoreHandle sensorMutex = NULL;
static xSemaphoreHandle sensor1semaph = NULL;
static xSemaphoreHandle sensor2semaph = NULL;
static xSemaphoreHandle sensor3semaph = NULL;



int main(void)
{
xTimerHandle xExampleSoftwareTimer = NULL;

	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	initializeProgram();


	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	queueSensor1 = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint32_t ) );	/* The size of each item the queue holds. */
	queueSensor2 = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint32_t ) );
	queueSensor3 = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint32_t ) );						
	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( queueSensor1, "queueSensor1" );
	vQueueAddToRegistry( queueSensor2, "queueSensor2" );
	vQueueAddToRegistry( queueSensor3, "queueSensor3" );


	/* Create the semaphore used by the FreeRTOS tick hook function and the
	event semaphore task. */
	vSemaphoreCreateBinary( sensorMutex );
	vSemaphoreCreateBinary( sensor1semaph );
	vSemaphoreCreateBinary( sensor2semaph );
	vSemaphoreCreateBinary( sensor3semaph );
	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( sensorMutex, "sensorMutex" );
	vQueueAddToRegistry( sensor1semaph, "sensor1semaph" );
	vQueueAddToRegistry( sensor2semaph, "sensor2semaph" );
	vQueueAddToRegistry( sensor3semaph, "sensor3semaph" );


	// /* Create the queue receive task as described in the comments at the top
	// of this	file.  http://www.freertos.org/a00125.html */
	// xTaskCreate( 	prvQueueReceiveTask,			/* The function that implements the task. */
	// 				"Rx", 		/* Text name for the task, just to help debugging. */
	// 				configMINIMAL_STACK_SIZE, 		/* The size (in words) of the stack that should be created for the task. */
	// 				NULL, 							/* A parameter that can be passed into the task.  Not used in this simple demo. */
	// 				mainQUEUE_RECEIVE_TASK_PRIORITY,/* The priority to assign to the task.  tskIDLE_PRIORITY (which is 0) is the lowest priority.  configMAX_PRIORITIES - 1 is the highest priority. */
	// 				NULL );							/* Used to obtain a handle to the created task.  Not used in this simple demo, so set to NULL. */


	/* Create the queue send task in exactly the same way.  Again, this is
	described in the comments at the top of the file. */
	xTaskCreate( 	taskCheckModule1,
					"taskCheckModule1",
					configMINIMAL_STACK_SIZE,
					NULL,
					tskIDLE_PRIORITY + 2,
					NULL );
	xTaskCreate( 	taskCheckModule2,
					"taskCheckModule2",
					configMINIMAL_STACK_SIZE,
					NULL,
					tskIDLE_PRIORITY + 2,
					NULL );
	xTaskCreate( 	taskCheckModule3,
					"taskCheckModule3",
					configMINIMAL_STACK_SIZE,
					NULL,
					tskIDLE_PRIORITY + 2,
					NULL );
	xTaskCreate( 	taskCheckDistances,
					"taskCheckDistances",
					configMINIMAL_STACK_SIZE,
					NULL,
					tskIDLE_PRIORITY + 1,
					NULL );


	/* Create the task that is synchronised with an interrupt using the
	xEventSemaphore semaphore. */


	/* Create the software timer as described in the comments at the top of
	this file.  http://www.freertos.org/FreeRTOS-timers-xTimerCreate.html. */
	

	/* Start the created timer.  A block time of zero is used as the timer
	command queue cannot possibly be full here (this is the first timer to
	be created, and it is not yet running).
	http://www.freertos.org/FreeRTOS-timers-xTimerStart.html */
	

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details.  http://www.freertos.org/a00111.html */
	for( ;; );
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

static void taskCheckModule1( void *pvParameters )
{

uint32_t sensor1value = 0;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	for( ;; )
	{
		xSemaphoreTake( sensor1semaph, portMAX_DELAY );
		xSemaphoreTake( sensorMutex, portMAX_DELAY );
		(TIM3)->CNT = 0;
		TIM_Cmd(TIM3, ENABLE);
		while(!TIM_GetFlagStatus(TIM3, TIM_FLAG_Update));
		TIM_Cmd(TIM3, DISABLE);
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
		sensor1value =  (TIM_GetCapture2(TIM3)-TIM_GetCapture1(TIM3))*165/1000;
		xQueueSend( queueSensor1, &sensor1value, 0 );
		xSemaphoreGive(sensorMutex);
	}
}

static void taskCheckModule2( void *pvParameters )
{
uint32_t sensor2value = 0;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	for( ;; )
	{
		xSemaphoreTake( sensor2semaph, portMAX_DELAY );
		xSemaphoreTake( sensorMutex, portMAX_DELAY );
		(TIM4)->CNT = 0;
		TIM_Cmd(TIM4, ENABLE);
		while(!TIM_GetFlagStatus(TIM4, TIM_FLAG_Update));
		TIM_Cmd(TIM4, DISABLE);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		sensor1value =  (TIM_GetCapture2(TIM4)-TIM_GetCapture1(TIM4))*165/1000;
		xQueueSend( queueSensor2, &sensor2value, 0 );
		xSemaphoreGive(sensorMutex);
	}
}
static void taskCheckModule3( void *pvParameters )
{
uint32_t sensor3value = 0;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	for( ;; )
	{
		xSemaphoreTake( sensor3semaph, portMAX_DELAY );
		xSemaphoreTake( sensorMutex, portMAX_DELAY );
		(TIM5)->CNT = 0;
		TIM_Cmd(TIM5, ENABLE);
		while(!TIM_GetFlagStatus(TIM5, TIM_FLAG_Update));
		TIM_Cmd(TIM5, DISABLE);
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		sensor3value =  (TIM_GetCapture2(TIM5)-TIM_GetCapture1(TIM5))*165/1000;
		xQueueSend( queueSensor3, &sensor3value, 0 );
		xSemaphoreGive(sensorMutex);
	}
}
/*-----------------------------------------------------------*/
static void processDistance( uint32_t distance, int sensor){
	uint32_t buz_pin;
	switch (sensor){
		case 1:
			buz_pin = GPIO_Pin_3;
			break;
		case 2:
			buz_pin = GPIO_Pin_4;
			break;
		case 3:
			buz_pin = GPIO_Pin_5;
			break;
		default:
			return;
	}	
	if (distance>3000){
		GPIO_SetBits(GPIOB, buz_pin);
		vTaskDelay(100/portTICK_RATE_MS);
		GPIO_ResetBits(GPIOB, buz_pin);
	}
	else if (distance>2000){
		GPIO_SetBits(GPIOB, buz_pin);
		vTaskDelay(200/portTICK_RATE_MS);
		GPIO_ResetBits(GPIOB, buz_pin);
	}
	else if (distance>1000){
		GPIO_SetBits(GPIOB, buz_pin);
		vTaskDelay(400/portTICK_RATE_MS);
		GPIO_ResetBits(GPIOB, buz_pin);
	}
	else if (distance>500){
		GPIO_SetBits(GPIOB, buz_pin);
		vTaskDelay(800/portTICK_RATE_MS);
		GPIO_ResetBits(GPIOB, buz_pin);
	}
	else if (distance>250){
		GPIO_SetBits(GPIOB, buz_pin);
		vTaskDelay(1600/portTICK_RATE_MS);
		GPIO_ResetBits(GPIOB, buz_pin);
	}
	
}

static void taskCheckDistances( void *pvParameters )
{
uint32_t S1ReceivedValue;
uint32_t S2ReceivedValue;
uint32_t S3ReceivedValue;
uint32_t totalDistance=0;
uint32_t nextDelay;
uint8_t mode = 0;
	for( ;; )
	{
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h.  http://www.freertos.org/a00118.html */
		xQueueReceive(queueSensor1, &S1ReceivedValue, portMAX_DELAY );
		processDistance(S1ReceivedValue,1);
		xQueueReceive(queueSensor2, &S2ReceivedValue, portMAX_DELAY );
		processDistance(S2ReceivedValue,2);
		xQueueReceive(queueSensor3, &S3ReceivedValue, portMAX_DELAY );
		processDistance(S3ReceivedValue,3);

		if (S1ReceivedValue>0){
			totalDistance =+ S1ReceivedValue;
		}
		if (S2ReceivedValue>0){
			totalDistance =+ S2ReceivedValue;
		}
		if (S3ReceivedValue>0){
			totalDistance =+ S3ReceivedValue;
		}

		if (totalDistance<500){
			nextDelay=5;
		}
		else if (totalDistance<1000){
			nextDelay = 50;
		}
		else if (totalDistance<2000){ 
			nextDelay = 100;
		}
		else {
			nextDelay = 500;
		}
		vTaskDelay(nextDelay/portTICK_RATE_MS);
		totalDistance = 0;

		/*  To get here something must have been received from the queue, but
		is it the expected value?  If it is, increment the counter. */
	
	}
}
/*-----------------------------------------------------------*/

static void initializeProgram( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );
    //Initialize Clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	//Initialize Pins
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_ClocksTypeDef RCC_ClocksStatus;
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	uint16_t prescaler = RCC_ClocksStatus.SYSCLK_Frequency / 1000000 - 1; //1 tick = 1us (1 tick = 0.165mm resolution)
	//TIM3
		TIM_DeInit(TIM3);
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_TimeBaseInitStruct.TIM_Prescaler = prescaler;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
		TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

		TIM_OCInitTypeDef TIM_OCInitStruct;
		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_Pulse = 15; //us
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC3Init(TIM3, &TIM_OCInitStruct);

		TIM_ICInitTypeDef TIM_ICInitStruct;
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
		TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStruct.TIM_ICFilter = 0;

		TIM_PWMIConfig(TIM3, &TIM_ICInitStruct);
		TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
		TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

		TIM_CtrlPWMOutputs(TIM3, ENABLE);

		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	//TIM4
		TIM_DeInit(TIM4);
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_TimeBaseInitStruct.TIM_Prescaler = prescaler;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
		TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

		TIM_OCInitTypeDef TIM_OCInitStruct;
		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_Pulse = 15; //us
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC3Init(TIM4, &TIM_OCInitStruct);

		TIM_ICInitTypeDef TIM_ICInitStruct;
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
		TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStruct.TIM_ICFilter = 0;

		TIM_PWMIConfig(TIM4, &TIM_ICInitStruct);
		TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
		TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);

		TIM_CtrlPWMOutputs(TIM4, ENABLE);

		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	//TIM5
		TIM_DeInit(TIM5);
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_TimeBaseInitStruct.TIM_Prescaler = prescaler;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
		TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);

		TIM_OCInitTypeDef TIM_OCInitStruct;
		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_Pulse = 15; //us
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC3Init(TIM5, &TIM_OCInitStruct);

		TIM_ICInitTypeDef TIM_ICInitStruct;
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
		TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStruct.TIM_ICFilter = 0;

		TIM_PWMIConfig(TIM5, &TIM_ICInitStruct);
		TIM_SelectInputTrigger(TIM5, TIM_TS_TI1FP1);
		TIM_SelectMasterSlaveMode(TIM5, TIM_MasterSlaveMode_Enable);

		TIM_CtrlPWMOutputs(TIM5, ENABLE);

		TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		


	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

