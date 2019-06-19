#include <stdint.h>

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



#define mainQUEUE_LENGTH					( 1 )

// DECLARACAO DAS FUNCOES A SEREM USADAS NO PROGRAMA
static void initializeProgram( void );


static void taskCheckModule1( void *pvParameters );
static void taskCheckModule2( void *pvParameters );
static void taskCheckModule3( void *pvParameters );
static void taskCheckDistances( void *pvParameters );
static void processDistance( uint32_t distance, int sensor);



// DECLARACAO DOS HANDLES DAS FILAS E SEMAFOROS
static xQueueHandle queueSensor1 = NULL;
static xQueueHandle queueSensor2 = NULL;
static xQueueHandle queueSensor3 = NULL;


static xSemaphoreHandle sensorMutex = NULL;
static xSemaphoreHandle sensor1semaph = NULL;
static xSemaphoreHandle sensor2semaph = NULL;
static xSemaphoreHandle sensor3semaph = NULL;



int main(void)
{

// INICIALIZACAO DO PROGRAMA
	initializeProgram();

//INICIALIZACAO DAS FILAS
	queueSensor1 = xQueueCreate( 	mainQUEUE_LENGTH,
							sizeof( uint32_t ) );
	queueSensor2 = xQueueCreate( 	mainQUEUE_LENGTH,
							sizeof( uint32_t ) );
	queueSensor3 = xQueueCreate( 	mainQUEUE_LENGTH,
							sizeof( uint32_t ) );

	vQueueAddToRegistry( queueSensor1, "queueSensor1" );
	vQueueAddToRegistry( queueSensor2, "queueSensor2" );
	vQueueAddToRegistry( queueSensor3, "queueSensor3" );


//INICIALIZACAO DOS SEMAFOROS
	vSemaphoreCreateBinary( sensorMutex );
	vSemaphoreCreateBinary( sensor1semaph );
	vSemaphoreCreateBinary( sensor2semaph );
	vSemaphoreCreateBinary( sensor3semaph );

	vQueueAddToRegistry( sensorMutex, "sensorMutex" );
	vQueueAddToRegistry( sensor1semaph, "sensor1semaph" );
	vQueueAddToRegistry( sensor2semaph, "sensor2semaph" );
	vQueueAddToRegistry( sensor3semaph, "sensor3semaph" );


// CRIACAO DAS TASKS COM SUAS RESPECTIVAS FUNCOES, PRIORIDADES, ETC
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


//INICIALIZACAO DO SCHEDULER
	vTaskStartScheduler();


	for( ;; );
}

static void taskCheckModule1( void *pvParameters )
{
//TASK DE VERIFICACAO DO MODULO 1
uint32_t sensor1value = 0;


	for( ;; ) //LOOP INFINITO
	{
		xSemaphoreTake( sensor1semaph, portMAX_DELAY );  // LIBERACAO DA TASK POR PARTE DO CONSUMIDOR DOS DADOS
		xSemaphoreTake( sensorMutex, portMAX_DELAY );   // MUTEX QUE IMPEDE VERIFICACOES PARALELAS DE DISTANCIA
		// CODIGO DE ATIVACAO DAS PORTAS DOS CANAIS DO RESPECTIVO TIMER PARA ATENDER
		//           O PROTOCOLO DE VERIFICACAO DE DISTANCIA DO MODULO
		(TIM3)->CNT = 0;
		TIM_Cmd(TIM3, ENABLE);
		while(!TIM_GetFlagStatus(TIM3, TIM_FLAG_Update));
		TIM_Cmd(TIM3, DISABLE);
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
		sensor1value =  (TIM_GetCapture2(TIM3)-TIM_GetCapture1(TIM3))*165/1000;

		xQueueSend( queueSensor1, &sensor1value, 0 ); // ENVIO DA DISTANCIA PARA A FILA
		xSemaphoreGive(sensorMutex);   // DEVOLUCAO DO MUTEX PARA OUTRO SENSOR PODER SER LIDO
	}
}

static void taskCheckModule2( void *pvParameters )
{
uint32_t sensor2value = 0;

	for( ;; )
	{
		xSemaphoreTake( sensor2semaph, portMAX_DELAY );
		xSemaphoreTake( sensorMutex, portMAX_DELAY );
		(TIM4)->CNT = 0;
		TIM_Cmd(TIM4, ENABLE);
		while(!TIM_GetFlagStatus(TIM4, TIM_FLAG_Update));
		TIM_Cmd(TIM4, DISABLE);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		sensor2value =  (TIM_GetCapture2(TIM4)-TIM_GetCapture1(TIM4))*165/1000;
		xQueueSend( queueSensor2, &sensor2value, 0 );
		xSemaphoreGive(sensorMutex);
	}
}
static void taskCheckModule3( void *pvParameters )
{
uint32_t sensor3value = 0;


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

static void processDistance( uint32_t distance, int sensor){
	// FUNCAO DE PROCESSAMENTO DA DISTANCIA, DANDO A RESPOSTA ESPERADA NOS PINOS DO MICROCONTROLADOR

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

	// TASK DE CONSUMO DAS INFORMACOES DA FILA, PROCESSAMENTO DAS DISTANCIAS EM RESPOSTA PARA USUARIO
	//                  E GERENCIAMENTO DE SLEEP
uint32_t S1ReceivedValue;
uint32_t S2ReceivedValue;
uint32_t S3ReceivedValue;
uint32_t totalDistance=0;
uint32_t nextDelay;

	for( ;; )
	{

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


	}
}


static void initializeProgram( void )
{
	// INICIALIZACAO DO PROGRAMA

	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );


    //INICIALIZACAO DOS CLOCKS DOS PERIFERICOS
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);


	//INICIALIZACAO DOS PINOS
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
	//CONFIGURACAO DOS TIMERS E OS PINOS CORRESPONDENTES
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

		TIM_TimeBaseInitStruct.TIM_Prescaler = prescaler;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
		TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);


		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_Pulse = 15; //us
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC3Init(TIM4, &TIM_OCInitStruct);


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

		TIM_TimeBaseInitStruct.TIM_Prescaler = prescaler;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
		TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);


		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_Pulse = 15; //us
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC3Init(TIM5, &TIM_OCInitStruct);


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




}




void vApplicationTickHook( void )
{



}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{

	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;


	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;


	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{

	}
}
