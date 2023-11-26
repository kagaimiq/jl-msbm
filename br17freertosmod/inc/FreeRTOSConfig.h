#ifndef _FREERTOS_CONFIG_H
#define _FREERTOS_CONFIG_H

/*=================================================================*/

#define configAPPLICATION_ALLOCATED_HEAP		0
#define configTOTAL_HEAP_SIZE				49152

/*=================================================================*/

#define configTICK_RATE_HZ				100

/*=================================================================*/

#define configUSE_16_BIT_TICKS				0
#define configMINIMAL_STACK_SIZE			128
#define configMAX_PRIORITIES				8
#define configUSE_PREEMPTION				1

/*=================================================================*/

#define configUSE_IDLE_HOOK				1
#define configUSE_TICK_HOOK				0

/*=================================================================*/

#define configUSE_TRACE_FACILITY			1

#define configGENERATE_RUN_TIME_STATS			1

extern uint32_t runtimectr;

#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()	{ runtimectr = 0; }
#define portGET_RUN_TIME_COUNTER_VALUE()		runtimectr

/*=================================================================*/

#define INCLUDE_vTaskDelay				1
#define INCLUDE_xTaskDelayUntil				1
#define INCLUDE_vTaskDelete				1

#endif
