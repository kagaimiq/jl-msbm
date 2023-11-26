#ifndef _PORTMACRO_H
#define _PORTMACRO_H

/* ---------------- type definitions ------------------- */
#define portCHAR			char
#define portFLOAT			float
#define portDOUBLE			double
#define portLONG			long
#define portSHORT			short
#define portSTACK_TYPE			uint32_t
#define portBASE_TYPE			long

typedef portSTACK_TYPE			StackType_t;
typedef long				BaseType_t;
typedef unsigned long			UBaseType_t;

#if configTICK_TYPE_WIDTH_IN_BITS == TICK_TYPE_WIDTH_16_BITS
	typedef uint16_t			TickType_t;
	#define portMAX_DELAY			(TickType_t)0xFFFF
#elif configTICK_TYPE_WIDTH_IN_BITS == TICK_TYPE_WIDTH_32_BITS
	typedef uint32_t			TickType_t;
	#define portMAX_DELAY			(TickType_t)0xFFFFFFFF
	#define portTICK_TYPE_IS_ATOMIC		1
#else
	#error you popa b
#endif

/* ------------------ arch specifics ------------------- */

#define portSTACK_GROWTH			(-1)
#define portTICK_PERIOD_MS			((TickType_t)1000 / configTICK_RATE_HZ)
#define portBYTE_ALIGNMENT			8
#define portDONT_DISCARD			__attribute__((used))

#define portYIELD()			asm volatile ("swi 3")
#define portEND_SWITCHING_ISR(swreq)	{ if (swreq != pdFALSE) portYIELD(); }
#define portYIELD_FROM_ISR(x)		portEND_SWITCHING_ISR(x)

/* ------------------ critical sections ---------------- */
extern void vPortEnterCritical(void);
extern void vPortExitCritical(void);

#define portSET_INTERRUPT_MASK_FROM_ISR()		0
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)		{}
#define portDISABLE_INTERRUPTS()			asm volatile ("cli")
#define portENABLE_INTERRUPTS()				asm volatile ("sti")
#define portENTER_CRITICAL()				vPortEnterCritical()
#define portEXIT_CRITICAL()				vPortExitCritical()

/*============================================================*/

#define portTASK_FUNCTION_PROTO(func, param)		void func(void *param)
#define portTASK_FUNCTION(func, param)			void func(void *param)

/* ---------------- arch specifics opts --------------- */

#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
	#define configUSE_PORT_OPTIMISED_TASK_SELECTION		0
#endif

/* there are none! */

/*============================================================*/

#define portNOP()					asm ("nop")

#define portINLINE					inline

#ifndef portFORCE_INLINE
	#define portFORCE_INLINE			inline __attribute__((always_inline))
#endif

/*============================================================*/

#define portMEMORY_BARRIER()				asm volatile ("" ::: "memory")

#endif
