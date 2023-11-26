#include "FreeRTOS.h"
#include "task.h"

#include <jl_br25_regs.h>
#include <jl_irq.h>

#include <xprintf.h>

/*------------------------------------------------------------------------------*/

static UBaseType_t uxCriticalNesting = 0xAAAAAAAA;

/*------------------------------------------------------------------------------*/

static void __attribute__((naked)) prvPortStartFirstTask(void) {
	asm (
		"r0 = pxCurrentTCB\n"		/* pxCurrentTCB address */
		"r0 = [r0]\n"			/* grab the current TCB */
		"r0 = [r0]\n"			/* read pxTopOfStack */
		"sp = r0\n"			/* set it as our stack pointer */
		"{r15-r0} = [sp++]\n"		/* restore the context */
		"{psr, rets, reti} = [sp++]\n"
		"sti\n"				/* enable interrupts */
		"rti\n"				/* return */
	);
}

static void __attribute__((naked)) prvPortSwitchTask(void) {
	asm (
		"sp = usp\n"			/* use the user stack */
		"[--sp] = {psr, rets, reti}\n"	/* store the current task's context */
		"[--sp] = {r15-r0}\n"

		"r0 = 63\n"			/* int #63 (sw int 3) */
		"call irq_latch_clear\n"	/* clear latch */

		"r4 = pxCurrentTCB\n"		/* pxCurrentTCB pointer */

		"r5 = [r4]\n"			/* grab the current TCB */
		"r6 = sp\n"
		"[r5] = r6\n"			/* set pxTopOfStack with the new stack */

		"call vTaskSwitchContext\n"	/* switch context */

		"r5 = [r4]\n"			/* grab the current TCB */
		"r5 = [r5]\n"			/* read pxTopOfStack */
		"sp = r5\n"

		"{r15-r0} = [sp++]\n"		/* restore the next task's context */
		"{psr, rets, reti} = [sp++]\n"
		"usp = sp\n"			/* move the user stack to a new place */

		"sp = ssp\n"			/* <- in order to keep the supervisor stack */
		"rti\n"				/* return */
	);
}

static void IRQ_HANDLER prvPortTickHandler(void) {
	portDISABLE_INTERRUPTS();

	JL_CPU->TTMR_CON |= (1<<6);

	if (xTaskIncrementTick() != pdFALSE)
		portYIELD();

	portENABLE_INTERRUPTS();
}

/*------------------------------------------------------------------------------*/

StackType_t *pxPortInitialiseStack(StackType_t *stack, TaskFunction_t func, void *param) {
	stack -= 3;	/* SFRs */
	stack[ 0] = (StackType_t)func;		/* reti */
	stack[ 1] = 0x555a494d;			/* rets */
	stack[ 2] = 0x00000000;			/* psr */

	stack -= 16;	/* GPRs */
	stack[ 0] = (StackType_t)param;		/* r0 */
	stack[ 1] = 0x20313021;			/* r1 */
	stack[ 2] = 0x20323021;			/* r2 */
	stack[ 3] = 0x20333021;			/* r3 */
	stack[ 4] = 0x20343021;			/* r4 */
	stack[ 5] = 0x20353021;			/* r5 */
	stack[ 6] = 0x20363021;			/* r6 */
	stack[ 7] = 0x20373021;			/* r7 */
	stack[ 8] = 0x20383021;			/* r8 */
	stack[ 9] = 0x20393021;			/* r9 */
	stack[10] = 0x20303121;			/* r10 */
	stack[11] = 0x20313121;			/* r11 */
	stack[12] = 0x20323121;			/* r12 */
	stack[13] = 0x20333121;			/* r13 */
	stack[14] = 0x20343121;			/* r14 */
	stack[15] = 0x20353121;			/* r15 */

	return stack;
}

BaseType_t xPortStartScheduler(void) {
	portDISABLE_INTERRUPTS();

	uxCriticalNesting = 0;

	irq_attach(irqn_TICK_TMR, prvPortTickHandler, 0);
	irq_attach(63, prvPortSwitchTask, 0);

	JL_CPU->TTMR_CON = 0;
	JL_CPU->TTMR_CNT = 0;
	JL_CPU->TTMR_PRD = (192000000 / configTICK_RATE_HZ) - 1;
	JL_CPU->TTMR_CON |= 1;

	prvPortStartFirstTask();

	return 0;
}

void vPortEndScheduler(void) {
}

void vPortEnterCritical(void) {
	/*portDISABLE_INTERRUPTS();

	uxCriticalNesting++;*/
}

void vPortExitCritical(void) {
	/*uxCriticalNesting--;

	if (uxCriticalNesting == 0)
		portENABLE_INTERRUPTS();*/
}
