#include <jl_br17_regs.h>
#include <jl_irq.h>
#include <jl_irtc.h>
#include <xprintf.h>

#include <FreeRTOS.h>
#include <task.h>
#include <stream_buffer.h>

void uart_putc(int c) {
	uint32_t tmp;
	asm ("cli %0" : "=r"(tmp));

	while (!(JL_UART2->CON & (1<<15)));
	JL_UART2->BUF = c;

	asm ("sti %0" :: "r"(tmp));
}

void hexdump(void *ptr, int len) {
	for (int i = 0; i < len; i += 16) {
		xprintf("%08x: ", ptr+i);

		for (int j = 0; j < 16; j++) {
			if (i+j < len)
				xprintf("%02X ", *(uint8_t*)(ptr+i+j));
			else
				xputs("-- ");
		}

		xputs(" |");

		for (int j = 0; j < 16; j++) {
			uint8_t c = ' ';
			if (i+j < len) {
				c = *(uint8_t*)(ptr+i+j);
				if (c < 0x20 || c >= 0x7f) c = '.';
			}
			xputc(c);
		}

		xputs("|\n");
	}

	xputc('\n');
}

/*=========================================================================*/

void clk_init(void) {
	/* reset */
	JL_CLOCK->CLK_CON0 = 0x00000001;
	JL_CLOCK->CLK_CON1 = 0x00000000;
	JL_CLOCK->CLK_CON2 = 0x00555015;

	/* pll config */
	JL_CLOCK->PLL_CON =
		(0<<20)|	/* PLL_TEST */
		(0<<17)|	/* PLL_REF_SEL   -> btosc_clk */
		(0<<16)|	/* PLL_RSEL12 */
		(0<<12)|	/* PLL_TSEL */
		(0<<11)|	/* PLL_DSMS */
		(0<<10)|	/* PLL_DIVS */
		(0<<8)|		/* PLL_RSEL */
		(1<<7)|		/* PLL_REFDE */
		(((16/2)-2)<<2)	/* PLL_REFDS */
	;

	/* pll enable */
	JL_CLOCK->PLL_CON |= (1<<0);
	for (volatile int i = 1000; i; i--);

	/* pll reset release */
	JL_CLOCK->PLL_CON |= (1<<1);
	for (volatile int i = 1000; i; i--);

	/* clock cfg */
	JL_CLOCK->CLK_CON2 &= ~(0x3f<<0);
	JL_CLOCK->CLK_CON2 |= (0<<4)|(1<<2)|(1<<0);	/* pll_sys_clk <- div1 <- div3 <- pll_480m */

	/* system clock */
	JL_CLOCK->SYS_DIV =
		(0<<0)|		/* hsb_clk/... = org_clk - div1 */
		(1<<8)		/* lsb_clk = div2 */
	;

	JL_CLOCK->CLK_CON0 |= (3<<6);	/* (sel) <- pll_sys_clk */
	JL_CLOCK->CLK_CON0 |= (1<<8);	/* #src_clk <--- (sel) */
}


void vApplicationIdleHook(void) {
	/* feed watchdog */
	JL_SYSTEM->WDT_CON |= (1<<6);

	asm ("idle");
}


void demitask(void *param) {
	irtc_wsmask(IRTC_PORTR_DIE, 6, 1, 0);	/* no long press reset */

	irtc_wsmask(IRTC_PORTR_OUT, 4+2, 1, 0);
	irtc_wsmask(IRTC_PORTR_OUT, 0+2, 1, 1);

	for (;;) {
		irtc_write(IRTC_PORTR_OUT, irtc_read(IRTC_PORTR_OUT) ^ (1<<2));
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}



void meintask(void *param) {
	xprintf("MeinTask %x\n", param);

	{
		uint32_t sp;
		asm ("%0 = sp\n" : "=r"(sp));
		xprintf("SP = %x\n", sp);
	}

	xTaskCreate(demitask, "demitask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	extern void Hakase(void *param);
	xTaskCreate(Hakase, "Hakase", 1024, NULL, 2, NULL);

	/*====================================================================*/

	TickType_t stime = xTaskGetTickCount();

	TaskStatus_t *oldstatus = NULL;
	UBaseType_t oldntasks = 0;
	uint32_t oldruntime = 0;

	for (;;) {
		UBaseType_t ntasks = uxTaskGetNumberOfTasks();
		TaskStatus_t *status = pvPortMalloc(ntasks * sizeof(TaskStatus_t));

		if (status) {
			uint32_t runtime;
			ntasks = uxTaskGetSystemState(status, ntasks, &runtime);

			int rtdiff = runtime - oldruntime;

			xprintf("\e[25;1H----------- %d tasks, total %u (diff %u) ----------\n", ntasks, runtime, rtdiff);

			for (int i = 0; i < ntasks; i++) {
				TaskStatus_t *task = &status[i];

				int tasktdiff = 0;

				for (int j = 0; j < oldntasks; j++) {
					TaskStatus_t *otask = &oldstatus[j];

					if (otask->xTaskNumber == task->xTaskNumber) {
						tasktdiff = task->ulRunTimeCounter - otask->ulRunTimeCounter;
						break;
					}
				}

				if (tasktdiff < 0)
					tasktdiff = 0;
				if (tasktdiff > rtdiff)
					tasktdiff = rtdiff;

				xprintf("{%2d: @%x [%-16s] #%d | %d | %d/%d | %3d%% | @%x/%d}\n",
					i, task->xHandle, task->pcTaskName, task->xTaskNumber,
					task->eCurrentState,
					task->uxCurrentPriority, task->uxBasePriority,
					tasktdiff * 100 / rtdiff,
					task->pxStackBase, task->usStackHighWaterMark);
			}

			xputc('\n');

			oldruntime = runtime;
		}

		if (oldstatus)
			vPortFree(oldstatus);

		oldstatus = status;
		oldntasks = ntasks;

		xTaskDelayUntil(&stime, pdMS_TO_TICKS(1000));
	}
}

/*------------------------------------------------------------------*/

uint32_t runtimectr;

void IRQ_HANDLER timer0_irqhandler(void) {
	runtimectr++;

	JL_TIMER0->CON |= (1<<14);
	irq_latch_clear(irqn_TMR0);
}

/*------------------------------------------------------------------*/

void JieLi(void) {
	clk_init();

	reg_wsmask(JL_CLOCK->CLK_CON1, 10, 0x3, 0x1);	/* uart_clk = pll_48m */

	JL_UART2->CON = 1;
	JL_UART2->BAUD = (48000000 / 4 / 921600) - 1;

	reg_wsmask(JL_IOMAP->CON1, 14, 0x3, 'A'-'A');
	reg_wsmask(JL_IOMAP->CON3, 8, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<11);

	JL_PORTA->DIR &= ~(1<<3);	/* PA3 out */

	xdev_out(uart_putc);
	xputs("\e[H\e[2J\e[3J"); // clear screen
	xputs("\n\nhello br17 "__DATE__" "__TIME__"\n");

	extern char ExceptionHandler[];
	irq_set_handler(irqn_EXCEPTION, ExceptionHandler);

	/*---------------------------------------------------*/

	JL_TIMER0->CON = (7<<4);
	JL_TIMER0->CNT = 0;
	JL_TIMER0->PRD = (80000000 / 128 / 1000) - 1;
	JL_TIMER0->CON |= (1<<0);

	irq_attach(irqn_TMR0, timer0_irqhandler, 3);

	xTaskCreate(meintask, "Mein Task", configMINIMAL_STACK_SIZE, (void *)0x89023154, 1, NULL);

	vTaskStartScheduler();
}
