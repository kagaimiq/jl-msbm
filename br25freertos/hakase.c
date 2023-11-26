#include <jl_br25_regs.h>
#include <jl_irq.h>
#include <jl_p33.h>
#include <xprintf.h>

#include <FreeRTOS.h>
#include <task.h>
#include <stream_buffer.h>

void uart_putc(int c) {
	JL_UART2->BUF = c;
	while (!(JL_UART2->CON0 & (1<<15)));
	JL_UART2->CON0 |= (1<<13);
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


void cpuloops(int n) {
	while (n--)
		asm volatile ("nop");
}



void vApplicationIdleHook(void) {
	/* feed watchdog */
	p33_xfer(0, P33_OP_RMW_OR, 0x80, 0x40);

	asm ("idle");
}




void demitask(void *param) {
	JL_UART0->CON0 = 0;
	JL_PORTB->DIR &= ~(1<<5);

	for (;;) {
		JL_PORTB->OUT ^= (1<<5);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}



void meintask(void *param) {
	xprintf("MeinTask %x\n", param);

	xTaskCreate(demitask, "demitask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	extern void Hakase(void *param);
	xTaskCreate(Hakase, "Hakase", 4096, NULL, 2, NULL);

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
}

/*------------------------------------------------------------------*/


void clk_init(void) {
	int ref = 24000000; /* taken from boot arg list + 0x4 */

	/* don't use the PLL clock for now */
	JL_CLOCK->CLK_CON0 &= ~(1<<8);

	/* Init PLL */
	JL_CLOCK->PLL_CON  = 0x34400000 | (((ref / 2000000) - 2) << 2);
	JL_CLOCK->PLL_CON1 = 0x3F803000 | ((480 / 2) - 2);
	JL_CLOCK->PLL_INTF = 0x00000004;

	/* Enable PLL */
	JL_CLOCK->PLL_CON |= (1<<0);
	cpuloops(100);

	/* Release PLL reset */
	JL_CLOCK->PLL_CON |= (1<<1);
	cpuloops(100);

	/* USB clock sel = pll_48m */
	reg_wsmask(JL_CLOCK->CLK_CON1, 0, 3, 0);	/* 0: pll_48m */

	/* pll_sys config */
	reg_wsmask(JL_CLOCK->CLK_CON2, 0, 3, 0);	/* 0: pll_192m, 1: pll_137m, 2: pll_320m, 3: pll_480m */
	reg_wsmask(JL_CLOCK->CLK_CON2, 2, 3, 0);	/* 0: /1, 1: /3, 2: /5, 3: /7 */
	reg_wsmask(JL_CLOCK->CLK_CON2, 4, 3, 0);	/* 0: /1, 1: /2, 2: /4, 3: /8 */

	/* use pll_sys for the main clock */
	JL_CLOCK->CLK_CON0 |= (3<<6);
	cpuloops(100);

	/* HSB, LSB and SFC clock div */
	JL_CLOCK->SYS_DIV = ((1-1) << 0) | ((4-1) << 8) | ((1-1) << 12);

	/* use the selected main clock! */
	JL_CLOCK->CLK_CON0 |= (1<<8);
	cpuloops(100);
}


void JieLi(void) {
	clk_init();

	reg_wsmask(JL_CLOCK->CLK_CON1, 10, 0x3, 0x0);	/* uart_clk = pll_48m */

	#if 1	/* UART2 at PA3(TX)/PA4(RX) */
	JL_UART2->CON0 = 1;
	JL_UART2->BAUD = (48000000 / 4 / 921600) - 1;

	reg_wsmask(JL_IOMAP->CON1, 14, 0x3, 'A'-'A');
	reg_wsmask(JL_IOMAP->CON3, 8, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<11);

	JL_PORTA->DIR &= ~(1<<3);	/* PA3 out */
	JL_PORTA->DIE |= (1<<4);	/* PA4 digital in en */
	JL_PORTA->PU |= (1<<4);		/* PA4 pullup */
	#endif

	#if 0	/* UART0 at PB5(TX/RX) */
	JL_UART0->CON0 = 1;
	JL_UART0->BAUD = (48000000 / 4 / 921600) - 1;

	reg_wsmask(JL_IOMAP->CON0, 3, 0x3, 'C'-'A');
	reg_wsmask(JL_IOMAP->CON3, 0, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<3);

	JL_PORTB->DIR &= ~(1<<5);
	JL_PORTB->DIE |= (1<<5);
	JL_PORTB->PU |= (1<<5);
	#endif

	xdev_out(uart_putc);
	xputs("\n\n\e[1;33mhello br25\e[0m\n");

	{
		uint32_t tmp;

		asm ("%0 = sp\n" : "=r"(tmp));
		xprintf("sp %08x\n", tmp);

		asm ("%0 = usp\n" : "=r"(tmp));
		xprintf("usp %08x\n", tmp);

		asm ("%0 = ssp\n" : "=r"(tmp));
		xprintf("ssp %08x\n", tmp);
	}

	/*---------------------------------------------------*/

	JL_TIMER0->CON = (7<<4);
	JL_TIMER0->CNT = 0;
	JL_TIMER0->PRD = (48000000 / 128 / 1000) - 1;
	JL_TIMER0->CON |= (1<<0);

	irq_attach(irqn_TMR0, timer0_irqhandler, 3);

	xTaskCreate(meintask, "Mein Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	vTaskStartScheduler();
}
