#ifndef _JL_IRQ_H
#define _JL_IRQ_H

#include <stdint.h>

enum {
	irqn_EXCEPTION = 0,
	irqn_TICK_TMR,

	irqn_TMR0,
	irqn_TMR1,
	irqn_TMR2,
	irqn_TMR3,
	irqn_FUSB_SOF,
	irqn_FUSB_CTL,
	irqn_RTC,
	irqn_ALNK,
	irqn_AUDIO,
	irqn_PORT,
	irqn_SPI0,
	irqn_SPI1,
	irqn_SDC0,
	irqn_SDC1,
	irqn_UART0,
	irqn_UART1,
	irqn_UART2,
	irqn_PAP,
	irqn_IIC,
	irqn_SARADC,
	irqn_FM_HWFE,
	irqn_FM,
	irqn_FM_LOFC,
	irqn_BT_BREDR,
	irqn_BT_CLKN,
	irqn_BT_DBG,
	irqn_BT_PCM,
	irqn_SRC,
	irqn_FFT,	/* maybe? */
	irqn_EQ,
	irqn_BT_BLE = 36,
	irqn_CTM = 43,
};

#define IRQ_HANDLER		__attribute__((interrupt))

#define IRQ_VECTABLE_BASE		0x1E000

void irq_enable(int no);
void irq_disable(int no);
void irq_latch_clear(int no);
void irq_set_priority(int no, int priority);

void irq_set_handler(int no, void *handler);

void irq_attach(int no, void *handler, int priority);
void irq_detach(int no);

#endif
