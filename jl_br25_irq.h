#ifndef _JL_IRQ_H
#define _JL_IRQ_H

#include <stdint.h>

enum {
	irqn_EMUEXCPT = 0,
	irqn_EXCEPTION,
	irqn_SYSCALL,
	irqn_TICK_TMR,

	irqn_TMR0,
	irqn_TMR1,
	irqn_TMR2,
	irqn_TMR3,
	irqn_FUSB_SOF,
	irqn_FUSB_CTL,
	irqn_RTC,
	irqn_ALNK0,
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
	irqn_PLNK,
	irqn_RDEC0,
	irqn_LRCT,
	irqn_BT_BREDR,
	irqn_BT_CLKN,
	irqn_BT_DBG,
	irqn_BT_LOFC,
	irqn_SRC,
	irqn_FFT,
	irqn_EQ,
	irqn_PD_TMR0,
	irqn_PD_TMR1,
	irqn_ALNK1,
	irqn_OSC_SAFE,
	irqn_BT_BLE,
	irqn_BT_EVENT,
	irqn_AES,
	irqn_MCTMRX,
	irqn_CHX_PWM,
	irqn_FMRX_HWFE,
	irqn_SPI2,
	irqn_SBC,
	irqn_GPC,
	irqn_FMTX_HWFE,
	irqn_DCP,
	irqn_RDEC1,
	irqn_RDEC2,
	irqn_SPDIF,
	irqn_PWM,
	irqn_CTM,
	irqn_TMR4,
	irqn_TMR5
};

#define IRQ_HANDLER		__attribute__((interrupt))

#define IRQ_VECTABLE_BASE		0x31F00

void irq_enable(int no);
void irq_disable(int no);
void irq_latch_clear(int no);
void irq_set_priority(int no, int priority);

void irq_set_handler(int no, void *handler);

void irq_attach(int no, void *handler, int priority);
void irq_detach(int no);

#endif
