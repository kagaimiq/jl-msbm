#include <jl_regs.h>
#include <jl_irq.h>
#include "wallclk.h"
#include <stdio.h>

// hardcoded...
#define PERIOD_1MS		(48000000 / 1000)

volatile uint32_t msecs;

static void IRQ_HANDLER TickerTimer(void) {
	msecs++;
	reg32_wsmask(CORE_base+CORE_TTMR_CON_cpnd, 1);
};


void wallclk_init(void) {
	reg32_write(CORE_base+CORE_TTMR_CON, 0);
	reg32_write(CORE_base+CORE_TTMR_CNT, 0);
	reg32_write(CORE_base+CORE_TTMR_PRD, PERIOD_1MS);
	reg32_wsmask(CORE_base+CORE_TTMR_CON_enable, 1);
	irq_attach(3, TickerTimer);
}

void wallclk_deinit(void) {
	irq_detach(3);
	reg32_wsmask(CORE_base+CORE_TTMR_CON_enable, 0);
}


uint64_t micros(void) {
	return (msecs * 1000ull) + (reg32_read(CORE_base+CORE_TTMR_CNT) / (PERIOD_1MS / 1000));
}

void usleep(uint32_t us) {
	for (uint64_t target = micros() + us; micros() < target; );
}


uint32_t millis(void) {
	return msecs;
}

void delay(uint32_t ms) {
	for (uint32_t target = millis() + ms + 1; millis() < target; );
}
