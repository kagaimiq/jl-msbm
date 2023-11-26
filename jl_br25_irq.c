#include "jl_irq.h"
#include "jl_br25_regs.h"

void irq_enable(int no) {
	JL_CPU->ICFGn[no / 8] |= (1 << ((no % 8) * 4));
}

void irq_disable(int no) {
	JL_CPU->ICFGn[no / 8] &= ~(1 << ((no % 8) * 4));
}

void irq_latch_clear(int no) {
	JL_CPU->ILAT_CLR = no;
}

void irq_set_priority(int no, int priority) {
	reg_wsmask(JL_CPU->ICFGn[no / 8], (no % 8) * 4 + 1, 0x7, priority);
}

void irq_set_handler(int no, void *handler) {
	void **vectors = (void *)IRQ_VECTABLE_BASE;
	if (no < 0 || no >= 64) return;

	vectors[no] = handler;
}

void irq_attach(int no, void *handler, int priority) {
	irq_disable(no);
	irq_set_handler(no, handler);
	irq_set_priority(no, priority);
	irq_enable(no);
}

void irq_detach(int no) {
	irq_disable(no);
	irq_set_handler(no, (void *)0xacebeca1);
}
