#include "jl_irq.h"
#include "jl_br17_regs.h"

void irq_enable(int no) {
	uint32_t tmp;

	if (no < 32) {
		asm ("%0 = ie0" : "=r"(tmp));
		tmp |= (1 << (no - 0));
		asm ("ie0 = %0" :: "r"(tmp));
	} else if (no < 64) {
		asm ("%0 = ie1" : "=r"(tmp));
		tmp |= (1 << (no - 32));
		asm ("ie1 = %0" :: "r"(tmp));
	}
}

void irq_disable(int no) {
	uint32_t tmp;

	if (no < 32) {
		asm ("%0 = ie0" : "=r"(tmp));
		tmp &= ~(1 << (no - 0));
		asm ("ie0 = %0" :: "r"(tmp));
	} else if (no < 64) {
		asm ("%0 = ie1" : "=r"(tmp));
		tmp &= ~(1 << (no - 32));
		asm ("ie1 = %0" :: "r"(tmp));
	}
}

void irq_latch_clear(int no) {
	if (no < 32) {
		JL_NVIC->ILAT0_CLR = (1 << (no - 0));
	} else if (no < 64) {
		JL_NVIC->ILAT1_CLR = (1 << (no - 32));
	}
}

void irq_set_priority(int no, int priority) {
	if (no < 16) {
		reg_wsmask(JL_NVIC->IPCON0, (no - 0) * 2, 3, priority);
	} else if (no < 32) {
		reg_wsmask(JL_NVIC->IPCON1, (no - 16) * 2, 3, priority);
	} else if (no < 48) {
		reg_wsmask(JL_NVIC->IPCON2, (no - 32) * 2, 3, priority);
	} else if (no < 64) {
		reg_wsmask(JL_NVIC->IPCON3, (no - 48) * 2, 3, priority);
	}
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
	irq_set_handler(no, (void*)0x4d495a55);
}
