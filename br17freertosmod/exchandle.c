#include <jl_br17_regs.h>
#include <xprintf.h>

extern void hexdump(void *ptr, int len);

struct exc_stack_frame {
	uint32_t r[15];

	uint32_t reti, rete;
	uint32_t maccl, macch;
	uint32_t rets;
	uint32_t psr;
	uint32_t ssp, usp;
	uint32_t ie1, ie0, icfg;
};




const char *dbg_msg_bits[] = {
	"reserved",
	"reserved",
	"prp_ex_limit_err",
	"sdr_wr_limit_err",

	"reserved",
	"reserved",
	"reserved",
	"reserved",

	"reserved",
	"reserved",
	"reserved",
	"reserved",

	"reserved",
	"reserved",
	"reserved",
	"reserved",

	"dsp_pc_limit_err",
	"dsp_ex_limit_err",
	"dsp_illegal",
	"dsp_misaligned",

	"reserved",
	"reserved",
	"reserved",
	"reserved",

	"dsp_if_bus_inv",
	"dsp_of_bus_inv",
	"dsp_ex_bus_inv",
	"prp_bus_inv",

	"reserved",
	"reserved",
	"reserved",
	"reserved",
};




void ExceptionHandlerMain(void *stack) {
	struct exc_stack_frame *sf = stack;

	xputs("\e[1;33;41m=============== JIELI DIES ===============\e[0m\n");

	xputc('\n');

	xprintf("Message: %08x (", JL_DEBUG->MSG);

	if (JL_DEBUG->MSG) {
		for (int i = 0, n = 0; i < 32; i++) {
			if (!(JL_DEBUG->MSG & (1<<i)))
				continue;

			if (n++ > 0) xputs(", ");

			xputs(dbg_msg_bits[i]);
		}
	} else {
		xputs("none");
	}

	xputs(")\n");

	xprintf("PRP_MMU_MSG      = %08x\n", JL_DEBUG->PRP_MMU_MSG);
	xprintf("LSB_MMU_MSG_CH   = %08x\n", JL_DEBUG->LSB_MMU_MSG_CH);
	xprintf("PRP_WR_LIMIT_MSG = %08x\n", JL_DEBUG->PRP_WR_LIMIT_MSG);
	xprintf("LSB_WR_LIMIT_CH  = %08x\n", JL_DEBUG->LSB_WR_LIMIT_CH);

	xputc('\n');

	xprintf("r0  <%08x>  r1  <%08x>  r2  <%08x>  r3  <%08x>\n", sf->r[ 0], sf->r[ 1], sf->r[ 2], sf->r[ 3]);
	xprintf("r4  <%08x>  r5  <%08x>  r6  <%08x>  r7  <%08x>\n", sf->r[ 4], sf->r[ 5], sf->r[ 6], sf->r[ 7]);
	xprintf("r8  <%08x>  r9  <%08x>  r10 <%08x>  r11 <%08x>\n", sf->r[ 8], sf->r[ 9], sf->r[10], sf->r[11]);
	xprintf("r12 <%08x>  r13 <%08x>  r14 <%08x>\n",             sf->r[12], sf->r[13], sf->r[14]);

	xputc('\n');

	xprintf("reti <%08x>  rete <%08x>  maccl <%08x>  macch <%08x>\n", sf->reti, sf->rete, sf->maccl, sf->macch);
	xprintf("rets <%08x>  psr  <%08x>  ssp   <%08x>  usp   <%08x>\n", sf->rets, sf->psr, sf->ssp, sf->usp);

	xprintf("ie1  <%08x>  ie0  <%08x>  icfg  <%08x>\n",               sf->ie1, sf->ie0, sf->icfg);

	xputc('\n');

	xputs("Near User stack:\n");
	hexdump((void *)(sf->usp - 0x40), 0x80);

	xputs("Near crash:\n");
	hexdump((void *)(sf->reti - 0x40), 0x80);

	JL_POWER->CON |= (1<<4);	/* reset chip */
}

void __attribute__((naked)) ExceptionHandler(void) {
	asm (
		"[--sp] = {icfg, ie0, ie1}\n"
		"[--sp] = {usp, ssp, psr, rets, macch, maccl, rete, reti}\n"
		"[--sp] = {r14-r0}\n"
		"r0 = sp\n"
		"call ExceptionHandlerMain\n"
		"1: goto 1b\n"
	);
}
