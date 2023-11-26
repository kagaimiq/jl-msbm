#include <jl_br17_regs.h>
#include <jl_irq.h>
#include <jl_irtc.h>


void cpuloops(int n) {
	while (n--)
		asm volatile ("nop");
}


void clk_init(void) {
	/* reset */
	JL_CLOCK->CLK_CON0 = 0x00000001;
	JL_CLOCK->CLK_CON1 = 0x00000000;
	JL_CLOCK->CLK_CON2 = 0x15555555; //0x00555015;

	JL_CLOCK->CLK_CON0 |= (1<<1);	/* use pat_clk as src_clk */
	JL_CLOCK->CLK_CON0 |= (3<<4);	/* osc_clk is pat_clk */

#if 0
	/* pll config */
	JL_CLOCK->PLL_CON =
		(0<<20)|	/* PLL_TEST */
		(3<<17)|	/* PLL_REF_SEL   -> 0:btosc_clk, 3:pat_clk */
		(0<<16)|	/* PLL_RSEL12 */
		(0<<12)|	/* PLL_TSEL */
		(0<<11)|	/* PLL_DSMS */
		(0<<10)|	/* PLL_DIVS */
		(0<<8)|		/* PLL_RSEL */
		(1<<7)|		/* PLL_REFDE */
		(((4/2)-2)<<2)	/* PLL_REFDS */
	;

	/* pll enable */
	JL_CLOCK->PLL_CON |= (1<<0);
	cpuloops(100);

	/* pll reset release */
	JL_CLOCK->PLL_CON |= (1<<1);
	cpuloops(100);

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
#endif
}




void putc(char c) {
	short val = (c << 1) | 0x200;

	for (short mask = 0x001; mask != 0x400; mask <<= 1) {
		irtc_wsmask(IRTC_PORTR_OUT, 2, 1, !!(mask & val));
		cpuloops(1800);
	}
}

void puts(char *s) {
	while (*s)
		putc(*s++);
}

void puthexdig(char h) {
	if (h < 10)
		putc('0' + h);
	else
		putc('A' + h - 10);
}

void puthex32(uint32_t val) {
	putc('0');
	putc('x');
	puthexdig(val >> 28);
	puthexdig((val >> 24) & 0xf);
	puthexdig((val >> 20) & 0xf);
	puthexdig((val >> 16) & 0xf);
	puthexdig((val >> 12) & 0xf);
	puthexdig((val >> 8) & 0xf);
	puthexdig((val >> 4) & 0xf);
	puthexdig(val & 0xf);
	putc('\n');
}



void JieLi(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
	JL_CRC->REG = 0x6EA5;	/* unlock MODE_CON! */
	JL_SYSTEM->MODE_CON = 0;	/* enter normal mode (don't use clock from ISP) */

	clk_init();

	irtc_wsmask(IRTC_PORTR_DIE, 6, 1, 0);	/* no long press reset */

	irtc_wsmask(IRTC_PORTR_OUT, 4+2, 1, 0);
	irtc_wsmask(IRTC_PORTR_OUT, 0+2, 1, 1);

	for (int i = 0; i < 100; i++) putc(0x7e);

	puts("Regs:\n");
	puthex32(r0);
	puthex32(r1);
	puthex32(r2);
	puthex32(r3);

	puts("Mode Con:\n");
	puthex32(JL_SYSTEM->WDT_CON);
	puthex32(JL_SYSTEM->MODE_CON);
	puthex32(JL_CLOCK->CLK_CON0);
	puthex32(JL_CLOCK->CLK_CON1);
	puthex32(JL_CLOCK->CLK_CON2);

	for (;;) {
		putc(0x55);
	}
}
