#include <jl_br25_regs.h>
#include <jl_p33.h>

void putc(int c) {
	JL_UART0->BUF = c;
	while (!(JL_UART0->CON0 & (1<<15)));
	JL_UART0->CON0 |= (1<<13);
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




void cpuloops(int cnt) {
	while (cnt--)
		asm volatile ("nop");
}



void JieLi(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
	/* Enter normal mode */
	JL_CRC->REG = 0x6EA5;	/* Unlock MODE_CON */
	JL_MODE->CON = 0;	/* Clear MODE_CON */
	JL_CRC->REG = 0;	/* Lock it back */

	/* Init UART0 at PB5 */
	reg_wsmask(JL_CLOCK->CLK_CON1, 10, 0x3, 0x2);

	JL_UART0->CON0 = 1;
	JL_UART0->BAUD = (16000000 / 4 / 38400) - 1;

	reg_wsmask(JL_IOMAP->CON0, 3, 0x3, 'C'-'A');
	reg_wsmask(JL_IOMAP->CON3, 0, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<3);

	JL_PORTB->DIR &= ~(1<<5);
	JL_PORTB->DIE |= (1<<5);
	JL_PORTB->PU |= (1<<5);

	puts("hOI!\n");

	for (int i = 0; i < 100; i++) putc(0x7e);
	putc('\n');

	puts("Regs:\n");
	puthex32(r0);
	puthex32(r1);
	puthex32(r2);
	puthex32(r3);

	for (;;) {
		puts("hai hai!!\n");
		cpuloops(1000000);
	}
}
