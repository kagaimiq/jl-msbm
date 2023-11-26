#include <jl_br25_regs.h>
#include <jl_irq.h>
#include <jl_p33.h>

void cpuloops(int cnt) {
	while (cnt--)
		asm volatile ("nop");
}

void JieLi(uint32_t r0) {
#if 0
	JL_PORTB->DIR &= ~(1<<5);

	for (;;) {
		p33_xfer(0, P33_OP_RMW_OR, 0x80, 0x40);

		JL_PORTB->OUT ^= (1<<5);
		cpuloops(100000);
	}
#endif

	/* PD0=SCK, PD1=MOSI, PD2=MISO, PD3=SCK, PD4=Vdd */
	JL_PORTD->DIR &= ~0x1B;
	JL_PORTD->DIR |=  0x04;
	JL_PORTD->OUT |=  0x18;

	JL_IOMAP->CON0 &= ~(1<<2);

	JL_SPI0->BAUD = 48-1;
	JL_SPI0->CON = (1<<14) | (1<<5) | (1<<0) | (1<<3);

	cpuloops(1000);

	JL_PERIENC->CON = (1<<0) | (1<<1);
	JL_PERIENC->KEY = 0x77A;
	JL_PERIENC->ADR = 0x10;

	JL_PORTD->OUT &= ~(1<<3);	/* CS low */

	JL_SPI0->CON &= ~(1<<12);	/* send */

	JL_SPI0->BUF = 0x03;
	while (!(JL_SPI0->CON & (1<<15)));
	JL_SPI0->CON |= (1<<14);

	JL_SPI0->BUF = 0x00;
	while (!(JL_SPI0->CON & (1<<15)));
	JL_SPI0->CON |= (1<<14);

	JL_SPI0->BUF = 0x10;
	while (!(JL_SPI0->CON & (1<<15)));
	JL_SPI0->CON |= (1<<14);

	JL_SPI0->BUF = 0x00;
	while (!(JL_SPI0->CON & (1<<15)));
	JL_SPI0->CON |= (1<<14);

	JL_SPI0->CON |= (1<<12);	/* receive */

	JL_SPI0->ADR = 0x18000;
	JL_SPI0->CNT = 256;
	while (!(JL_SPI0->CON & (1<<15)));
	JL_SPI0->CON |= (1<<14);

	JL_PORTD->OUT |= (1<<3);	/* CS high */
}
