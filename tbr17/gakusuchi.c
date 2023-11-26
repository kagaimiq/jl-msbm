#include <jl_br17_regs.h>
#include <jl_irq.h>
#include <xprintf.h>

void uputc(int c) {
	while (!(JL_UART2->CON & (1<<15)));
	JL_UART2->BUF = c;
//	JL_UART2->CON |= (1<<13);
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

/*=========================================================================*/

void JieLi(uint32_t args[4]) {
	reg_wsmask(JL_CLOCK->CLK_CON1, 10, 0x3, 0x1);	/* uart_clk = pll_48m */

	JL_UART2->CON = 1;
	JL_UART2->BAUD = (48000000 / 4 / 115200) - 1;

	reg_wsmask(JL_IOMAP->CON1, 14, 0x3, 'A'-'A');
	reg_wsmask(JL_IOMAP->CON3, 8, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<11);

	JL_PORTA->DIR &= ~(1<<3);	/* PA3 out */

	xdev_out(uputc);
	xputs("\n\nhello br17\n");

	
}
