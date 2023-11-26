#include <jl_br25_regs.h>
#include <jl_irq.h>
#include <jl_p33.h>
#include <xprintf.h>

void uputc(int c) {
	JL_UART0->BUF = c;
	while (!(JL_UART0->CON0 & (1<<15)));
	JL_UART0->CON0 |= (1<<13);
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

void usleep(unsigned usecs) {
	/* quick'n'dirty delays via a timer */

	JL_TIMER4->CON = (1<<14);
	JL_TIMER4->CNT = 0;
	JL_TIMER4->PRD = usecs * 48;	/* 48 MHz */
	JL_TIMER4->CON |= (1<<0);
	while (!(JL_TIMER4->CON & (1<<15)));
	JL_TIMER4->CON = (1<<14);
}

void delay(unsigned msecs) {
	while (msecs--) usleep(1000);
}

void init_sfc(void) {
	/*
	 * PD0 = SCK
	 * PD1 = DI / IO0 (mosi)
	 * PD2 = DO / IO1 (miso)
	 * PD3 = CS#
	 * PD4 = Power
	 */

	/* Init I/O */
	JL_PORTD->DIR &= ~((1<<0)|(1<<1)|(1<<3)|(1<<4));
	JL_PORTD->DIR |= (1<<2);
	JL_PORTD->PU |= (1<<1)|(1<<2);
	JL_PORTD->OUT |= (1<<3)|(1<<4);

	reg_wsmask(JL_IOMAP->CON0, 2, 0x1, 'A'-'A');	/* spi0 map A */
	reg_wsmask(JL_IOMAP->CON1, 5, 0x1, 'A'-'A');	/* sfc map A */

	/* SFC init */
	JL_SFC->CON = 0xf00000;
	JL_SFC->CON = 0;

	JL_SFC->BAUD = 16-1;

	JL_SFC->BASE_ADR = 0x10000;

	JL_SFC->CON =
		(0<<25) |	/* read JEDEC ID (command 0x9F) */
		(2<<20) |	/* ? */
		(8<<16) |	/* Dummy bit count */
		(2<<8) |	/* SPI mode */
		(1<<7) |	/* ? */
		(0<<3)		/* DO/DI combine */
	;

	JL_SFCENC->CON = 0;

	/* Enable SFC */
	JL_SPI0->CON &= ~(1<<0);	/* disable SPI0 */
	JL_SFC->CON |= (1<<0);		/* enable SFC */

	/* Enable SFC map */
	JL_DSP->CON &= ~(1<<8);		/* disable SFC map */
	memset((void *)0xf8000, 0x55, 0x4000);	/* clear icache data */
	memset((void *)0xfc000, 0x00, 0x1c00);	/* clear icache tag */
	JL_DSP->CON |= (1<<8);		/* enable SFC map */
}

/*=========================================================================*/

extern void feed_to_dac(int16_t *sdata, int cnt);
extern void modplayerstart(void);

#define FMTX_BUFF_SAMPS		128

static int16_t fm_tx_buffer[2][FMTX_BUFF_SAMPS][2];

void __attribute__((interrupt)) FMTX_IntHandler(void) {
	if (JL_FM->TX_CON1 & (1<<7)) {
		JL_FM->TX_CON1 |= (1<<6);

		int buff = (JL_FM->TX_CON0 >> 4) & 1;

		feed_to_dac((void *)&fm_tx_buffer[buff], FMTX_BUFF_SAMPS);
	}
}


void fmtx_on(void) {
	JL_FM->TX_CON0 |= (1<<7)|(1<<0);
}

void fmtx_off(void) {
	JL_FM->TX_CON0 &= ~((1<<7)|(1<<0));
}

void fmtx_init(void) {
	JL_FM->CON &= ~(1<<14);	/* TX mode / RX disable / ??? */

	JL_FM->TX_BASE_ADR = (uint32_t)fm_tx_buffer;

	JL_FM->TX_CON0 |= (1<<7)|(1<<0);

	JL_FM->TX_MUL = 59;
	JL_FM->TX_PILOT = 166;
	JL_FM->TX_SYN_GAIN = 4096;
	JL_FM->TX_FREQ = 0x15400000;

	/* fill stuff with zero, presumeably */
	JL_FM->TX_LEN = 0;
	for (int i = 160; i >= 0; i--) {
		JL_FM->TX_ADR = i;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);
	}

	/* seems like it writes some waveform there, a sinc?? */
	{
		/* -13 13 -39 24 -451 1945 -451 24 -39 13 -13 */
		/* 16371, 11, 16328, 31, 15875, 2091, 15875, 31, 16328, 11, 16371 */

		/* 14-bit! */
		const int sincthing[11] = {16371, 11, 16328, 31, 15875, 2091, 15875, 31, 16328, 11, 16371}; //{-13, 13, -39, 34, -451, 1945, -451, 24, -39, 13, -13};

		for (int i = 0; i < 11; i++) {
			JL_FM->TX_LEN = sincthing[i];

			JL_FM->TX_ADR = 0x100 + (i * 4);
			usleep(5);
			JL_FM->TX_CON0 |= (1<<5);
			usleep(5);
		}

		#if 0
		/* -13 to 256, 296 */
		JL_FM->TX_LEN = -13;

		JL_FM->TX_ADR = 0x100;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);

		JL_FM->TX_ADR = 0x128;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);

		/* 13 to 260, 292 */
		JL_FM->TX_LEN = 13;

		JL_FM->TX_ADR = 0x104;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);

		JL_FM->TX_ADR = 0x124;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);

		/* -39 to 264, 288 */
		JL_FM->TX_LEN = -39;

		JL_FM->TX_ADR = 0x108;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);

		JL_FM->TX_ADR = 0x120;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);

		/* 24 to 268, 284 */
		JL_FM->TX_LEN = 24;

		JL_FM->TX_ADR = 0x10C;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);

		JL_FM->TX_ADR = 0x11C;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);

		/* -451 to 272, 280 */
		JL_FM->TX_LEN = -451;

		JL_FM->TX_ADR = 0x110;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);

		JL_FM->TX_ADR = 0x118;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);

		/* 1945 to 276 */
		JL_FM->TX_LEN = 1945;

		JL_FM->TX_ADR = 0x114;
		usleep(5);
		JL_FM->TX_CON0 |= (1<<5);
		usleep(5);
		#endif
	}

	usleep(200);

	JL_FM->TX_CON0 |= (1<<0);
	usleep(10);

	JL_FM->TX_CON1 |= (1<<6);
	usleep(10);

	reg_wsmask(JL_FM->TX_CON0, 1, 1, 1);	/* stereo flag */

	JL_FM->TX_CON1 |= (1<<0);
	JL_FM->TX_CON1 &= 0xf;
	JL_FM->TX_CON1 |= 0x8;

	JL_FM->TX_FREQ = 0x15400000;

	JL_FM->TX_LEN = FMTX_BUFF_SAMPS;

	usleep(500);

	JL_FM->TX_CON0 |= (1<<3);
}

void fm_emitter_set_freq(uint16_t freq) {
	/* corresponds to ratios 1/1, 1/2, 1/3, 1/4, 1/5, 1/6, 1/7 and 1/8 */
	static const uint8_t divs[8] = {0x0, 0x4, 0x1, 0x8, 0x2, 0x5, 0x3, 0xC};

	char div = (4800. / freq) + .5;
	if (div > 8) div = 8;

	int fvco = freq * 100 * div;

	int fint = (fvco / 1000) / 24. - 2;
	int ffrac = ((fvco / 24. / 1000.) - fint - 2) * 0x1000000;

	xprintf("Divider: %d\nFVco: %d\n\nInt %02x, frac %06x\n\n", div, fvco, fint, ffrac);

	#if 1	/* The clock will become the one from the synth PLL ... */
	JL_CLOCK->PLL_CON1 &= ~(1<<31);

	JL_CLOCK->PLL_CON |= (1<<11);		/* DSMS - 0:CIFF/1:MASH111 */

	reg_wsmask(JL_CLOCK->PLL_CON1, 22, 0x3, 0x1);

	JL_ANA->WLA_CON19 |= (1<<21);

	reg_wsmask(JL_CLOCK->CLK_CON2, 2, 0xf, divs[div-1]);	/* <<<div_arr[%12 - 1] {0418253C} */
	reg_wsmask(JL_CLOCK->CLK_CON2, 0, 0x3, 0x3);

	reg_wsmask(JL_CLOCK->PLL_CON, 24, 0x7, 0x1);	/* ~ICPS */
	reg_wsmask(JL_CLOCK->PLL_CON, 27, 0x7, 0x3);	/* ~LPFR2 */

	JL_CLOCK->PLL_CON |= (1<<10);		/* ?? test? */

	JL_CLOCK->PLL_CON1 |= (1<<19);	/* ?? MSEL */

	JL_CLOCK->PLL_CON1 |= (1<<31);		/* ???? FM TX ownership! */

	JL_CLOCK->PLL_CON |= (1<<0);	/* enable pll */
	usleep(150);
	JL_CLOCK->PLL_CON |= (1<<1);	/* release pll reset */
	#endif

	//JL_CLOCK->CLK_CON0 &= ~(1<<8);		/* don't use this pll clock! */
	//JL_CLOCK->SYS_DIV = ((3-1)<<0) | ((4-1)<<8);	/* hsb div, lsb div */

	// 0 = 192 MHz    - f/2.5
	// 1 = 137 MHz    - f/3.5
	// 2 = 160 MHz    - f/3
	// 3 = 480 MHz    - f/1

	/* WHY?? */
	reg_wsmask(JL_CLOCK->CLK_CON2, 2, 0xf, divs[div-1]);
	reg_wsmask(JL_CLOCK->CLK_CON2, 0, 0x3, 0x3);		/* 0:pll_192m, 1:pll_137m, 2:pll_160m, 3:pll_480m */

	/* (%18 << 24) | %26 */
	JL_FM->TX_FREQ = (fint << 24) | ffrac;
}

void fmtx_analog_init(void) {
	JL_ANA->WLA_CON18 |= (1<<0);
	JL_ANA->WLA_CON21 &= ~(1<<11);
}

void fm_emitter_init(void) {
	irq_attach(47, FMTX_IntHandler, 0);

	reg_wsmask(JL_CLOCK->CLK_CON1, 24, 0x3, 0x1);	/* fm synth clock ... 0:fm_fb_clk, 1:osc_clk, 2:lsb_clk, 3:dis */

	JL_FM->TX_CON1 |= (1<<3);

	fmtx_init();

	#if 0	/* TX port: PB1 */
	reg_wsmask(JL_CLOCK->CLK_CON1, 12, 0x3, 0x1);
	JL_PORTB->DIR &= ~(1<<1);	/* PB1 out */
	#else	/* TX port: PB3 */
	reg_wsmask(JL_CLOCK->CLK_CON1, 12, 0x3, 0x2);
	JL_PORTB->DIR &= ~(1<<3);	/* PB3 out */
	JL_PORTB->HD |= (1<<3);		/* PB3 highdrive */
	JL_PORTB->HD0 |= (1<<3);	/* PB3 highdrive */
	usleep(50);
	#endif
}

/*=========================================================================*/

void JieLi(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
	reg_wsmask(JL_CLOCK->CLK_CON1, 10, 0x3, 0x0);	/* uart_clk = 0:pll_48m / 1:osc_clk / 2:lsb_clk */

	/* UART0 at PB5(TX/RX) */
	JL_UART0->CON0 = 1;
	JL_UART0->BAUD = (48000000 / 4 / 115200) - 1;

	reg_wsmask(JL_IOMAP->CON0, 3, 0x3, 'C'-'A');
	reg_wsmask(JL_IOMAP->CON3, 0, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<3);

	JL_PORTB->DIR &= ~(1<<5);
	JL_PORTB->DIE |= (1<<5);
	JL_PORTB->PU |= (1<<5);

	xdev_out(uputc);
	xputs("\n\n\e[1;33mhello br25\e[0m\n\n");

	/*---------------------------------------------------*/

	p33_xfer(0, P33_OP_WRITE, 0x80, 0x00);

	init_sfc();

	fm_emitter_init();
	fm_emitter_set_freq(880);

	JL_UART0->BAUD = (44000000 / 4 / 115200) - 1;

	xputs("hai????\nhai????\nhai????\n");

	modplayerstart();
}
