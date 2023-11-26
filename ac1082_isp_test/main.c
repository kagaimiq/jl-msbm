#include "ac109n.h"

char _sdcc_external_startup(void) {
	/* do not do anything on init! */
	return 1;
}

/*
   rc_clk   = ~260 kHz
   osc_clk  = X32KI/X32KO :: 32768 Hz / 1..12 MHz
   htc_clk  = ~5 MHz
   hosc_clk = P00/P01 :: 1..12 MHz

CLK_CON0:
	b0 = RC osc enable
	b1 = HOSC enable
	b2 = PLL enable
	b3 = osc_clk source:
		0 = OSC on X32KI/X32KO
		1 = HOSC on P00/P01
	b4-b5 = pll reference:
		00 = DPLL (262.144 kHz)
		01 = HTC (~5 MHz)
		10 = OSC (32768 Hz / 1..12 MHz)
		11 = HOSC (1..12 MHz)
	b6 = IO reset system enable
		0 = Disabled
		1 = If P35/P07 is low for 4 seconds then the system resetes
	b7 = IO reset system selection
		0 = P35
		1 = P07

CLK_CON1:
	b0-b1 = main clock select:
		00 = osc_clk
		01 = rc_clk
		10 = htc_clk
		11 = pll_clk
	b2 = multiply clock by 2
	b3 = main clock switch
		0 = runs on RC clock
		1 = runs on something else
	b4 = decoder clock enable (active low)
	b5 = OTP clock enable (active low)
	b6 = OTP clock divide by 2

CLK_CON2:
	b0 = DAC clock selection:
		0 = pll_clk/2
		1 = osc_clk
	b1 = LCD clock selection:
		0 = osc_clk
		1 = wclk
	b2-b4 = P05 output
		0xx = GPIO
		100 = osc_clk
		101 = sys_clk
		110 = htc_clk
		111 - pll_clk
	b5 = DPLL track prohibit
		0 = allow tracking of 32768 Hz clock
		1 = deny tracking of 32768 Hz clock, DPLL is in free-running state
	b6 = CPU breakpoint enable (active low)

CLK_CON3:
	b0-b7 = system clock divider (n+1)

CLK_GAT:
	b0 = disable TIMER/IRDA
	b1 = disable SD
	b2 = disable IIC/SPI/UART
	b3 = disable ISP
	b4 = disable USB
	b5 = disable LCD
	b6 = disable DAC
	b7 = disable DPLL

PLL_CON2:
	b0-b7 = NF[9:2]

PLL_CON1:
	b6-b7 = NF[1:0]
	b0-b5 = NR[13:8]

PLL_CON0:
	b0-b7 = NR[7:0]
*/


void clk_init(void) {
#if 0
	CLK_CON0 &= ~(1<<1);	/* Disable HOSC */
	CLK_CON0 &= ~(1<<3);	/* osc_clk is from OSC */
#else
	CLK_CON0 |= (1<<1);	/* Enable HOSC */
	CLK_CON0 |= (1<<3);	/* osc_clk is from HOSC */
#endif

#if 0
	/* wa */

	for (char i = 255; i; i--);

	pll_init();

	CLK_CON0 = (CLK_CON0 & ~0x30) | 0x30;	/* PLL clock source */
	CLK_CON0 |= (1<<2);	/* Enable PLL */

	for (char i = 255; i; i--);
#endif

	CLK_CON1 &= ~(1<<2);	/* do not multiply by 2 */
	CLK_CON3 = 0;	/* system clock div1 */

	CLK_CON1 = (CLK_CON1 & ~0x03) | 0x00;	/* use osc_clk as the system clock */
	CLK_CON1 |= (1<<3);	/* run on something else */

#if 0
	CLK_CON1 = (CLK_CON1 & ~0x03) | 0x03;	/* use pll_clk as the system clock */
	CLK_CON1 |= (1<<3);	/* run on something else */

	for (char i = 255; i; i--);

	CLK_CON2 = (CLK_CON2 & 0xE3) | 0x1C;
#endif
}



void uart_init(char portmap) {
	if (portmap == 0) {
		/* P06 / P07 = TX/RX */
		P0PU &= ~0xC0;
		P0PD &= ~0xC0;
		P0DIR = (P0DIR & ~0x40) | 0x80;
	} else
	if (portmap == 1) {
		/* P24 / P25 = TX/RX */
		P2PU &= ~0x30;
		P2PD &= ~0x30;
		P2DIR = (P2DIR & ~0x10) | 0x20;
	} else
	if (portmap == 2) {
		/* P32 / P33 = TX/RX */
		P3PU &= ~0x0C;
		P3PD &= ~0x0C;
		P3DIR = (P3DIR & ~0x04) | 0x08;
	} else
	if (portmap == 3) {
		/* P36 (USBDP) / P37 (USBDM) = TX/RX */
		P3PU &= ~0xC0;
		P3PD &= ~0xC0;
		P3DIR = (P3DIR & ~0x40) | 0x80;
	}

	IO_MC0 = (IO_MC0 & ~0xC0) | (portmap << 6);

	// 260 kHz
	int baud = (4000000 / 8 / 38400) - 1;
	UART_BAUD = baud >> 8;
	UART_BAUD = baud;
	UART_CON = 0x01;
}


void putc(char c) {
	UART_BUF = c;
	while (!(UART_STA & 0x80));
}

void puthex(char h) {
	if (h < 10)
		putc('0' + h);
	else
		putc('A' + h - 10);
}

void puthex8(char h) {
	puthex(h >> 4);
	puthex(h & 0xf);
}

void puthex16(unsigned int h) {
	puthex(h >> 12);
	puthex((h >> 8) & 0xf);
	puthex((h >> 4) & 0xf);
	puthex(h & 0xf);
}


void main(void) {
	//MODE_CON = 0;	/* Work normally (don't use clock from ISP_CLK) */

	//clk_init();
	uart_init(1);

	for (char i = 0; i < 100; i++) putc(0x7e);
	putc('\n');

	puthex16(0xdead);
	puthex16(0xbeef);
	puthex16(0x1234);
	putc('\n');

	SPH = 0x1E;
	SP = 0xFF;

	BANK_SEL |= 0x10;
	BANK_SEL |= 0x04;

	CRC_REGH = 0x7E;
	CRC_REGL = 0xA4;
	OTP_CMD0 |= 0x40;

	#if 1
	for (long i = 0xb000; i <= 0xbfff; ) {
		//puthex16(i);
		//putc(':');

		for (char j = 0; j < 16; j++) {
			putc(' ');
			puthex8(*(char __code *)(i++));
		}

		putc('\n');
	}
	#endif

	P4DIR &= ~(1<<6);

	for (;;) {
		P4 ^= (1<<6);

		if (UART_STA & 0x40) {
			char c = UART_BUF;
			UART_STA = 0x10;
			putc(c);
		}

		for (int i = 10000; i; i--);
	}
}
