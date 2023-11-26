#include <jl_br17_regs.h>
#include <jl_irq.h>
#include <jl_audio.h>
#include <xprintf.h>

void uputc(int c) {
	while (!(JL_UART2->CON & (1<<15)));
	JL_UART2->BUF = c;
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

void clk_init(void) {
	/* reset */
	JL_CLOCK->CLK_CON0 = 0x00000001;
	JL_CLOCK->CLK_CON1 = 0x00000000;
	JL_CLOCK->CLK_CON2 = 0x00555015;

	/* pll config */
	JL_CLOCK->PLL_CON =
		(0<<20)|	/* PLL_TEST */
		(0<<17)|	/* PLL_REF_SEL   -> btosc_clk */
		(0<<16)|	/* PLL_RSEL12 */
		(0<<12)|	/* PLL_TSEL */
		(0<<11)|	/* PLL_DSMS */
		(0<<10)|	/* PLL_DIVS */
		(0<<8)|		/* PLL_RSEL */
		(1<<7)|		/* PLL_REFDE */
		(((16/2)-2)<<2)	/* PLL_REFDS */
	;

	/* pll enable */
	JL_CLOCK->PLL_CON |= (1<<0);
	for (volatile int i = 1000; i; i--);

	/* pll reset release */
	JL_CLOCK->PLL_CON |= (1<<1);
	for (volatile int i = 1000; i; i--);

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
}


void init_sfc(void) {
	/*
	 * PD0 = SCK
	 * PD1 = DI / IO0 (mosi)
	 * PD2 = DO / IO1 (miso)
	 * PD3 = CS#
	 */

	/* Init I/O */
	JL_PORTD->DIR &= ~((1<<0)|(1<<1)|(1<<3));
	JL_PORTD->DIR |= (1<<2);
	JL_PORTD->PU |= (1<<1)|(1<<2);
	JL_PORTD->OUT |= (1<<3);

	//reg_wsmask(JL_IOMAP->CON0, 2, 0x1, 'A'-'A');	/* spi0 map A */
	//reg_wsmask(JL_IOMAP->CON1, 5, 0x1, 'A'-'A');	/* sfc map A */

	/* SFC init */
	JL_SFC->CON = 0xf00000;
	JL_SFC->CON = 0;

	JL_SFC->BAUD = 128-1;

	JL_SFC->BASE_ADR = 0x4000;

	JL_SFC->CON =
		(0<<25) |	/* read JEDEC ID (command 0x9F) */
		(2<<20) |	/* ? */
		(8<<16) |	/* Dummy bit count */
		(2<<8) |	/* SPI mode */
		(1<<7) |	/* ? */
		(0<<3)		/* DO/DI combine */
	;

	JL_ENC->CON &= ~(1<<3);

	/* Enable SFC */
	JL_SPI0->CON &= ~(1<<0);	/* disable SPI0 */
	JL_SFC->CON |= (1<<0);		/* enable SFC */

	/* Enable SFC map */
	JL_DSP->CON &= ~(1<<8);		/* disable SFC map */
	memset((void *)0x1a000, 0x55, 0x4000);	/* clear icache data */
	memset((void *)0x48000, 0x00, 0x1800);	/* clear icache tag */
	JL_DSP->CON |= (1<<8);		/* enable SFC map */
}

void usleep(int us) {
	/* TODO */
	for (volatile int i = 100; i; i--);
}

void delay(int ms) {
	while (ms--)
		usleep(1000);
}

/*=========================================================================*/

struct s3m_header {
	char name[28];
	char ch_1A;		/* 0x1A */
	uint8_t type;
	uint8_t res1E[2];
	uint16_t ord_num;
	uint16_t ins_num;
	uint16_t pat_num;
	uint16_t flags;
	uint16_t cwt_ver;
	uint16_t file_fmt_info;
	uint32_t tag;		/* 'SCRM' */
	uint8_t global_vol;
	uint8_t ini_speed;
	uint8_t ini_tempo;
	uint8_t master_vol;
	uint8_t ultraclick;
	uint8_t default_pan;
	uint8_t res36[8];
	uint16_t special;
	uint8_t chansets[32];
};

struct s3m_samplehdr {
	uint8_t type;
	char filename[12];
	uint8_t memseg[3];
	uint32_t length;
	uint32_t loopbeg;
	uint32_t loopend;
	uint8_t volume;
	uint8_t res1D[1];
	uint8_t pack;
	uint8_t flags;
	uint32_t c2spd;
	uint8_t res24[4];
	uint16_t int_gravisptr;
	uint16_t int_512;
	uint32_t int_lastused;
	char name[28];
	uint32_t tag;		/* 'SCRS' */
};

struct s3m_module {
	void *file;

	/* header */
	struct s3m_header *hdr;
	uint8_t *orders;
	uint16_t *ins_pptr;
	uint16_t *pat_pptr;
	uint8_t *def_pan;
};

/*-------------------------------*/

const char *note_names[12] = {"C-", "C#", "D-", "D#", "E-", "F-", "F#", "G-", "G#", "A-", "A#", "B-"};

/*-------------------------------*/

#define CHANNERU_CNT		8

struct channeru {
	int state;
	void *data;
	int pos, len;
	int loop_st, loop_len;
	char volume, pan;
	uint32_t freq, facc;
};

struct channeru dachans[CHANNERU_CNT];

void feed_to_dac(int16_t *sdata, int cnt) {
	for (int s = 0; s < cnt; s++) {
		int16_t mixed = 0;

		for (int ch = 0; ch < CHANNERU_CNT; ch++) {
			struct channeru *chan = &dachans[ch];

			if (chan->state > 0) {
				int8_t *data = chan->data + chan->pos;

				/* TODO: interpolate */
				mixed += data[0] * chan->volume;

				chan->facc += chan->freq;
				while (chan->facc >= 0x10000) {
					chan->facc -= 0x10000;

					chan->pos++;

					if (chan->state == 1) {
						if (chan->pos >= chan->len) {
							if (chan->loop_len > 2) {
								chan->state = 2;
							} else {
								chan->state = 0;
							}
						}
					} else {
						if (chan->pos - chan->loop_st >= chan->loop_len)
							chan->pos = chan->loop_st;
					}
				}
			}
		}

		sdata[s*2+0] = mixed;
		sdata[s*2+1] = mixed;
	}
}

/*-------------------------------*/

struct s3m_module s3mod;



void __attribute__((interrupt)) Timer0_IRQ(void) {
	

	JL_TIMER0->CON |= (1<<14);
	irq_latch_clear(irqn_TMR0);
}

void __attribute__((interrupt)) Timer1_IRQ(void) {
	

	JL_TIMER1->CON |= (1<<14);
	irq_latch_clear(irqn_TMR1);
}




void JieLi(uint32_t args[4]) {
	clk_init();

	reg_wsmask(JL_CLOCK->CLK_CON1, 10, 0x3, 0x1);	/* uart_clk = pll_48m */

	JL_UART2->CON = 1;
	JL_UART2->BAUD = (48000000 / 4 / 921600) - 1;

	reg_wsmask(JL_IOMAP->CON1, 14, 0x3, 'A'-'A');
	reg_wsmask(JL_IOMAP->CON3, 8, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<11);

	JL_PORTA->DIR &= ~(1<<3);	/* PA3 out */

	xdev_out(uputc);
	xputs("\e[H\e[2J\e[3J"); // clear screen
	xputs("\n\nhello br17 "__DATE__" "__TIME__"\n");

	init_sfc();

	if (s3m_parse(&s3mod, (void *)0x1000000))
		return;

	return;

	static struct jl_audio_initdata aud_id = {
		.vcomo_en = 0,
		.ldo_sel = 2,
		.hp_type = 1,
		.vcm_rsel = 0,
		.isel_cfg = 0,
		.dac_cb = feed_to_dac
	};

	jl_audio_init(&aud_id);

	/* mod timer - 125 bpm (50 Hz) */
	JL_TIMER0->CON = (1<<14) | (8<<4);
	JL_TIMER0->CNT = 0;
	JL_TIMER0->PRD = 80000000 / 256 / (125 * 4 / 10) - 1;
	JL_TIMER0->CON |= 1;

	/* display timer - 25 Hz */
	JL_TIMER1->CON = (1<<14) | (8<<4);
	JL_TIMER1->CNT = 0;
	JL_TIMER1->PRD = 80000000 / 256 / 25 - 1;
	JL_TIMER1->CON |= 1;

	irq_attach(irqn_TMR0, Timer0_IRQ, 1);
	irq_attach(irqn_TMR1, Timer1_IRQ, 0);

	/* enable dac! */

	jl_audio_start(JL_AUDIO_SR_44100);

	asm ("sti");

	/* halt */
	for (;;) {
		JL_SYSTEM->WDT_CON |= (1<<6);
		asm ("idle");
	}
}
