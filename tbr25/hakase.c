#include <jl_br25_regs.h>
#include <jl_irq.h>
#include <xprintf.h>

void uputc(int c) {
	while (!(JL_UART0->CON0 & (1<<15)));
	JL_UART0->BUF = c;
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

	JL_TIMER5->CON = (1<<14);
	JL_TIMER5->CNT = 0;
	JL_TIMER5->PRD = usecs * 48;	/* 48 MHz */
	JL_TIMER5->CON |= (1<<0);
	while (!(JL_TIMER5->CON & (1<<15)));
	JL_TIMER5->CON = (1<<14);
}

void delay(unsigned msecs) {
	while (msecs--) usleep(1000);
}

/*=========================================================================*/

static int32_t fm_rx_buffer[2][512];

void __attribute__((interrupt)) FMRX_IntHandler(void) {
	if (JL_FM->HF_CON1 & (1<<7)) {
		JL_FM->HF_CON1 |= (1<<6);

		int buff = (JL_FM->HF_CON1 >> 5) & 1;
		xprintf("\n-----FMRX (%d)-----\n", buff);

		for (int i = 0; i < 16; i++)
			xprintf(" %d", fm_rx_buffer[buff][i]);

		xputc('\n');
	}
}

/***************************************************\
 * btctrler.a/RF.o                                 *
\***************************************************/

void bt_pll_para(int osc, int sys, char low_power, char xosc) {
	xprintf("...btapll : osc-%d, sys-%d, lowpower-%d, xosc-%d\n",
		osc, sys, low_power, xosc);
}

/***************************************************\
 * btctrler.a/analog.o                             *
\***************************************************/

uint8_t bta_pll_bank_limit_new[54];

uint8_t get_bta_pll_bank(uint8_t val) {
	uint8_t bank = 0;

	xprintf("//// get bta pll bank -> %02x ////\n", val);

	for (int i = 1; i < 53; i++) {
		if (bta_pll_bank_limit_new[i] > val) {
			bank = bta_pll_bank_limit_new[53] + i - 1;
			break;
		}
	}

	xprintf("got: %02x\n", bank);
	return bank;
}

uint8_t bta_pll_bank_scan(int freq) {
	xprintf("Bank Scanner: %d\n", freq);

	JL_ANA->WLA_CON11 = 0xDD527FD;
	JL_ANA->WLA_CON12 = 0xD177;

	JL_ANA->WLA_CON10 |= (1<<5);

	JL_ANA->WLA_CON21 &= ~(0x7<<7);
	JL_ANA->WLA_CON21 |= (1<<7);

	JL_ANA->WLA_CON12 &= ~0xff;
	JL_ANA->WLA_CON12 |= 0x0C;

	/* ... */

	JL_ANA->WLA_CON10 &= ~(1<<5);

	return 0x00;
}

/***************************************************\
 * cpu.a/fm_rf.o                                   *
\***************************************************/

void fm_set_freq(int freq, char fs, char pll_mode) {
	switch (fs) {
	case 2:	/* 1.5 MHz */
		reg_wsmask(JL_CLOCK->CLK_CON1, 22, 0x3, 0x0);

		JL_FM->ADC_CON &= ~(0x3<<2);
		usleep(10);
		JL_FM->ADC_CON |= (0x2<<2);
		usleep(10);

		freq -= 1500;
		break;

	case 1:	/* 1.875 MHz */
		reg_wsmask(JL_CLOCK->CLK_CON1, 22, 0x3, 0x1);

		JL_FM->ADC_CON &= ~(0x3<<2);
		usleep(10);
		JL_FM->ADC_CON |= (0x3<<2);
		usleep(10);

		freq -= 1875;
		break;

	case 0:	/* 2.143 MHz */
		reg_wsmask(JL_CLOCK->CLK_CON1, 22, 0x3, 0x1);

		JL_FM->ADC_CON &= ~(0x3<<2);
		usleep(10);
		JL_FM->ADC_CON |= (0x1<<2);
		usleep(10);

		freq -= 2143;
		break;
	};

	if (pll_mode == 1) {
		JL_ANA->WLA_CON11 = 0x14E6D2;
		JL_ANA->WLA_CON12 = 0x377;
	} else if (pll_mode == 2) {
		JL_ANA->WLA_CON11 = 0x146524;
		JL_ANA->WLA_CON12 = 0x377;
	}

	JL_ANA->WLA_CON19 &= ~(0x7<<5);
	JL_ANA->WLA_CON11 &= ~(0x3F<<22);

	int freq_set = freq;

	if (freq <= 74000) {		/* ...69.4 - 74 MHz */
		JL_ANA->WLA_CON19 |= (0x7<<5);
		freq_set *= 68;
	} else if (freq <= 78600) {	/* 74 - 78.6 MHz */
		JL_ANA->WLA_CON19 |= (0x6<<5);
		freq_set *= 64;
	} else if (freq <= 84100) {	/* 78.6 - 84.1 MHz */
		JL_ANA->WLA_CON19 |= (0x5<<5);
		freq_set *= 60;
	} else if (freq <= 90400) {	/* 84.1 - 90.4 MHz */
		JL_ANA->WLA_CON19 |= (0x4<<5);
		freq_set *= 56;
	} else if (freq <= 97600) {	/* 90.4 - 97.6 MHz */
		JL_ANA->WLA_CON19 |= (0x3<<5);
		freq_set *= 52;
	} else if (freq <= 106100) {	/* 97.6 - 106.1 MHz */
		JL_ANA->WLA_CON19 |= (0x2<<5);
		freq_set *= 48;
	} else if (freq <= 115000) {	/* 106.1 - 115 MHz */
		JL_ANA->WLA_CON19 |= (0x1<<5);
		freq_set *= 44;
	} else {
		/* wrong frequency... [69.4 - 115 MHz] */
		return;
	}

	/* freq is ... [4668400 - 5060000] = 2.3342 - 2.530 GHz ? */

	int fm_vco_freq = freq_set / 2000;

	xprintf("Freq: %d / %d\n", freq_set, fm_vco_freq);

	uint8_t fm_bank = get_bta_pll_bank(239 + fm_vco_freq);    // 239= -17 or -2321?

	JL_ANA->WLA_CON11 |= ((fm_bank & 0x7f) << 22);

	if (pll_mode == 2) {
		/* TODO */
	} else if (pll_mode == 1) {
		/* TODO */
	} else {
		JL_FM->TX_FREQ = 0;
	}

	JL_ANA->WLA_CON9 &= ~(1<<7);
	usleep(50);

	JL_ANA->WLA_CON9 |= (1<<7);
	usleep(1000);
}

void fm_rf_init(void) {
	bt_pll_para(24000000, 24000000, 0, 0);

	JL_ANA->WLA_CON20 = 0;
	JL_ANA->WLA_CON21 = 0;

	JL_ANA->WLA_CON21 |= (1<<7);

	JL_ANA->WLA_CON0 = 0x1503E5;
	JL_ANA->WLA_CON1 = 0xC000000;
	JL_ANA->WLA_CON2 = 0xE413E00;
	JL_ANA->WLA_CON3 = 0x7432E961;
	JL_ANA->WLA_CON4 = 0x29;
	JL_ANA->WLA_CON5 = 0x8B4038C;
	JL_ANA->WLA_CON6 = 0x15FFD5E7;
	JL_ANA->WLA_CON7 = 0x1000000;
	JL_ANA->WLA_CON9 |= 0xA5800E0;
	JL_ANA->WLA_CON10 = 0x6B4010C;
	JL_ANA->WLA_CON13 = 0x4000000;
	JL_ANA->WLA_CON18 = 0xFF58000;
	JL_ANA->WLA_CON19 = 0x129;

	JL_ANA->WLA_CON11 = ((bta_pll_bank_scan(2414) & 0x7f) << 22) | 0x146524;

	JL_ANA->WLA_CON12 = 0x3377;
	JL_ANA->WLA_CON13 |= 0x19BFD4;

	JL_ANA->WLA_CON9 |= (1<<5);
	usleep(150);

	JL_ANA->WLA_CON9 |= (1<<7);

	// clk_set_default_osc_cap();
}

/***************************************************\
 * cpu.a/fm_init.o                                 *
\***************************************************/

/* set_fm_pcm_out_fun */

/* fm_dem_init */

void fm_hw_init(void) {
	JL_FM->CON = (1<<14)|(1<<0);
	usleep(10);

	JL_FM->CON |= (1<<3)|(1<<0);
	usleep(10);

	JL_FM->ADC_CON1 |= (1<<31);

	/* cs75 = China Standard 75 khz (deviation)?? */
	static const int cs75_coeff[] = {
		-  720902,   2031609,   15925363,  27722097,
		  8520024, -20709512, -  3342628,  29425932,
		- 4521656, -42861049,   22216373,  62260127,
		-63569710, -98371356,  226754942, 569907718,
	};

	for (int i = 0; i < 16; i++) {
		JL_FM->HF_CRAM = cs75_coeff[i];
		usleep(2);
	}

	JL_FM->ADC_CON  = 0x0000001A;
	usleep(10);

	JL_FM->ADC_CON1 = 0x8007F9FE;
	usleep(10);

	JL_FM->BASE = (uint32_t)fm_rx_buffer;
	usleep(10);

	JL_FM->HF_CON1 = 1;
	usleep(10);

	JL_FM->HF_CON1 |= (1<<13);
	usleep(10);

	JL_FM->ADC_CON |= (1<<0);
}

void fm_lofc_init(void) {
	JL_FM->LF_CON = 0;

	JL_FM->LF_CON |= (1<<0);
	usleep(100);

	JL_FM->LF_CON |= (1<<2);
	usleep(100);

	reg_wsmask(JL_FM->LF_CON, 4, 0xf, 0x2);
	usleep(100);

	reg_wsmask(JL_FM->LF_CON, 8, 0xff, 0x3C);
	usleep(50);
}

/* fmhw_isr --> FMRX_IntHandler */
/*  @ fm_dem (r0 = adc input data, r1 = fm_param struct, r2 = pcm output) */

/* fm_mem_init */

void fm_init(void) {
	JL_CLOCK->PLL_CON1 |= (1<<24);

	//irq_attach(43, FMRX_IntHandler, 0);

	JL_CLOCK->CLK_CON1 &= ~(0x3<<22);

	// fm_dem_init();
	fm_rf_init();
	fm_hw_init();

	JL_FM->CON |= (1<<4);
	usleep(10);

	JL_FM->CON |= (1<<0);
	usleep(10);

	JL_FM->ADC_CON |= (1<<4);
	usleep(10);

	// fm_agc();

	if (!(JL_AUDIO->DAC_CON & (1<<24))) {
		reg_wsmask(JL_ANA->DAA_CON0, 19, 0x3, 0x1);
		JL_AUDIO->DAC_CON |= (1<<24);
	}
}

void fm_close(void) {
	JL_ANA->WLA_CON18 = 0;
	usleep(10);

	JL_ANA->WLA_CON19 = 0;
	usleep(10);

	JL_ANA->WLA_CON18 |= (1<<15);
	usleep(10);

	JL_FM->ADC_CON1 = 0;
	usleep(10);

	JL_FM->ADC_CON = 0;
	usleep(10);

	JL_FM->CON &= ~0x9;
	usleep(10);

	#if 1	/* fm_mode_dac_clk_sw_flg */
	JL_ANA->DAA_CON0 |= (0x3<<19);
	JL_AUDIO->DAC_CON &= ~(1<<24);
	#endif

	irq_detach(43);
}

/*=========================================================================*/



void JieLi(uint32_t args[4]) {
	reg_wsmask(JL_CLOCK->CLK_CON1, 10, 0x3, 0x0);	/* uart_clk = pll_48m */

	#if 0	/* UART2 at PA3(TX)/PA4(RX) */
	JL_UART2->CON0 = 1;
	JL_UART2->BAUD = (48000000 / 4 / 115200) - 1;

	reg_wsmask(JL_IOMAP->CON1, 14, 0x3, 'A'-'A');
	reg_wsmask(JL_IOMAP->CON3, 8, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<11);

	JL_PORTA->DIR &= ~(1<<3);	/* PA3 out */
	JL_PORTA->DIE |= (1<<4);	/* PA4 digital in en */
	JL_PORTA->PU |= (1<<4);		/* PA4 pullup */
	#endif

	#if 1	/* UART0 at PB5(TX/RX) */
	JL_UART0->CON0 = 1;
	JL_UART0->BAUD = (48000000 / 4 / 1000000) - 1;

	reg_wsmask(JL_IOMAP->CON0, 3, 0x3, 'C'-'A');
	reg_wsmask(JL_IOMAP->CON3, 0, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<3);

	JL_PORTB->DIR &= ~(1<<5);
	JL_PORTB->DIE |= (1<<5);
	JL_PORTB->PU |= (1<<5);
	#endif

	xdev_out(uputc);
	xputs("\n\nhello br25\n");

	/*---------------------------------------------------*/

	xprintf("Chip ID: %08x\n", JL_SYSTEM->CHIP_ID);

	JL_TIMER4->CON = (1<<14)|(2<<2);	/* OSC clk */
	JL_TIMER4->PRD = 16000 * 2;		/* 2x RC clk freq */
	JL_TIMER4->CNT = 0;

	JL_TIMER5->CON = (1<<14)|(3<<2);	/* RC clk */
	JL_TIMER5->PRD = ~0;
	JL_TIMER5->CNT = 0;

	JL_TIMER4->CON |= (1<<0);
	JL_TIMER5->CON |= (1<<0);

	while (!(JL_TIMER4->CON & (1<<15)));

	JL_TIMER4->CON = (1<<14);
	JL_TIMER5->CON = (1<<14);

	xprintf("%d/%d :: osc frequency: ~%d kHz\n",
		JL_TIMER4->CNT, JL_TIMER5->CNT,
		(JL_TIMER4->CNT + 16000 * 2) * 16000 / JL_TIMER5->CNT
	);

	/*---------------*/

	fm_init();
	fm_set_freq(106400, 0, 2);
}
