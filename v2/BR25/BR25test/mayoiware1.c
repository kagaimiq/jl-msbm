#include <stdint.h>
#include <stdio.h>
#include <xprintf.h>
#include <jl_regs.h>
#include <jl_irq.h>
#include <wallclk.h>
#include <maskrom_stuff.h>

void uputc(int c) {
	reg32_write(UART0_base+UARTx_BUF, c);
	while (!reg32_rsmask(UART0_base+UARTx_CON0_tpnd));
	reg32_wsmask(UART0_base+UARTx_CON0_clrtpnd, 1);
}


/* probably messy but it works... */
void lputc(int c) {
	static char newline = 1;
	static uint32_t ts = 0;

	if (ts == 0)
		ts = micros();

	if (newline) {
		xprintf("[%5u.%06u] ", ts / 1000000, ts % 1000000);
		newline = 0; ts = 0;
	}

	if (c == '\n')
		newline = 1;

	xputc(c);
}


/*===========================================================*/

void dbi_send_cmd(uint8_t cmd) {
	//printf("dbi: cmd %02x\n", cmd);

	reg32_wsmask(PORTB_base+PORTx_OUTn(4), 0); // cmd
	reg32_wsmask(SPI2_base+SPIx_CON_dir, 0); // tx
	reg32_write(SPI2_base+SPIx_BUF, cmd);
	while (!reg32_rsmask(SPI2_base+SPIx_CON_pnd));
	reg32_wsmask(SPI2_base+SPIx_CON_pclr, 1);
}

void dbi_send_data(uint8_t data) {
	//printf("dbi: data %02x\n", data);

	reg32_wsmask(PORTB_base+PORTx_OUTn(4), 1); // dat
	reg32_wsmask(SPI2_base+SPIx_CON_dir, 0); // tx
	reg32_write(SPI2_base+SPIx_BUF, data);
	while (!reg32_rsmask(SPI2_base+SPIx_CON_pnd));
	reg32_wsmask(SPI2_base+SPIx_CON_pclr, 1);
}

void dbi_send_data_buff(const void *ptr, int len) {
	//printf("dbi: data @%p, %d\n", ptr, len);

	reg32_wsmask(PORTB_base+PORTx_OUTn(4), 1); // dat
	reg32_wsmask(SPI2_base+SPIx_CON_dir, 0); // tx
	reg32_write(SPI2_base+SPIx_ADR, (uint32_t)ptr);
	reg32_write(SPI2_base+SPIx_CNT, len); // trigger DMA
	while (!reg32_rsmask(SPI2_base+SPIx_CON_pnd));
	reg32_wsmask(SPI2_base+SPIx_CON_pclr, 1);
}

void dbi_setwindow(int x, int y, int w, int h) {
	w = x + w - 1;
	h = y + h - 1;

	dbi_send_cmd(0x2A);
		dbi_send_data(x>>8);
		dbi_send_data(x>>0);
		dbi_send_data(w>>8);
		dbi_send_data(w>>0);

	dbi_send_cmd(0x2B);
		dbi_send_data(y>>8);
		dbi_send_data(y>>0);
		dbi_send_data(h>>8);
		dbi_send_data(h>>0);
}

void Disp_init(void) {
	reg32_wsmask(PORTB_base+PORTx_DIRn(4), 0); // PB4 out = RS
	reg32_wsmask(PORTB_base+PORTx_DIRn(6), 0); // PB6 out = SCL
	reg32_wsmask(PORTB_base+PORTx_DIRn(7), 1); // PB7 out = SDA
	reg32_wsmask(PORTC_base+PORTx_DIRn(4), 0); // PC4 out = RES
	reg32_wsmask(PORTC_base+PORTx_DIRn(5), 0); // PC5 out = CS

	reg32_wsmask(PORTB_base+PORTx_OUTn(4), 0); // PB4 (RS) low
	reg32_wsmask(PORTC_base+PORTx_OUTn(4), 0); // PC4 (RES) low
	reg32_wsmask(PORTC_base+PORTx_OUTn(5), 1); // PC5 (CS) hi

	reg32_wsmask(IOMAP_base+IOMAP_CON1_spi2ios, 'A'-'A'); // SPI2 - DO=PB7, CLK=PB6, DI=PB5

	reg32_write(SPI2_base+SPIx_CON, REG_SMASK(SPIx_CON_pclr)|REG_SMASK(SPIx_CON_ue));
	reg32_write(SPI2_base+SPIx_BAUD, 1-1);
	reg32_wsmask(SPI2_base+SPIx_CON_spie, 1);

	delay(50);
	reg32_wsmask(PORTC_base+PORTx_OUTn(4), 1); // PC4 (RES) hi

	/* Send init commands */ 
	reg32_wsmask(PORTC_base+PORTx_OUTn(5), 0); // PC5 (CS) low
	{
		const uint8_t initcmds[] = {
			1, 0x01,			// soft reset
			1, 0x11,			// get out of sleep mode
			1, 0x28,			// turn off display

			1, 0xfe,
			1, 0xef,
			2, 0xb3, 0x2b,
			2, 0xb5, 0x01,
			2, 0xb6, 0x11,
			2, 0xac, 0x0b,
			2, 0xb4, 0x21,
			2, 0xb1, 0xc8,
			2, 0xc0, 0xc3,
			3, 0xc6, 0x1f, 0x00,
			3, 0xf8, 0x80, 0x06,
			3, 0xf3, 0x01, 0x03,
			3, 0xf5, 0x48, 0x90,
			2, 0xb2, 0x0d,
			2, 0xea, 0x65,
			3, 0xeb, 0x75, 0x66,
			2, 0xb0, 0x3d,
			2, 0xc2, 0x0d,
			2, 0xc3, 0x4c,
			2, 0xc4, 0x10,
			2, 0xc5, 0x10,
			3, 0xe6, 0x40, 0x27,
			3, 0xe7, 0x60, 0x07,
			2, 0xa3, 0x12,
			15, 0xf0, 0x17, 0x39, 0x1b, 0x4e, 0x95, 0x32, 0x33, 0x00, 0x12, 0x0a, 0x09, 0x12, 0x12, 0x0f,
			15, 0xf1, 0x17, 0x39, 0x1b, 0x4e, 0x65, 0x32, 0x33, 0x00, 0x12, 0x0a, 0x09, 0x0e, 0x0c, 0x0f,
			1, 0xfe,
			1, 0xef,

			1, 0x13,			// set normal display (partial mode off)
			1, 0x29,			// turn on display

			0,
		};

		for (const uint8_t *cmd = initcmds; *cmd; ) {
			uint8_t clen = *cmd++;

			dbi_send_cmd(cmd[0]);
			if (cmd[0] == 0x01)
				delay(150);

			for (int i = 1; i < clen; i++)
				dbi_send_data(cmd[i]);

			cmd += clen;
		}
	}
	reg32_wsmask(PORTC_base+PORTx_OUTn(5), 1); // PC5 (CS) hi
}

/*===========================================================*/

//--------- fm_rf ----------//

void fm_set_freq(int freq, char v1, char v2) { // freq is in kHz
	switch (v1) {
	case 0: /* IF 2.143 MHz */
		reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1, 22, 0x3, 0x1);

		reg32_wsmask(FM_base+FM_ADC_CON, 2, 0x3, 0x0);
		usleep(10);

		reg32_wsmask(FM_base+FM_ADC_CON, 2, 1, 1);
		usleep(10);

		freq -= 2143;
		break;

	case 1: /* IF 1.875 MHz */
		reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1, 22, 0x3, 0x1);

		reg32_wsmask(FM_base+FM_ADC_CON, 2, 0x3, 0x0);
		usleep(10);

		reg32_wsmask(FM_base+FM_ADC_CON, 2, 1, 1);
		reg32_wsmask(FM_base+FM_ADC_CON, 3, 1, 1);
		usleep(10);

		freq -= 1875;
		break;

	case 2: /* IF 1.5 MHz */
		reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1, 22, 0x3, 0x0);

		reg32_wsmask(FM_base+FM_ADC_CON, 2, 0x3, 0x0);
		usleep(10);

		reg32_wsmask(FM_base+FM_ADC_CON, 3, 1, 1);
		usleep(10);

		freq -= 1500;
		break;
	}

	switch (v2) {
	case 0: /* none */
		break;
	case 1: /* a */
		reg32_write(ANA_base+ANA_WLA_CON11, 0x0014E6D2);
		reg32_write(ANA_base+ANA_WLA_CON12, 0x00000377);
		break;
	case 2: /* b */
		reg32_write(ANA_base+ANA_WLA_CON11, 0x00146524);
		reg32_write(ANA_base+ANA_WLA_CON12, 0x00000377);
		break;
	}

	//---------------------------------------------------//

	reg32_wsmask(ANA_base+ANA_WLA_CON19, 5, 0x7, 0x0);
	reg32_wsmask(ANA_base+ANA_WLA_CON11, 22, 0x3F, 0x00);

	printf("Freq---%d\n", freq);


	// each band covers 2368 - 2515.2 sads


	if (freq < 74000) {
		puts("band7");
		reg32_wsmask(ANA_base+ANA_WLA_CON19, 5, 0x7, 0x7);
		freq *= 68;
	} else if (freq < 78600) {
		puts("band6");
		reg32_wsmask(ANA_base+ANA_WLA_CON19, 5, 0x7, 0x6);
		freq *= 64;
	} else if (freq < 84100) {
		puts("band5");
		reg32_wsmask(ANA_base+ANA_WLA_CON19, 5, 0x7, 0x5);
		freq *= 60;
	} else if (freq < 90400) {
		puts("band4");
		reg32_wsmask(ANA_base+ANA_WLA_CON19, 5, 0x7, 0x4);
		freq *= 56;
	} else if (freq < 97600) {
		puts("band3");
		reg32_wsmask(ANA_base+ANA_WLA_CON19, 5, 0x7, 0x3);
		freq *= 52;
	} else if (freq < 106100) {
		puts("band2");
		reg32_wsmask(ANA_base+ANA_WLA_CON19, 5, 0x7, 0x2);
		freq *= 48;
	} else if (freq < 115000) {
		puts("band1");
		reg32_wsmask(ANA_base+ANA_WLA_CON19, 5, 0x7, 0x1);
		freq *= 44;
	} else {
		// Out of range ... 74-115 MHz essentially
	}

	freq /= 2000;
	freq += 239;

	printf("Freq---%02x\n", freq & 0xff);
	// get_bta_pll_bank(freq & 0xff) & 0x7f;

	// WHATETA
	reg32_wsmask(ANA_base+ANA_WLA_CON11, 22, 0x7f, 0);

	if (v2 == 1 || v2 == 2) {
		
	}
}

void fm_rf_init(void) {
	puts("FM RF init...");

	// BT PLL!?!??! // bt_pll_para(24000000, 24000000, 0, 0);

	reg32_write(ANA_base+ANA_WLA_CON20, 0);
	reg32_write(ANA_base+ANA_WLA_CON21, 0);

	reg32_wsmask(ANA_base+ANA_WLA_CON21, 7, 1, 1);

	reg32_write(ANA_base+ANA_WLA_CON0, 0x001503E5);
	reg32_write(ANA_base+ANA_WLA_CON1, 0x0C000000);
	reg32_write(ANA_base+ANA_WLA_CON2, 0x0E413E00);
	reg32_write(ANA_base+ANA_WLA_CON3, 0x7432E961);
	reg32_write(ANA_base+ANA_WLA_CON4, 0x00000029);
	reg32_write(ANA_base+ANA_WLA_CON5, 0x08B4038C);
	reg32_write(ANA_base+ANA_WLA_CON6, 0x15FFD5E7);
	reg32_write(ANA_base+ANA_WLA_CON7, 0x01000000);

	reg32_wsmask(ANA_base+ANA_WLA_CON9, 0, 0x0A5800E0, ~0);

	reg32_write(ANA_base+ANA_WLA_CON10, 0x06B4010C);
	reg32_write(ANA_base+ANA_WLA_CON13, 0x04000000);
	reg32_write(ANA_base+ANA_WLA_CON18, 0x0FF58000);
	reg32_write(ANA_base+ANA_WLA_CON19, 0x00000129);

	// BTA PLL BANK SCAN 2414 ???!!!!>!>!>  //         \|/ = bta_pll_bank_scan(2414) & 0x7f;
	reg32_write(ANA_base+ANA_WLA_CON11, 0x00146524 | (0x00<<22));

	reg32_write(ANA_base+ANA_WLA_CON12, 0x00003377);

	reg32_wsmask(ANA_base+ANA_WLA_CON13, 0, 0xFFFFFF, 0x19BFD4);

	reg32_wsmask(ANA_base+ANA_WLA_CON9, 5, 1, 1);
	usleep(150);

	reg32_wsmask(ANA_base+ANA_WLA_CON9, 7, 1, 1);
	// clk_set_default_osc_cap();

	puts("FM RF init don!");
}

//--------- fm_init --------//

static int32_t fm_adc_buff[2][512];

void fm_hw_init(void) {
	puts("FM HW init...");

	reg32_write(FM_base+FM_CON, 0x4001);
	usleep(10);

	reg32_wsmask(FM_base+FM_CON, 3, 1, 1);
	reg32_wsmask(FM_base+FM_CON, 0, 1, 1);
	usleep(10);

	reg32_wsmask(FM_base+FM_ADC_CON1, 31, 1, 1);

	// .. fm_hf_cram ???! .. //
	int32_t cs75_coeff[16] = {
		-0x000B0006, //-   720902,
		 0x001EFFF9, //   2031609,

		 0x00F30073, //  15925363,
		 0x01A70171, //  27722097,

		 0x00820158, //   8520024,
		-0x013C0088, //- 20709512,

		-0x00330124, //-  3342628,
		 0x01C1010C, //  29425932,

		 0x0044FEB8, //-  4521656,
		-0x028E01F9, //- 42861049,

		 0x0152FEB5, //  22216373,
		 0x03B6039F, //  62260127,

		-0x03C9FF2E, //- 63569710,
		-0x05DD071C, //- 98371356,

		 0x0D84017E, // 226754942,
		 0x21F81A06, // 569907718
	};

	for (int i = 0; i < 16; i++) {
		reg32_write(FM_base+FM_HF_CRAM, cs75_coeff[i]);
		usleep(2);
	}

	reg32_write(FM_base+FM_ADC_CON, 0x1a);
	usleep(10);

	reg32_write(FM_base+FM_ADC_CON1, 0x7FF80602);
	usleep(10);

	reg32_write(FM_base+FM_BASE, (uint32_t)fm_adc_buff);
	usleep(10);

	reg32_write(FM_base+FM_HF_CON1, 1);
	usleep(10);

	reg32_wsmask(FM_base+FM_HF_CON1, 13, 1, 1);
	usleep(10);

	reg32_wsmask(FM_base+FM_ADC_CON, 0, 1, 1);

	puts("FM HW init don!");
}

#if 0 // not used??
void fm_lofc_init(void) {
	puts("FM LOFC init...");

	reg32_write(FM_base+FM_LF_CON, 0);

	reg32_wsmask(FM_base+FM_LF_CON, 0, 1, 1);
	usleep(100);

	reg32_wsmask(FM_base+FM_LF_CON, 2, 1, 1);
	usleep(100);

	reg32_wsmask(FM_base+FM_LF_CON, 8, 0xFF, 0x3C);
	usleep(50);

	puts("FM LOFC init don!");
}
#endif

void IRQ_HANDLER fmhw_isr(void) {
	reg32_wsmask(FM_base+FM_HF_CON1, 6, 1, 1);
	reg32_wsmask(FM_base+FM_HF_CON1, 6, 1, 1);
	int slice = reg32_rsmask(FM_base+FM_HF_CON1, 5, 1);

	//printf("!! FM IRQ; slice ->%d ==> %08x\n", slice, fm_adc_buff[slice][0]);

	//printf("%08x %08x\n", fm_adc_buff[slice][0], fm_adc_buff[slice][1]);
}

void fm_init(void) {
	puts("FM radio init...");

	reg32_wsmask(CLOCK_base+CLOCK_PLL_CON1, 24, 1, 1);

	irq_attach(43, fmhw_isr);

	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1, 22, 0x3, 0x0);

	//fm_dem_init(); // pure software
	fm_rf_init();
	fm_hw_init();

	reg32_wsmask(FM_base+FM_CON, 4, 1, 1);
	usleep(10);

	reg32_wsmask(FM_base+FM_CON, 0, 1, 1);
	usleep(10);

	reg32_wsmask(FM_base+FM_ADC_CON, 4, 1, 1);
	usleep(10);

	//fm_agc();

	// ... audio dac configs?? //

	puts("FM radio init done!");
}

void fm_close(void) {
	puts("FM radio close...");

	reg32_write(ANA_base+ANA_WLA_CON18, 0);
	usleep(10);

	reg32_write(ANA_base+ANA_WLA_CON19, 0);
	usleep(10);

	reg32_wsmask(ANA_base+ANA_WLA_CON18, 14, 1, 1);
	usleep(10);

	reg32_write(FM_base+FM_ADC_CON1, 0);
	usleep(10);

	reg32_write(FM_base+FM_ADC_CON, 0);
	usleep(10);

	reg32_wsmask(FM_base+FM_CON, 0, 1, 0);
	reg32_wsmask(FM_base+FM_CON, 3, 1, 0);
	usleep(10);

	// ... audio dac configs?? //

	irq_detach(43);

	puts("FM radio close done!");
}

/*===========================================================*/




void JieLi(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1, 10, 0x3, 0x0); // uart_clk <- pll_48m?

	// init UART0 on PB5
	reg32_write(UART0_base+UARTx_CON0, 1); // 8n1, en
	reg32_write(UART0_base+UARTx_BAUD, (48000000 / 4 / 921600) - 1);
	reg32_wsmask(IOMAP_base+IOMAP_CON0_ut0ios, 0x2); // UART0 to PB5
	reg32_wsmask(IOMAP_base+IOMAP_CON3_ut0mxs, 0x0); // UART0 muxsel -> iomux
	reg32_wsmask(IOMAP_base+IOMAP_CON3_ut0ioen, 1); // UART0 I/O enable
	reg32_wsmask(PORTB_base+PORTx_DIRn(5), 0); // PB5 out

	xdev_out(uputc);
	xputs("\e[H\e[2J\e[3J"); // clear screen
	xputs("\e[1;37;41m==== JieLi AC6965A! "__DATE__" "__TIME__" ====\e[0m\n");
	xprintf("r0: <%08x>  r1: <%08x>  r2: <%08x>  r3: <%08x>\n", r0,r1,r2,r3);

	/*==================================================================*/

	p33_tx_1byte(0x80, 0); // disable wdt!

	wallclk_init();

	*(void **)0x31cdc = lputc; // mask_putchar funcptr

	puts("Mayoi-ware <mayoi katase> 5.0 @ P90215.500Liner(O).BuildKey=00049494992");
	puts("build @ "__DATE__" --- "__TIME__);

	//--------------------------------------------------------------------//

	Disp_init();

	fm_init();

	fm_set_freq(106400, 0, 2);

	//--------------------------------------------------------------------//

#if 0
	wallclk_deinit();

	p33_tx_1byte(0x80, 0x1c); // enable wdt back! (5 sec)
#else
	for (;;) {
		//asm volatile ("idle");

		reg32_wsmask(PORTC_base+PORTx_OUTn(5), 0); // PC5 (CS) low
		{
			dbi_send_cmd(0x3A);
			dbi_send_data(0x05);

			dbi_send_cmd(0x36);
			dbi_send_data(
				(1 << 7)|	// MY    - reverse Y axis direction (memory write)
				(1 << 6)|	// MX    - reverse X axis direction (memory write)
				(0 << 5)|	// MV    - swap X/Y axis (memory write)
				(0 << 3)	// BGR   - swap R/B channels
			);

			dbi_setwindow(24, 0, 80, 160);
			//dbi_setwindow(0, 24, 160, 80);

			dbi_send_cmd(0x2C);

			for (int i = 0; i < 160; i++) {
				uint16_t line[80];

				int v = fm_adc_buff[0][i] / 0x1000;
				if (v < -40) v = -40;
				if (v > 39) v = 39;
				v += 40;

				for (int j = 0; j < 80; j++) {
					if (v == j)
						line[j] = 0x00ff;
					else
						line[j] = 0x0000;
				}

				dbi_send_data_buff(line, sizeof line);
			}
		}
		reg32_wsmask(PORTC_base+PORTx_OUTn(5), 1); // PC5 (CS) hi


	}
#endif
}
