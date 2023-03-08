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




void dbi_send_cmd(uint8_t cmd) {
	printf("dbi: cmd %02x\n", cmd);

	reg32_wsmask(PORTB_base+PORTx_OUTn(4), 0); // cmd
	reg32_wsmask(SPI2_base+SPIx_CON_dir, 0); // tx
	reg32_write(SPI2_base+SPIx_BUF, cmd);
	while (!reg32_rsmask(SPI2_base+SPIx_CON_pnd));
	reg32_wsmask(SPI2_base+SPIx_CON_pclr, 1);
}

void dbi_send_data(uint8_t data) {
	printf("dbi: data %02x\n", data);

	reg32_wsmask(PORTB_base+PORTx_OUTn(4), 1); // dat
	reg32_wsmask(SPI2_base+SPIx_CON_dir, 0); // tx
	reg32_write(SPI2_base+SPIx_BUF, data);
	while (!reg32_rsmask(SPI2_base+SPIx_CON_pnd));
	reg32_wsmask(SPI2_base+SPIx_CON_pclr, 1);
}

void dbi_send_data_buff(const void *ptr, int len) {
	printf("dbi: data @%p, %d\n", ptr, len);

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




void SFC_init(void) {
	reg32_wsmask(PORTD_base+PORTx_DIRn(0), 0); // PD0 out  -> SCK
	reg32_wsmask(PORTD_base+PORTx_DIRn(1), 0); // PD1 out  -> MOSI
	reg32_wsmask(PORTD_base+PORTx_DIRn(2), 1); // PD2 in   -> MISO
	reg32_wsmask(PORTD_base+PORTx_DIRn(3), 0); // PD3 out  -> CS
	reg32_wsmask(PORTD_base+PORTx_DIRn(4), 0); // PD4 out  -> ?? HOLD? Power?!

	reg32_wsmask(PORTD_base+PORTx_PUn(1), 1); // PD1 pullup
	reg32_wsmask(PORTD_base+PORTx_PUn(2), 1); // PD2 pullup

	reg32_wsmask(PORTD_base+PORTx_OUTn(3), 1); // PD3 high
	reg32_wsmask(PORTD_base+PORTx_OUTn(4), 1); // PD4 high

	reg32_wsmask(IOMAP_base+IOMAP_CON0_spi0ios, 0x0); // SPI0 on PD3/PD2/PD1/PD0
	reg32_wsmask(IOMAP_base+IOMAP_CON1_sfcios, 0x0); // SFC on PD3/PD2/PD1/PD0

	reg32_write(SFC_base+SFC_CON, 0xf00000);
	reg32_write(SFC_base+SFC_CON, 0);

	reg32_write(SFC_base+SFC_BAUD, 1-1);

	/*
	 * mask -> 6ff0fbd : 0110'1111'1111'0000'1111'1011'1101
	 *         0000034   0000'0000'0000'0000'0000'0011'0100
	 *
	 * write ->6ff0f89   0110'1111'1111'0000'1111'1000'1001
	 *
	 *          200088   0000'0010'0000'0000'0000'1000'1000
	 *
	 * b0 = enable
	 * b3 = combine MOSI and MISO
	 * b7 = 
	 * b8~b11 = 
	 * b16~b19 = dummy bits count
	 * b20~b23 = 
	 * b25 = if set, then the JEDEC ID is readed out (cmd 9F)
	 *
	 * 00000011 Aaaaaaaa:aaaaaaaa:aaaaaaaa          <Data>
	 * 00001011 Aaaaaaaa:aaaaaaaa:aaaaaaaa XXXXXXXX <Data>
	 * 00111011 Aaaaaaaa:aaaaaaaa:aaaaaaaa XXXXXXXX <Dual data>
	 * 01101011 Aaaaaaaa:aaaaaaaa:aaaaaaaa XXXXXXXX <Quad data>
	 * 10111011 AA.aa.aa.aa:aa.aa.aa.aa:... XX.XX.XX.XX <Dual data>
	 * 11101011 AAAA.aaaa:aaaa.aaaa:.... XXXX.XXXX.XXXX.XXXX.XXXX.XXXX <Quad data>
	 * xxxxyzzz
	 * 
	 *
	 * SINGLE IO
	 * 0x00200088 <== [r2 != 4] && [r1 != 8]  >>>> cmd 03 - read
	 * 0x00280188 <== [r2 != 4] && [r1 == 8]  >>>> cmd 0B - read fast
	 *
	 * DUAL IO
	 * 0x00200080 <== [r2 == 2] && [r1 != 8]  >>>> cmd 03 - read
	 * 0x00280280 <== [r2 == 2] && [r1 == 4]  >>>> cmd 3B - read dual out
	 * 0x00240480 <== [r2 == 2] && [r1 == 8]  >>>> cmd BB - read dual io
	 * 0x00240680 <== [r2 == 2] && [r1 == 12] >>>> 
	 *
	 * QUAD IO
	 * 0x00200080 <== [r2 == 4] && [r1 != 8]  >>>> cmd 03 - read
	 * 0x00280380 <== [r2 == 4] && [r1 == 4]  >>>> cmd 6B - read quad out
	 * 0x00260580 <== [r2 == 4] && [r1 == 8]  >>>> cmd EB - read quad io
	 * 0x00260780 <== [r2 == 4] && [r1 == 12] >>>> 
	 */

	reg32_wsmask(SFC_base+SFC_CON, 0, 0x6ff0f88, 0x280280);

	reg32_write(SFC_base+SFC_BASE_ADR, 0);

	reg32_write(SFCENC_base+SFCENC_CON, 0); //b0 = enable, b1=?
	reg32_write(SFCENC_base+SFCENC_KEY, 0xdead);
	//reg32_write(SFCENC_base+SFCENC_UNENC_ADRH, 0);
	//reg32_write(SFCENC_base+SFCENC_UNENC_ADRL, 0);
	//reg32_write(SFCENC_base+SFCENC_LENC_ADRH,  0);
	//reg32_write(SFCENC_base+SFCENC_LENC_ADRL,  0);
	//reg32_wsmask(SFCENC_base+SFCENC_CON, 1, 1, 0);
	reg32_wsmask(SFCENC_base+SFCENC_CON_enable, 1);

	reg32_wsmask(DSP_base+DSP_CON, 8, 1, 0); // disable sfc map
	memset((void *)0xfc000, 0x00, 0x1c00);   // clear sfc cache tags
	reg32_wsmask(DSP_base+DSP_CON, 8, 1, 1); // enable sfc map

	reg32_wsmask(SPI0_base+SPIx_CON_spie, 0); // disable spi
	reg32_wsmask(SFC_base+SFC_CON_enable, 1); // enable sfc
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

	SFC_init();
	Disp_init();

	/* Draw a pic! */
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

		// 0x01000 - ts1      160x80
		// 0x08000 - kgb      80x160
		// 0x0f000 - tkr      160x80
		// 0x16000 - x        80x160
		// 0x1d000 - arb      80x160
		// 0x24000 - tsm      142x80
		// 0x2a000 - tsj      80x160
		// 0x31000 - ts2      160x80
		// 0x38000 - tem      128x64
		// 0x3d000 - may      160x80
		// 0x44000 - ss       128x64
		// 0x49000 - dpc      160x80
		// 0x50000 - tsx      160x80

		dbi_send_cmd(0x2C);
		dbi_send_data_buff(NULL, 160*80*2);
		dbi_send_data_buff((void *)(0x1000000 + 0x2a000 + 8), 160*80*2);
	}
	reg32_wsmask(PORTC_base+PORTx_OUTn(5), 1); // PC5 (CS) hi

	puts("all done");

	//--------------------------------------------------------------------//

	wallclk_deinit();

	p33_tx_1byte(0x80, 0x1c); // enable wdt back! (5 sec)
}
