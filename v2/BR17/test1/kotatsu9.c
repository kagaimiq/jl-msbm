#include <stdint.h>
#include <stdio.h>
#include <xprintf.h>
#include <jl_regs.h>

void uputc(int c) {
	reg32_write(UART2_base+UART2_BUF, c);
	while (!reg32_rsmask(UART2_base+UARTx_CON0_tpnd));
	reg32_wsmask(UART2_base+UARTx_CON0_clrtpnd, 1);
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



void InitSFC(void) {
	reg32_wsmask(PORTD_base+PORTx_DIRn(0), 0); // PD0 out  -> SCK
	reg32_wsmask(PORTD_base+PORTx_DIRn(1), 0); // PD1 out  -> MOSI
	reg32_wsmask(PORTD_base+PORTx_DIRn(2), 1); // PD2 in   -> MISO
	reg32_wsmask(PORTD_base+PORTx_DIRn(3), 0); // PD3 out  -> CS

	reg32_wsmask(PORTD_base+PORTx_OUTn(3), 1); // PD3 high

	reg32_write(SFC_base+SFC_CON, 0xf00000);
	reg32_write(SFC_base+SFC_CON, 0);

	reg32_write(SFC_base+SFC_BAUD, 1 - 1);

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

	reg32_write(SFC_base+SFC_BASE_ADR, 0x10000);

	reg32_write(ENC_base+ENC_CON, 0);
	reg32_write(ENC_base+ENC_KEY, 0xffff);
	reg32_write(ENC_base+ENC_ADR, 0);
	reg32_write(ENC_base+ENC_UNENC_ADRH, 0x1000140);
	reg32_write(ENC_base+ENC_UNENC_ADRL, 0x10000c0);
	reg32_wsmask(ENC_base+ENC_CON_en_sfc, 0);
	reg32_wsmask(ENC_base+ENC_CON_en_sfc_unenc, 0);

	reg32_wsmask(DSP_base+DSP_CON, 8, 1, 0); // disable sfc map
	memset((void *)0x48000, 0x00, 0x1800);   // clear icache tags
	memset((void *)0x1a000, 0x55, 0x4000);   // clear icache to see what's going on
	reg32_wsmask(DSP_base+DSP_CON, 8, 1, 1); // enable sfc map

	reg32_wsmask(SPI0_base+SPIx_CON_spie, 0);	// disable SPI0 to not conflict with SFC
	reg32_wsmask(SFC_base+SFC_CON_enable, 1);	// enable SFC
}


void clk_init(void) {
	/* switch to a safe clock (the btosc) */
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON0_cksel, 0x8); // src_clk <- btosc_clk

	/* the osc_clk we want to use is the btosc */
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON0_osc_sel, 0x0); // osc_clk <- btosc_clk

	/* disable pll */
	reg32_wsmask(CLOCK_base+CLOCK_PLL_CON_pll_en, 0);

	/* reset pll */
	reg32_wsmask(CLOCK_base+CLOCK_PLL_CON_pll_rst, 0);
	for (volatile int i = 1000; i; i--);
	reg32_wsmask(CLOCK_base+CLOCK_PLL_CON_pll_rst, 1);

	/* config pll */
	reg32_wsmask(CLOCK_base+CLOCK_PLL_CON_pll_rsel, 0); // PLL reference is btosc
	reg32_wsmask(CLOCK_base+CLOCK_PLL_CON_pll_refds, (16 / 2) - 2); // pll ref divider for 2 MHz ref
	reg32_wsmask(CLOCK_base+CLOCK_PLL_CON_pll_refde, 1); // enable ref divider

	/* enable pll */
	reg32_wsmask(CLOCK_base+CLOCK_PLL_CON_pll_en, 1);

	/* configure the rest */
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON2_pll_sys_sel, 1); // pll_sys_clk <- pll_480m
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON2_pll_sys_div0, 0); //   div1
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON2_pll_sys_div1, 1); //     div2 ===> 240 MHz !!

	reg32_wsmask(CLOCK_base+CLOCK_SYS_DIV_orgdiv, 1 - 1); // div1 --> hsb_clk/cpu_clk/mem_clk (240 MHz)
	reg32_wsmask(CLOCK_base+CLOCK_SYS_DIV_lsbdiv, 4 - 1); //        div4 --> lsb_clk (80 MHz)

	/* switch to pll */
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON0_cksel, 0xf); // src_clk <- pll_clk
}





void JieLi(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
	//clk_init();

	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1_uart_cksel, 0x1); // uart_clk <- pll_48m

	// init UART2 on PA3
	reg32_write(UART2_base+UARTx_CON0, 1); // 8n1, en
	reg32_write(UART2_base+UARTx_BAUD, (48000000 / 4 / 921600) - 1);
	reg32_wsmask(IOMAP_base+IOMAP_CON1_ut2ios, 0x0); // UART2 to TX/RX => PA3/PA4
	reg32_wsmask(IOMAP_base+IOMAP_CON3_ut2mxs, 0x0); // UART2 muxsel -> iomux
	reg32_wsmask(IOMAP_base+IOMAP_CON3_ut2ioen, 1); // UART2 I/O enable
	reg32_wsmask(PORTA_base+PORTx_DIRn(3), 0); // PA3 out

	xdev_out(uputc);
	xputs("\e[H\e[2J\e[3J"); // clear screen
	xputs("\e[1;37;41m==== JieLi AC6908C! "__DATE__" "__TIME__" ====\e[0m\n");
	xprintf("r0: <%08x>  r1: <%08x>  r2: <%08x>  r3: <%08x>\n", r0,r1,r2,r3);

	/*==================================================================*/

	InitSFC();

	void *sfc_map = (void *)0x1000000;
	void *icache_data = (void *)0x1a000;
	uint32_t *icache_tag1 = (void *)0x48000;
	uint32_t *icache_tag2 = (void *)0x49000;

	for (int i = 0; i < 128; i++) {
		// dummy read
		*(volatile uint32_t *)(sfc_map + i * 0x1000);

		reg32_wsmask(DSP_base+DSP_CON, 8, 1, 0); // disable sfc map
		xprintf("%3d: |%04x %04x %04x %04x| %x\n", i,
			icache_tag1[0], icache_tag1[1], icache_tag1[2], icache_tag1[3],
			icache_tag2[0]);
		reg32_wsmask(DSP_base+DSP_CON, 8, 1, 1); // enable sfc map
	}

}
