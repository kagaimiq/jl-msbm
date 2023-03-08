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


struct usb_msc_cbw {
	uint32_t sign;		// 0x55534243 "USBC"
	uint32_t tag;
	uint32_t xfer_len;
	uint8_t flags;
	uint8_t lun;
	uint8_t cdb_len;
	uint8_t cdb[16];
};

struct JieLi_LoaderArgs {
	int (*msd_send)(void *ptr, int len);		// send request data
	int (*msd_recv)(void *ptr, int len);		// receive request data
	int (**msd_hook)(struct usb_msc_cbw *cbw);	// SCSI request hook
	uint32_t arg;		// Argument
	uint32_t wtw_10;	// set to zero?! toggles??
};

struct JieLi_LoaderArgs *largs;


int KonaHook(struct usb_msc_cbw *cbw) {
	xputs("A SCSI command came...\n[ ");
	for (int i = 0; i < cbw->cdb_len; i++) xprintf("%02x ", cbw->cdb[i]);
	xputs("]\n\n");

	return 0;
}


void sflash_init(void) {
	reg32_wsmask(PORTD_base+PORTx_DIRn(0), 0); // PD0 out  -> SCK
	reg32_wsmask(PORTD_base+PORTx_DIRn(1), 0); // PD1 out  -> MOSI
	reg32_wsmask(PORTD_base+PORTx_DIRn(2), 1); // PD2 in   -> MISO
	reg32_wsmask(PORTD_base+PORTx_DIRn(3), 0); // PD3 out  -> CS

	reg32_wsmask(PORTD_base+PORTx_OUTn(3), 1); // PD3 high

	reg32_wsmask(IOMAP_base+IOMAP_CON0_spi0ios, 0x0); // SPI0 on PD3/PD2/PD1/PD0

	reg32_write(SPI0_base+SPIx_CON, 0x20);
	reg32_write(SPI0_base+SPIx_BAUD, 32); // some clock speed
	reg32_wsmask(SPI0_base+SPIx_CON_bidir, 1); // full duplex

	reg32_wsmask(SFC_base+SFC_CON_enable, 0); // disable SFC
	reg32_wsmask(SPI0_base+SPIx_CON_spie, 1); // enable SPI0

	for (volatile int i = 100000; i; i--); // stutter
}

void sflash_sel(char sel) {
	reg32_wsmask(PORTD_base+PORTx_OUTn(3), !sel); // CS
}

uint8_t sflash_bytexfer(uint8_t val) {
	reg32_write(SPI0_base+SPIx_BUF, val);
	while (!reg32_rsmask(SPI0_base+SPIx_CON_pnd));
	reg32_wsmask(SPI0_base+SPIx_CON_pclr, 1);
	return reg32_read(SPI0_base+SPIx_BUF);
}

void sflash_dmaxfer(void *ptr, int len, int dir) {
	reg32_wsmask(SPI0_base+SPIx_CON_dir, !!dir); //1=recv, 0=send
	reg32_write(SPI0_base+SPIx_ADR, (uint32_t)ptr);
	reg32_write(SPI0_base+SPIx_CNT, len);
	while (!reg32_rsmask(SPI0_base+SPIx_CON_pnd));
	reg32_wsmask(SPI0_base+SPIx_CON_pclr, 1);
}


void JieLi(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
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

	largs = (void *)r0;

	//*largs->msd_hook = KonaHook;


	sflash_init();

#if 0
	static uint8_t buff[512];

	// erase a 4k sector at 000000
	sflash_sel(1);
	sflash_bytexfer(0x06);
	sflash_sel(0);

	sflash_sel(1);
	sflash_bytexfer(0x20);
	sflash_bytexfer(0x00);
	sflash_bytexfer(0x00);
	sflash_bytexfer(0x00);
	sflash_sel(0);

	sflash_sel(1);
	sflash_bytexfer(0x05);
	while (sflash_bytexfer(0xff) & 0x01);
	sflash_sel(0);

	// program to 000010
	sflash_sel(1);
	sflash_bytexfer(0x06);
	sflash_sel(0);

	sflash_sel(1);
	sflash_bytexfer(0x02);
	sflash_bytexfer(0x00);
	sflash_bytexfer(0x00);
	sflash_bytexfer(0x10);
	sflash_dmaxfer("\x01\x02\x04\x08\x10\x20\x40\x80konachan", 16, 0);
	sflash_sel(0);

	sflash_sel(1);
	sflash_bytexfer(0x05);
	while (sflash_bytexfer(0xff) & 0x01);
	sflash_sel(0);

	// program to 000080

	sflash_sel(1);
	sflash_bytexfer(0x06);
	sflash_sel(0);

	sflash_sel(1);
	sflash_bytexfer(0x02);
	sflash_bytexfer(0x00);
	sflash_bytexfer(0x00);
	sflash_bytexfer(0x80);
	sflash_dmaxfer("Oh my god no way!!", 18, 0);
	sflash_sel(0);

	sflash_sel(1);
	sflash_bytexfer(0x05);
	while (sflash_bytexfer(0xff) & 0x01);
	sflash_sel(0);

	// read 000000

	sflash_sel(1);
	sflash_bytexfer(0x03);
	sflash_bytexfer(0x00);
	sflash_bytexfer(0x00);
	sflash_bytexfer(0x00);
	sflash_dmaxfer(buff, sizeof buff, 1);
	sflash_sel(0);

	hexdump(buff, sizeof buff);
#endif

#if 1
	reg32_write(SFC_base+SFC_CON, 0xf00000);
	reg32_write(SFC_base+SFC_CON, 0);

	reg32_write(SFC_base+SFC_BAUD, 0xff);

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

	reg32_wsmask(SFC_base+SFC_CON, 0, 0x6ff0f88, 0x0240480);

	reg32_write(SFC_base+SFC_BASE_ADR, 0x12000);

	reg32_write(ENC_base+ENC_CON, 0);
	reg32_write(ENC_base+ENC_KEY, 0xffff);
	reg32_write(ENC_base+ENC_ADR, 0);
	reg32_write(ENC_base+ENC_UNENC_ADRH, 0x1000140);
	reg32_write(ENC_base+ENC_UNENC_ADRL, 0x10000c0);
	reg32_wsmask(ENC_base+ENC_CON_en_sfc, 0);
	reg32_wsmask(ENC_base+ENC_CON_en_sfc_unenc, 1);

	reg32_wsmask(DSP_base+DSP_CON, 8, 1, 0); // disable sfc map
	memset((void *)0x48000, 0x00, 0x1800);   // clear sfc cache tags
	reg32_wsmask(DSP_base+DSP_CON, 8, 1, 1); // enable sfc map

	reg32_wsmask(SPI0_base+SPIx_CON_spie, 0);	// disable SPI0 to not conflict with SFC
	reg32_wsmask(SFC_base+SFC_CON_enable, 1);	// enable SFC

	hexdump((void *)0x1000000, 0x200);
#endif
}
