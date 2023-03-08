#include <stdint.h>
#include <xprintf.h>
#include <jl_regs.h>
#include <jl_irq.h>
#include <wallclk.h>
#include <maskrom_stuff.h>
#include <ff.h>

void uputc(int c) {
	reg32_write(UART0_base+UARTx_BUF, c);
	while (!reg32_rsmask(UART0_base+UARTx_CON0_tpnd));
	reg32_wsmask(UART0_base+UARTx_CON0_clrtpnd, 1);
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




int sd_send_cmd(uint8_t cmd, uint32_t arg, int rtype, uint32_t *resp) {
	//xprintf("---sd-send-cmd---- %02x %08x %d @%x---\n", cmd,arg,rtype,resp);

	uint8_t cbuff[32] = {
		0x40 | (cmd & 0x3f),
		arg >> 24,
		arg >> 16,
		arg >> 8,
		arg >> 0,
	};

	reg32_write(SD0_base+SDx_CPTR, (uint32_t)cbuff);

	reg32_wsmask(SD0_base+SDx_CON1, 6, 1, 0);
	reg32_wsmask(SD0_base+SDx_CON0, 4, 1, 1);

	reg32_wsmask(SD0_base+SDx_CON0_firecmd, rtype == 2 ? 0x6 : 0x5);

	// timeout...
	while (!reg32_rsmask(SD0_base+SDx_CON0_pnd_cmd));
	reg32_wsmask(SD0_base+SDx_CON0_cpnd_cmd, 1);

	reg32_wsmask(SD0_base+SDx_CON0, 4, 1, 0);

	asm volatile ("csync");

	if (rtype == 1) {
		resp[0] = (cbuff[ 7] << 24) | (cbuff[ 8] << 16) | (cbuff[ 9] << 8) | cbuff[10];
		//xprintf("RESP->%08x\n", resp[0]);
	} else if (rtype == 2) {
		resp[0] = (cbuff[ 6] << 24) | (cbuff[ 7] << 16) | (cbuff[ 8] << 8) | cbuff[ 9];
		resp[1] = (cbuff[12] << 24) | (cbuff[13] << 16) | (cbuff[14] << 8) | cbuff[15];
		resp[2] = (cbuff[16] << 24) | (cbuff[17] << 16) | (cbuff[18] << 8) | cbuff[19];
		resp[3] = (cbuff[20] << 24) | (cbuff[21] << 16) | (cbuff[22] << 8) | cbuff[23];
		//xprintf("RESP->%08x %08x %08x %08x\n", resp[0],resp[1],resp[2],resp[3]);
	}

	wdt_clr();

	return 0;
}



#include <diskio.h>

DSTATUS dStatus;
uint16_t cRCA;
uint8_t cCSD[16], cCID[16];

DSTATUS disk_initialize(BYTE drv) {
	dStatus |= STA_NOINIT;

	// iocfg
	reg32_wsmask(IOMAP_base+IOMAP_CON0_sd0ios, 'f'-'a');
	reg32_wsmask(IOMAP_base+IOMAP_CON0_sd0cken, 1);
	reg32_wsmask(IOMAP_base+IOMAP_CON0_sd0dten, 1);
	// pullup
	reg32_wsmask(PORTB_base+PORTx_PUn(4), 1); //PB4 pullup
	reg32_wsmask(PORTB_base+PORTx_PUn(6), 1); //PB6 pullup
	reg32_wsmask(PORTB_base+PORTx_PUn(7), 1); //PB7 pullup
	// pulldown
	reg32_wsmask(PORTB_base+PORTx_PDn(4), 0); //PB4 no pulldown
	reg32_wsmask(PORTB_base+PORTx_PDn(6), 0); //PB6 no pulldown
	reg32_wsmask(PORTB_base+PORTx_PDn(7), 0); //PB7 no pulldown
	// output
	reg32_wsmask(PORTB_base+PORTx_DIRn(4), 0); // PB4 output
	reg32_wsmask(PORTB_base+PORTx_DIRn(6), 0); // PB6 output
	reg32_wsmask(PORTB_base+PORTx_DIRn(7), 0); // PB7 output
	// digital in
	reg32_wsmask(PORTB_base+PORTx_DIEn(4), 1); // PB4 digital in
	reg32_wsmask(PORTB_base+PORTx_DIEn(6), 1); // PB6 digital in
	reg32_wsmask(PORTB_base+PORTx_DIEn(7), 1); // PB7 digital in

	// disable encrypting
	reg32_wsmask(PERIENC_base+PERIENC_CON_en_sd0data, 0);

	// hw CLOSE
	reg32_write(SD0_base+SDx_CON0, REG_SMASK(SDx_CON0_cpnd_cmd)|REG_SMASK(SDx_CON0_cpnd_dat));
	reg32_write(SD0_base+SDx_CON1, 0);
	reg32_write(SD0_base+SDx_CON2, REG_MKVAL(SDx_CON2_blksize, 512-1));
	reg32_write(SD0_base+SDx_CTU_CON, REG_SMASK(SDx_CTU_CON_done)); //waa
	reg32_write(SD0_base+SDx_CTU_CNT, 0);

	// set BUAD
	reg32_wsmask(SD0_base+SDx_CON1_baud, (48000 / 400) - 1);
	//reg32_wsmask(SD0_base+SDx_CON1_baud, 255);

	// hw bit enable
	reg32_wsmask(SD0_base+SDx_CON1_enable, 1);
	reg32_wsmask(SD0_base+SDx_CTU_CON_enable, 1);

	// idle clock enable
	reg32_wsmask(SD0_base+SDx_CON1_idleclken, 1);

	// wait for some time (at least 74 clock ticks should be sent to the card)
	for (volatile int i = 10000; i; i--) {};

	//---------------------------------------------------

	uint32_t resp[4];

	sd_send_cmd(0, 0, 0, NULL);		// CMD0 (GO_IDLE_STATE)

	//----------------------idle state-----------------------//

	sd_send_cmd(8, 0x1aa, 1, resp);		// CMD8 (SEND_IF_COND)

	for (int i = 0; i < 100; i++) {
		sd_send_cmd(55, 0, 1, resp);		// CMD55 (APP_CMD)

		sd_send_cmd(41, 0x40ff8000, 1, resp);	// ACMD41 (APP_SEND_OP_COND)

		if (resp[0] & 0x80000000) break;
	}

	//---------------------ready state-----------------------//

	sd_send_cmd(2, 0, 2, resp);		// CMD2 (APP_SEND_CID)
	xprintf("CID => %08x %08x %08x %08x\n", resp[0],resp[1],resp[2],resp[3]);

	for (int i = 0; i < 16; i++)
		cCID[i] = resp[i / 4] >> (8 * (3 - (i % 4)));

	//---------------------ident state-----------------------//

	sd_send_cmd(3, 0, 1, resp);		// CMD3 (SEND_RELATIVE_ADDR)
	cRCA = resp[0] >> 16;
	xprintf("Card address => %04x\n", cRCA);

	//----------------------stby state-----------------------//

	sd_send_cmd(9, cRCA << 16, 2, resp);	// CMD9 (SEND_CSD)
	xprintf("CSD => %08x %08x %08x %08x\n", resp[0],resp[1],resp[2],resp[3]);

	for (int i = 0; i < 16; i++)
		cCSD[i] = resp[i / 4] >> (8 * (3 - (i % 4)));

	sd_send_cmd(7, cRCA << 16, 1, resp);	// CMD7 (SELECT_CARD)

	//----------------------tran state-----------------------//

	reg32_wsmask(SD0_base+SDx_CON1_baud, (48000 / 4000) - 1);

	dStatus &= ~STA_NOINIT;

	return dStatus;
}

DSTATUS disk_status(BYTE drv) {
	return dStatus;
}

DRESULT disk_read(BYTE drv, BYTE *ptr, LBA_t lba, UINT cnt) {
	wdt_clr();

	uint32_t resp;

	for (int i = 10000; i >= 0; i--) {
		if (i == 0) return RES_ERROR;

		sd_send_cmd(13, cRCA << 16, 1, &resp);
		if ((resp & 0x1e00) == 0x800) break;
	}

	reg32_wsmask(SD0_base+SDx_CON0, 12, 1, 0);

	reg32_write(SD0_base+SDx_DPTR, (uint32_t)ptr);
	reg32_wsmask(SD0_base+SDx_CON1, 7, 1, 0);

	reg32_wsmask(SD0_base+SDx_CTU_CON, 1, 0x3, 0x0);

	reg32_wsmask(SD0_base+SDx_CTU_CON_clr_done, 1);
	reg32_wsmask(SD0_base+SDx_CTU_CON_clr_err, 1);

	reg32_write(SD0_base+SDx_CON2, REG_MKVAL(SDx_CON2_blksize, 512-1)); //block size
	reg32_wsmask(SD0_base+SDx_CTU_CON_firedat, 0x6); // data rx
	reg32_write(SD0_base+SDx_CTU_CNT, cnt-1); // block count

	sd_send_cmd(cnt > 1 ? 18 : 17, lba, 1, &resp);

	while (!reg32_rsmask(SD0_base+SDx_CON0_pnd_dat));
	reg32_wsmask(SD0_base+SDx_CON0_cpnd_dat, 1); // clr data pend

	reg32_wsmask(SD0_base+SDx_CTU_CON_clr_done, 1);
	reg32_wsmask(SD0_base+SDx_CTU_CON_clr_err, 1);

	if (cnt > 1) sd_send_cmd(12, 0, 1, &resp); //stop

	asm volatile ("csync");

	return RES_OK; //RES_ERROR;
}

DRESULT disk_write(BYTE drv, const BYTE *ptr, LBA_t lba, UINT cnt) {
	uint32_t resp;

	asm volatile ("csync");

	for (int i = 10000; i >= 0; i--) {
		if (i == 0) return RES_ERROR;

		sd_send_cmd(13, cRCA << 16, 1, &resp);
		if ((resp & 0x1e00) == 0x800) break;
	}

	if (cnt > 1) {
		sd_send_cmd(55, cRCA << 16, 1, &resp);
		sd_send_cmd(23, cnt, 1, &resp);
	}

	sd_send_cmd(cnt > 1 ? 25 : 24, lba, 1, &resp);

	reg32_wsmask(SD0_base+SDx_CON0, 12, 1, 0);

	reg32_write(SD0_base+SDx_DPTR, (uint32_t)ptr);
	reg32_wsmask(SD0_base+SDx_CON1, 7, 1, 0);

	reg32_wsmask(SD0_base+SDx_CTU_CON, 1, 0x3, 0x0);

	reg32_wsmask(SD0_base+SDx_CTU_CON_clr_done, 1);
	reg32_wsmask(SD0_base+SDx_CTU_CON_clr_err, 1);

	reg32_write(SD0_base+SDx_CON2, REG_MKVAL(SDx_CON2_blksize, 512-1)); //block size
	reg32_wsmask(SD0_base+SDx_CTU_CON, 8, 0x7, 0x5); // data tx
	reg32_write(SD0_base+SDx_CTU_CNT, cnt-1); // block count

	while (!reg32_rsmask(SD0_base+SDx_CON0_pnd_dat));

	reg32_wsmask(SD0_base+SDx_CON0_cpnd_dat, 1); // clr data pend

	reg32_wsmask(SD0_base+SDx_CTU_CON_clr_done, 1);
	reg32_wsmask(SD0_base+SDx_CTU_CON_clr_err, 1);

	if (cnt > 1) sd_send_cmd(12, 0, 1, &resp);

	return RES_OK; //RES_ERROR;
}

DRESULT disk_ioctl(BYTE drv, BYTE cmd, void *buff) {
	switch (cmd) {
	case CTRL_SYNC:
		return RES_OK;

	default:
		return RES_PARERR;
	}

	return RES_ERROR;
}




void sflash_init(void) {
	reg32_wsmask(PORTD_base+PORTx_DIRn(0), 0); // PD0 out  -> SCK
	reg32_wsmask(PORTD_base+PORTx_DIRn(1), 0); // PD1 out  -> MOSI
	reg32_wsmask(PORTD_base+PORTx_DIRn(2), 1); // PD2 in   -> MISO
	reg32_wsmask(PORTD_base+PORTx_DIRn(3), 0); // PD3 out  -> CS
	reg32_wsmask(PORTD_base+PORTx_DIRn(4), 0); // PD4 out  -> ?? HOLD? Power?!

	reg32_wsmask(PORTD_base+PORTx_OUTn(3), 1); // PD3 high
	reg32_wsmask(PORTD_base+PORTx_OUTn(4), 1); // PD4 high

	reg32_wsmask(IOMAP_base+IOMAP_CON0_spi0ios, 0x0); // SPI0 on PD3/PD2/PD1/PD0

	reg32_write(SPI0_base+SPIx_CON, 0x20);
	reg32_write(SPI0_base+SPIx_BAUD, 32); // some clock speed
	reg32_wsmask(SPI0_base+SPIx_CON_bidir, 1); // full duplex
	reg32_wsmask(SPI0_base+SPIx_CON_spie, 1);

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
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1, 10, 0x3, 0x0); // uart_clk <- pll_48m?

	// init UART0 on PB5
	reg32_write(UART0_base+UARTx_CON0, 1); // 8n1, en
	reg32_write(UART0_base+UARTx_BAUD, (48000000 / 4 / 921600) - 1);
	reg32_wsmask(IOMAP_base+IOMAP_CON0_ut0ios, 0x2); // UART0 to PB5
	reg32_wsmask(IOMAP_base+IOMAP_CON3_ut0mxs, 0x0); // UART0 muxsel -> iomux
	reg32_wsmask(IOMAP_base+IOMAP_CON3_ut0ioen, 1); // UART0 I/O enable
	reg32_wsmask(PORTB_base+PORTx_PUn(5), 1); // PB5 pullup
	reg32_wsmask(PORTB_base+PORTx_DIRn(5), 0); // PB5 out

	xdev_out(uputc);
	xputs("\e[H\e[2J\e[3J"); // clear screen
	xputs("\e[1;37;41m==== JieLi AC6965A! "__DATE__" "__TIME__" ====\e[0m\n");
	xprintf("r0: <%08x>  r1: <%08x>  r2: <%08x>  r3: <%08x>\n", r0,r1,r2,r3);

	/*==================================================================*/

	wallclk_init();
	//irq_attach(1, ExceptionHandler_entry);

	reg32_write(PERIENC_base+PERIENC_CON, 0);
	reg32_write(PERIENC_base+PERIENC_KEY, 0x0000);
	reg32_write(PERIENC_base+PERIENC_ADR, 0x0);

	sflash_init();
	disk_initialize(0);

	reg32_wsmask(PERIENC_base+PERIENC_CON_en_sd0data, 1);
	reg32_wsmask(PERIENC_base+PERIENC_CON_en_spi0dma, 1);

	static uint8_t tmp[512];

	/*reg32_wsmask(PERIENC_base+PERIENC_CON_rstkey, 1);
	memset(tmp, 0, sizeof tmp);
	strcpy((void *)&tmp[0], "kagami");
	strcpy((void *)&tmp[1], "hiiragi");

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 64; j++)
			tmp[i][32+j] = reg32_read(RAND_base+RAND_R64L);

	disk_write(0, (void *)tmp, 2, sizeof tmp / 512);

	for (int i = 0; i < 100; i++) {
		uint32_t resp;
		sd_send_cmd(13, cRCA << 16, 1, &resp);
		if ((resp & 0x1e00) == 0x800) break;
	}*/

	reg32_wsmask(PERIENC_base+PERIENC_CON_rstkey, 1);

	disk_read(0, (void *)tmp, 0, sizeof tmp / 512);
	hexdump(tmp, sizeof tmp);

	reg32_wsmask(PERIENC_base+PERIENC_CON_rstkey, 1);

	sflash_sel(1);
	sflash_bytexfer(0x03);
	sflash_bytexfer(0x00);
	sflash_bytexfer(0x00);
	sflash_bytexfer(0x00);
	sflash_dmaxfer(tmp, sizeof tmp, 1);
	sflash_sel(0);

	hexdump(tmp, sizeof tmp);

	xputs("byte\n");

	wallclk_deinit();
}
