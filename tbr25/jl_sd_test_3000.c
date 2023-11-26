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

	JL_TIMER5->CON = (1<<14);
	JL_TIMER5->CNT = 0;
	JL_TIMER5->PRD = usecs * 48;	/* 48 MHz clock source */
	JL_TIMER5->CON |= (1<<0);
	while (!(JL_TIMER5->CON & (1<<15)));
	JL_TIMER5->CON = (1<<14);
}

void delay(unsigned msecs) {
	while (msecs--) usleep(1000);
}

/*=========================================================================*/

/*
 0 : idle
 1 : ? .. CMD12 .. CMD7
 2 : 
 3 : 
 4 : 
 5 : 48 bit resp    ... CMD8, CMD55, etc .. CMD3
 6 : 136 bit resp   ... CMD0 (#1) ... CMD2, CMD9
 7 :                ... CMD0 (#2)

 ====================

 0: <no operation>
 1: 48-bit response     ...<used on cmd12 & cmd7>
 2: 136-bit response (only 128 bits received)
 3: No response
 4: <no operation>
 5: 48-bit response
 6: 136-bit response (only 128 bits received)
 7: No response
*/



/*
 * CON0:
 *   [15] dat interrupt pending
 *   [14] dat interrupt clear
 *   [13] bus width:
 *      0 = 1-bit
 *      1 = 4-bit
 *   [12] data 8 clocks popa
 *   [11] data crc error
 *   [10] ? <together with [9:8], usually set to 1>
 *   [9:8] data xfer fire:
 *      00 = no action
 *      01 = send data
 *      10 = recv data
 *   [7] cmd interrupt pending
 *   [6] cmd interrupt clear
 *   [5] command timeout
 *   [4] command 8 clocks popa
 *   [3] command CRC error
 *   [2] wait for busy: (used together with [1:0])
 *      0 = wait for busy
 *      1 = do not wait for busy
 *   [1:0] cmd xfer fire:
 *      00 = no action
 *      01 = 48-bit response
 *      10 = 136-bit response (only 128 bits received)
 *      11 = no response
 *
 * CON1:
 *   [15:8] clock divider <Fsd = Fbus / (n + 1)>
 *   [7] data int enable
 *   [6] cmd int enable
 *   [5] enable clock on idle
 *   [0] ~~enable
 *
 * CON2:
 *   [8:0] block size <size = n + 1>
 *
 * CTU_CON:
 *   [10] ? <together with [9:8], usually 1>
 *   [9:8] data xfer cmd:
 *     00 = no action
 *     01 = send
 *     10 = receive
 *   [7] flag1
 *   [6] clear flag1
 *   [5] flag0
 *   [4] clear flag0
 *   [2] flag1 int enable
 *   [1] flag0 int enable
 *   [0] ~~enable
 *
 * CTU_CNT:
 *   [15:0] block count <count = n + 1>
 *
 *
 * the CTU is used to transfer multiple blocks at a time,
 *  so the plain sdc was able to do one block at a time....
 *     This means the HW part is the same as e.g. in BR17 - BR21?
 *        or even older, back to AC4100 ?!
 */


struct JL_SD_regs *sdh = JL_SD0;
uint8_t sdh_cbuff[6+16];



#define SDHCMD(cmd, typ)		((cmd) << 8 | typ)

//#define SDH_NOISE



int sdh_sendcmd(uint16_t cmd, uint32_t arg) {
	int res = 0;

#ifdef SDH_NOISE
	xprintf("\e[1;31m>>>>> cmd=%04x arg=%08x <<<<<\e[0m\n", cmd, arg);
	xprintf("////////// CMD%d\n", (cmd >> 8) & 0x3f);
#endif

	/* Prepare the command buffer */
	memset(sdh_cbuff, 0, sizeof(sdh_cbuff));
	sdh_cbuff[0] = 0x40 | ((cmd >> 8) & 0x3f);	/* start bit (0), tran bit (1), opcode (xxxxxx) */
	sdh_cbuff[1] = arg >> 24;			/* argumemt [31:24] */
	sdh_cbuff[2] = arg >> 16;			/* argument [23:16] */
	sdh_cbuff[3] = arg >> 8;			/* argument [15:8] */
	sdh_cbuff[4] = arg >> 0;			/* argument [7:0] */
	sdh->CPTR = (uint32_t)sdh_cbuff;

	sdh->CON1 &= ~(1<<6);		/* disable cmd int */
	sdh->CON0 |= (1<<4);		/* set ??? 4 */

	sdh->CON0 |= (1<<6);		/* clr cmd int pend */

	/* fire cmd */
	sdh->CON0 &= ~7;
	sdh->CON0 |= cmd & 7;

	/* wait for cmd int */
	while (!(sdh->CON0 & (1<<7)));

	/* Timed out? */
	if (sdh->CON0 & (1<<5))
		res = -1;

#ifdef SDH_NOISE
	xprintf("CON0 = %04x || %s, %s\n",
		sdh->CON0,
		(sdh->CON0 & (1<<5)) ? "Timed out":"-",
		(sdh->CON0 & (1<<3)) ? "CRC error":"-"
	);
#endif

	sdh->CON0 |= (1<<6);		/* clr cmd int pend */

#ifdef SDH_NOISE
	xputs("\e[1;32m[ ");
	for (int i = 0; i < sizeof sdh_cbuff; i++) xprintf("%02x ", sdh_cbuff[i]);
	xputs("]\e[0m\n");
#endif

	sdh->CON0 &= ~(1<<4);	/* clr ??? 4 */

	return res;
}

void sdh_data_rx(void *ptr, int blocks, int blksize) {
	//xprintf("---- data rx: %x, %d x %d\n", ptr, blocks, blksize);

	sdh->CON0 &= ~(1<<12);	/* clr ??? 12 */
	sdh->CON1 &= ~(1<<5);	/* disable idle clock! */

	sdh->DPTR = (uint32_t)ptr;	/* set data buffer address */

	sdh->CON1 &= ~(1<<7);	/* disable data int */
	sdh->CON0 |= (1<<14);	/* clr data int pend */

	sdh->CTU_CON &= ~((1<<2)|(1<<1));	/* disable flag ints */
	sdh->CTU_CON |= (1<<6)|(1<<4);		/* clear the flags */

	sdh->CON2 = blksize - 1;	/* block size */

	if (blocks > 1) {
		/* receive blocks via CTU */
		sdh->CTU_CON &= ~(7<<8);
		sdh->CTU_CON |= (6<<8);

		/* set block count */
		sdh->CTU_CNT = blocks - 1;
	} else {
		/* receive block */
		sdh->CON0 &= ~(7<<8);
		sdh->CON0 |= (6<<8);
	}
}

void sdh_data_tx(void *ptr, int blocks, int blksize) {
	//xprintf("---- data tx: %x, %d x %d\n", ptr, blocks, blksize);

	sdh->CON0 &= ~(1<<12);	/* clr ??? 12 */

	sdh->DPTR = (uint32_t)ptr;	/* set data buffer address */

	sdh->CON1 &= ~(1<<7);	/* disable data int */
	sdh->CON0 |= (1<<14);	/* clr data int pend */

	sdh->CTU_CON &= ~((1<<2)|(1<<1));	/* disable flag ints */
	sdh->CTU_CON |= (1<<6)|(1<<4);	/* clear the flags */

	sdh->CON2 = blksize - 1;	/* block size */

	if (blocks > 1) {
		/* send blocks via CTU */
		sdh->CTU_CON &= ~(7<<8);
		sdh->CTU_CON |= (5<<8);

		/* set block count */
		sdh->CTU_CNT = blocks - 1;
	} else {
		/* send block */
		sdh->CON0 &= ~(7<<8);
		sdh->CON0 |= (5<<8);
	}
}

int sdh_data_wait(void) {
	int res = 0;

	/* wait for dat int */
	//while (!(sdh->CON0 & (1<<15)));

	for (int i = 1000; i >= 0; i--) {
		/* timed out? */
		if (i == 0) res = -1;

		if (sdh->CON0 & (1<<15)) break;

		delay(1);
	}

#ifdef SDH_NOISE
	xprintf("CON0 = %04x, CTU_CON = %04x, CTU_CNT = %04x\n",
		sdh->CON0, sdh->CTU_CON, sdh->CTU_CNT);
#endif

	sdh->CON0 |= (1<<14);	/* clr dat int pend */
	sdh->CTU_CON |= (1<<6);	/* clear flag1 */
	sdh->CTU_CON |= (1<<4);	/* clear flag0 */

	return res;
}

/*---------------------------------------------------------------------------------*/

/* [[Remember!]]
 * R1 response: 48 bits, valid opcode, valid crc     [normal response]
 * R2 response: 136 bits, invalid opcode, valid crc  [CID/CSD register]
 * R3 response: 48 bits, invalid opcode, invalid crc [OCR register]
 * R4 response: 48 bits, opcode == CMD39, valid crc  [fast I/O]
 * R5 response: 48 bits, opcode == CMD40, valid crc  [interrupt request]
 * R6 response: 48 bits, opcode == CMD3, valid crc   [published RCA response]
 * R7 response: 48 bits, opcode == CMD8, valid crc   [card interface condition]
 */

#define CMD_GO_IDLE_STATE			0
#define CMD_SEND_OP_COND			1	/* MMC */
#define CMD_ALL_SEND_CID			2
#define CMD_SEND_RELATIVE_ADDR			3
#define CMD_SET_DSR				4
#define CMD_IO_SEND_OP_COND			5	/* SDIO */
#define CMD_SWITCH				6
#define CMD_SELECT_CARD				7
#define CMD_SEND_IF_COND			8	/* SDCv2 */
#define CMD_SEND_EXT_CSD			8	/* MMCv4 */
#define CMD_SEND_CSD				9
#define CMD_SEND_CID				10
#define CMD_STOP_TRANSMISSION			12
#define CMD_SEND_STATUS				13
#define CMD_GO_INACTIVE_STATUS			15
#define CMD_SET_BLOCKLEN			16
#define CMD_READ_SINGLE_BLOCK			17
#define CMD_READ_MULTIPLE_BLOCKS		18
#define CMD_SET_BLK_COUNT			23	/* MMC */
#define CMD_WRITE_BLOCK				24
#define CMD_WRITE_MULTIPLE_BLOCK		25
#define CMD_PROGRAM_CSD				27
#define CMD_ERASE_ER_BLK_START			32
#define CMD_ERASE_ER_BLK_END			33
#define CMD_ERASE				38
#define CMD_IO_RW_DIRECT			52	/* SDIO */
#define CMD_IO_RW_EXTENDED			53	/* SDIO */
#define CMD_APP_CMD				55
/* sent after APP_CMD */
#define ACMD_SET_BUS_WIDTH			(0x80|6)	/* SDC */
#define ACMD_SD_STATUS				(0x80|13)	/* SDC */
#define ACMD_SET_WR_BLK_ERASE_COUNT		(0x80|23)	/* SDC */
#define ACMD_SEND_OP_COND			(0x80|41)	/* SDC */


#define CT_MMC		0x03
#define CT_MMCV3	0x01
#define CT_MMCV4	0x02
#define CT_SDC		0x0C
#define CT_SDCV1	0x04
#define CT_SDCV2	0x08
#define CT_BLOCK	0x10


uint16_t sdc_rca;
uint8_t sdc_ctype;
uint8_t sdc_cid[16];
uint8_t sdc_csd[16];


int sdc_sendcmd(uint16_t cmd, uint32_t arg, uint32_t *resp) {
	int ret;

	if (cmd & 0x8000) {	/* Send APP_CMD first */
		ret = sdh_sendcmd(SDHCMD(CMD_APP_CMD, 5), sdc_rca << 16);
		if (ret)
			return ret;
	}

	ret = sdh_sendcmd(cmd, arg);
	if (ret)
		return ret;

	if (resp) {
		switch (cmd & 3) {
		case 1:	/* 48-bit response */
			resp[0] = (sdh_cbuff[7] << 24) | (sdh_cbuff[8] << 16) | (sdh_cbuff[9] << 8) | sdh_cbuff[10];
			break;

		case 2:	/* 136-bit response (almost) */
			resp[0] = (sdh_cbuff[ 7]<<24) | (sdh_cbuff[ 8]<<16) | (sdh_cbuff[ 9]<<8) | sdh_cbuff[10];
			resp[1] = (sdh_cbuff[11]<<24) | (sdh_cbuff[12]<<16) | (sdh_cbuff[13]<<8) | sdh_cbuff[14];
			resp[2] = (sdh_cbuff[15]<<24) | (sdh_cbuff[16]<<16) | (sdh_cbuff[17]<<8) | sdh_cbuff[18];
			resp[3] = (sdh_cbuff[19]<<24) | (sdh_cbuff[20]<<16) | (sdh_cbuff[21]<<8) | 0x01;
			break;
		}
	}

	return 0;
}


int sdc_probe(void) {
	uint8_t cmd, type;
	uint32_t arg, resp[4];

	xputs("\e[1;33m ===================== SDH FUN ======================= \e[0m\n");

	sdc_rca = 0;

	sdc_sendcmd(SDHCMD(CMD_GO_IDLE_STATE, 7), 0, NULL);

	/*------------------------- Idle state -----------------------------*/

	if (!sdc_sendcmd(SDHCMD(CMD_SEND_IF_COND, 5), 0x1aa, resp) && ((resp[0] & 0xfff) == 0x1aa)) {
		/* SDv2 */
		cmd = ACMD_SEND_OP_COND;
		arg = 0x40ff8000;
		type = CT_SDCV2;
	} else {
		if (!sdc_sendcmd(SDHCMD(ACMD_SEND_OP_COND, 5), 0x00ff8000, NULL)) {
			/* SDv1 */
			cmd = ACMD_SEND_OP_COND;
			arg = 0x00ff8000;
			type = CT_SDCV1;
		} else {
			/* MMC */
			cmd = CMD_SEND_OP_COND;
			arg = 0x40ff8000;
			type = CT_MMCV3;
		}
	}

	for (int i = 1000; i >= 0; i--) {
		if (i == 0)
			return 1;

		if (sdc_sendcmd(SDHCMD(cmd, 5), arg, resp))
			return 1;

		if (resp[0] & 0x80000000)
			break;
	}

	if (resp[0] & 0x40000000)
		type |= CT_BLOCK;

	/*------------------------- Ready state ----------------------------*/

	sdc_sendcmd(SDHCMD(CMD_ALL_SEND_CID, 6), 0, NULL);
	for (int i = 0; i < 14; i++) sdc_cid[i] = sdh_cbuff[7+i];

	/*----------------------- Identify state ---------------------------*/

	xputs("CID: ");
	for (int i = 0; i < 16; i++) xprintf("%02x", sdc_cid[i]);
	xputc('\n');

	if (type & CT_SDC) {
		sdc_sendcmd(SDHCMD(CMD_SEND_RELATIVE_ADDR, 5), 0, resp);
		sdc_rca = resp[0] >> 16;
	} else {
		sdc_rca = 1337;
		sdc_sendcmd(SDHCMD(CMD_SEND_RELATIVE_ADDR, 5), sdc_rca << 16, NULL);
	}

	xprintf("RCA: %04x\n", sdc_rca);

	sdc_sendcmd(SDHCMD(CMD_SEND_CSD, 6), sdc_rca << 16, NULL);
	for (int i = 0; i < 15; i++) sdc_csd[i] = sdh_cbuff[7+i];

	xputs("CSD: ");
	for (int i = 0; i < 16; i++) xprintf("%02x", sdc_csd[i]);
	xputc('\n');

	/*------------------------ Standby state ---------------------------*/

	sdc_sendcmd(SDHCMD(CMD_SELECT_CARD, 1), sdc_rca << 16, NULL);

	/*----------------------- Transfer state ---------------------------*/

	xprintf("CARD TYPE == %02x\n", type);
	sdc_ctype = type;

	return 0;
}

int sdc_read(uint32_t blk, void *ptr, int cnt) {
	if (!(sdc_ctype & CT_BLOCK))
		blk *= 512;

	

	return 0;
}


uint8_t buffer[4096];


void sdhfun_sdc(void) {
	uint32_t resp[4];
	uint32_t capacity;

	if ((sdc_ctype & CT_SDCV2) && (sdc_csd[0] == 0x40)) {
		capacity = sdc_csd[9] | (sdc_csd[8] << 8) | ((sdc_csd[7] & 0x3f) << 16);
		capacity = (capacity + 1) << 10;
	} else {
		int d = ((sdc_csd[8] >> 6) | (sdc_csd[7] << 2) | ((sdc_csd[6] & 3) << 10)) + 1;

		if (sdc_ctype & CT_MMC) {
			int b = ((sdc_csd[5] & 0xf) | ((sdc_csd[10] & 0x80) >> 7) | ((sdc_csd[9] & 3) << 1)) - 7;
			capacity = d << b;
		} else {
			capacity = d << 10;
		}
	}

	xprintf("Capacity=%u blocks\n", capacity);



	JL_PERIENC->CON &= ~(1<<2);

	sdc_sendcmd(SDHCMD(CMD_SEND_STATUS, 5), sdc_rca << 16, resp);
	xprintf("Status = %08x\n", resp[0]);

	#if 0
	/* SD Status */
	sdh_sendcmd_rxdata(SDHCMD(ACMD_SD_STATUS, 5), 0, buffer, 64);
	hexdump(buffer, 64);

	/* SCR */
	sdh_sendcmd_rxdata(SDHCMD(0x80|51, 5), 0, buffer, 64);
	hexdump(buffer, 64);
	#endif

	JL_PERIENC->KEY = 0xffff;
	JL_PERIENC->CON |= (1<<2);

	#if 0
	sdc_sendcmd(SDHCMD(CMD_WRITE_BLOCK, 5), 1*512, resp);
	xprintf("write---%08x\n", resp[0]);

	memset(buffer, 0x24, sizeof buffer);

	sdh_data_tx(buffer, 1, 512);
	sdh_data_wait();
	#endif

	#if 0
	sdc_sendcmd(SDHCMD(CMD_ERASE_ER_BLK_START, 5), 0, resp);
	sdc_sendcmd(SDHCMD(CMD_ERASE_ER_BLK_END, 5), capacity * 512 - 1, resp);
	sdc_sendcmd(SDHCMD(CMD_ERASE, 5), 1, resp);
	xprintf("resp===%08x\n", resp[0]);

	for (int i = 0; i < 1000; i++) {
		sdc_sendcmd(SDHCMD(CMD_SEND_STATUS, 5), sdc_rca << 16, resp);
		xprintf("Status = %08x\n", resp[0]);
		if ((resp[0] & 0x1e00) == 0x800) break;
		delay(100);
	}
	#endif

	JL_PERIENC->CON |= (1<<7);	/* reset key */
	//JL_PERIENC->CON &= ~(1<<2);

	#if 1
	sdh_data_rx(buffer, 1, 512);
	sdc_sendcmd(SDHCMD(CMD_READ_SINGLE_BLOCK, 5), 0*512, resp);
	xprintf("read---%08x\n", resp[0]);
	sdh_data_wait();
	hexdump(buffer, 512);
	#endif

	#if 0
	sdh_data_rx(buffer, 1, 512);
	sdc_sendcmd(SDHCMD(CMD_READ_MULTIPLE_BLOCKS, 5), 0*512, resp);
	xprintf("read---%08x\n", resp[0]);

	sdh_data_wait();
	hexdump(buffer, sizeof buffer);

	JL_PERIENC->CON |= (1<<7);	/* reset key */

	sdh_data_rx(buffer, 1, 512);
	sdh_data_wait();
	hexdump(buffer, sizeof buffer);

	sdc_sendcmd(SDHCMD(CMD_STOP_TRANSMISSION, 1), 0, resp);
	xprintf("stop---%08x\n", resp[0]);
	#endif
}



#include "ff/ff.h"
#include "ff/diskio.h"

DSTATUS disk_initialize(BYTE drv) {
	if (drv != 0) return STA_NOINIT;

	return 0;
}

DSTATUS disk_status(BYTE drv) {
	if (drv != 0) return STA_NOINIT;

	return 0;
}

DRESULT disk_read(BYTE drv, BYTE *ptr, LBA_t lba, UINT cnt) {
	uint32_t addr = lba, resp;
	uint8_t cmd;

	if (drv != 0) return RES_NOTRDY;
	if (cnt < 1) return RES_PARERR;

	if (!(sdc_ctype & CT_BLOCK))
		addr *= 512;

	JL_PERIENC->CON |= (1<<7);	/* reset key */
	sdh_data_rx(ptr, 1, 512);

	cmd = cnt > 1 ? CMD_READ_MULTIPLE_BLOCKS : CMD_READ_SINGLE_BLOCK;
	if (!sdc_sendcmd(SDHCMD(cmd, 5), addr, &resp) && !(resp & 0xc0580000)) {
		while (cnt > 0) {
			if (sdh_data_wait())
				break;	/* error occured */

			/* Still need to receive blocks? */
			if (--cnt > 0) {
				JL_PERIENC->CON |= (1<<7);	/* reset key */

				ptr += 512;
				sdh_data_rx(ptr, 1, 512);
			}
		}
	}

	/* error or multiblock read */
	if (cnt || cmd == CMD_READ_MULTIPLE_BLOCKS)
		sdc_sendcmd(SDHCMD(CMD_STOP_TRANSMISSION, 1), 0, NULL);

	return cnt ? RES_ERROR : RES_OK;
}

DRESULT disk_write(BYTE drv, const BYTE *ptr, LBA_t lba, UINT cnt) {
	uint32_t addr = lba, resp;
	uint8_t cmd;

	if (drv != 0) return RES_NOTRDY;
	if (cnt < 1) return RES_PARERR;

	if (!(sdc_ctype & CT_BLOCK))
		addr *= 512;

	if (cnt > 1) {
		cmd = (sdc_ctype & CT_SDC) ? ACMD_SET_WR_BLK_ERASE_COUNT : CMD_SET_BLK_COUNT;
		if (sdc_sendcmd(SDHCMD(cmd, 5), cnt, &resp) || (resp & 0xc0580000))
			return RES_ERROR;

		cmd = CMD_WRITE_MULTIPLE_BLOCK;
	} else {
		cmd = CMD_WRITE_BLOCK;
	}

	if (!sdc_sendcmd(SDHCMD(cmd, 5), addr, &resp) && !(resp & 0xc0580000)) {
		while (cnt > 0) {
			JL_PERIENC->CON |= (1<<7);	/* reset key */

			sdh_data_tx((void *)ptr, 1, 512);
			if (sdh_data_wait())
				break;	/* error occured */

			ptr += 512;
			cnt--;
		}
	}

	/* error or SDC multiblock write */
	if (cnt || (cmd == CMD_WRITE_MULTIPLE_BLOCK && (sdc_ctype & CT_SDC)))
		sdc_sendcmd(SDHCMD(CMD_STOP_TRANSMISSION, 1), 0, NULL);

	return cnt ? RES_ERROR : RES_OK;
}

DRESULT disk_ioctl(BYTE drv, BYTE cmd, void *ptr) {
	if (drv != 0) return RES_NOTRDY;

	switch (cmd) {
	case CTRL_SYNC:
		return RES_OK;

	case GET_SECTOR_COUNT:
		*(LBA_t*)ptr = 16384;
		return RES_OK;

	case GET_BLOCK_SIZE:
		*(DWORD*)ptr = 1;
		return RES_OK;
	}

	return RES_PARERR;
}

DWORD get_fattime(void) {
	return 0x12345678;
}



void lsdirs(char *path) {
	DIR dir;
	FILINFO finfo;

	if (f_opendir(&dir, path) == FR_OK) {
		for (;;) {
			if (f_readdir(&dir, &finfo) != FR_OK || !finfo.fname[0])
				break;

			xprintf("[%s] (%d) %02x\n", finfo.fname, finfo.fsize, finfo.fattrib);
		}

		f_closedir(&dir);
	}
}


void fftest(void) {
	static FATFS ftfs;
	static FIL fil;
	FRESULT fr;

	JL_PERIENC->CON |= (1<<2);

	#if 0
	MKFS_PARM parm = {
		.fmt = FM_FAT|FM_SFD,
		.n_fat = 2,
		.align = 1,
		.n_root = 512,
		.au_size = 4096,
	};
	fr = f_mkfs("0:", &parm, buffer, sizeof buffer);
	xprintf("---->%d\n", fr);
	#endif

	fr = f_mount(&ftfs, "0:", 1);
	xprintf("---->%d\n", fr);
	if (fr != FR_OK) return;

	#if 0
	if ((fr = f_open(&fil, "0:moachen.txt", FA_WRITE|FA_CREATE_ALWAYS)) == FR_OK) {
		DWORD cnt;

		f_puts("Is it even fair?\n", &fil);
		f_printf(&fil, "Text file: %02x\n", sdc_ctype);
		f_printf(&fil, "Random String: %08x\n", JL_RAND->R64L);

		f_close(&fil);
	}
	#endif

	#if 1
	if ((fr = f_open(&fil, "0:moachen.txt", FA_READ)) == FR_OK) {
		DWORD cnt;

		f_read(&fil, buffer, sizeof buffer, &cnt);
		hexdump(buffer, cnt);

		f_close(&fil);
	}
	#endif

	#if 1
	if ((fr = f_open(&fil, "0:gochon.bin", FA_READ)) == FR_OK) {
		DWORD cnt;

		f_read(&fil, (void *)0x20000, 0x10000, &cnt);

		f_close(&fil);
	}
	#endif

	lsdirs("0:");
}


void sdhfun(void) {
	/*============ Init GPIOs =============*/
	/* PB4 = DAT0, PB6 = CMD, PB7 = CLK */
	reg_wsmask(JL_IOMAP->CON0, 8, 0x7, 'F'-'A');	/* sd0 map F */
	JL_IOMAP->CON0 |= (1<<0)|(1<<1);	/* en sd0 clk & sd0 cmd/dat */

	JL_PORTB->PU |= (1<<4)|(1<<6);		/* dat0/cmd pullup */
	JL_PORTB->PU &= ~(1<<7);		/* clk no pullup */
	JL_PORTB->PD |= (1<<7);			/* clk pulldown */
	JL_PORTB->PD &= ~((1<<4)|(1<<6));	/* dat0/cmd no pulldown */

	JL_PORTB->DIR &= ~((1<<4)|(1<<6)|(1<<7));	/* dat0/cmd/clk out */
	JL_PORTB->DIE |= (1<<4)|(1<<6)|(1<<7);		/* dat0/cmd/clk digital in en */

	/*============ The Controller ========== */
	/* reset all regs */
	sdh->CON0 = (1<<14)|(1<<6);	/* clr pend dat and cmd */
	sdh->CON1 = 0;
	sdh->CON2 = 512-1;
	sdh->CTU_CON = (1<<6);	/* clear flag1 */
	sdh->CTU_CNT = 0;

	/* hw enable */
	sdh->CON1 |= (1<<0);
	sdh->CTU_CON |= (1<<0);

	/* idle clock enable */
	/* sdh->CON1 |= (1<<5); */

	/* 4-bit bus width */
	/* sdh->CON0 &= ~(1<<13); */

	/* clock divider */
	reg_wsmask(sdh->CON1, 8, 0xff, 256 - 1);

	/*============ ha do yo do ============*/

	if (!sdc_probe())
		sdhfun_sdc();

	reg_wsmask(sdh->CON1, 8, 0xff, (48000000 / 2000000) - 1);

	fftest();
}



#if 0
struct usb_msc_cbw {
	uint32_t sign;
	uint32_t tag;
	uint32_t data_len;
	uint8_t flags;
	uint8_t lun;
	uint8_t cb_len;
	uint8_t cb[16];
};

struct jl_loader_args {
	void (*msd_send)(void *ptr, int len);
	void (*msd_recv)(void *ptr, int len);
	int (**msd_hook)(struct usb_msc_cbw *cbw, void *buff);
	uint32_t arg;
	uint32_t wtw;
};

struct jl_loader_args *largs;

int msc_hook(struct usb_msc_cbw *cbw, void *buff) {
	xprintf(">>> HOOK: %x, %x <<<\n", cbw, buff);

	xprintf("tag=%08x, datalen=%d, flags=%02x, lun=%d\n",
		cbw->tag, cbw->data_len, cbw->flags, cbw->lun);

	xputs("cb: ");
	for (int i = 0; i < cbw->cb_len; i++) xprintf("%02x", cbw->cb[i]);
	xputc('\n');

	return 0;
}
#endif




void JieLi(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
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

	xprintf("got args: r0=%x r1=%x r2=%x r3=%x\n", r0, r1, r2, r3);

	p33_xfer(0, P33_OP_WRITE, 0x80, 0x00);

	/*---------------------------------------------------*/

	sdhfun();
}
