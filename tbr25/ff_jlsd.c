#include "ff/ff.h"
#include "ff/diskio.h"

#include <jl_br25_regs.h>
#include <xprintf.h>


extern void hexdump(void *ptr, int len);
extern void delay(int ms);

/*=================================================================*/

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

/*=================================================================*/

DSTATUS disk_initialize(BYTE drv) {
	if (drv != 0) return STA_NOINIT;

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

	if (sdc_probe())
		return STA_NOINIT;

	reg_wsmask(sdh->CON1, 8, 0xff, 16 - 1);

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
