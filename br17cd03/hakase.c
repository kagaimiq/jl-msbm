#include <jl_br17_regs.h>
#include <jl_irtc.h>
#include <loader_args.h>

/*=========================================================================================*/

void uart_init(int baud) {
	reg_wsmask(JL_CLOCK->CLK_CON1, 10, 0x3, 0x1);	/* uart_clk = pll_48m */

	JL_UART2->CON = 1;
	JL_UART2->BAUD = (48000000 / 4 / baud) - 1;

	reg_wsmask(JL_IOMAP->CON1, 14, 0x3, 'A'-'A');
	reg_wsmask(JL_IOMAP->CON3, 8, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<11);

	JL_PORTA->DIR &= ~(1<<3);	/* PA3 out */
}

void putc(int c) {
	while (!(JL_UART2->CON & (1<<15)));
	JL_UART2->BUF = c;
}

void puts(char *s) {
	while (*s)
		putc(*s++);
}

void puthex4(int v) {
	if (v < 10)
		putc('0' + v);
	else
		putc('A' + v - 10);
}

void puthex32(uint32_t v) {
	puthex4(v >> 24);
	puthex4(v >> 20 & 0xf);
	puthex4(v >> 16 & 0xf);
	puthex4(v >> 12 & 0xf);
	puthex4(v >> 8 & 0xf);
	puthex4(v >> 4 & 0xf);
	puthex4(v & 0xf);
}

void puthex16(uint16_t v) {
	puthex4(v >> 12);
	puthex4(v >> 8 & 0xf);
	puthex4(v >> 4 & 0xf);
	puthex4(v & 0xf);
}

void puthex8(uint8_t v) {
	puthex4(v >> 4);
	puthex4(v & 0xf);
}

/*=========================================================================================*/

void cpuloops(int n) {
	while (n--)
		asm volatile ("nop");
}


const uint8_t scsi_inquiry[] = {
	/* Qualifier / Device type */
	0x00,
	/* RMB */
	0x80,	/* Removable */
	/* Version */
	0x00,	/* No standard conformance! */
	/* NORMACA / HISUP / Response data format */
	0x00,
	/* Additional length */
	0x1F,	/* 31? but there's 32 more! */
	/* SCCS / ACC / TPGS / 3PC / PROTECT */
	0x00,
	/* ENSERV / VS / MULTIP */
	0x00,
	/* CMDQUE / VS */
	0x00,
	/* Vendor */
	'C','D','0','3',' ',' ',' ',' ',
	/* Product */
	'D','E','V','I','C','E',' ','V','1','.','0','0',' ',' ',' ',' ',
	/* Revision */
	'1','.','0','0',
};


msd_send_func msd_send;
msd_recv_func msd_recv;
msd_hook_func l2hook;

uint8_t usb_buffer[512];


uint16_t crc16(void *ptr, int len) {
	JL_CRC->REG = 0;

	while (len--)
		JL_CRC->FIFO = *(uint8_t *)(ptr++);

	asm volatile ("csync");
	return JL_CRC->REG;
}

/*---------------------------------------------------*/

void jlcmd_write_memory(struct usb_msc_cbw *cbw) {
	uint32_t addr = cbw->cb[2] << 24 | cbw->cb[3] << 16 | cbw->cb[4] << 8 | cbw->cb[5];
	uint16_t len = cbw->cb[6] << 8 | cbw->cb[7];

	puts("---- Write memory ---- ");
	puthex32(addr); putc(' '); puthex16(len);
	putc('\n');

	msd_recv((void *)addr, len);
}

void jlcmd_jump_memory(struct usb_msc_cbw *cbw, uint8_t *pbuff) {
	uint32_t addr = cbw->cb[2] << 24 | cbw->cb[3] << 16 | cbw->cb[4] << 8 | cbw->cb[5];
	uint16_t wtw = cbw->cb[6] << 8 | cbw->cb[7];

	puts("---- Jump memory ---- ");
	puthex32(addr); putc(' '); puthex16(wtw);
	putc('\n');

	l2hook = 0;

	/* same calling convention as the v1 protocol */
	((loader_call_v1)addr)(msd_send, msd_recv, &l2hook);

	pbuff[0] = 0xFB;
	pbuff[1] = 0x09;
	msd_send(pbuff, 16);
}


void jlcmd_erase_block(struct usb_msc_cbw *cbw, uint8_t *pbuff) {
	uint32_t addr = cbw->cb[2] << 24 | cbw->cb[3] << 16 | cbw->cb[4] << 8 | cbw->cb[5];

	/* /// */
	puts("---- Erase block ---- ");
	puthex32(addr);
	putc('\n');

	pbuff[0] = 0xFB;
	pbuff[1] = 0x00;
	msd_send(pbuff, 16);
}

void jlcmd_erase_sector(struct usb_msc_cbw *cbw, uint8_t *pbuff) {
	uint32_t addr = cbw->cb[2] << 24 | cbw->cb[3] << 16 | cbw->cb[4] << 8 | cbw->cb[5];

	/* /// */
	puts("---- Erase sector ---- ");
	puthex32(addr);
	putc('\n');

	pbuff[0] = 0xFB;
	pbuff[1] = 0x03;
	msd_send(pbuff, 16);
}

void jlcmd_erase_chip(struct usb_msc_cbw *cbw, uint8_t *pbuff) {
	/* /// */
	puts("---- Erase chip ----\n");
	pbuff[0] = 0xFB;
	pbuff[1] = 0x02;
	msd_send(pbuff, 16);
}

void jlcmd_write_flash(struct usb_msc_cbw *cbw) {
	uint32_t addr = cbw->cb[2] << 24 | cbw->cb[3] << 16 | cbw->cb[4] << 8 | cbw->cb[5];
	uint16_t len = cbw->cb[6] << 8 | cbw->cb[7];
	uint16_t crc = cbw->cb[9] | cbw->cb[10] << 8;

	puts("---- Write flash ---- ");
	puthex32(addr); putc(' '); puthex16(len); putc(' '); puthex16(crc);
	putc('\n');

	msd_recv(usb_buffer, len);

	if (crc16(usb_buffer, len) == crc) {
		/* /// */
		puts("** CRC matched **\n");
	}
}

void jlcmd_read_flash(struct usb_msc_cbw *cbw) {
	uint32_t addr = cbw->cb[2] << 24 | cbw->cb[3] << 16 | cbw->cb[4] << 8 | cbw->cb[5];
	uint16_t len = cbw->cb[6] << 8 | cbw->cb[7];

	/* /// */
	puts("---- Read flash ---- ");
	puthex32(addr); putc(' '); puthex16(len);
	putc('\n');

	msd_send(usb_buffer, len);
}

void jlcmd_get_id(struct usb_msc_cbw *cbw, uint8_t *pbuff) {
	puts("---- Get ID ----\n");

	pbuff[0] = 0xFC;
	pbuff[1] = 0x00;

	/* /// */
	pbuff[2] = 0xEF;
	pbuff[3] = 0x40;
	pbuff[4] = 0x16;

	msd_send(pbuff, 16);
}

void jlcmd_online_device(struct usb_msc_cbw *cbw, uint8_t *pbuff) {
	puts("---- Online device ----\n");

	pbuff[0] = 0xFC;
	pbuff[1] = 0x0B;

	/* /// */
	pbuff[2] = 1;

	msd_send(pbuff, 16);
}

void jlcmd_select_flash(struct usb_msc_cbw *cbw, uint8_t *pbuff) {
	uint8_t sel = cbw->cb[2];

	puts("---- Select flash ---- ");
	puthex8(sel);
	putc('\n');

	/* /// */

	pbuff[0] = 0xFC;
	pbuff[1] = 0x0C;
	msd_send(pbuff, 16);
}

/*---------------------------------------------------*/

int msd_hook(struct usb_msc_cbw *cbw, void *pbuff) {
	puts("<<< Command ");
	puthex8(cbw->cb[0]); puthex8(cbw->cb[1]); putc(' '); puthex32((uint32_t)pbuff);
	puts(" >>>\n");

	/* Handle some SCSI stuff beforehand... */
	switch (cbw->cb[0]) {
	case 0x12:
		{
			uint16_t len = cbw->cb[3] << 8 | cbw->cb[4];
			msd_send(scsi_inquiry, len);
		}
		return 1;
	}

	/* Handle an additional hook */
	if (l2hook) {
		puts("... HOOK\n");
		if (l2hook(cbw, pbuff))
			return 1;
	}

	puts("Handle!\n");

	/* Handle the loader commands themselve */
	switch (cbw->cb[0]) {
	case 0xFB:
		switch (cbw->cb[1]) {
		case 0x00:
			jlcmd_erase_block(cbw, pbuff);
			return 1;
		case 0x01:
			jlcmd_write_flash(cbw);
			return 1;
		case 0x02:
			jlcmd_erase_chip(cbw, pbuff);
			return 1;
		case 0x03:
			jlcmd_erase_sector(cbw, pbuff);
			return 1;
		case 0x04:
			jlcmd_write_memory(cbw);
			return 1;
		case 0x09:
			jlcmd_jump_memory(cbw, pbuff);
			return 1;
		}
		break;

	case 0xFC:
		switch (cbw->cb[1]) {
		case 0x00:
			jlcmd_get_id(cbw, pbuff);
			return 1;
		case 0x0B:
			jlcmd_online_device(cbw, pbuff);
			return 1;
		case 0x0C:
			jlcmd_select_flash(cbw, pbuff);
			return 1;
		};
		break;

	case 0xFD:
		switch (cbw->cb[1]) {
		case 0x00:
			jlcmd_get_id(cbw, pbuff);
			return 1;
		case 0x01:
			jlcmd_read_flash(cbw);
			return 1;
		};
		break;
	}

	return 0;
}


void loader_main(struct jl_loader_args *largs) {
	uart_init(115200);
	puts("---- Konnichiwa, JIELI ----\n");

	/* Setup the MSD hook */
	msd_send = largs->msd_send;
	msd_recv = largs->msd_recv;
	*largs->msd_hook = msd_hook;

	/* replug the device in order to re-enumerate it */
	JL_USB->IO_CON &= ~(1<<6);
	cpuloops(10000);
	JL_USB->IO_CON |= (1<<6);

	/* return back into ROM */
}
