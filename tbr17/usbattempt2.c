#include <jl_br17_regs.h>
#include <jl_irq.h>
#include <xprintf.h>

void uputc(int c) {
	while (!(JL_UART2->CON & (1<<15)));
	JL_UART2->BUF = c;
//	JL_UART2->CON |= (1<<13);
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

	JL_SFC->BAUD = 32-1;

	JL_SFC->BASE_ADR = 0x10000;

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


void cpuloops(int n) {
	while (n--)
		asm volatile ("nop");
}

/*=========================================================================*/

/* Common USB registers */
#define MC_FAddr		0x00	/* 00 */
#define MC_Power		0x01	/* 01 */
#define 	MC_Power_EnableSuspendM		(1<<0)
#define 	MC_Power_SuspendMode		(1<<1)
#define 	MC_Power_Resume			(1<<2)
#define 	MC_Power_Reset			(1<<3)
#define 	MC_Power_HSMode			(1<<4)
#define 	MC_Power_HSEnab			(1<<5)
#define 	MC_Power_SoftConn		(1<<6)
#define 	MC_Power_ISOUpdate		(1<<7)
#define MC_IntrTxL		0x02	/* 02 */
#define MC_IntrTxH		0x03	/* 03 */
#define MC_IntrRxL		0x04	/* 04 */
#define MC_IntrRxH		0x05	/* 05 */
#define MC_IntrTxEL		0x07	/* 06 */
#define MC_IntrTxEH		0x08	/* 07 */
#define MC_IntrRxEL		0x09	/* 08 */
#define MC_IntrRxEH		0x0A	/* 09 */
#define MC_IntrUSB		0x06	/* 0A */
#define 	MC_IntrUSB_Suspend		(1<<0)
#define 	MC_IntrUSB_Resume		(1<<1)
#define 	MC_IntrUSB_Reset		(1<<2)	/* peri */
#define 	MC_IntrUSB_Babble		(1<<2)	/* host */
#define 	MC_IntrUSB_SOF			(1<<3)
#define 	MC_IntrUSB_Conn			(1<<4)
#define 	MC_IntrUSB_Discon		(1<<5)
#define 	MC_IntrUSB_SessReq		(1<<6)
#define 	MC_IntrUSB_VBusError		(1<<7)
#define MC_IntrUSBE		0x0B	/* 0B */
#define MC_FrameL		0x0C	/* 0C */
#define MC_FrameH		0x0D	/* 0D */
#define MC_Index		0x0E	/* 0E */
					/* 0F */
/* Indexed endpoint regs */
#define MC_TxMaxP		0x10	/* 10 */
					/* 11 */
#define MC_CSR0			0x11	/* 12 */
#define 	MC_CSR0_RxPktRdy		(1<<0)
#define 	MC_CSR0_TxPktRdy		(1<<1)
#define 	MC_CSR0_SentStall		(1<<2)	/* peri */
#define 	MC_CSR0_RxStall			(1<<2)	/* host */
#define 	MC_CSR0_DataEnd			(1<<3)	/* peri */
#define 	MC_CSR0_SetupPkt		(1<<3)	/* host */
#define 	MC_CSR0_SetupEnd		(1<<4)	/* peri */
#define 	MC_CSR0_Error			(1<<4)	/* host */
#define 	MC_CSR0_SendStall		(1<<5)	/* peri */
#define 	MC_CSR0_ReqPkt			(1<<5)	/* host */
#define 	MC_CSR0_ServicedRxPktRdy	(1<<6)	/* peri */
#define 	MC_CSR0_StatusPkt		(1<<6)	/* host */
#define 	MC_CSR0_ServicedSetupEnd	(1<<7)	/* peri */
#define 	MC_CSR0_NAKTimeout		(1<<7)	/* host */
#define MC_TxCSRL		0x11	/* 12 */
#define 	MC_TxCSRL_TxPktRdy		(1<<0)
#define 	MC_TxCSRL_FIFONotEmpty		(1<<1)
#define 	MC_TxCSRL_UnderRun		(1<<2)	/* peri */
#define 	MC_TxCSRL_Error			(1<<2)	/* host */
#define 	MC_TxCSRL_FlushFIFO		(1<<3)
#define 	MC_TxCSRL_SendStall		(1<<4)	/* peri */
#define 	MC_TxCSRL_SetupPkt		(1<<4)	/* host */
#define 	MC_TxCSRL_SentStall		(1<<5)	/* peri */
#define 	MC_TxCSRL_RxStall		(1<<5)	/* host */
#define 	MC_TxCSRL_ClrDataTog		(1<<6)
#define 	MC_TxCSRL_IncompTx		(1<<7)
#define 	MC_TxCSRL_NAKTimeout		(1<<7)	/* host */
#define MC_TxCSRH		0x12	/* 13 */
#define 	MC_TxCSRH_DataToggle		(1<<0)	/* host */
#define 	MC_TxCSRH_DataToggleWrEnable	(1<<1)	/* host */
#define 	MC_TxCSRH_DMAReqMode		(1<<2)
#define 	MC_TxCSRH_FrcDataTog		(1<<3)
#define 	MC_TxCSRH_DMAReqEnab		(1<<4)
#define 	MC_TxCSRH_Mode			(1<<5)
#define 	MC_TxCSRH_ISO			(1<<6)	/* peri */
#define 	MC_TxCSRH_AutoSet		(1<<7)
#define MC_RxMaxP		0x13	/* 14 */
					/* 15 */
#define MC_RxCSRL		0x14	/* 16 */
#define 	MC_RxCSRL_RxPktRdy		(1<<0)
#define 	MC_RxCSRL_FIFOFull		(1<<1)
#define 	MC_RxCSRL_OverRun		(1<<2)	/* peri */
#define 	MC_RxCSRL_Error			(1<<2)	/* host */
#define 	MC_RxCSRL_DataError		(1<<3)
#define 	MC_RxCSRL_NAKTimeout		(1<<3)	/* host */
#define 	MC_RxCSRL_FlushFIFO		(1<<4)
#define 	MC_RxCSRL_SendStall		(1<<5)	/* peri */
#define 	MC_RxCSRL_ReqPkt		(1<<5)	/* host */
#define 	MC_RxCSRL_SentStall		(1<<6)	/* peri */
#define 	MC_RxCSRL_RxStall		(1<<6)	/* host */
#define 	MC_RxCSRL_ClrDataTog		(1<<7)
#define MC_RxCSRH		0x15	/* 17 */
#define 	MC_RxCSRL_IncompRx		(1<<0)
#define 	MC_RxCSRL_DataToggle		(1<<1)	/* host */
#define 	MC_RxCSRL_DataToggleWrEnable	(1<<2)	/* host */
#define 	MC_RxCSRL_DMAReqMode		(1<<3)
#define 	MC_RxCSRL_DisNyet		(1<<4)	/* peri */
#define 	MC_RxCSRL_PIDError		(1<<4)
#define 	MC_RxCSRL_DMAReqEnab		(1<<5)
#define 	MC_RxCSRL_ISO			(1<<6)	/* peri */
#define 	MC_RxCSRL_AutoReq		(1<<6)	/* host */
#define 	MC_RxCSRL_AutoClear		(1<<7)
#define MC_Count0		0x16	/* 18 */
#define MC_RxCountL		0x16	/* 18 */
#define MC_RxCountH		0x17	/* 19 */
#define MC_TxType		0x18	/* 1A */
#define MC_TxInterval		0x19	/* 1B */
#define MC_RxType		0x1A	/* 1C */
#define MC_RxInterval		0x1B	/* 1D */
					/* 1E */
					/* 1F */
/* Additional control&config regs */
#define MC_DevCtl		0x0F	/* 60 */

uint8_t usb_ep0_buff[64];
uint8_t usb_ep1r_buff[2][64];

char usb_ep1r_buff_sel;


uint8_t usb_mc_read(char addr) {
	JL_USB->CON1 = (addr << 8) | (1<<14);
	while (!(JL_USB->CON1 & (1<<15)));
	return JL_USB->CON1 & 0xff;
}

void usb_mc_write(char addr, uint8_t val) {
	JL_USB->CON1 = (addr << 8) | val;
	while (!(JL_USB->CON1 & (1<<15)));
}

void usb_mc_wmask(char addr, uint8_t mask, uint8_t val) {
	usb_mc_write(addr, (usb_mc_read(addr) & ~mask) | (val & mask));
}



#define MSC_CBW_SIGN		0x43425355	/* USBC */
#define MSC_CSW_SIGN		0x53425355	/* USBS */

struct usb_msc_cbw {
	uint32_t	sign;		/* "USBC" 0x43425355 */
	uint32_t	tag;
	uint32_t	xfer_len;
	uint8_t		flags;
	uint8_t		lun;
	uint8_t		cb_len;
	uint8_t		cb[31];
};

struct usb_msc_csw {
	uint32_t	sign;		/* "USBS" 0x53425355 */
	uint32_t	tag;
	uint32_t	rem_len;
	uint8_t		status;		/* 0 = ok, 1 = cmd failed, 2 = phase error */
};




struct usb_setup_pkt {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
};



const uint8_t usb_device_desc[] = {
	18,			/* bLength */
	0x01,			/* bDescriptorType */
	0x10,0x01,		/* bcdUSB */
	0,			/* bDeviceClass */
	0,			/* bDeviceSubClass */
	0,			/* bDeviceProtocol */
	0x40,			/* bMaxPacketSize */
	0xca,0x67,		/* idVendor */		/*0x67ca = hiiragi, 0x6c34 = mizu*/
	0x69,0xac,		/* idProduct */
	0x95,0x01,		/* bcdDevice */
	1,			/* iManufacturer */
	2,			/* iProduct */
	3,			/* iSerialNumber */
	1			/* bNumConfigurations */
};

const uint8_t usb_config_desc[] = {
	9,			/* bLength */
	0x02,			/* bDescriptorType */
	0x20,0x00,		/* wTotalLength */
	1,			/* bNumInterfaces */
	1,			/* bConfigurationValue */
	2,			/* iConfiguration */
	0x80,			/* bmAttributes */
	100/2,			/* bMaxPower */

		9,			/* bLength */
		0x04,			/* bDescriptorType */
		0,			/* bInterfaceNumber */
		0,			/* bAlternateSetting */
		2,			/* bNumEndpoints */
		0x08,			/* bInterfaceClass */
		0x06,			/* bInterfaceSubClass */
		0x50,			/* bInterfaceProtocol */
		0,			/* iInterface */

			/* EP1 -> Bulk In */
			7,			/* bLength */
			0x05,			/* bDescriptorType */
			0x81,			/* bEndpointAddress */
			0x02,			/* bmAttributes */
			0x40,0x00,		/* wMaxPacketSize */
			0,			/* bInterval */

			/* EP1 -> Bulk Out */
			7,			/* bLength */
			0x05,			/* bDescriptorType */
			0x01,			/* bEndpointAddress */
			0x02,			/* bmAttributes */
			0x40,0x00,		/* wMaxPacketSize */
			0,			/* bInterval */
};

const uint8_t usb_string_desc_0[] = {
	2+2,		/* bLength */
	0x03,		/* bDescriptorType */
	0x09,0x04,	/* wLANGID[0] */
};

const uint8_t usb_string_desc_1[] = {
	2+8*2,		/* bLength */
	0x03,		/* bDescriptorType */
	'M',0, 'i',0, 'z',0, 'u',0, '-',0, 'D',0, 'E',0, 'C',0,
};

const uint8_t usb_string_desc_2[] = {
	2+(6+6+9)*2,	/* bLength */
	0x03,		/* bDescriptorType */
	'J',0, 'i',0, 'e',0, 'L',0, 'i',0, ' ',0,
	'A',0, 'C',0, '6',0, '9',0, 'x',0, 'x',0, ' ',0,
	'U',0, 'S',0, 'B',0, ' ',0, 'T',0, 'e',0, 's',0, 't',0,
};

const uint8_t usb_string_desc_3[] = {
	2+(12)*2,	/* bLength */
	0x03,		/* bDescriptorType */
	'2',0, '0',0, '1',0, '9',0, '0',0, '7',0, '1',0, '8',0,
	'1',0, '0',0, '3',0, '1',0,
};


char newaddress = 0;


char ep0_state = 0;
void *ep0_dataptr;
int ep0_datalen;



void usb_reset_state(void) {
	newaddress = 0;
	ep0_state = 0;
}

void usb_reset_regs(void) {
	static const uint8_t regs[] = {
		MC_Power,	MC_Power_ISOUpdate|MC_Power_EnableSuspendM,

		/* USB ints */
		MC_IntrUSBE,	MC_IntrUSB_Reset|MC_IntrUSB_Suspend,

		/* TX endpoint ints */
		MC_IntrTxEL,	(1<<0),		/* only EP0 */
		MC_IntrTxEH,	0,

		/* RX endpoint ints */
		MC_IntrRxEL,	0,
		MC_IntrRxEH,	0,

		/* EP1rx reset */
		MC_Index,	1,
		MC_RxCSRL,	MC_RxCSRL_ClrDataTog|MC_RxCSRL_FlushFIFO,
		MC_RxCSRH,	0,
		MC_RxMaxP,	8,	/* ?? */

		/* EP1tx reset */
		MC_Index,	1,
		MC_TxCSRL,	MC_TxCSRL_ClrDataTog|MC_TxCSRL_FlushFIFO,
		MC_TxCSRH,	0,
		MC_TxMaxP,	8,	/* ?? */
	};

	irq_disable(irqn_FUSB_CTL);

	/* write registers from the table */
	for (int i = 0; i < sizeof regs; i += 2)
		usb_mc_write(regs[i+0], regs[i+1]);

	irq_enable(irqn_FUSB_CTL);
}



void ep0_state_tx(void) {
	uint8_t csr0 = usb_mc_read(MC_CSR0);
	xprintf("<><> TX state, CSR0 = %02x\n", csr0);

	if (csr0 & MC_CSR0_TxPktRdy)
		return;

	xprintf("gonna! %x, %d\n", ep0_dataptr, ep0_datalen);

	int n = ep0_datalen > 64 ? 64 : ep0_datalen;

	memcpy(&usb_ep0_buff, ep0_dataptr, n);
	JL_USB->EP0_CNT = n;

	ep0_dataptr += n;
	ep0_datalen -= n;

	if (ep0_datalen == 0) {
		xputs("NO data left\n");
		usb_mc_write(MC_CSR0, MC_CSR0_TxPktRdy|MC_CSR0_DataEnd);
		ep0_state = 0;
	} else {
		xputs("Still going!\n");
		usb_mc_write(MC_CSR0, MC_CSR0_TxPktRdy);
	}
}

void ep0_state_rx(void) {
	uint8_t csr0 = usb_mc_read(MC_CSR0);
	xprintf("<><> RX state, CSR0 = %02x\n", csr0);

	if (!(csr0 & MC_CSR0_RxPktRdy))
		return;
}



int usb_setup_get_desc(struct usb_setup_pkt *setup) {
	const uint8_t *desc = NULL;

	switch (setup->wValue >> 8) {
	case 1: desc = usb_device_desc; break;
	case 2: desc = usb_config_desc; break;

	case 3:
		switch (setup->wValue & 0xff) {
		case 0: desc = usb_string_desc_0; break;
		case 1: desc = usb_string_desc_1; break;
		case 2: desc = usb_string_desc_2; break;
		case 3: desc = usb_string_desc_3; break;
		}
		break;
	}

	if (desc) {
		uint16_t len = desc[0];
		if (desc[1] == 0x02) len = desc[2] | (desc[3] << 8);
		if (len > setup->wLength) len = setup->wLength;

		usb_mc_write(MC_CSR0, MC_CSR0_ServicedRxPktRdy);

		ep0_dataptr = (void *)desc;
		ep0_datalen = len;
		ep0_state = 1;

		ep0_state_tx();

		return 0;
	}

	return -1;
}

void ep0_state_idle(void) {
	uint8_t csr0 = usb_mc_read(MC_CSR0);
	xprintf("<><> Idle state, CSR0 = %02x\n", csr0);

	if (!(csr0 & MC_CSR0_RxPktRdy))
		return;

	struct usb_setup_pkt *setup = (void *)&usb_ep0_buff;
	char stall = 1, ack = 1;

	xprintf("\e[36m//// type:%02x req:%02x val:%04x idx:%04x len:%04x ////\e[0m\n",
		setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);

	switch (setup->bmRequestType & 0x7f) {
	/* Standard, Device */
	case 0x00:
		switch (setup->bRequest) {
		case 0x00: /* Get status */
			if (!(setup->bmRequestType & 0x80)) break;

			usb_mc_write(MC_CSR0, MC_CSR0_ServicedRxPktRdy);
			usb_ep0_buff[0] = 0x01;
			usb_ep0_buff[1] = 0x00;
			JL_USB->EP0_CNT = 2;
			usb_mc_write(MC_CSR0, MC_CSR0_TxPktRdy|MC_CSR0_DataEnd);

			stall = ack = 0;
			break;

		case 0x05: /* Set address */
			if (setup->bmRequestType & 0x80) break;
			newaddress = setup->wValue;
			stall = 0;
			break;

		case 0x06: /* Get descriptor */
			if (!(setup->bmRequestType & 0x80)) break;
			stall = usb_setup_get_desc(setup);
			ack = 0;
			break;

		case 0x09: /* Set configuration */
			if (setup->bmRequestType & 0x80) break;
			stall = 0;
			break;
		}
		break;

	/* Standard, Endpoint */
	case 0x02:
		usb_mc_write(MC_Index, setup->wIndex & 3);
		xprintf("Rx %02x-%02x / Tx %02x-%02x\n",
			usb_mc_read(MC_RxCSRL), usb_mc_read(MC_RxCSRH),
			usb_mc_read(MC_TxCSRL), usb_mc_read(MC_TxCSRH));
		usb_mc_write(MC_Index, 0);
		break;
	}

	if (stall) {
		usb_mc_write(MC_CSR0, MC_CSR0_ServicedRxPktRdy|MC_CSR0_SendStall);
	} else
	if (ack) {
		usb_mc_write(MC_CSR0, MC_CSR0_ServicedRxPktRdy|MC_CSR0_DataEnd);
	}
}




void usb_ep0_irq(void) {
	uint8_t tmp;

	/* select EP0 */
	usb_mc_write(MC_Index, 0);

	/* set the address, if there is one to set */
	if (newaddress) {
		usb_mc_write(MC_FAddr, newaddress);
		newaddress = 0;
	}

	tmp = usb_mc_read(MC_CSR0);
	xprintf("*** CSR0=%02x\n", tmp);

	if (tmp & MC_CSR0_SentStall) {
		xputs("--- sent stall! ---\n");
		usb_mc_write(MC_CSR0, 0);

		ep0_state = 0;
	}

	if (tmp & MC_CSR0_SetupEnd) {
		xputs("--- setup end! ---\n");
		usb_mc_write(MC_CSR0, MC_CSR0_ServicedSetupEnd);

		ep0_state = 0;
	}

	switch (ep0_state) {
	case 0: ep0_state_idle(); break;
	case 1: ep0_state_tx();   break;
	case 2: ep0_state_rx();   break;
	}
}




void IRQ_HANDLER USB_Ctrl_Handler(void) {
	uint8_t oldidx, istat;

	xputs("\n\e[1;33m========> USB IRQ (Ctrl)\e[0m\n");

	/* Select EP0 */
	oldidx = usb_mc_read(MC_Index);
	usb_mc_write(MC_Index, 0);

	/*------ USB Interrupts ------*/
	istat = usb_mc_read(MC_IntrUSB);
	if (istat) xprintf("//// IntrUSB: %02x\n", istat);

	if (istat & MC_IntrUSB_Resume) {
		xputs("(((( Resume ))))\n");
	}

	if (istat & MC_IntrUSB_Reset) {
		xputs("(((( Reset ))))\n");

		usb_reset_state();
		usb_reset_regs();
	}

	/*------ Endpoint interrupts ------*/
	istat = usb_mc_read(MC_IntrTxL);
	if (istat) xprintf("//// IntrTxL: %02x\n", istat);

	if (istat & (1<<0)) {
		xputs("(((( Endpoint0 ))))\n");

		usb_ep0_irq();
	}

	istat = usb_mc_read(MC_IntrRxL);
	if (istat) xprintf("//// IntrRxL: %02x\n", istat);

	/* Restore the index */
	usb_mc_write(MC_Index, oldidx);

	irq_latch_clear(irqn_FUSB_CTL);

	xputs("\n\e[1;33m<========\e[0m\n");
}



void usb_init(void) {
	irq_disable(irqn_FUSB_CTL);

	/* disable */
	JL_USB->IO_CON &= ~(1<<6);	/* disable D+ pullup */
	cpuloops(10000);

	/* clear stuff */
	JL_USB->CON0 &= ~0x40B7;
	JL_USB->CON1 = 0;

	/* enable phy */
	JL_USB->CON0 |= (1<<0);

	/* pullup D+, pulldown D- */
	JL_USB->IO_CON &= ~(3<<4);	/* no pulldown */
	JL_USB->IO_CON &= ~(3<<6);	/* no pullup */
	JL_USB->IO_CON |= (1<<6);	/* D+ pullup */
	JL_USB->IO_CON |= (1<<5);	/* D- pulldown */

	/* ?? vbus, cid, tm1, usb_nrst */
	JL_USB->CON0 |= 0x3C;

	/* setup some buffers */
	JL_USB->EP0_ADR  = (uint32_t)usb_ep0_buff;
	JL_USB->EP1_RADR = (uint32_t)&usb_ep1r_buff[0];

	usb_ep1r_buff_sel = 0;

	/* Enable SIE interrupts */
	JL_USB->CON0 |= (1<<11);

	irq_enable(irqn_FUSB_CTL);
}



struct usb_msc_cbw thecbw;
struct usb_msc_csw thecsw;


int usb_ep1_rx(void *ptr, int len) {
	return 1;
}

int usb_ep1_tx(void *ptr, int len) {
	return 1;
}



void usb_mainloop(void) {
	char got_cbw = 0;

	usb_mc_write(MC_Index, 1);
	if (usb_mc_read(MC_RxCSRL) & MC_RxCSRL_RxPktRdy) {
		int n = usb_mc_read(MC_RxCountL);
		xprintf("\e[1;32m***** Received:: %d\e[m\n", n);

		void *buff = (void *)&usb_ep1r_buff[usb_ep1r_buff_sel];

		/* swap rx buffers */
		usb_ep1r_buff_sel = !usb_ep1r_buff_sel;
		JL_USB->EP1_RADR = (uint32_t)&usb_ep1r_buff[usb_ep1r_buff_sel];

		/* reset buffer write pointer */
		usb_mc_write(MC_RxCSRL, MC_RxCSRL_FlushFIFO);

		hexdump(buff, n);
		memcpy(&thecbw, buff, sizeof thecbw);

		if (thecbw.sign == MSC_CBW_SIGN) {
			thecsw.sign = MSC_CSW_SIGN;
			thecsw.tag = thecbw.tag;
			thecsw.rem_len = thecbw.xfer_len;

			got_cbw = 1;
		} else {
			usb_mc_write(MC_RxCSRL, MC_RxCSRL_SendStall);
		}
	}

	if (got_cbw) {
		if (thecbw.xfer_len > 0) {
			if (thecbw.flags & 0x80)
				usb_mc_write(MC_TxCSRL, MC_TxCSRL_SendStall);
			else
				usb_mc_write(MC_RxCSRL, MC_RxCSRL_SendStall);
		}

		/* failed... */
		thecsw.status = 1;
		usb_ep1_tx(&thecsw, sizeof thecsw);
	}
}

void usb_fun(void) {
	irq_attach(irqn_FUSB_CTL, USB_Ctrl_Handler, 0);

	usb_init();

	for (;;) {
		JL_SYSTEM->WDT_CON |= (1<<6);
		usb_mainloop();
	}
}

/*=========================================================================*/

void JieLi(uint32_t args[4]) {
	reg_wsmask(JL_CLOCK->CLK_CON1, 10, 0x3, 0x1);	/* uart_clk = pll_48m */

	JL_UART2->CON = 1;
	JL_UART2->BAUD = (48000000 / 4 / 115200) - 1;

	reg_wsmask(JL_IOMAP->CON1, 14, 0x3, 'A'-'A');
	reg_wsmask(JL_IOMAP->CON3, 8, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<11);

	JL_PORTA->DIR &= ~(1<<3);	/* PA3 out */

	xdev_out(uputc);
	xputs("\n\nhello br17\n");

	init_sfc();
	hexdump((void *)0x1000000, 0x100);

	usb_fun();
}
