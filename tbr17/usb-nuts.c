#include <stdio.h>		/* There was a Micropython */
#include <stdint.h>
#include <string.h>

#include "jl_br17_regs.h"
#include "jl_irq.h"

extern void cpuloops(int cnt);

/*----------------------------------------------------------*/

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

/*----------------------------------------------------------*/

struct usb_setup_pkt {
	uint8_t		bmRequestType;
	uint8_t		bRequest;
	uint16_t	wValue;
	uint16_t	wIndex;
	uint16_t	wLength;
};

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

/*----------------------------------------------------------*/

struct {
	int (*msc_recv)(void *ptr, int len);
	int (*msc_send)(const void *ptr, int len);
	int (**msc_hook)(struct usb_msc_cbw *, void *);
	int arg;
} loader_arglist;

int usb_curr_rxbuff;					/* 00A9C */

struct {
	uint8_t scsi_sensei;
	uint8_t scsi_sense_key;
	uint8_t scsi_sense_code;
	uint8_t scsi_sense_qual;
	uint8_t field5_0x4;
} USB_Data1;

uint8_t var_AA8;

/* hole */

uint8_t MSC_MaxLUN;
uint8_t var_AB3;

struct {
	uint8_t		newdevaddr;		/* +00 */
	uint8_t		ep0state;		/* +01 */
	uint16_t	ep0txlen;		/* +02 */
	uint8_t		field3_0x4[7];		/* +04 */
	uint8_t		field4_0xb;		/* +0B */
	void		*ep0txptr;		/* +0C */
	uint8_t		txepstalled;		/* +10 */
	uint8_t		rxepstalled;		/* +11 */
	uint8_t		selconfig;		/* +12 */
	uint8_t		ep0forcetx;		/* +13 */
} USB_Data2;

int (*MSC_Hook)(struct usb_msc_cbw *, void *);
int (*MSC_Recv)(void *, int);
int (*MSC_Send)(const void *, int);
void (*MSC_SendTheCSW)(void);
int (*MSC_RecvTheCBW)(void);

uint8_t var_ADC;

/*-----------*/

uint8_t usb_ep1r_buff[2][64];		/* 40000, 40040 */
uint8_t usb_ep0_buff[64];		/* 40080 */
uint8_t msc_csw_buff[64];		/* 400C0 */
uint8_t TempBuff[64];			/* 40100 */
uint8_t msc_cbw_buff[64];		/* 40140 */
uint8_t TempBuff2[512];			/* 40180 */
uint8_t TempBuff3[512];			/* 40380 */

/*-----------*/

const uint8_t scsi_modesense[] = {
	0x03, 0x00, 0x00, 0x00
};

const uint8_t scsi_inquiry[] = {
	0x00, 0x80, 0x00, 0x00, 36-4-1, 0x00, 0x00, 0x00,
	'B','R','1','7',' ',' ',' ', ' ',
	'U','B','O','O','T','1','.','0','0',' ',' ',' ',' ',' ',' ',' ',
	'1','.','2','3',
};

const uint8_t usb_devicedesc[] = {
	18,		/* bLength */
	1,		/* bDescriptorType: device */
	0x10,0x01,	/* bcdUSB */
	0x00,		/* bDeviceClass */
	0x00,		/* bDeviceSubClass */
	0x00,		/* bDeviceProtocol */
	8,		/* bMaxPacketSize0 */
	0xB5,0xE2,	/* idVendor */
	0x11,0x69,	/* idProduct */
	0x00,0x01,	/* bcdDevice */
	1,		/* iManufacturer */
	2,		/* iProduct */
	0,		/* iSerial */
	1,		/* bNumConfigurations */
};

const uint8_t usb_configdesc[] = {
	9,		/* bLength */
	2,		/* bDescriptorType: configuration */
	0x20,0x00,	/* wTotalLength */
	1,		/* bNumInterfaces */
	1,		/* bConfigurationValue */
	0,		/* iConfiguration */
	0x80,		/* bmAttributes */
	400/2,		/* MaxPower */
};

const uint8_t usb_ifacedesc[] = {
	9,		/* bLength */
	4,		/* bDescriptorType: interface */
	0,		/* bInterfaceNumber */
	0,		/* bAlternateSetting */
	2,		/* bNumEndpoints */
	0x08,		/* bInterfaceClass - Mass storage */
	0x06,		/* bInterfaceSubClass - SCSI */
	0x50,		/* bInterfaceProtocol - Bulk-only*/
	0,		/* iInterface */

	7,		/* bLength */
	5,		/* bDescriptorType: endpoint */
	0x81,		/* bEndpointAddress: EP1, in */
	0x02,		/* bmAttributes: bulk */
	0x40,0x00,	/* wMaxPacketSize */
	1,		/* bInterval */

	7,		/* bLength */
	5,		/* bDescriptorType: endpoint */
	0x01,		/* bEndpointAddress: EP1, out */
	0x02,		/* bmAttributes: bulk */
	0x40,0x00,	/* wMaxPacketSize */
	1,		/* bInterval */
};

const uint8_t usb_stringdesc_0[] = {
	4,		/* bLength */
	3,		/* bDescriptorType */
	0x09,0x04,	/* lang id 0x0409 */
};

const uint8_t usb_stringdesc_1[] = {
	2+5*2,		/* bLength */
	3,		/* bDescriptorType */
	'J',0, 'i',0, 'e',0, 'L',0, 'i',0,
};

const uint8_t usb_stringdesc_2[] = {
	2+(4+1+5)*2,	/* bLength */
	3,		/* bDescriptorType */
	'B',0, 'R',0, '1',0, '7',0, ' ',0, 'U',0, 'B',0, 'o',0, 'o',0, 't',0,
};

/*----------------------------------------------------------*/

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

/*--------------------------------*/

void usb_state_reset(void) {
	memset(&USB_Data2, 0, sizeof USB_Data2);
	memset(&USB_Data1, 0, sizeof USB_Data1);
	memset(TempBuff, 0, sizeof TempBuff);
}

void usb_regs_reset(void) {
	irq_disable(irqn_FUSB_CTL);

	static const uint8_t regs[] = {
		MC_Power,	MC_Power_ISOUpdate|MC_Power_EnableSuspendM,

		MC_IntrUSBE,	MC_IntrUSB_Reset|MC_IntrUSB_Suspend,

		/* Tx EP int enable: EP0 int only */
		MC_IntrTxEL,	0x01,
		MC_IntrTxEH,	0x00,

		/* Rx EP int enable: None */
		MC_IntrRxEL,	0x00,
		MC_IntrTxEH,	0x00,

		/* Rx EP1 config */
		MC_Index,	1,
		MC_RxCSRL,	MC_RxCSRL_ClrDataTog|MC_RxCSRL_FlushFIFO,
		MC_TxCSRH,	0x00,
		MC_RxMaxP,	8,

		/* Tx EP1 config */
		MC_Index,	1,
		MC_TxCSRL,	MC_TxCSRL_ClrDataTog|MC_TxCSRL_FlushFIFO,
		MC_TxCSRH,	0x00,
		MC_TxMaxP,	8,
	};

	for (int i = 0; i < sizeof regs; i += 2) {
		usb_mc_write(regs[i+0], regs[i+1]);
	}

	irq_enable(irqn_FUSB_CTL);
}

/*--------------------------------*/

void usb_ep0_state_tx(void) {
	uint8_t stat;
	uint16_t len;

	stat = usb_mc_read(MC_CSR0);

	/* Still transmitting? */
	if (stat & MC_CSR0_TxPktRdy)
		return;

	len = USB_Data2.ep0txlen;
	if (len) {
		/* transmit up to 8 bytes at a time ... why ep0 needs to be so tiny in FS? */
		if (len > 8) len = 8;

		/* copy into the buffer */
		memcpy(usb_ep0_buff, USB_Data2.ep0txptr, len);
		USB_Data2.ep0txptr += len;
		USB_Data2.ep0txlen -= len;

		/* prepare to send x bytes */
		JL_USB->EP0_CNT = len;

		/* there are still data in buffer or it's forced to send */
		if (USB_Data2.ep0txlen || USB_Data2.ep0forcetx == 1) {
			/* finally transmit data */
			usb_mc_write(MC_CSR0, MC_CSR0_TxPktRdy);
			return;
		}
	} else {
		/* a null packet! */
		JL_USB->EP0_CNT = 0;
	}

	/* we're done sending! */
	usb_mc_write(MC_CSR0, MC_CSR0_TxPktRdy|MC_CSR0_DataEnd);

	/* idle state */
	USB_Data2.ep0state = 0;
}

void usb_ep0_state_rx(void) {
	/* ha! */
	var_AA8 = 0;

	usb_mc_wmask(MC_CSR0, MC_CSR0_ServicedRxPktRdy, ~0);		/* not sure why */
	usb_mc_write(MC_CSR0, MC_CSR0_ServicedRxPktRdy|MC_CSR0_DataEnd);

	USB_Data2.ep0state = 0;
}

void usb_prepare_config_desc(void) {
	memcpy(&TempBuff2[0], usb_configdesc, sizeof usb_configdesc);
	memcpy(&TempBuff2[9], usb_ifacedesc, sizeof usb_ifacedesc);
	TempBuff2[2] = 0x20;	/* wTotalLength */
	TempBuff2[3] = 0x00;		/* ,,,, */
	TempBuff2[4] = 1;	/* bNumInterfaces */
}

void usb_ep0_setup_pkt(void) {
	struct usb_setup_pkt *setup = (void *)usb_ep0_buff;
	int txlen, stall = 1;

	/* We arrive here after the RxPktRdy flag was checked to be set */
	printf("Setup packet, bmRequestType=%02x, bRequest=%02x, wValue=%04x, wIndex=%04x, wLength=%d\n",
		setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);

	switch ((setup->bmRequestType >> 5) & 3) {	/* <- like a classical approach */
	case 1:	/* Class */
		if (setup->bmRequestType == 0x21 && setup->bRequest == 0xff) {
			/* msc reset */
			stall = 0;
		} else
		if (setup->bmRequestType == 0xA1 && setup->bRequest == 0xfe) {
			/* get max lun */
			usb_mc_wmask(MC_CSR0, MC_CSR0_ServicedRxPktRdy, ~0);
			usb_ep0_buff[0] = MSC_MaxLUN;
			JL_USB->EP0_CNT = 1;
			usb_mc_write(MC_CSR0, MC_CSR0_TxPktRdy|MC_CSR0_DataEnd);

			var_ADC |= 4;
			USB_Data2.ep0state = 0;
			return;
		}
		break;

	case 0:	/* Standard */
		switch (setup->bRequest) {
		case 0:		/* Get status */
			usb_mc_wmask(MC_CSR0, MC_CSR0_ServicedRxPktRdy, ~0);
			usb_ep0_buff[0] = 1;
			usb_ep0_buff[1] = 0;
			JL_USB->EP0_CNT = 2;
			usb_mc_write(MC_CSR0, MC_CSR0_TxPktRdy|MC_CSR0_DataEnd);
			return;

		case 1:		/* Clear feature */
			
			break;

		case 5:		/* Set address */
			USB_Data2.newdevaddr = setup->wValue;
			stall = 0;
			break;

		case 6:		/* Get descriptor */
			txlen = -1;

			switch (setup->wValue >> 8) {
			case 1:	/* Device */
				USB_Data2.ep0txptr = (void *)usb_devicedesc;
				txlen = sizeof usb_devicedesc;
				break;

			case 2:	/* Configuration */
				usb_prepare_config_desc();
				USB_Data2.ep0txptr = TempBuff2;
				txlen = *(uint16_t *)&TempBuff2[2];	/* <- full length */
				break;

			case 3:	/* String */
				switch (setup->wValue & 0xff) {
				case 0:		/* Language ID */
					USB_Data2.ep0txptr = (void *)usb_stringdesc_0;
					txlen = sizeof usb_stringdesc_0;
					break;
				case 1:	/* other strings */
					USB_Data2.ep0txptr = (void *)usb_stringdesc_1;
					txlen = sizeof usb_stringdesc_1;
					break;
				case 2:
					USB_Data2.ep0txptr = (void *)usb_stringdesc_2;
					txlen = sizeof usb_stringdesc_2;
					break;
				}
				break;
			}

			if (txlen < 0) {
				/* well, it should stall, right? */
				usb_mc_wmask(MC_CSR0, MC_CSR0_ServicedRxPktRdy, ~0);
				usb_mc_write(MC_CSR0, MC_CSR0_TxPktRdy|MC_CSR0_DataEnd);
				return;
			} else {
				usb_mc_wmask(MC_CSR0, MC_CSR0_ServicedRxPktRdy, ~0);

				USB_Data2.ep0txlen = setup->wLength < txlen ? setup->wLength : txlen;
				USB_Data2.ep0forcetx = setup->wLength < txlen;
				USB_Data2.ep0state = 1;
				usb_ep0_state_tx();
			}
			return;

		case 9:		/* Set configuration */
			USB_Data2.selconfig = setup->wValue;
			stall = 0;
			break;

		case 11:	/* Set interface */
			stall = 0;
			break;
		}
		break;
	}

	if (stall) {
		/* send stall */
		usb_mc_write(MC_CSR0, MC_CSR0_ServicedRxPktRdy|MC_CSR0_SendStall);
	} else {
		/* end the packet */
		usb_mc_write(MC_CSR0, MC_CSR0_ServicedRxPktRdy|MC_CSR0_DataEnd);
	}
}

void usb_ep0_irq(void) {
	uint8_t stat;

	usb_mc_write(MC_Index, 0);

	var_AB3 |= 1;

	/*
	 * The device address should be set after sending an ack for the request.
	 *  In this case, it will happen when a transmission completion interrupt fires
	 *  for the null ack packet.
	 */
	if (USB_Data2.newdevaddr) {
		usb_mc_write(MC_FAddr, USB_Data2.newdevaddr);
		USB_Data2.newdevaddr = 0;
	}

	/* read CSR0 to determine interrupt source */
	stat = usb_mc_read(MC_CSR0);

	/* SentStall: clear this bit */
	if (stat & MC_CSR0_SentStall) {
		usb_mc_write(MC_CSR0, 0);
		USB_Data2.ep0state = 0;
		return;
	}

	/* SetupEnd: set ServicedSetupEnd */
	if (stat & MC_CSR0_SetupEnd) {
		usb_mc_write(MC_CSR0, MC_CSR0_ServicedSetupEnd);
		USB_Data2.ep0state = 0;
	}

	/* Handle the approprieate ep0 state */
	switch (USB_Data2.ep0state) {
	case 0:		/* idle */
		if (stat & MC_CSR0_RxPktRdy)
			usb_ep0_setup_pkt();
		break;
	case 1:		/* tx */
		usb_ep0_state_tx();
		break;
	case 2:		/* rx */
		usb_ep0_state_rx();
		break;
	}
}

void IRQ_HANDLER USBCtrl_handler(void) {
	uint8_t oldepidx, stat;

	oldepidx = usb_mc_read(MC_Index);
	usb_mc_write(MC_Index, 0);

	stat = usb_mc_read(MC_IntrUSB);

	if (stat & MC_IntrUSB_Resume) {
		USB_Data2.selconfig = 0;
	}

	if (stat & MC_IntrUSB_Reset) {
		usb_state_reset();
		usb_regs_reset();
	}

	stat = usb_mc_read(MC_IntrTxL);

	if (stat & (1<<0)) {
		usb_ep0_irq();
	}

	usb_mc_write(MC_Index, oldepidx);

	irq_latch_clear(irqn_FUSB_CTL);
}

/*--------------------------------*/

void usb_rxep_stall(int ep) {
	irq_disable(irqn_FUSB_CTL);

	usb_mc_write(MC_Index, ep);
	usb_mc_write(MC_RxCSRL, MC_RxCSRL_SendStall);
	USB_Data2.rxepstalled = 1;

	irq_enable(irqn_FUSB_CTL);

	while (var_ADC & 2) {
		if (USB_Data2.rxepstalled != 1 || USB_Data2.selconfig == 0)
			break;
	}

	/* ??? */
	usb_mc_write(MC_Index, ep);
}

void usb_txep_stall(int ep) {
	irq_disable(irqn_FUSB_CTL);

	usb_mc_write(MC_Index, ep);
	usb_mc_write(MC_TxCSRL, MC_TxCSRL_SendStall);
	USB_Data2.txepstalled = 1;

	irq_enable(irqn_FUSB_CTL);

	while (var_ADC & 2) {
		if (USB_Data2.txepstalled != 1)
			break;
	}

	/* ??? */
	usb_mc_write(MC_Index, ep);
}

int usb_sendep1(const void *buff, int len) {
	uint8_t stat;

	while (len > 0) {
		int n = len > 64 ? 64 : len;

		/* prepare to send */
		JL_USB->EP1_TADR = (uint32_t)buff;
		JL_USB->EP1_CNT = n;

		/* adjust stuff */
		len -= n;
		buff += n;

		/* send */
		irq_disable(irqn_FUSB_CTL);
		usb_mc_write(MC_Index, 1);
		usb_mc_write(MC_TxCSRL, MC_TxCSRL_TxPktRdy);
		irq_enable(irqn_FUSB_CTL);

		/* wait */
		do {
			JL_SYSTEM->WDT_CON |= (1<<6);		/* clear wdt */

			irq_disable(irqn_FUSB_CTL);
			usb_mc_write(MC_Index, 1);
			stat = usb_mc_read(MC_TxCSRL);
			irq_enable(irqn_FUSB_CTL);

			/* reset occured */
			if (USB_Data2.selconfig == 0)
				return 0;

			if (!(var_ADC & 2))
				return 0;
		} while (stat & MC_TxCSRL_TxPktRdy);
	}

	return 1;	/* Succeed */
}

int usb_recvep1(void *buff, int len) {
	uint8_t stat, *epbuff;

	while (len > 0) {
		int n = len > 64 ? 64 : len;

		do {
			JL_SYSTEM->WDT_CON |= (1<<6);		/* clear wdt */

			if (!(var_ADC & 2))
				return 0;

			irq_disable(irqn_FUSB_CTL);
			usb_mc_write(MC_Index, 1);
			stat = usb_mc_read(MC_RxCSRL);
			irq_enable(irqn_FUSB_CTL);

			/* Reset occured */
			if (USB_Data2.selconfig == 0)
				return 0;
		} while (!(stat & (MC_RxCSRL_RxPktRdy|MC_RxCSRL_FIFOFull)));

		epbuff = (uint8_t *)usb_ep1r_buff[usb_curr_rxbuff];
		usb_curr_rxbuff = !usb_curr_rxbuff;
		JL_USB->EP1_RADR = (uint32_t)usb_ep1r_buff[usb_curr_rxbuff];

		irq_disable(irqn_FUSB_CTL);
		usb_mc_write(MC_Index, 1);
		usb_mc_write(MC_RxCSRL, MC_RxCSRL_FlushFIFO);	/* reset write offset */
		irq_enable(irqn_FUSB_CTL);

		memcpy(buff, epbuff, len);

		len -= n;
		buff += n;
	}

	return 1;
}

int msc_recvcbw(void) {
	struct usb_msc_cbw *cbw = (void *)msc_cbw_buff;
	struct usb_msc_csw *csw = (void *)msc_csw_buff;
	uint8_t stat;

	irq_disable(irqn_FUSB_CTL);

	usb_mc_write(MC_Index, 1);
	stat = usb_mc_read(MC_RxCSRL);

	if (stat & MC_RxCSRL_RxPktRdy) {
		uint8_t *buff = (uint8_t *)&usb_ep1r_buff[usb_curr_rxbuff];
		usb_curr_rxbuff = !usb_curr_rxbuff;
		JL_USB->EP1_RADR = (uint32_t)usb_ep1r_buff[usb_curr_rxbuff];

		/* reset buffer write pointer, at least */
		usb_mc_write(MC_RxCSRL, MC_RxCSRL_FlushFIFO);

		memcpy(cbw, buff, sizeof (*cbw));

		if (cbw->sign == MSC_CBW_SIGN) {
			irq_enable(irqn_FUSB_CTL);

			csw->tag = cbw->tag;
			csw->rem_len = cbw->xfer_len;	/* there was a memcpy but it is dumb */
			return 0;	/* Received */
		} else {
			usb_rxep_stall(1);
		}
	}

	irq_enable(irqn_FUSB_CTL);
	return 1;	/* Did not receive */
}

void msc_sendcsw(void) {
	struct usb_msc_csw *csw = (void *)msc_csw_buff;

	csw->sign = MSC_CSW_SIGN;
	MSC_Send(csw, 13);		/* alignment!!! */

	memset(msc_csw_buff, 0, sizeof (*csw));
	memset(TempBuff, 0, sizeof TempBuff);
}

int msc_default_hook(struct usb_msc_cbw *cbw, void *buff) {
	return 0;
}

/*--------------------------------*/

void msc_failed(int dir) {
	struct usb_msc_csw *csw = (void *)msc_csw_buff;

	if (dir) {
		usb_rxep_stall(1);
	} else {
		usb_txep_stall(1);
	}

	csw->rem_len = 0;	/* <- was a pointless memset */
	csw->status = 0;
}

void msc_failed_sensei(char sensei) {
	struct usb_msc_csw *csw = (void *)msc_csw_buff;

	msc_failed(0);

	USB_Data1.scsi_sensei = sensei;
	csw->status = 1;
}

void msc_scsi_testunitready(void) {
	struct usb_msc_csw *csw = (void *)msc_csw_buff;

	csw->status = 1;
	USB_Data1.scsi_sensei = 5;
}

void msc_scsi_requestsense(void) {
	struct usb_msc_cbw *cbw = (void *)msc_cbw_buff;
	struct usb_msc_csw *csw = (void *)msc_csw_buff;

	if (cbw->xfer_len == 0) {
		usb_txep_stall(1);
		USB_Data1.scsi_sensei = 2;
		csw->rem_len = 0;
		csw->status = 1;
	} else {
		int len = cbw->xfer_len > 18 ? 18 : cbw->xfer_len;

		memset(TempBuff, 0, sizeof TempBuff);

		TempBuff[0] = 0x70;
		TempBuff[2] = USB_Data1.scsi_sense_key;
		TempBuff[7] = 18-8-1;
		TempBuff[12] = USB_Data1.scsi_sense_code;
		TempBuff[13] = USB_Data1.scsi_sense_key;

		MSC_Send(TempBuff, len);

		csw->rem_len -= len;
		csw->status = 0;
	}
}

void msc_scsi_inquiry(void) {
	struct usb_msc_csw *csw = (void *)msc_csw_buff;

	int len = sizeof scsi_inquiry;
	if (csw->rem_len < len) len = csw->rem_len;

	memcpy(TempBuff, scsi_inquiry, len);
	MSC_Send(TempBuff, len);

	csw->rem_len -= len;
	csw->status = 0;
}

void msc_scsi_modesense(void) {
	struct usb_msc_csw *csw = (void *)msc_csw_buff;
	int len = sizeof scsi_modesense;
	memcpy(TempBuff, scsi_modesense, len);
	MSC_Send(scsi_modesense, len);

	csw->rem_len = 0;	/* <- was a pointless memset */
	csw->status = 0;
}

void msc_scsi_cmd1E(void) {
	struct usb_msc_cbw *cbw = (void *)msc_cbw_buff;
	struct usb_msc_csw *csw = (void *)msc_csw_buff;

	if (cbw->cb[4] != 0x00) {
		USB_Data1.scsi_sensei = 2;
		csw->status = 1;
	}

	csw->rem_len = 0;
}

void msc_scsi_verify(void) {
	struct usb_msc_csw *csw = (void *)msc_csw_buff;

	csw->rem_len = 0;
	csw->status = 0;
}

void jlcmd_read_memory(void) {
	struct usb_msc_cbw *cbw = (void *)msc_cbw_buff;

	uint32_t addr = cbw->cb[2] << 24 | cbw->cb[3] << 16 | cbw->cb[4] << 8 | cbw->cb[5];
	uint16_t len = cbw->cb[6] << 8 | cbw->cb[7];

	memset(TempBuff2, 0xff, len);
	memcpy(TempBuff2, (void *)addr, len);
	MSC_Send(TempBuff2, len);
}

void jlcmd_write_memory(void) {
	struct usb_msc_cbw *cbw = (void *)msc_cbw_buff;

	uint32_t addr = cbw->cb[2] << 24 | cbw->cb[3] << 16 | cbw->cb[4] << 8 | cbw->cb[5];
	uint16_t len = cbw->cb[6] << 8 | cbw->cb[7];
	uint16_t crc = *(uint16_t *)&cbw->cb[9];

	MSC_Recv(TempBuff3, len);

	//if (crc == crc16(TempBuff3, len))
	memcpy((void *)addr, TempBuff3, len);
}

void jlcmd_jump_memory(void) {
	struct usb_msc_cbw *cbw = (void *)msc_cbw_buff;

	uint32_t addr = cbw->cb[2] << 24 | cbw->cb[3] << 16 | cbw->cb[4] << 8 | cbw->cb[5];
	uint16_t arg = cbw->cb[6] << 8 | cbw->cb[7];

	TempBuff[0] = 0xFB;
	TempBuff[1] = 0x08;

	loader_arglist.msc_send = MSC_Send;
	loader_arglist.msc_recv = MSC_Recv;
	loader_arglist.msc_hook = &MSC_Hook;
	loader_arglist.arg = arg;

	((void (*)(void *))addr)(&loader_arglist);

	MSC_Send(TempBuff, 16);
}

int usb_mainloop(void) {
	struct usb_msc_cbw *cbw = (void *)msc_cbw_buff;
	struct usb_msc_csw *csw = (void *)msc_csw_buff;
	int rc = 0;

	if (MSC_RecvTheCBW())
		return 0;

	if (MSC_Hook(cbw, TempBuff)) {
		/* woo */
		USB_Data1.scsi_sensei = 0;
		csw->rem_len = 0;
		csw->status = 0;
	} else {
		/* handle! */
		switch (cbw->cb[0]) {
		case 0x00:
			msc_scsi_testunitready();
			break;
		case 0x03:
			msc_scsi_requestsense();
			break;
		case 0x12:
			msc_scsi_inquiry();
			break;
		case 0x1A:
			msc_scsi_modesense();
			break;
		case 0x1E:
			msc_scsi_cmd1E();
			break;
		case 0x23:
		case 0x25:
			msc_failed_sensei(5);
			break;
		case 0x28:
			var_AB3 |= 2;
			msc_failed_sensei(15);
			break;
		case 0x2F:
			msc_scsi_verify();
			break;

		case 0xFB:
			switch (cbw->cb[1]) {
			case 0x06:
				jlcmd_write_memory();
				break;
			case 0x08:
				jlcmd_jump_memory();
				break;
			}
			break;
		//case 0xFC:
		//	break;
		case 0xFD:
			switch (cbw->cb[1]) {
			case 0x07:
				jlcmd_read_memory();
				break;
			}
			break;

		default:
			msc_failed(0);
			break;
		}
	}

	switch (USB_Data1.scsi_sensei) {
	case 0:		/* all good */
		USB_Data1.scsi_sense_key = 0x0;		/* no sense */
		USB_Data1.scsi_sense_code = 0;
		USB_Data1.scsi_sense_qual = 0;
		break;
	case 2:		/* parameter error */
		USB_Data1.scsi_sense_key = 0x5;		/* illegal request */
		USB_Data1.scsi_sense_code = 0x24;	/* invalid field */
		USB_Data1.scsi_sense_qual = 0;		/* <--' */
		break;
	case 5:		/* not ready - no unit - poasdf */
		USB_Data1.scsi_sense_key = 0x2;		/* not ready */
		USB_Data1.scsi_sense_code = 0x3a;	/* ? */
		USB_Data1.scsi_sense_qual = 0;
		break;
	case 6:		/* unknown opcode */
		USB_Data1.scsi_sense_key = 0xB;		/* aborted command */
		USB_Data1.scsi_sense_code = 0x20;	/* invalid opcode */
		USB_Data1.scsi_sense_qual = 0;
		break;
	}

	MSC_SendTheCSW();
	return rc;
}

void usb_handlersetup(int wa) {
	MSC_Hook = msc_default_hook;
	MSC_Send = usb_sendep1;
	MSC_Recv = usb_recvep1;
	MSC_RecvTheCBW = msc_recvcbw;
	MSC_SendTheCSW = msc_sendcsw;
}

void usb_init(void) {
	irq_disable(irqn_FUSB_CTL);

	JL_USB->CON0 &= ~0x40B7;
	JL_USB->CON1 = 0;

	JL_USB->CON0 |= (1<<0);		/* enable phy */

	JL_USB->IO_CON &= ~(3<<4);
	JL_USB->IO_CON &= ~(3<<6);
	JL_USB->IO_CON |= (1<<6);	/* D+ pullup */
	JL_USB->IO_CON |= (1<<5);	/* D- pullup */

	JL_USB->CON0 |= 0x3C;

	JL_USB->EP0_ADR = (uint32_t)usb_ep0_buff;
	JL_USB->EP1_RADR = (uint32_t)usb_ep1r_buff[0];

	usb_curr_rxbuff = 0;

	JL_USB->CON0 |= (1<<11);

	irq_enable(irqn_FUSB_CTL);
}

/*--------------------------------*/

void usbnuts(void) {
	irq_attach(irqn_FUSB_CTL, USBCtrl_handler, 0);

	usb_handlersetup(1);

	MSC_MaxLUN = 0;
	var_ADC = 2;

	usb_init();

	asm ("sti");

	for (;;) {
		JL_SYSTEM->WDT_CON |= (1<<6);		/* clear watchdog */
		usb_mainloop();
	}
}
