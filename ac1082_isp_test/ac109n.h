/*
 * Definitions for JieLi AC109N (CD02)
 */
#ifndef _AC109N_H
#define _AC109N_H

/*==================== SFR ===================*/

__sfr __at(0x80)	P0;			/* Port 0 */
__sfr __at(0x81)	SP;			/* Stack pointer */
__sfr __at(0x82)	DP0L;			/* Data pointer 0 (low) */
__sfr __at(0x83)	DP0H;			/* Data pointer 0 (high) */
__sfr __at(0x84)	DP1L;			/* Data pointer 1 (low) */
__sfr __at(0x85)	DP1H;			/* Data pointer 1 (high) */
__sfr __at(0x86)	DPCON;			/* Data pointer control */
__sfr __at(0x87)	SPH;			/* Stack pointer high? */

__sfr __at(0x88)	P0HD;			/* Port 0 high-drive */
__sfr __at(0x89)	SINT;
__sfr __at(0x8A)	IP0L;			/* Interrupt priority 0 (low) */
__sfr __at(0x8B)	IP0H;			/* Interrupt priority 0 (high) */
__sfr __at(0x8C)	IP1L;			/* Interrupt priority 1 (low) */
__sfr __at(0x8D)	IP1H;			/* Interrupt priority 1 (high) */
__sfr __at(0x8E)	WKUPPND;		/* Wakeup pending */
__sfr __at(0x8F)	WKUPCON;		/* Wakeup control */

__sfr __at(0x90)	P1;			/* Port 1 */
__sfr __at(0x91)	PWR_CON;		/* Power control */
__sfr __at(0x92)	WDT_CON;		/* Watchdog control */
__sfr __at(0x93)	BANK_SEL;
__sfr __at(0x94)	PBANK;
__sfr __at(0x95)	IE1;			/* Interrupt enable 1 */
__sfr __at(0x96)	IO_MC0;			/* IO map control 0 */
__sfr __at(0x97)	IO_MC1;			/* IO map control 1 */

__sfr __at(0x98)	P1HD;			/* Port 1 high-drive */
__sfr __at(0x99)	SPI_BUF;		/* SPI data buffer */
__sfr __at(0x9A)	P2PD;			/* Port 2 pulldown */
__sfr __at(0x9B)	P3PD;			/* Port 3 pulldown */
__sfr __at(0x9C)	P2PU;			/* Port 2 pullup */
__sfr __at(0x9D)	P3PU;			/* Port 3 pullup */
__sfr __at(0x9E)	SD_CON0;		/* SD host control 0 */
__sfr __at(0x9F)	SD_CON1;		/* SD host control 1 */

__sfr __at(0xA0)	P2;			/* Port 2 */
__sfr __at(0xA1)	IRFLT_CON;		/* IR filter control */
__sfr __at(0xA2)	P0DIR;			/* Port 0 direction */
__sfr __at(0xA3)	P1DIR;			/* Port 1 direction */
__sfr __at(0xA4)	P2DIR;			/* Port 2 direction */
__sfr __at(0xA5)	P3DIR;			/* Port 3 direction */
__sfr __at(0xA6)	LCD_CON0;		/* LCD control 0 */
__sfr __at(0xA7)	LCD_CON1;		/* LCD control 1 */

__sfr __at(0xA8)	IE0;			/* Interrupt enable 0 */
__sfr __at(0xA9)	SD_CON2;		/* SD host control 2 */
__sfr __at(0xAA)	DAC_CON0;		/* DAC control 0 */
__sfr __at(0xAB)	DAC_CON1;		/* DAC control 1 */
__sfr __at(0xAC)	DAC_VLML;		/* [W] DAC volume left ..? */
__sfr __at(0xAC)	DAC_RDLL;		/* [R] DAC read left (low)..? */
__sfr __at(0xAD)	DAC_VLMR;		/* [W] DAC volume right ..? */
__sfr __at(0xAD)	DAC_RDLH;		/* [R] DAC read left (high) ..? */
__sfr __at(0xAE)	SD_CPTR;		/* [W] SD host command buffer pointer */
__sfr __at(0xAE)	DAC_RDRL;		/* [R] DAC read right (low) ..? */
__sfr __at(0xAF)	SD_DPTR;		/* [W] SD host data buffer pointer */
__sfr __at(0xAF)	DAC_RDRH;		/* [R] DAC read right (high) ..? */

__sfr __at(0xB0)	P3;			/* Port 3 */
__sfr __at(0xB1)	P4;			/* Port 4 */
__sfr __at(0xB2)	P4DIR;			/* Port 4 direction */
__sfr __at(0xB3)	P4PU;			/* Port 4 pullup */
__sfr __at(0xB4)	P4PD;			/* Port 4 pulldown */
__sfr __at(0xB5)	P4HD;			/* Port 4 high-drive */
__sfr __at(0xB6)	P2HD;			/* Port 2 high-drive */
__sfr __at(0xB7)	P3HD;			/* Port 3 high-drive */

__sfr __at(0xB8)	P0DIE;			/* Port 0 digital input enable */
__sfr __at(0xB9)	P2DIE;			/* Port 2 digital input enable */
__sfr __at(0xBA)	P3DIE;			/* Port 3 digital input enable */
__sfr __at(0xBB)	P4DIE;			/* Port 4 digital input enable */
__sfr __at(0xBC)	CLK_CON0;		/* Clock control 0 */
__sfr __at(0xBD)	CLK_CON1;		/* Clock control 1 */
__sfr __at(0xBE)	CLK_CON2;		/* Clock control 2 */
__sfr __at(0xBF)	CLK_CON3;		/* Clock control 3 */

__sfr __at(0xC0)	P0PU;			/* Port 0 pullup */
__sfr __at(0xC1)	UART_CON;		/* UART control */
__sfr __at(0xC2)	UART_STA;		/* UART status */
__sfr __at(0xC3)	UART_BUF;		/* UART data buffer */
__sfr __at(0xC4)	MRES0;			/* Multiply result 0 */
__sfr __at(0xC5)	MRES1;			/* Multiply result 1 */
__sfr __at(0xC6)	MRES2;			/* Multiply result 2 */
__sfr __at(0xC7)	MRES3;			/* Multiply result 3 */

__sfr __at(0xC8)	P0PD;			/* Port 0 pulldown */
__sfr __at(0xC9)	RTC_BUF;		/* RTC data buffer */
__sfr __at(0xCA)	RTC_CON0;		/* RTC control 0 */
__sfr __at(0xCB)	RTC_CON1;		/* [W] RTC control 1 */
__sfr __at(0xCB)	CHIP_VERSION;		/* [R] Chip version */
__sfr __at(0xCC)	ADC_CON0;		/* ADC control 0 */
__sfr __at(0xCD)	ADC_CON1;		/* [W] ADC control 1 */
__sfr __at(0xCE)	SPI_CON;		/* SPI control */
__sfr __at(0xCF)	SPI_STA;		/* SPI status */

__sfr __at(0xD0)	PSW;			/* Processor status word */
__sfr __at(0xD1)	USB_CON2;		/* USB control 2 */
__sfr __at(0xD2)	TMR0_CON0;		/* Timer 0 control 0 */
__sfr __at(0xD3)	TMR0_CON1;		/* Timer 0 control 1 */
__sfr __at(0xD4)	TMR0_CNT;		/* Timer 0 counter */
__sfr __at(0xD5)	TMR0_PRD;		/* Timer 0 period */
__sfr __at(0xD6)	TMR0_PWM0;		/* Timer 0 PWM duty 0 */
__sfr __at(0xD7)	TMR0_PWM1;		/* Timer 0 PWM duty 1 */

__sfr __at(0xD8)	P1DIE;			/* Port 1 digital input enable */
__sfr __at(0xD9)	CRC_FIFO;		/* CRC FIFO */
__sfr __at(0xDA)	TMR1_CON0;		/* Timer 1 control 0 */
__sfr __at(0xDB)	TMR1_CON1;		/* Timer 1 control 1 */
__sfr __at(0xDC)	TMR1_CNT;		/* Timer 1 counter */
__sfr __at(0xDD)	TMR1_PRD;		/* Timer 1 period */
__sfr __at(0xDE)	TMR1_PWM0;		/* Timer 1 PWM duty 0 */
__sfr __at(0xDF)	TMR1_PWM1;		/* Timer 1 PWM duty 1 */

__sfr __at(0xE0)	ACC;			/* Accumulator register */
__sfr __at(0xE1)	CLK_GAT;		/* Clock gate */
__sfr __at(0xE2)	TMR2_CON;		/* Timer 2 control */
__sfr __at(0xE3)	TMR2_CNTL;		/* Timer 2 counter (low) */
__sfr __at(0xE4)	TMR2_CNTH;		/* Timer 2 counter (high) */
__sfr __at(0xE5)	TMR2_PRDL;		/* Timer 2 period (low) */
__sfr __at(0xE6)	TMR2_PRDH;		/* Timer 2 period (high) */
__sfr __at(0xE7)	USB_DAT;		/* USB register data */

__sfr __at(0xE8)	P1PU;			/* Port 1 pullup */
__sfr __at(0xE9)	USB_CON1;		/* USB control 1 */
__sfr __at(0xEA)	TMR3_CON;		/* Timer 3 control */
__sfr __at(0xEB)	TMR3_CNTL;		/* Timer 3 counter (low) */
__sfr __at(0xEC)	TMR3_CNTH;		/* Timer 3 counter (high) */
__sfr __at(0xED)	TMR3_PRDL;		/* Timer 3 period (low) */
__sfr __at(0xEE)	TMR3_PRDH;		/* Timer 3 period (high) */
__sfr __at(0xEF)	DAA_CON1;		/* DAC analog control 1 */

__sfr __at(0xF0)	BREG;			/* B register */
__sfr __at(0xF1)	HBITS;			/* [W] */
__sfr __at(0xF1)	ADC_DATL;		/* [R] ADC data low */
__sfr __at(0xF2)	MP_CON0;		/* [W] */
__sfr __at(0xF2)	ADC_DATH;		/* [R] ADC data high */
__sfr __at(0xF3)	MP_CON1;
__sfr __at(0xF4)	MP_CON2;
__sfr __at(0xF5)	MP_CON3;
__sfr __at(0xF6)	WPTRL;
__sfr __at(0xF7)	WPTRH;

__sfr __at(0xF8)	P1PD;			/* Port 1 pulldown */
__sfr __at(0xF9)	DAA_CON2;		/* DAC analog control 2 */
__sfr __at(0xFA)	HUFF_CON0;		/* Huffman control 0 */
__sfr __at(0xFB)	HUFF_CON1;		/* Huffman control 1 */
__sfr __at(0xFC)	VLENL;
__sfr __at(0xFD)	VLENH;
__sfr __at(0xFE)	SFTOUTH;
__sfr __at(0xFF)	SFTOUTL;

/*==================== XFR ===================*/

char __xdata __at(0xFF00)	OTP_CMD1;	/* [W] OTP command 1 */
char __xdata __at(0xFF01)	OTP_CMD0;	/* OTP command 0 */
char __xdata __at(0xFF03)	OSC_DET_CON;	/* OSC detect control */
char __xdata __at(0xFF04)	UART_BAUD;	/* [W] UART baudrate */
char __xdata __at(0xFF05)	SD_CON3;	/* [W] SD host control 3 */
char __xdata __at(0xFF06)	LVD_CON;	/* Low voltage detector control */
char __xdata __at(0xFF07)	STACK_LMT;	/* [W, FPGA only] Stack limit */
char __xdata __at(0xFF08)	ISD_CON;	/* ISD control */
char __xdata __at(0xFF09)	ISD_BUF;	/* ISD data buffer */
char __xdata __at(0xFF0A)	ISD_ADR;	/* [W] ISD DMA address */
char __xdata __at(0xFF0B)	ISD_CNT;	/* [W] ISD DMA length */
char __xdata __at(0xFF0C)	MODE_CON;	/* Mode control */
char __xdata __at(0xFF0D)	PWM4_CON;	/* [W] PWM4 control */
char __xdata __at(0xFF0E)	CRC_REGL;	/* CRC register (low) */
char __xdata __at(0xFF0F)	CRC_REGH;	/* CRC register (high) */

char __xdata __at(0xFF10)	USB_CON0;	/* USB control 0 */
char __xdata __at(0xFF14)	USB_EP0_ADR;	/* [W] USB endpoint 0 address */
char __xdata __at(0xFF15)	USB_EP0_CNT;	/* [W] USB endpoint 0 count */
char __xdata __at(0xFF16)	USB_EPX_CNT;	/* [W] USB endpoint X count */
char __xdata __at(0xFF17)	USB_EP1TX_ADR;	/* [W] USB endpoint 1.TX address */
char __xdata __at(0xFF17)	EP1_RX_ADDRL;	/* [R] Endpoint 1.RX address (low) */
char __xdata __at(0xFF18)	USB_EP1RX_ADR;	/* [W] USB endpoint 1.RX address */
char __xdata __at(0xFF18)	EP1_RX_ADDRH;	/* [R] Endpoint 1.RX address (high) */
char __xdata __at(0xFF19)	USB_EP2TX_ADR;	/* [W] USB endpoint 2.TX address */
char __xdata __at(0xFF1A)	USB_EP2RX_ADR;	/* [W] USB endpoint 2.RX address */

char __xdata __at(0xFF20)	LOFC_CON;
char __xdata __at(0xFF21)	LOFC_PR;
char __xdata __at(0xFF22)	LOFC_CNTH;
char __xdata __at(0xFF23)	LOFC_CNTM;
char __xdata __at(0xFF24)	LOFC_CNTL;
char __xdata __at(0xFF26)	BP_ADRL;
char __xdata __at(0xFF27)	BP_ADRH;
char __xdata __at(0xFF28)	PLL_CON0;	/* PLL control 0 */
char __xdata __at(0xFF29)	PLL_CON1;	/* PLL control 1 */
char __xdata __at(0xFF2A)	PLL_CON2;	/* PLL control 2 */
char __xdata __at(0xFF2B)	SEG_EN0;	/* LCD segment enable 0 */
char __xdata __at(0xFF2C)	SEG_EN1;	/* LCD segment enable 1 */

char __xdata __at(0xFF30)	DAA_CON0;	/* DAC analog control 0 */
char __xdata __at(0xFF33)	DAA_CON3;	/* DAC analog control 3 */
char __xdata __at(0xFF34)	DAA_CON4;	/* DAC analog control 4 */
char __xdata __at(0xFF36)	DAC_TRML;	/* [W] DAC trim left */
char __xdata __at(0xFF37)	DAC_TRMR;	/* [W] DAC trim right */

char __xdata __at(0xFF40)	OBUF_NUM;	/* [R] */
char __xdata __at(0xFF41)	OBUF_CON;
char __xdata __at(0xFF42)	OBUF_DATL;	/* [W, 16-bit only] */
char __xdata __at(0xFF44)	OBUF_DATR;	/* [W, 16-bit only] */
char __xdata __at(0xFF46)	KV_DAT;		/* [W, 16-bit only] Key Voice data */
char __xdata __at(0xFF48)	KV_START;	/* [W] Key Voice start address */
char __xdata __at(0xFF49)	KV_END;		/* [W] Key Voice end address */
char __xdata __at(0xFF4A)	KV_CNT;		/* [W] Key Voice repeat count */
char __xdata __at(0xFF4B)	KV_VLM;		/* [W] Key Voice volume */
char __xdata __at(0xFF4C)	IIC_CON;	/* I2C control */
char __xdata __at(0xFF4D)	IIC_STA;	/* I2C status */
char __xdata __at(0xFF4E)	IIC_BAUD;	/* [W] I2C clock divider */
char __xdata __at(0xFF4F)	IIC_BUF;	/* I2C data buffer */

char __xdata __at(0xFF52)	SPI_BAUD;	/* [W] SPI clock divider */
char __xdata __at(0xFF53)	SPI_CNT;	/* [W] SPI DMA length */
char __xdata __at(0xFF54)	SPI_ADR;	/* [W] SPI DMA address */
char __xdata __at(0xFF55)	LDO_CON;	/* LDO control */
char __xdata __at(0xFF56)	HTC_CON;	/* HTC control */

#endif
