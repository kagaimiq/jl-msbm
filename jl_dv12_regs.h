/*
 * Register defitinion for JieLi DV12 (AC520N)
 */
#ifndef _JL_BR17_REGS_H
#define _JL_BR17_REGS_H

#include <stdint.h>
#include "regaccess.h"

/*============================================================================*/

#if 0
/* Low-speed bus */
#define LSBUS_base		0x1F60000
	#define LS_IO_base		LSBUS_base + 0x0000
		#define PORTA_base		LS_IO_base + 0x000
		#define PORTB_base		LS_IO_base + 0x040
		#define PORTC_base		LS_IO_base + 0x080
		#define PORTD_base		LS_IO_base + 0x0C0
		#define PORTE_base		LS_IO_base + 0x100
		#define PORTF_base		LS_IO_base + 0x140
		#define PORTG_base		LS_IO_base + 0x180
		#define PORTH_base		LS_IO_base + 0x1C0
		#define PORTI_base		LS_IO_base + 0x200
		#define IOMAP_base		LS_IO_base + 0x240
		#define WAKEUP_base		LS_IO_base + 0x250
	#define LS_UART_base		LSBUS_base + 0x0400
		#define UART0_base		LS_UART_base + 0x000
		#define UART1_base		LS_UART_base + 0x040
		#define UART2_base		LS_UART_base + 0x080
		#define UART3_base		LS_UART_base + 0x0C0
	#define LS_SPI_base		LSBUS_base + 0x0800
		#define SPI0_base		LS_SPI_base + 0x000
		#define SPI1_base		LS_SPI_base + 0x040
	#define LS_SD_base		LSBUS_base + 0x0C00
		#define SD0_base		LS_SD_base + 0x000
		#define SD1_base		LS_SD_base + 0x040
	#define LS_TMR_base		LSBUS_base + 0x1000
		#define TIMER0_base		LS_TMR_base + 0x000
		#define TIMER1_base		LS_TMR_base + 0x040
		#define TIMER2_base		LS_TMR_base + 0x080
		#define TIMER3_base		LS_TMR_base + 0x0C0
	#define LS_FUSB_base		LSUSB_base + 0x1400
	#define LS_HUSB_base		LSUSB_base + 0x1800
	#define LS_ADDA_base		LSUSB_base + 0x1C00
	#define LS_CLK_base		LSUSB_base + 0x2000
	#deifne LS_OTH_base		LSUSB_base + 0x2400

/* High-speed bus */
#define HSBUS_base		0x1F70000
	#define HS_SDR_base		HSBUS_base + 0x0000
	/*...*/
	#define HS_CPU_base		HSBUS_base + 0x0400
	/*...*/
#endif

/*============================================================================*/
/* PORT - gpio port */

struct JL_PORT_regs {
	volatile uint32_t	OUT;		/* Output level */
	volatile uint32_t	IN;		/* Input level */
	volatile uint32_t	DIR;		/* Direction (1 = input) */
	volatile uint32_t	DIE;		/* Digital input enable */
	volatile uint32_t	PU;		/* Pullup enable */
	volatile uint32_t	PD;		/* Pulldown enable */
	volatile uint32_t	HD;		/* High-drive enable */
};

/*---------------------------------------*/
/* UART - universal asynchronous receiver-transmitter */

struct JL_UARTx_regs {
	volatile uint32_t	HRXCNT;		/* RX DMA count H */
	volatile uint32_t	OTCNT;		/* Overtime/Timeout count */
	volatile uint32_t	TXADR;		/* TX DMA address */
	volatile uint32_t	TXCNT;		/* TX DMA length */
	volatile uint32_t	RXEADR;		/* RX DMA end address */
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	BAUD;		/* Baudrate divider */
	volatile uint32_t	BUF;		/* Data reg */
	volatile uint32_t	RXSADR;		/* RX DMA start address */
	volatile uint32_t	RXCNT;		/* RX DMA count */
};

struct JL_UARTl_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	BUF;		/* Data reg */
	volatile uint32_t	BAUD;		/* Baudrate divider */
};

/*---------------------------------------*/
/* SPI - serial peripheral interface */

struct JL_SPI_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	BAUD;		/* Clock divider */
	volatile uint32_t	BUF;		/* Data reg */
	volatile uint32_t	ADR;		/* DMA address */
	volatile uint32_t	CNT;		/* DMA length */
};

/*---------------------------------------*/
/* SD - sd/mmc host */

struct JL_SD_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	CON2;		/* Control reg 2 */
	volatile uint32_t	CPTR;		/* Command buffer address */
	volatile uint32_t	DPTR;		/* Data buffer address */
	volatile uint32_t	CTU_CON;	/* continuous transfer unit control reg */
	volatile uint32_t	CTU_CNT;	/* continuous transfer unit block count */
};

/*---------------------------------------*/
/* TIMER - period/pwm timer */

struct JL_TIMER_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	CNT;		/* Counter */
	volatile uint32_t	PRD;		/* Period */
	volatile uint32_t	PWM;		/* PWM compare (duty) */
};

/*---------------------------------------*/
/* PWM - special pwm timer (MCPWM?) */

struct JL_PWM_regs {
	volatile uint32_t	TMR_CON;	/* Timer control register */
	volatile uint32_t	TMR_CNT;	/* Timer counter */
	volatile uint32_t	TMR_PR;		/* Timer period */
	volatile uint32_t	CMP1;		/* Output 1 compare */
	volatile uint32_t	CMP2;		/* Output 2 compare */
	volatile uint32_t	CMP3;		/* Output 3 compare */
	volatile uint32_t	PWM_CON;	/* PWM control reg */
	volatile uint32_t	DT_CON;		/* Dead time control reg */
	volatile uint32_t	FPIN_CON;	/* .?.fpin.?. control reg */
};

/*---------------------------------------*/
/* FUSB - fullspeed usb */

struct JL_FUSB_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	EP0_CNT;	/* EP0 buffer size */
	volatile uint32_t	EP1_CNT;	/* EP1 buffer size */
	volatile uint32_t	EP2_CNT;	/* EP2 buffer size */
	volatile uint32_t	EP3_CNT;	/* EP3 buffer size */
	volatile uint32_t	EP0_ADR;	/* EP0 buffer address */
	volatile uint32_t	EP1_TADR;	/* EP1.IN buffer address */
	volatile uint32_t	EP1_RADR;	/* EP1.OUT buffer address */
	volatile uint32_t	EP2_TADR;	/* EP2.IN buffer address */
	volatile uint32_t	EP2_RADR;	/* EP2.OUT buffer address */
	volatile uint32_t	EP3_TADR;	/* EP3.IN buffer address */
	volatile uint32_t	EP3_RADR;	/* EP3.OUT buffer address */
	volatile uint32_t	IO_CON0;	/* GPIO control */
}

/*---------------------------------------*/
/* HUSB - highspeed usb (DMA interface & phy/misc control) */

struct JL_HUSB_regs {
	volatile uint32_t	SIE_CON;	/* SIE control reg */
	volatile uint32_t	EP0_CNT;	/* EP0 buffer size */
	volatile uint32_t	EP1_CNT;	/* EP1 buffer size */
	volatile uint32_t	EP2_CNT;	/* EP2 buffer size */
	volatile uint32_t	EP3_CNT;	/* EP3 buffer size */
	volatile uint32_t	EP4_CNT;	/* EP4 buffer size */
	volatile uint32_t	EP0_ADR;	/* EP0 buffer address */
	volatile uint32_t	EP1_TADR;	/* EP1.IN buffer address */
	volatile uint32_t	EP1_RADR;	/* EP1.OUT buffer address */
	volatile uint32_t	EP2_TADR;	/* EP2.IN buffer address */
	volatile uint32_t	EP2_RADR;	/* EP2.OUT buffer address */
	volatile uint32_t	EP3_TADR;	/* EP3.IN buffer address */
	volatile uint32_t	EP3_RADR;	/* EP3.OUT buffer address */
	volatile uint32_t	EP4_TADR;	/* EP4.IN buffer address */
	volatile uint32_t	EP4_RADR;	/* EP4.OUT buffer address */
	uint32_t	Reserved_3C;
	volatile uint32_t	COM_CON;	/* .?.com.?. control reg */
	volatile uint32_t	PHY_CON0;	/* phy control reg 0 */
	volatile uint32_t	PHY_CON1;	/* phy control reg 1 */
	volatile uint32_t	PHY_CON2;	/* phy control reg 2 */
}

/*---------------------------------------*/
/* ALNK - audio link (inter-integrated sound / i2s) */

struct JL_ALNK_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	ADR0;		/* Buffer address 0 */
	volatile uint32_t	ADR1;		/* Buffer address 1 */
	volatile uint32_t	ADR2;		/* Buffer address 2 */
	volatile uint32_t	ADR3;		/* Buffer address 3 */
	volatile uint32_t	CON2;		/* Control reg 2 */
	volatile uint32_t	LEN;		/* Buffer length */
};

/*---------------------------------------*/
/* LADC - .L. analog-digital converter (audio) */

struct JL_LADC_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	ADR;		/* Buffer address */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	CON2;		/* Control reg 2 */
	volatile uint32_t	RES12;		/* Result 12 .? */
	volatile uint32_t	LEN;		/* Buffer length */
};

/*---------------------------------------*/
/* ADC - analog-digital converter */

struct JL_ADC_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	RES;		/* Conversion result */
};

/*---------------------------------------*/
/* DAC - digital-analog converter (audio) */

struct JL_DAC_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	TRML;		/* Left channel trim */
	volatile uint32_t	TRMR;		/* Right channel trim */
	volatile uint32_t	ADR;		/* Buffer address */
	volatile uint32_t	DAA_CON0;	/* Analog control reg 0 */
	volatile uint32_t	DAA_CON1;	/* Analog control reg 1 */
	volatile uint32_t	DAA_CON2;	/* Analog control reg 2 */
	volatile uint32_t	DAA_CON3;	/* Analog control reg 3 */
	volatile uint32_t	LEN;		/* Buffer length */
};

/*---------------------------------------*/
/* CLOCK - clock config */

struct JL_CLOCK_regs {
	volatile uint32_t	PWR_CON;	/* Power control reg */
	volatile uint32_t	CLK_CON0;	/* Clock control reg 0 */
	volatile uint32_t	CLK_CON1;	/* Clock control reg 1 */
	volatile uint32_t	CLK_CON2;	/* Clock control reg 2 */
	volatile uint32_t	LCLK_GAT;	/* LSB clock gating */
	volatile uint32_t	HCLK_GAT;	/* HSB clock gating */
	volatile uint32_t	ACLK_GAT;	/* A... clock gating */
	volatile uint32_t	PLL0_NF;
	volatile uint32_t	PLL0_NR;
	volatile uint32_t	PLL1_NF;
	volatile uint32_t	PLL1_NR;
	volatile uint32_t	OSA_CON;	/* .?.OSA.?. control reg */
	volatile uint32_t	PLL_CON0;	/* PLL control reg 0 */
	volatile uint32_t	PLL_CON1;	/* PLL control reg 1 */
};

/*---------------------------------------*/
/* SYSTEM - system stuff */

struct JL_SYSTEM_regs {
	volatile uint32_t	HTC_CON;	/* .?.HTC.?. control reg */
	volatile uint32_t	LDO_CON;	/* LDO control reg */
	volatile uint32_t	LVD_CON;	/* LVD control reg */
	uint32_t	Reserved_0C[2];
	volatile uint32_t	MODE_CON;	/* Mode control reg */
	uint32_t	Reserved_18[2];
	volatile uint32_t	WDT_CON;	/* WDT control reg */
	volatile uint32_t	CHIP_ID;	/* Chip ID */
	uint32_t	Reserved_28[17];
	volatile uint32_t	EFUSE_CON;	/* eFuse control reg */
};

/*---------------------------------------*/
/* IRTC - internal real time clock/counter */

struct JL_IRTC_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	BUF;		/* Data buffer */
};

/*---------------------------------------*/
/* CRC - cyclic redundancy check (CRC16-CCITT) */

struct JL_CRC_regs {
	volatile uint32_t	FIFO;		/* CRC FIFO */
	volatile uint32_t	REG;		/* CRC shift register */
};

/*---------------------------------------*/
/* IIC - inter-integrated circuit (i2c) */

struct JL_IIC_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	BUF;		/* Data reg */
	volatile uint32_t	BAUD;		/* Clock divider */
	volatile uint32_t	DMA_ADR;	/* DMA address */
	volatile uint32_t	DMA_CNT;	/* DMA length */
	volatile uint32_t	DMA_NRATE;	/* DMA nrate.. */
};

/*---------------------------------------*/
/* PWM8 - pwm timer 8 */

struct JL_PWM8_regs {
	volatile uint32_t	CON;		/* Control reg */
};

/*---------------------------------------*/
/* PAP - parallel active port */

struct JL_PAP_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	BUF;		/* Data buffer */
	volatile uint32_t	ADR;		/* DMA address */
	volatile uint32_t	CNT;		/* DMA length */
	volatile uint32_t	DAT0;		/* Data reg 0 */
	volatile uint32_t	DAT1;		/* Data reg 1 */
};

/*---------------------------------------*/
/* LCDC - lcd controller */

struct JL_LCDC_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	SEG_IOEN;	/* Segment IO enable */
};

/*---------------------------------------*/
/* MPU - memory protection unit */

struct JL_MPU_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	START;		/* Start area */
	volatile uint32_t	END;		/* End area */
	volatile uint32_t	CATCH0;		/* Catch 0 */
	volatile uint32_t	CATCH1;		/* Catch 1 */
};

/*---------------------------------------*/
/* PLCNT - pulse counter */

struct JL_PLCNT_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	VL;		/* Value */
};

/*---------------------------------------*/
/* CS - ??? */

struct JL_CS_regs {
	volatile uint32_t	CON;		/* Contol reg */
	volatile uint32_t	REG;		/* register */
	volatile uint32_t	FIFO;		/* fifo */
	volatile uint32_t	RADR;		/* r address */
	volatile uint32_t	RCNT;		/* r count */
};

/*---------------------------------------*/
/* RAND - (pseudo-)random number generator (64 bit) */

struct JL_RAND_regs {
	volatile uint32_t	R64L;		/* Low 32 bits */
	volatile uint32_t	R64H;		/* High 32 bits */
};

/*===========================================================*/
/********* High-speed bus *********/
/* SDRAM - sdram controller */

struct JL_SDRAM_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	SPTR;		/* // */
	volatile uint32_t	QPTR;		/* // */
	volatile uint32_t	REFREG;		
	volatile uint32_t	DMACNT;		/* // */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	REFSUM;		
	volatile uint32_t	DBG;		/* Debug reg */
	volatile uint32_t	CON2;		/* Control reg 2 */
}

/*============================================================================*/

/********* Low-speed bus **********/

/* ls_io */
#define JL_PORTA	((struct JL_PORT_regs   *)0x1F60000)
#define JL_PORTB	((struct JL_PORT_regs   *)0x1F60040)
#define JL_PORTC	((struct JL_PORT_regs   *)0x1F60080)
#define JL_PORTD	((struct JL_PORT_regs   *)0x1F600C0)
#define JL_PORTE	((struct JL_PORT_regs   *)0x1F60100)
#define JL_PORTF	((struct JL_PORT_regs   *)0x1F60140)
#define JL_PORTG	((struct JL_PORT_regs   *)0x1F60180)
#define JL_PORTH	((struct JL_PORT_regs   *)0x1F601C0)
#define JL_PORTI	((struct JL_PORT_regs   *)0x1F60200)	/* input only? */
#define JL_IOMAP	((struct JL_IOMAP_regs  *)0x1F60240)
#define JL_WAKEUP	((struct JL_WAKEUP_regs *)0x1F60250)

/* ls_uart */
#define JL_UART0	((struct JL_UARTx_regs  *)0x1F60400)
#define JL_UART1	((struct JL_UARTx_regs  *)0x1F60440)
#define JL_UART2	((struct JL_UARTl_regs  *)0x1F60480)
#define JL_UART3	((struct JL_UARTl_regs  *)0x1F604C0)

/* ls_spi */
#define JL_SPI0		((struct JL_SPI_regs    *)0x1F60800)
#define JL_SPI1		((struct JL_SPI_regs    *)0x1F60840)

/* ls_sd */
#define JL_SD0		((struct JL_SD_regs     *)0x1F60C00)
#define JL_SD1		((struct JL_SD_regs     *)0x1F60C40)

/* ls_tmr */
#define JL_TIMER0	((struct JL_TIMER_regs  *)0x1F61000)
#define JL_TIMER1	((struct JL_TIMER_regs  *)0x1F61040)
#define JL_TIMER2	((struct JL_TIMER_regs  *)0x1F61080)
#define JL_TIMER3	((struct JL_TIMER_regs  *)0x1F610C0)
#define JL_PWM		((struct JL_PWM_regs    *)0x1F61100)

/* ls_fusb */
#define JL_FUSB		((struct JL_FUSB_regs   *)0x1F61400)

/* ls_husb */
#define JL_HUSB		((struct JL_HUSB_regs   *)0x1F61800)

/* ls_adda */
#define JL_ALNK		((struct JL_ALNK_regs   *)0x1F61C00)
#define JL_LADC		((struct JL_LADC_regs   *)0x1F61C40)
#define JL_ADC		((struct JL_ADC_regs    *)0x1F61C80)
#define JL_DAC		((struct JL_DAC_regs    *)0x1F61CC0)

/* ls_clk */
#define JL_CLOCK	((struct JL_CLOCK_regs  *)0x1F62000)

/* ls_oth */
#define JL_SYSTEM	((struct JL_SYSTEM_regs *)0x1F62400)
#define JL_IRTC		((struct JL_IRTC_regs   *)0x1F6240C)
#define JL_CRC		((struct JL_CRC_regs    *)0x1F62418)
#define JL_IRFLT	((struct JL_IRFLT_regs  *)0x1F62428)
#define JL_IIC		((struct JL_IIC_regs    *)0x1F62434)
#define JL_PWM8		((struct JL_PWM8_regs   *)0x1F62444)
#define JL_PAP		((struct JL_PAP_regs    *)0x1F62448)
#define JL_LCDC		((struct JL_LCDC_regs   *)0x1F62460)
#define JL_MPU		((struct JL_MPU_regs    *)0x1F62470)
#define JL_PLCNT	((struct JL_PLCNT_regs  *)0x1F62484)
#define JL_CS		((struct JL_CS_regs     *)0x1F6248C)
#define JL_RAND		((struct JL_RAND_regs   *)0x1F624A0)

/********* High-speed bus *********/

/* hs_sdr */
#define JL_SDRAM	((struct JL_SDRAM_regs  *)0x1F70000)

/* hs_isp */

/* hs_cpu */
#define JL_NVIC		((struct JL_NVIC_regs   *)0x1F70400)
#define JL_DSP		((struct JL_DSP_regs    *)0x1F704

/* hs_sfc */

/* hs_jpg */

/* hs_oth */

/* hs_dbg */

/* hs_cve */

/*============================================================================*/

#endif
