/*
 * Register defitinion for JieLi BR25 (AC696N/AC636N)
 */
#ifndef _JL_BR25_REGS_H
#define _JL_BR25_REGS_H

#include <stdint.h>
#include "regaccess.h"

/*============================================================================*/

#if 0
/* CPU core space */
#define CORE_base		0x100000
	#define SDTAP_base		CORE_base + 0x0100	/* 100100 */
	#define MMU_base		CORE_base + 0x0300	/* 100300 */
	#define DSP_base		CORE_base + 0x1000	/* 101000 */
	#define DEBUG_base		CORE_base + 0x1040	/* 101040 */
	#define FFT_base		CORE_base + 0x2000	/* 102000 */
	#define CPU_base		CORE_base + 0xF000	/* 10F000 */

/* Low-speed bus */
#define LSBUS_base		0x1E0000
	#define CLOCK_base		LSBUS_base + 0x0000	/* 1E0000 */
	#define RST_base		LSBUS_base + 0x00C0	/* 1E00C0 */
	#define MODE_base		LSBUS_base + 0x0100	/* 1E0100 */
	#define SYSTEM_base		LSBUS_base + 0x0200	/* 1E0200 */
	#define TIMER0_base		LSBUS_base + 0x0400	/* 1E0400 */
	#define TIMER1_base		LSBUS_base + 0x0500	/* 1E0500 */
	#define TIMER2_base		LSBUS_base + 0x0600	/* 1E0600 */
	#define TIMER3_base		LSBUS_base + 0x0700	/* 1E0700 */
	#define TIMER4_base		LSBUS_base + 0x0800	/* 1E0800 */
	#define TIMER5_base		LSBUS_base + 0x0900	/* 1E0900 */
	#define PCNT_base		LSBUS_base + 0x1000	/* 1E1000 */
	#define GPCNT_base		LSBUS_base + 0x1100	/* 1E1100 */
	#define SD0_base		LSBUS_base + 0x1400	/* 1E1400 */
	#define SD1_base		LSBUS_base + 0x1500	/* 1E1500 */
	#define USB_base		LSBUS_base + 0x1800	/* 1E1800 */
	#define ANA_base		LSBUS_base + 0x1900	/* 1E1900 */
	#define SPI0_base		LSBUS_base + 0x1C00	/* 1E1C00 */
	#define SPI1_base		LSBUS_base + 0x1D00	/* 1E1D00 */
	#define SPI2_base		LSBUS_base + 0x1E00	/* 1E1E00 */
	#define UART0_base		LSBUS_base + 0x2000	/* 1E2000 */
	#define UART1_base		LSBUS_base + 0x2100	/* 1E2100 */
	#define UART2_base		LSBUS_base + 0x2200	/* 1E2200 */
	#define IIC_base		LSBUS_base + 0x2400	/* 1E2400 */
	#define PAP_base		LSBUS_base + 0x2800	/* 1E2800 */
	#define SS_base			LSBUS_base + 0x2B00	/* 1E2B00 */
	#define RDEC0_base		LSBUS_base + 0x2C00	/* 1E2C00 */
	#define PLNK_base		LSBUS_base + 0x2D00	/* 1E2D00 */
	#define ALNK0_base		LSBUS_base + 0x2E00	/* 1E2E00 */
	#define AUDIO_base		LSBUS_base + 0x2F00	/* 1E2F00 */
	#define MCPWM_base		LSBUS_base + 0x3000	/* 1E3000 */
	#define ADC_base		LSBUS_base + 0x3100	/* 1E3100 */
	#define IR_base			LSBUS_base + 0x3200	/* 1E3200 */
	#define ALNK1_base		LSBUS_base + 0x3300	/* 1E3300 */
	#define OSA_base		LSBUS_base + 0x3400	/* 1E3400 */
	#define CRC_base		LSBUS_base + 0x3500	/* 1E3500 */
	#define LRCT_base		LSBUS_base + 0x3600	/* 1E3600 */
	#define EFUSE_base		LSBUS_base + 0x3700	/* 1E3700 */
	#define RAND64_base		LSBUS_base + 0x3B00	/* 1E3B00 */
	#define CTM_base		LSBUS_base + 0x3C00	/* 1E3C00 */
	#define P33_base		LSBUS_base + 0x3E00	/* 1E3E00 */
	#define DMA_base		LSBUS_base + 0x3F00	/* 1E3F00 */
	#define PERIENC_base		LSBUS_base + 0x4100	/* 1E4100 */
	#define SBC_base		LSBUS_base + 0x4200	/* 1E4200 */
	#define AES_base		LSBUS_base + 0x4300	/* 1E4300 */
	#define RDEC0_base		LSBUS_base + 0x4400	/* 1E4400 */
	#define RDEC1_base		LSBUS_base + 0x4500	/* 1E4500 */
	#define PORTA_base		LSBUS_base + 0x5000	/* 1E5000 */
	#define PORTB_base		LSBUS_base + 0x5040	/* 1E5040 */
	#define PORTC_base		LSBUS_base + 0x5080	/* 1E5080 */
	#define PORTD_base		LSBUS_base + 0x50C0	/* 1E50C0 */
	#define USB_IO_base		LSBUS_base + 0x5100	/* 1E5100 */
	#define IOMAP_base		LSBUS_base + 0x5108	/* 1E5108 */
	#define WAKEUP_base		LSBUS_base + 0x5118	/* 1E5118 */
	#define PLED_base		LSBUS_base + 0x5200	/* 1E5200 */
	#define LCDC_base		LSBUS_base + 0x5300	/* 1E5300 */

/* High-speed bus */
#define HSBUS_base		0x1F0000
	#define SFC_base		HSBUS_base + 0x0200	/* 1F0200 */
	#define SFCENC_base		HSBUS_base + 0x0300	/* 1F0300 */
	#define DCP_base		HSBUS_base + 0x1300	/* 1F1300 */
	#define EQ_base			HSBUS_base + 0x1500	/* 1F1500 */
	#define SRC_base		HSBUS_base + 0x1600	/* 1F1600 */
	#define FM_base			HSBUS_base + 0x1700	/* 1F1700 */
	#define WL_base			HSBUS_base + 0x1800	/* 1F1800 */
#endif

/*============================================================================*/
/************ Core SFR ************/
/* SDTAP - serial debug tap?? */
struct JL_SDTAP_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	KEY;		/* Key ?? */
};

/*---------------------------------------*/
/* MMU - memory management/mapping unit */

struct JL_MMU_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	TLB1_BEG;	/* TLB1 begin address */
	volatile uint32_t	TLB1_END;	/* TLB1 end address */
};

/* TLB entry:
 *   16 bits per entry,
 *   b0-b12 = Page no. (pages are 128 bytes each iirc)
 *   b13 = Valid
 */

/*---------------------------------------*/
/* DSP - dsp stuff */

struct JL_DSP_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	RING_OSC;
	volatile uint32_t	CPASS_CON;
	volatile uint32_t	CPASS_ADRH;
	volatile uint32_t	CPASS_ADRL;
	volatile uint32_t	CPASS_BUF_LAST;
	volatile uint32_t	CPREFETCH_ADRH;
	volatile uint32_t	CPREFETCH_ADRL;
	volatile uint32_t	CACHE_MSG_CH;
	volatile uint32_t	MEM_CON;	/* Memory control */
};

/*---------------------------------------*/
/* DEBUG - debug stuff */

struct JL_DEBUG_regs {
	volatile uint32_t	DSP_BF_CON;
	volatile uint32_t	WR_EN;
	volatile uint32_t	MSG;
	volatile uint32_t	MSG_CLR;
	volatile uint32_t	CPU_WR_LIMH;
	volatile uint32_t	CPU_WR_LIML;
	volatile uint32_t	PRP_WR_LIMH;
	volatile uint32_t	PRP_WR_LIML;
	volatile uint32_t	PRP_MMU_MSG;
	volatile uint32_t	LSB_MMU_MSG_CH;
	volatile uint32_t	PRP_WR_LIMIT_MSG;
	volatile uint32_t	LSB_WR_LIMIT_CH;
	volatile uint32_t	CPU_PC_LIMH0;
	volatile uint32_t	CPU_PC_LIML0;
	volatile uint32_t	CPU_PC_LIMH1;
	volatile uint32_t	CPU_PC_LIML1;
	volatile uint32_t	PRP_SRM_INV_MSG;
	volatile uint32_t	LSB_SRM_INV_CH;
};

/*---------------------------------------*/
/* FFT - fast fourier transform */

struct JL_FFT_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	CADR;		/* Address */
	volatile uint32_t	TEST0;		/* Test reg 0 */
	volatile uint32_t	TEST1;		/* Test reg 1 */
};

/*---------------------------------------*/
/* CPU - cpu regs (q32DSP) */

struct JL_CPU_regs {
	/* 0x00 ~ CPU registers */
	union {	/* General purpose registers */
		struct {
			volatile uint32_t	DR00;
			volatile uint32_t	DR01;
			volatile uint32_t	DR02;
			volatile uint32_t	DR03;
			volatile uint32_t	DR04;
			volatile uint32_t	DR05;
			volatile uint32_t	DR06;
			volatile uint32_t	DR07;
			volatile uint32_t	DR08;
			volatile uint32_t	DR09;
			volatile uint32_t	DR10;
			volatile uint32_t	DR11;
			volatile uint32_t	DR12;
			volatile uint32_t	DR13;
			volatile uint32_t	DR14;
			volatile uint32_t	DR15;
		};
		volatile uint32_t DRn[16];
	};
	union {	/* Special function registers */
		struct {
			volatile uint32_t	RETI;		/* Interrupt return address */
			volatile uint32_t	RETE;		/* Emulation return address */
			volatile uint32_t	RETX;		/* eXception return address */
			volatile uint32_t	RETS;		/* Subroutine return address */
			volatile uint32_t	SR04;
			volatile uint32_t	PSR;		/* Processor status register */
			volatile uint32_t	CNUM;
			volatile uint32_t	SR07;
			volatile uint32_t	SR08;
			volatile uint32_t	SR09;
			volatile uint32_t	SR10;
			volatile uint32_t	ICFG;		/* Interrupt config */
			volatile uint32_t	USP;		/* User stack pointer */
			volatile uint32_t	SSP;		/* Supervisor stack pointer */
			volatile uint32_t	SP;		/* Current stack pointer */
			volatile uint32_t	PCRS;
		};
		volatile uint32_t SRn[16];
	};

	/* 0x80 ~ MPU */
	volatile uint32_t	BPCON;
	volatile uint32_t	BSP;
	volatile uint32_t	BP0;
	volatile uint32_t	BP1;
	volatile uint32_t	BP2;
	volatile uint32_t	BP3;
	volatile uint32_t	CMD_PAUSE;
	uint32_t Reserved_9C[9];

	/* 0xC0 ~ PMU */
	volatile uint32_t	PMU_CON;
	uint32_t Reserved_C4[3];

	/* 0xD0 ~ EMU */
	volatile uint32_t	EMU_CON;
	volatile uint32_t	EMU_MSG;
	volatile uint32_t	EMU_SSP_H;
	volatile uint32_t	EMU_SSP_L;
	volatile uint32_t	EMU_USP_H;
	volatile uint32_t	EMU_USP_L;
	uint32_t Reserved_E8;

	/* 0xEC ~ Tick timer */
	volatile uint32_t	TTMR_CON;	/* Tick timer control */
	volatile uint32_t	TTMR_CNT;	/* Tick timer counter */
	volatile uint32_t	TTMR_PRD;	/* Tick timer period */

	/* 0xF8 ~ Bank thing */
	volatile uint32_t	BANK_CON;
	volatile uint32_t	BANK_NUM;

	/* 0x100 ~ Interrupt controller */
	union {	/* Interrupt config - 8 bits per interupt */
		struct {
			volatile uint32_t	ICFG00;
			volatile uint32_t	ICFG01;
			volatile uint32_t	ICFG02;
			volatile uint32_t	ICFG03;
			volatile uint32_t	ICFG04;
			volatile uint32_t	ICFG05;
			volatile uint32_t	ICFG06;
			volatile uint32_t	ICFG07;
			volatile uint32_t	ICFG08;
			volatile uint32_t	ICFG09;
			volatile uint32_t	ICFG10;
			volatile uint32_t	ICFG11;
			volatile uint32_t	ICFG12;
			volatile uint32_t	ICFG13;
			volatile uint32_t	ICFG14;
			volatile uint32_t	ICFG15;
			volatile uint32_t	ICFG16;
			volatile uint32_t	ICFG17;
			volatile uint32_t	ICFG18;
			volatile uint32_t	ICFG19;
			volatile uint32_t	ICFG20;
			volatile uint32_t	ICFG21;
			volatile uint32_t	ICFG22;
			volatile uint32_t	ICFG23;
			volatile uint32_t	ICFG24;
			volatile uint32_t	ICFG25;
			volatile uint32_t	ICFG26;
			volatile uint32_t	ICFG27;
			volatile uint32_t	ICFG28;
			volatile uint32_t	ICFG29;
			volatile uint32_t	ICFG30;
			volatile uint32_t	ICFG31;
		};
		volatile uint32_t ICFGn[32];
	};
	union {	/* Interrupt pending - 1 bit per interrupt */
		struct {
			volatile uint32_t	IPND0;
			volatile uint32_t	IPND1;
			volatile uint32_t	IPND2;
			volatile uint32_t	IPND3;
			volatile uint32_t	IPND4;
			volatile uint32_t	IPND5;
			volatile uint32_t	IPND6;
			volatile uint32_t	IPND7;
		};
		volatile uint32_t IPNDn[8];
	};
	volatile uint32_t	ILAT_SET;	/* Interrupt latch set */
	volatile uint32_t	ILAT_CLR;	/* Interrupt latch clear */
	volatile uint32_t	IPMASK;		/* Interrupt priority mask */
	uint32_t Reserved_1AC[5];

	/* 0x1C0 ~ things */
	volatile uint32_t	ETM_CON;
	volatile uint32_t	ETM_PC0;
	volatile uint32_t	ETM_PC1;
	volatile uint32_t	ETM_PC2;
	volatile uint32_t	ETM_PC3;
	volatile uint32_t	WP0_ADRH;
	volatile uint32_t	WP0_ADRL;
	volatile uint32_t	WP0_DATH;
	volatile uint32_t	WP0_DATL;
	volatile uint32_t	WP0_PC;
};

/*===========================================================*/
/********* Low-speed bus **********/
/* CLOCK - clock config */

struct JL_CLOCK_regs {
	volatile uint32_t	PWR_CON;	/* [00] ~Power control */
	volatile uint32_t	HTC_CON;	/* [04] ~HTC control */
	volatile uint32_t	SYS_DIV;	/* [08] System clock divider */
	volatile uint32_t	CLK_CON0;	/* [0C] Clock control reg 0 */
	volatile uint32_t	CLK_CON1;	/* [10] Clock control reg 1 */
	volatile uint32_t	CLK_CON2;	/* [14] Clock control reg 2 */
	volatile uint32_t	CLK_CON3;	/* [18] Clock control reg 3 */
	uint32_t Reserved_1C[9];
	volatile uint32_t	PLL_CON;	/* [40] PLL control reg */
	volatile uint32_t	PLL_CON1;	/* [44] PLL control reg 1 */
	volatile uint32_t	PLL_INTF;	/* [48] PLL integer/fraction */
	volatile uint32_t	PLL_DMAX;	/* [4C] PLL deviation max ?? */
	volatile uint32_t	PLL_DMIN;	/* [50] PLL deviation min ?? */
	volatile uint32_t	PLL_DSTP;	/* [54] PLL deviation step ?? */
};

/*---------------------------------------*/
/* RST - reset source */

struct JL_RST_regs {
	volatile uint32_t	SRC;		/* Reset source */
};

/*---------------------------------------*/
/* MODE - mode control */

struct JL_MODE_regs {
	volatile uint32_t	CON;		/* Mode control */
};

/*---------------------------------------*/
/* SYSTEM - system stuff */

struct JL_SYSTEM_regs {
	volatile uint32_t	CHIP_ID;	/* Chip ID */
	volatile uint32_t	MBIST_CON;	/* Memory BuiltInSelfTest<MStarSemi> control ?? */
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
/* PCNT - pulse counter */

struct JL_PLCNT_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	VAL;		/* Value */
};

/*---------------------------------------*/
/* GPCNT - gated pulse counter */

struct JL_GPCNT_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	NUM;		/* Number */
};

/*---------------------------------------*/
/* SD - sd/mmc host */

struct JL_SD_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	CON2;		/* Control reg 2 */
	volatile uint32_t	CPTR;		/* Command buffer address */
	volatile uint32_t	DPTR;		/* Data buffer address */
	volatile uint32_t	CTU_CON;	/* .?.ctu.?. control reg */
	volatile uint32_t	CTU_CNT;	/* .?.ctu.?. block count */
};

/*---------------------------------------*/
/* USB - universal serial bus */

struct JL_USB_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
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
};

/* USB indirect registers (MUSB!! but altered ... like Allwinner)
 * (common usb regs)
 * 00 - FADDR			00 FAddr
 * 01 - POWER			01 Power
 * 02 - INTRTX1			02 IntrTx
 * 03 - INTRTX2			03 IntrTx
 * 04 - INTRRX1			04 IntrRx
 * 05 - INTRRX2			05 IntrRx
 * 06 - INTRUSB			0A IntrUSB
 * 07 - INTRTX1E		06 IntrTxE
 * 08 - INTRTX2E		07 IntrTxE
 * 09 - INTRRX1E		08 IntrRxE
 * 0A - INTRRX2E		09 IntrRxE
 * 0B - INTRUSBE		0B IntrUSBE
 * 0C - FRAME1			0C Frame
 * 0D - FRAME2			0D Frame
 * 0E - INDEX			0E Index
 * (additional control & config regs)
 * 0F - DEVCTL			60 DevCtl
 * (indexed endpoint regs)
 * 10 - TXMAXP			10 TxMaxP
 * 11 - CSR0 / TXCSR1		12 CSR0 / TxCSRL
 * 12 - TXCSR2			13 TxCSRH
 * 13 - RXMAXP			14 RxMaxP
 * 14 - RXCSR1			16 RxCSRL
 * 15 - RXCSR2			17 RxCSRH
 * 16 - COUNT0 / RXCOUNT1	18 Count0 / RxCount
 * 17 - RXCOUNT2		19 RxCount
 * (--- host-only)
 * 18 - TXTYPE			1A TxType
 * 19 - TXINTERVAL		1B TxInterval
 * 1A - RXTYPE			1C RxType
 * 1B - RXINTERVAL		1D RxInterval
 */

/*---------------------------------------*/
/* ANA - analog control stuff */

struct JL_ANA_regs {
	volatile uint32_t	WLA_CON0;
	volatile uint32_t	WLA_CON1;
	volatile uint32_t	WLA_CON2;
	volatile uint32_t	WLA_CON3;
	volatile uint32_t	WLA_CON4;
	volatile uint32_t	WLA_CON5;
	volatile uint32_t	WLA_CON6;
	volatile uint32_t	WLA_CON7;
	volatile uint32_t	WLA_CON8;
	volatile uint32_t	WLA_CON9;
	volatile uint32_t	WLA_CON10;
	volatile uint32_t	WLA_CON11;
	volatile uint32_t	WLA_CON12;
	volatile uint32_t	WLA_CON13;
	volatile uint32_t	WLA_CON14;
	volatile uint32_t	WLA_CON15;
	volatile uint32_t	WLA_CON16;
	volatile uint32_t	WLA_CON17;
	volatile uint32_t	WLA_CON18;
	volatile uint32_t	WLA_CON19;
	volatile uint32_t	WLA_CON20;
	volatile uint32_t	WLA_CON21;
	volatile uint32_t	WLA_CON22;
	volatile uint32_t	WLA_CON23;
	volatile uint32_t	WLA_CON24;
	volatile uint32_t	WLA_CON25;
	volatile uint32_t	WLA_CON26;
	volatile uint32_t	WLA_CON27;
	volatile uint32_t	WLA_CON28;
	volatile uint32_t	WLA_CON29;
	volatile uint32_t	WLA_CON30;
	volatile uint32_t	WLA_CON31;
	volatile uint32_t	WLA_CON32;
	volatile uint32_t	WLA_CON33;
	volatile uint32_t	WLA_CON34;
	volatile uint32_t	WLA_CON35;
	volatile uint32_t	WLA_CON36;
	volatile uint32_t	WLA_CON37;
	volatile uint32_t	WLA_CON38;
	volatile uint32_t	WLA_CON39;
	uint32_t Reserved_A0[8];
	volatile uint32_t	DAA_CON0;	/* DAC analog control reg 0 */
	volatile uint32_t	DAA_CON1;	/* DAC analog control reg 1 */
	volatile uint32_t	DAA_CON2;	/* DAC analog control reg 2 */
	volatile uint32_t	DAA_CON3;	/* DAC analog control reg 3 */
	volatile uint32_t	DAA_CON4;	/* DAC analog control reg 4 */
	uint32_t Reserved_B4[2];
	volatile uint32_t	DAA_CON7;	/* DAC analog control reg 7 */
	volatile uint32_t	ADA_CON0;	/* ADC analog control reg 0 */
	volatile uint32_t	ADA_CON1;	/* ADC analog control reg 1 */
	volatile uint32_t	ADA_CON2;	/* ADC analog control reg 2 */
	volatile uint32_t	ADA_CON3;	/* ADC analog control reg 3 */
	volatile uint32_t	ADA_CON4;	/* ADC analog control reg 4 */
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
/* UART - universal asynchronous receiver-transmitter */

struct JL_UART_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	BAUD;		/* Baudrate divider */
	volatile uint32_t	BUF;		/* Data reg */
	volatile uint32_t	OTCNT;		/* Overtime/Timeout count */
	volatile uint32_t	TXADR;		/* TX DMA address */
	volatile uint32_t	TXCNT;		/* TX DMA length */
	volatile uint32_t	RXSADR;		/* RX DMA start address */
	volatile uint32_t	RXEADR;		/* RX DMA end address */
	volatile uint32_t	RXCNT;		/* RX DMA count */
	volatile uint32_t	HRXCNT;		/* RX DMA current count */
};

/*---------------------------------------*/
/* IIC - inter-integrated circuit (i2c) */

struct JL_IIC_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	BUF;		/* Data reg */
	volatile uint32_t	BAUD;		/* Clock divider */
	volatile uint32_t	CON1;		/* Control reg 1 */
};

/*---------------------------------------*/
/* PAP - parallel active port */

struct JL_PAP_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	DAT0;		/* Data reg 0 */
	volatile uint32_t	DAT1;		/* Data reg 1 */
	volatile uint32_t	BUF;		/* ~buffer reg */
	volatile uint32_t	ADR;		/* DMA address */
	volatile uint32_t	CNT;		/* DMA length */
};

/*---------------------------------------*/
/* SS - s/pdif slave */

struct JL_SS_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	SR_CNT;		/* ~sr count */
	volatile uint32_t	IO_CON;		/* IO control reg */
	volatile uint32_t	DMA_CON;	/* DMA control reg */
	volatile uint32_t	DMA_LEN;	/* DMA length */
	volatile uint32_t	DAT_ADR;	/* Data address */
	volatile uint32_t	INF_ADR;	/* Info address */
	volatile uint32_t	CSB0;
	volatile uint32_t	CSB1;
	volatile uint32_t	CSB2;
	volatile uint32_t	CSB3;
	volatile uint32_t	CSB4;
	volatile uint32_t	CSB5;
};

/*---------------------------------------*/
/* RDEC - rotation (quadrature) decoder */

struct JL_RDEC_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	DAT;		/* Data reg */
	volatile uint32_t	SMP;
};

/*---------------------------------------*/
/* PLNK - pdm link */

struct JL_PLNK_regs {
	volatile uint32_t	CON;		/* Control reg 0 */
	volatile uint32_t	SMR;
	volatile uint32_t	ADR;		/* Buffer address */
	volatile uint32_t	LEN;		/* Buffer length */
};

/*---------------------------------------*/
/* ALNK - audio link (inter-integrated sound / i2s) */

struct JL_ALNK_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	CON2;		/* Control reg 2 */
	volatile uint32_t	CON3;		/* Control reg 3 */
	volatile uint32_t	ADR0;		/* Buffer address 0 */
	volatile uint32_t	ADR1;		/* Buffer address 1 */
	volatile uint32_t	ADR2;		/* Buffer address 2 */
	volatile uint32_t	ADR3;		/* Buffer address 3 */
	volatile uint32_t	LEN;		/* Buffer length */
};

/*---------------------------------------*/
/* AUDIO - audio codec / adda / dac */

struct JL_AUDIO_regs {
	volatile uint32_t	DAC_CON;	/* DAC control reg */
	volatile uint32_t	DAC_ADR;	/* DAC buffer address */
	volatile uint32_t	DAC_LEN;	/* DAC buffer length */
	volatile uint32_t	DAC_PNS;
	volatile uint32_t	DAC_HRP;
	volatile uint32_t	DAC_SWP;
	volatile uint32_t	DAC_SWN;
	uint32_t Reserved_1C;
	volatile uint32_t	DAC_VL0;
	uint32_t Reserved_24;
	volatile uint32_t	DAC_TM0;
	uint32_t Reserved_2C[2];
	volatile uint32_t	DAC_DTB;
	uint32_t Reserved_38;
	volatile uint32_t	DAC_DPD;
	volatile uint32_t	DAC_COP;
	volatile uint32_t	ADC_CON;	/* ADC control reg */
	volatile uint32_t	ADC_ADR;	/* ADC buffer address */
	volatile uint32_t	ADC_LEN;	/* ADC buffer length */
	volatile uint32_t	ADC_PNS;
	volatile uint32_t	ADC_HWP;
	volatile uint32_t	ADC_SRP;
	volatile uint32_t	ADC_SRN;
};

/*---------------------------------------*/
/* MCPWM - motor control pwm */

struct JL_MCPWM_regs_TMR {
	volatile uint32_t	CON;	/* Timer control */
	volatile uint32_t	CNT;	/* Timer counter */
	volatile uint32_t	PR;	/* Timer period */
};

struct JL_MCPWM_regs_CH {
	volatile uint32_t	CON0;	/* Channel control reg 0 */
	volatile uint32_t	CON1;	/* Channel control reg 1 */
	volatile uint32_t	CMPH;	/* Channel compare (high) */
	volatile uint32_t	CMPL;	/* Channel compare (low) */
};

struct JL_MCPWM_regs {
	struct JL_MCPWM_regs_TMR	TMR[8];
	volatile uint32_t	FPIN_CON;	/* .?.fpin.?. control reg */
	struct JL_MCPWM_regs_CH		CH[8];
	volatile uint32_t	MCPWM_CON0;	/* MCPWM control reg 0 */
};

/*---------------------------------------*/
/* ADC - analog-digital converter */

struct JL_ADC_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	RES;		/* Conversion result */
};

/*---------------------------------------*/
/* IRFLT - infrared (signal) filter */

struct JL_IRFLT_regs {
	volatile uint32_t	CON;		/* Control reg */
};

/*---------------------------------------*/
/* OSA - ... */

struct JL_OSA_regs {
	volatile uint32_t	CON;		/* Control reg */
};

/*---------------------------------------*/
/* CRC - cyclic redundancy check (CRC16-CCITT) */

struct JL_CRC_regs {
	volatile uint32_t	FIFO;		/* CRC FIFO */
	volatile uint32_t	REG;		/* CRC shift register */
};

/*---------------------------------------*/
/* LRCT - ... */

struct JL_LRCT_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	NUM;		/* Number */
};

/*---------------------------------------*/
/* EFUSE - efuse thing (not the p33 one??) */

struct JL_EFUSE_regs {
	volatile uint32_t	CON;		/* Control reg */
	uint32_t Reserved_04[7];
	volatile uint32_t	ME;
};

/*---------------------------------------*/
/* RAND - (pseudo-)random number generator (64 bit) */

struct JL_RAND_regs {
	volatile uint32_t	R64L;		/* Low 32 bits */
	volatile uint32_t	R64H;		/* High 32 bits */
};

/*---------------------------------------*/
/* CTM - charge time measurement */

struct JL_CTM_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	ADR;		/* Address ?? */
};

/*---------------------------------------*/
/* P33 - p33 pmu interface */

struct JL_P33_regs {
	volatile uint32_t	PMU_CON;	/* PMU control reg */
	volatile uint32_t	RTC_CON;	/* RTC control reg */
	volatile uint32_t	SPI_CON;	/* SPI control reg */
	volatile uint32_t	SPI_DAT;	/* SPI data */
};

/*---------------------------------------*/
/* DMA - direct memory access ? */

struct JL_DMA_regs {
	volatile uint32_t	PRI0;
	volatile uint32_t	PRI1;
	volatile uint32_t	PRI2;
	volatile uint32_t	PRI3;
	uint32_t Reserved_10[4];
	volatile uint32_t	MSG;
	volatile uint32_t	MSG_CH;
	volatile uint32_t	RDL;
	volatile uint32_t	RDH;
	volatile uint32_t	WRL;
	volatile uint32_t	WRH;
};

/*---------------------------------------*/
/* PERIENC - Peripheral (SPI0/SD0) "encryptor" */

struct JL_PERIENC_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	KEY;		/* Key */
	volatile uint32_t	ADR;
};

/*---------------------------------------*/
/* SBC - sbc codec */

struct JL_SBC_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	DEC_SRC_ADR;	/* Decoder src address */
	volatile uint32_t	DEC_DST_ADR;	/* Decoder dest address */
	volatile uint32_t	DEC_PWM_WCNT;	/* Decoder PCM write cnt */
	volatile uint32_t	DEC_INBUF_LEN;	/* Decoder input buff length */
	volatile uint32_t	ENC_SRC_ADR;	/* Encoder src address */
	volatile uint32_t	ENC_DST_ADR;	/* Encoder dest address */
	volatile uint32_t	DEC_DST_BASE;
	volatile uint32_t	RAM_CFG;
};

/*---------------------------------------*/
/* AES - advanced encryption standard */

struct JL_AES_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	DATIN;		/* Data input */
	volatile uint32_t	KEY;		/* Key */
	volatile uint32_t	ENCRES0;	/* Encrypted result 0 */
	volatile uint32_t	ENCRES1;	/* Encrypted result 1 */
	volatile uint32_t	ENCRES2;	/* Encrypted result 2 */
	volatile uint32_t	ENCRES3;	/* Encrypted result 3 */
	volatile uint32_t	NONCE;		/* Nonce */
	volatile uint32_t	HEADER;		/* Header */
	volatile uint32_t	SRCADR;		/* Source address */
	volatile uint32_t	DSTADR;		/* Dest address */
	volatile uint32_t	CTCNT;		/* ct count */
	volatile uint32_t	TAGLEN;		/* Tag length */
	volatile uint32_t	TAGRES0;	/* Tag result 0 */
	volatile uint32_t	TAGRES1;	/* Tag result 1 */
	volatile uint32_t	TAGRES2;	/* Tag result 2 */
	volatile uint32_t	TAGRES3;	/* Tag result 3 */
};

/*---------------------------------------*/
/* PORT - gpio port */

struct JL_PORT_regs {
	volatile uint32_t	OUT;		/* Output level */
	volatile uint32_t	IN;		/* Input level */
	volatile uint32_t	DIR;		/* Direction (1 = input) */
	volatile uint32_t	DIE;		/* Digital input enable */
	volatile uint32_t	PU;		/* Pullup enable */
	volatile uint32_t	PD;		/* Pulldown enable */
	volatile uint32_t	HD0;		/* High-drive enable 0 */
	volatile uint32_t	HD;		/* High-drive enable */
	volatile uint32_t	DIEH;		/* ~ digital input enable h?? */
};

/*---------------------------------------*/
/* USBIO - usb gpio */

struct JL_USBIO_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
};

/*---------------------------------------*/
/* WAKEUP - port wakeup */

struct JL_WAKEUP_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	CON2;		/* Control reg 2 */
	volatile uint32_t	CON3;		/* Control reg 3 */
};

/*---------------------------------------*/
/* IOMAP - port io mapping */

struct JL_IOMAP_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	CON2;		/* Control reg 2 */
	volatile uint32_t	CON3;		/* Control reg 3 */
	volatile uint32_t	CON4;		/* Control reg 4 */
	volatile uint32_t	CON5;		/* Control reg 5 */
};

/*---------------------------------------*/
/* PLED - pwm led */

struct JL_PLED_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	CON2;		/* Control reg 2 */
	volatile uint32_t	CON3;		/* Control reg 3 */
	volatile uint32_t	BRI_PRDL;	/* Brightness period low */
	volatile uint32_t	BRI_PRDH;	/* Brightness period high */
	volatile uint32_t	BRI_DUTY0L;	/* Brightness duty 0 low */
	volatile uint32_t	BRI_DUTY0H;	/* Brightness duty 0 high */
	volatile uint32_t	BRI_DUTY1L;	/* Brightness duty 1 low */
	volatile uint32_t	BRU_DUTY1H;	/* Brightness duty 1 high */
	volatile uint32_t	PRD_DIVL;	/* Period divider low */
	volatile uint32_t	DUTY0;		/* Duty 0 */
	volatile uint32_t	DUTY1;		/* Duty 1 */
	volatile uint32_t	DUTY2;		/* Duty 2 */
	volatile uint32_t	DUTY3;		/* Duty 3 */
	volatile uint32_t	CNT_RD;		/* Counter read */
};

/*---------------------------------------*/
/* LCDC - lcd controller */

struct JL_LCDC_regs {
	volatile uint32_t	CON0;		/* Control reg */
	volatile uint32_t	SEG_IOEN0;	/* Segment IO enable 0 */
	volatile uint32_t	SEG_IOEN1;	/* Segment IO enable 1 */
};

/*===========================================================*/
/********* High-speed bus *********/
/* SFC - serial flash controller */

struct JL_SFC_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	BAUD;		/* Clock divider */
	volatile uint32_t	CODE;
	volatile uint32_t	BASE_ADR;	/* Flash base address */
	volatile uint32_t	QUCNT;
};

/*---------------------------------------*/
/* SFCENC - SFC "encryptor" */

struct JL_SFCENC_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	KEY;		/* Key */
	volatile uint32_t	UNENC_ADRH;	/* unenc address high boundary */
	volatile uint32_t	UNENC_ADRL;	/* unenc address low boundary */
	volatile uint32_t	LENC_ADRH;	/* lenc address high boundary */
	volatile uint32_t	LENC_ADRL;	/* lenc address low boundary */
};

/*---------------------------------------*/
/* DCP - ... */

struct JL_DCP_regs {
	volatile uint32_t	CON;		/* Control reg */
	volatile uint32_t	ADR;		/* Address */
};

/*---------------------------------------*/
/* EQ - equalizer */

struct JL_EQ_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	DATAI_ADR;	/* Input buffer address */
	volatile uint32_t	DATAO_ADR;	/* Output buffer address */
	volatile uint32_t	DATA_LEN;	/* Data length */
	volatile uint32_t	FLT_ADR;	/* .?.flt.?. address */
};

/*---------------------------------------*/
/* SRC - sample rate conversion */

struct JL_SRC_regs {
	volatile uint32_t	CON0;		/* Control reg 0 */
	volatile uint32_t	CON1;		/* Control reg 1 */
	volatile uint32_t	CON2;		/* Control reg 2 */
	volatile uint32_t	CON3;		/* Control reg 3 */
	volatile uint32_t	IDAT_ADR;	/* Input data address */
	volatile uint32_t	IDAT_LEN;	/* Input data length */
	volatile uint32_t	ODAT_ADR;	/* Output data address */
	volatile uint32_t	ODAT_LEN;	/* Output data length */
	volatile uint32_t	FLTB_ADR;	/* .?.fltb.?. address */
};

/*---------------------------------------*/
/* FM - fm receiver/transmitter */

struct JL_FM_regs {
	volatile uint32_t	CON;		/* [00] Control reg */
	volatile uint32_t	BASE;		/* [04] RX buffer base address */
	volatile uint32_t	ADC_CON;	/* [08] ADC control */
	volatile uint32_t	ADC_CON1;	/* [0C] ADC control 1 */
	volatile uint32_t	HF_CON0;	/* [10] HF control reg 0 */
	volatile uint32_t	HF_CON1;	/* [14] HF control reg 1 */
	volatile uint32_t	HF_CRAM;	/* [18] */
	volatile uint32_t	HF_CRAM2;	/* [1C] */
	volatile uint32_t	HF_DRAM;	/* [20] */
	volatile uint32_t	LF_CON;		/* [24] LF control reg 0 */
	volatile uint32_t	LF_RES;		/* [28] */
	volatile uint32_t	FMRX_CON4;	/* [2C] RX control reg 4 */
	volatile uint32_t	FMRX_CON5;	/* [30] RX control reg 5 */

	volatile uint32_t	TX_CON0;	/* [34] TX control reg 0 */
	volatile uint32_t	TX_CON1;	/* [38] TX control reg 1 */
	volatile uint32_t	TX_PILOT;	/* [3C] */
	volatile uint32_t	TX_SYN_GAIN;	/* [40] */
	volatile uint32_t	TX_MUL;		/* [44] */
	volatile uint32_t	TX_ADR;		/* [48] */
	volatile uint32_t	TX_LEN;		/* [4C] */
	volatile uint32_t	TX_FREQ;	/* [50] */
	volatile uint32_t	TX_BASE_ADR;	/* [54] TX buffer base address */
};

/*---------------------------------------*/
/* WL - wireless stuff */

struct JL_WL_regs {
	volatile uint32_t	CON0;
	volatile uint32_t	CON3;
	volatile uint32_t	LOFC_CON;
	volatile uint32_t	LOFC_RES;
};

/*============================================================================*/

/************ Core SFR ************/
#define JL_SDTAP	((struct JL_SDTAP_regs   *)0x100100)
#define JL_MMU		((struct JL_MMU_regs     *)0x100300)

#define JL_DSP		((struct JL_DSP_regs     *)0x101000)
#define JL_DEBUG	((struct JL_DEBUG_regs   *)0x101040)

#define JL_FFT		((struct JL_FFT_regs     *)0x102000)

#define JL_CPU		((struct JL_CPU_regs     *)0x10F000)	/* q32DSP */

/********* Low-speed bus **********/
#define JL_CLOCK	((struct JL_CLOCK_regs   *)0x1E0000)
#define JL_RST		((struct JL_RST_regs     *)0x1E00C0)

#define JL_MODE		((struct JL_MODE_regs    *)0x1E0100)
#define JL_SYSTEM	((struct JL_SYSTEM_regs  *)0x1E0200)
#define JL_TIMER0	((struct JL_TIMER_regs   *)0x1E0400)
#define JL_TIMER1	((struct JL_TIMER_regs   *)0x1E0500)
#define JL_TIMER2	((struct JL_TIMER_regs   *)0x1E0600)
#define JL_TIMER3	((struct JL_TIMER_regs   *)0x1E0700)
#define JL_TIMER4	((struct JL_TIMER_regs   *)0x1E0800)
#define JL_TIMER5	((struct JL_TIMER_regs   *)0x1E0900)
#define JL_PCNT		((struct JL_PCNT_regs    *)0x1E1000)
#define JL_GPCNT	((struct JL_GPCNT_regs   *)0x1E1100)
#define JL_SD0		((struct JL_SD_regs      *)0x1E1400)
#define JL_SD1		((struct JL_SD_regs      *)0x1E1500)
#define JL_USB		((struct JL_USB_regs     *)0x1E1800)
#define JL_ANA		((struct JL_ANA_regs     *)0x1E1900)
#define JL_SPI0		((struct JL_SPI_regs     *)0x1E1C00)
#define JL_SPI1		((struct JL_SPI_regs     *)0x1E1D00)
#define JL_SPI2		((struct JL_SPI_regs     *)0x1E1E00)
#define JL_UART0	((struct JL_UART_regs    *)0x1E2000)
#define JL_UART1	((struct JL_UART_regs    *)0x1E2100)
#define JL_UART2	((struct JL_UART_regs    *)0x1E2200)
#define JL_IIC		((struct JL_IIC_regs     *)0x1E2400)
#define JL_PAP		((struct JL_PAP_regs     *)0x1E2800)
#define JL_SS		((struct JL_SS_regs      *)0x1E2B00)
#define JL_RDEC0	((struct JL_RDEC_regs    *)0x1E2C00)
#define JL_PLNK		((struct JL_PLNK_regs    *)0x1E2D00)
#define JL_ALNK0	((struct JL_ALNK_regs    *)0x1E2E00)
#define JL_AUDIO	((struct JL_AUDIO_regs   *)0x1E2F00)
#define JL_MCPWM	((struct JL_MCPWM_regs   *)0x1E3000)
#define JL_ADC		((struct JL_ADC_regs     *)0x1E3100)
#define JL_IRFLT	((struct JL_IRFLT_regs   *)0x1E3200)	/* IR */
#define JL_ALNK1	((struct JL_ALNK_regs    *)0x1E3300)
#define JL_OSA		((struct JL_OSA_regs     *)0x1E3400)
#define JL_CRC		((struct JL_CRC_regs     *)0x1E3500)
#define JL_LRCT		((struct JL_LRCT_regs    *)0x1E3600)
#define JL_EFUSE	((struct JL_EFUSE_regs   *)0x1E3700)
#define JL_RAND		((struct JL_RAND_regs    *)0x1E3B00)	/* rand64 */
#define JL_CTM		((struct JL_CTM_regs     *)0x1E3C00)
#define JL_P33		((struct JL_P33_regs     *)0x1E3E00)
#define JL_DMA		((struct JL_DMA_regs     *)0x1E3F00)
#define JL_PERIENC	((struct JL_PERIENC_regs *)0x1E4100)
#define JL_SBC		((struct JL_SBC_regs     *)0x1E4200)
#define JL_AES		((struct JL_AES_regs     *)0x1E4300)
#define JL_RDEC1	((struct JL_RDEC_regs    *)0x1E4400)
#define JL_RDEC2	((struct JL_RDEC_regs    *)0x1E4500)
#define JL_PORTA	((struct JL_PORT_regs    *)0x1E5000)
#define JL_PORTB	((struct JL_PORT_regs    *)0x1E5040)
#define JL_PORTC	((struct JL_PORT_regs    *)0x1E5080)
#define JL_PORTD	((struct JL_PORT_regs    *)0x1E50C0)

#define JL_USBIO	((struct JL_USBIO_regs   *)0x1E5100)	/* USB_IO */
#define JL_WAKEUP	((struct JL_WAKEUP_regs  *)0x1E5108)
#define JL_IOMAP	((struct JL_IOMAP_regs   *)0x1E5118)

#define JL_PLED		((struct JL_PLED_regs    *)0x1E5200)
#define JL_LCDC		((struct JL_LCDC_regs    *)0x1E5300)	/* LCD */

/********* High-speed bus *********/
#define JL_SFC		((struct JL_SFC_regs     *)0x1F0200)
#define JL_SFCENC	((struct JL_SFCENC_regs  *)0x1F0300)
#define JL_DCP		((struct JL_DCP_regs     *)0x1F1300)
#define JL_EQ		((struct JL_EQ_regs      *)0x1F1500)
#define JL_SRC		((struct JL_SRC_regs     *)0x1F1600)
#define JL_FM		((struct JL_FM_regs      *)0x1F1700)
#define JL_WL		((struct JL_WL_regs      *)0x1F1800)

/*============================================================================*/

#endif
