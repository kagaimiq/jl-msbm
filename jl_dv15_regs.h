
/*==========================================================================*/
/* EVA - Ehanced Visual Architecture - space */

/* Low-speed bus */
#define LSBUS_base	0xF60000
	#define LS_IO_base	LSBUS_base + 0x0000
		#define PORTA_base	LS_IO_base + 0x000
		#define PORTB_base	LS_IO_base + 0x040
		#define PORTC_base	LS_IO_base + 0x080
		#define PORTD_base	LS_IO_base + 0x0C0
		#define PORTE_base	LS_IO_base + 0x100
		#define PORTF_base	LS_IO_base + 0x140
		#define PORTG_base	LS_IO_base + 0x180
		#define PORTH_base	LS_IO_base + 0x1C0
		#define IOMAP_base	LS_IO_base + 0x200
		#define WAKEUP_base	LS_IO_base + 0x214
	#define LS_UART_base	LSBUS_base + 0x0400
		#define UART0_base	LS_UART_base + 0x000
		#define UART1_base	LS_UART_base + 0x040
		#define UART2_base	LS_UART_base + 0x080
		#define UART3_base	LS_UART_base + 0x0C0	/* "Lite" UART */
	#define LS_SPI_base	LSBUS_base + 0x0800
		#define SPI0_base	LS_SPI_base + 0x000
		#define SPI1_base	LS_SPI_base + 0x040
	#define LS_SD_base	LSBUS_base + 0x0C00
		#define SD0_base	LS_SD_base + 0x000
		#define SD1_base	LS_SD_base + 0x040
		#define SD2_base	LS_SD_base + 0x080
	#define LS_TMR_base	LSBUS_base + 0x1000
		#define TIMER0_base	LS_TMR_base + 0x000
		#define TIMER1_base	LS_TMR_base + 0x040
		#define TIMER2_base	LS_TMR_base + 0x080
		#define TIMER3_base	LS_TMR_base + 0x0C0
		#define PWM_base	LS_TMR_base + 0x100	/* aka 'MCPWM' */
	#define LS_FUSB_base	LSBUS_base + 0x1400
	#define LS_HUSB_base	LSBUS_base + 0x1800
	#define LS_ADDA_base	LSBUS_base + 0x1C00
	#define LS_CLK_base	LSBUS_base + 0x2000
	#define LS_OTH_base	LSBUS_base + 0x2400
		#define IRTC_base	LS_OTH_base + 0x00C
		#define CRC0_base	LS_OTH_base + 0x018
		#define IIC_base	LS_OTH_base + 0x02C
		#define PAP_base	LS_OTH_base + 0x048
		#define CRC1_base	LS_OTH_base + 0x060
		#define MPU_base	LS_OTH_base + 0x070
		#define PLCNT_base	LS_OTH_base + 0x084
		#define CS_base		LS_OTH_base + 0x08C
		#define RAND_base	LS_OTH_base + 0x0A0
		#define GPADC_base	LS_OTH_base + 0x0A8	/* aka 'ADC' */
	#define LS_ALNK_base	LSBUS_base + 0x2800
	#define HUSB_base	LSBUS_base + 0x8000

/* High-speed bus */
#define HSBUS_base	0xF70000
	#define HS_CPU_base	HSBUS_base + 0x0000
	#define HS_DBG_base	HSBUS_base + 0x0400
	#define HS_SDR_base	HSBUS_base + 0x0800
	#define HS_EVA_base	HSBUS_base + 0x0C00
	#define HS_SFC_base	HSBUS_base + 0x1000
	#define HS_JPG_base	HSBUS_base + 0x1400
	#define HS_OTH_base	HSBUS_base + 0x1800
		#define ENC_base	HS_OTH_base + 0x000
		#define DMA_base	HS_OTH_base + 0x014
		#define AES_base	HS_OTH_base + 0x040
		#define GPDMA_RD_base	HS_OTH_base + 0x080
		#define GPDMA_WR_base	HS_OTH_base + 0x0C0

#define JPG0_base	0xF74000
#define JPG1_base	0xF75000
#define HS_DMACOPY_base	0xF77000

/* Enhanced Visual Architecture */
#define EVA_base	0xF78000
	#define XBUS_base	EVA_base + 0x0000
	#define ISC_base	EVA_base + 0x0800
	#define ISP0_base	EVA_base + 0x1000
	#define ISP1_base	EVA_base + 0x1800
	#define IMC_base	EVA_base + 0x2000
	#define IMB_base	EVA_base + 0x2800
	#define IMD_DMM_base	EVA_base + 0x3000
	#define IMD_DPI_base	EVA_base + 0x3200
	#define CVBS_base	EVA_base + 0x3800
	#define TVE_base	EVA_base + 0x3C00
	#define CVE_base	EVA_base + 0x3E00
	#define MIPI_base	EVA_base + 0x4000
		#define CSI_base	MIPI_base + 0x0000
		#define DSI_S_base	MIPI_base + 0x0200
		#define DSI_D_base	MIPI_base + 0x0300
		#define MP_PHY_base	MIPI_base + 0x0400
	#define IMR_base	EVA_base + 0x4800

/*==========================================================================*/

/*----------------------------------*/
/* XBUS */

struct JL_XBUS_regs {
	volatile uint32_t	CH00_LVL;
	volatile uint32_t	CH01_LVL;
	volatile uint32_t	CH02_LVL;
	volatile uint32_t	CH03_LVL;
	volatile uint32_t	CH04_LVL;
	volatile uint32_t	CH05_LVL;
	volatile uint32_t	CH06_LVL;
	volatile uint32_t	CH07_LVL;
	volatile uint32_t	CH08_LVL;
	volatile uint32_t	CH09_LVL;
	volatile uint32_t	CH10_LVL;
	volatile uint32_t	CH11_LVL;
	volatile uint32_t	CH12_LVL;
	volatile uint32_t	CH13_LVL;
	volatile uint32_t	CH14_LVL;
	volatile uint32_t	CH15_LVL;
	volatile uint32_t	CH16_LVL;
	volatile uint32_t	CH17_LVL;
	volatile uint32_t	CH18_LVL;
	volatile uint32_t	CH19_LVL;
	volatile uint32_t	CH20_LVL;
	volatile uint32_t	CH21_LVL;
	volatile uint32_t	CH22_LVL;
	volatile uint32_t	CH23_LVL;
	volatile uint32_t	CH24_LVL;
	volatile uint32_t	CH25_LVL;
	volatile uint32_t	CH26_LVL;
	volatile uint32_t	CH27_LVL;
	volatile uint32_t	CH28_LVL;
	volatile uint32_t	CH29_LVL;
	volatile uint32_t	CH30_LVL;
	volatile uint32_t	CH31_LVL;
	volatile uint32_t	LV1_PRD;
	volatile uint32_t	LV2_PRD;
	volatile uint32_t	DIST0_CON;
	volatile uint32_t	DIST1_CON;
};

/*----------------------------------*/
/* ISC */

struct JL_ISC_regs_SEN {
	volatile uint32_t	CON;		/* [+00] */
	volatile uint32_t	VBLK;		/* [+04] */
	volatile uint32_t	VACT;		/* [+08] */
	volatile uint32_t	HBLK;		/* [+0C] */
	volatile uint32_t	HACT;		/* [+10] */
	uint32_t	Reserved_14[3];
};

struct JL_ISC_regs {
	volatile uint32_t	PND_CON;		/* [00] */
	volatile uint32_t	DMX_CON0;		/* [04] */
	volatile uint32_t	DMX_CON1;		/* [08] */
	uint32_t	Reserved_0C[5];
	struct JL_ISC_regs_SEN		SEN0;		/* [20-3C] */
	struct JL_ISC_regs_SEN		SEN1;		/* [40-5C] */
	struct JL_ISC_regs_SEN		LCDS;		/* [60-7C] */
};

#define JL_ISC		((struct JL_ISC_regs *)ISC_base)

/*----------------------------------*/
/* ISP */

struct JL_ISP_regs {
	volatile uint32_t	PND_CON;
	volatile uint32_t	SCN_CON;
	uint32_t	Reserved_004[2];

	volatile uint32_t	SRC_CON;
	volatile uint32_t	SRC_HAW;
	volatile uint32_t	SRC_VAW;
	uint32_t	Reserved_018[2];

	volatile uint32_t	BLC_OFF_R;
	volatile uint32_t	BLC_OFF_GR;
	volatile uint32_t	BLC_OFF_GB;
	volatile uint32_t	BLC_OFF_B;

	volatile uint32_t	DPC_TH0;
	volatile uint32_t	DPC_TH1;
	volatile uint32_t	DPC_TH2;
	uint32_t	Reserved_03C[1];

	volatile uint32_t	LSC_CEN_X;
	volatile uint32_t	LSC_CEN_Y;
	volatile uint32_t	LSC_DTH_TH;
	volatile uint32_t	LSC_DTH_PRM0;
	volatile uint32_t	LSC_DTH_PRM1;
	volatile uint32_t	LSC_LUT_R;
	volatile uint32_t	LSC_LUT_G;
	volatile uint32_t	LSC_LUT_B;

	volatile uint32_t	AWB_GAIN_R;
	volatile uint32_t	AWB_GAIN_G;
	volatile uint32_t	AWB_GAIN_B;

	volatile uint32_t	DRC_LUT;

	volatile uint32_t	TNR_CON;
	volatile uint32_t	TNR_BASE;
	volatile uint32_t	TNR_SIZE;
	volatile uint32_t	TNR_2D_STR;
	volatile uint32_t	TNR_3D_TH0;
	volatile uint32_t	TNR_3D_TH1;
	volatile uint32_t	TNR_MT_TH;
	volatile uint32_t	TNR_WMAX;
	volatile uint32_t	TNR_WMIN;
	volatile uint32_t	TNR_WSLOPE;
	volatile uint32_t	TNR_BREAK;
	volatile uint32_t	TNR_SCALE0;
	volatile uint32_t	TNR_SCALE1;
	volatile uint32_t	TNR_SCALE2;
	volatile uint32_t	TNR_SCALE3;
	volatile uint32_t	TNR_SCALE4;
	volatile uint32_t	TNR_SCALE5;
	volatile uint32_t	TNR_SCALE6;
	volatile uint32_t	TNR_SCALE7;
	uint32_t	Reserved_0BC[1];

	volatile uint32_t	CCM_R_COE0;
	volatile uint32_t	CCM_R_COE1;
	volatile uint32_t	CCM_R_COE2;
	volatile uint32_t	CCM_R_OFF;
	volatile uint32_t	CCM_G_COE0;
	volatile uint32_t	CCM_G_COE1;
	volatile uint32_t	CCM_G_COE2;
	volatile uint32_t	CCM_G_OFF;
	volatile uint32_t	CCM_B_COE0;
	volatile uint32_t	CCM_B_COE1;
	volatile uint32_t	CCM_B_COE2;
	volatile uint32_t	CCM_B_OFF;

	volatile uint32_t	GMA_R_LUT;
	volatile uint32_t	GMA_G_LUT;
	volatile uint32_t	GMA_B_LUT;
	volatile uint32_t	CSC_Y_LUT;

	volatile uint32_t	DNR_SIM_TH;		/* [100] */
	volatile uint32_t	DNR_RNG_SGM;
	volatile uint32_t	DNR_GAUS_C00;
	volatile uint32_t	DNR_GAUS_C01;
	volatile uint32_t	DNR_GAUS_C02;
	volatile uint32_t	DNR_GAUS_C03;
	volatile uint32_t	DNR_GAUS_C11;
	volatile uint32_t	DNR_GAUS_C12;
	volatile uint32_t	DNR_GAUS_C13;
	volatile uint32_t	DNR_GAUS_C22;
	volatile uint32_t	DNR_GAUS_C23;
	volatile uint32_t	DNR_GAUS_C33;
	volatile uint32_t	DNR_CMID_EN;

	volatile uint32_t	SHP_LONE_TH;
	volatile uint32_t	SHP_ECH_MIN;
	volatile uint32_t	SHP_ECH_MAX;

	volatile uint32_t	SHP_HF_TH0;
	volatile uint32_t	SHP_HF_TH1;
	volatile uint32_t	SHP_HF_TH2;
	volatile uint32_t	SHP_HF_AMT;
	volatile uint32_t	SHP_HF_GAIN;
	volatile uint32_t	SHP_HF_C00;
	volatile uint32_t	SHP_HF_C01;
	volatile uint32_t	SHP_HF_C02;
	volatile uint32_t	SHP_HF_C10;
	volatile uint32_t	SHP_HF_C11;
	volatile uint32_t	SHP_HF_C12;
	volatile uint32_t	SHP_HF_C20;
	volatile uint32_t	SHP_HF_C21;
	volatile uint32_t	SHP_HF_C22;
	volatile uint32_t	SHP_HF_SFT;
	uint32_t	Reserved_17C[1];

	volatile uint32_t	SHP_MF_TH0;		/* [180] */
	volatile uint32_t	SHP_MF_TH1;
	volatile uint32_t	SHP_MF_AMT;
	volatile uint32_t	SHP_MF_GAIN;
	volatile uint32_t	SHP_MF_C00;
	volatile uint32_t	SHP_MF_C01;
	volatile uint32_t	SHP_MF_C02;
	volatile uint32_t	SHP_MF_C10;
	volatile uint32_t	SHP_MF_C11;
	volatile uint32_t	SHP_MF_C12;
	volatile uint32_t	SHP_MF_C20;
	volatile uint32_t	SHP_MF_C21;
	volatile uint32_t	SHP_MF_C22;
	volatile uint32_t	SHP_MF_SFT;
	uint32_t	Reserved_1B8[2];

	volatile uint32_t	SHP_CR_SMT_TH;		/* [1C0] */
	volatile uint32_t	SHP_CR_C00;
	volatile uint32_t	SHP_CR_C01;
	volatile uint32_t	SHP_CR_C02;
	volatile uint32_t	SHP_CR_C10;
	volatile uint32_t	SHP_CR_C11;
	volatile uint32_t	SHP_CR_C12;
	volatile uint32_t	SHP_CR_C20;
	volatile uint32_t	SHP_CR_C21;
	volatile uint32_t	SHP_CR_C22;
	volatile uint32_t	SHP_CR_SFT;
	uint32_t	Reserved_1EC[5];

	volatile uint32_t	CBS_Y_GAIN;		/* [200] */
	volatile uint32_t	CBS_U_GAIN;
	volatile uint32_t	CBS_V_GAIN;
	volatile uint32_t	CBS_Y_OFFS;
	volatile uint32_t	CBS_U_OFFS;
	volatile uint32_t	CBS_V_OFFS;
	uint32_t	Reserved_218[2];

	volatile uint32_t	OUT_HST;		/* [220] */
	volatile uint32_t	OUT_HED;
	volatile uint32_t	OUT_VST;
	volatile uint32_t	OUT_VED;
	uint32_t	Reserved_230[4];

	volatile uint32_t	STC_AE_BASE0;		/* [240] */
	volatile uint32_t	STC_AE_BASE1;
	volatile uint32_t	STC_AE_BASE2;
	volatile uint32_t	STC_AE_BASE3;
	volatile uint32_t	STC_AE_BASEX;
	volatile uint32_t	STC_AE_EN;
	volatile uint32_t	STC_AE_LV1;
	volatile uint32_t	STC_AE_LV2;
	volatile uint32_t	STC_AE_LV3;
	volatile uint32_t	STC_AE_LV4;
	volatile uint32_t	STC_AE_LV5;
	volatile uint32_t	STC_AE_LV6;
	volatile uint32_t	STC_AE_LV7;
	uint32_t	Reserved_274[3];

	volatile uint32_t	STC_WB_BASE0;		/* [280] */
	volatile uint32_t	STC_WB_BASE1;
	volatile uint32_t	STC_WB_BASE2;
	volatile uint32_t	STC_WB_BASE3;
	volatile uint32_t	STC_WB_BASEX;
	volatile uint32_t	STC_WB_EN;
	volatile uint32_t	STC_WB_R_TH;
	volatile uint32_t	STC_WB_G_TH;
	volatile uint32_t	STC_WB_B_TH;
	volatile uint32_t	STC_WB_W_TH;
	volatile uint32_t	STC_WB_Y_MIN;
	volatile uint32_t	STC_WB_Y_MAX;
	volatile uint32_t	STC_WB_RG_MIN;
	volatile uint32_t	STC_WB_RG_MAX;
	volatile uint32_t	STC_WB_BG_MIN;
	volatile uint32_t	STC_WB_BG_MAX;
};

#define JL_ISP0		((struct JL_ISP_base *)ISP0_base)
#define JL_ISP1		((struct JL_ISP_base *)ISP1_base)

