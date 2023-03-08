#ifndef _JL_REGS_H
#define _JL_REGS_H

#include <stdint.h>

static inline uint32_t reg32_read(uint32_t addr) {
	return *(volatile uint32_t*)addr;
}
static inline void reg32_write(uint32_t addr, uint32_t val) {
	*(volatile uint32_t*)addr = val;
}
static inline void reg32_wsmask(uint32_t addr, int shift, uint32_t mask, uint32_t val) {
	*(volatile uint32_t*)addr =
		(*(volatile uint32_t*)addr & ~(mask << shift)) | ((val & mask) << shift);
}
static inline uint32_t reg32_rsmask(uint32_t addr, int shift, uint32_t mask) {
	return (*(volatile uint32_t*)addr >> shift) & mask;
}

#define _REG_MKVAL(addr, shift, mask, val)	(((val) & (mask)) << (shift))
#define REG_MKVAL(regdef, val)			_REG_MKVAL(regdef, val)

#define _REG_GETVAL(addr, shift, mask, val)	(((val) >> (shift)) & (mask))
#define REG_GETVAL(regdef, val)			_REG_GETVAL(regdef, val)

#define _REG_SMASK(addr, shift, mask)	((mask) << (shift))
#define REG_SMASK(regdef)			_REG_SMASK(regdef)

/*////////////////////////////////////////////////////////////////////////////*/
/* Low Speed SFR */
/*=============== SYSTEM ================*/
// system stuff
#define SYSTEM_base		0x60000

#define CHIP_ID				0x00
#define MODE_CON			0x04
#define LDO_CON				0x84
#define LVD_CON				0x88
#define WDT_CON				0x8C
#define OSA_CON				0x90
#define EFUSE_CON			0x94

/*=============== WAKEUP ================*/
// wakeup stuff
#define WAKEUP_base		0x60008

#define WAKEUP_CON0			0x00
#define WAKEUP_CON1			0x04
#define WAKEUP_CON2			0x08
#define WAKEUP_CON3			0x0C

/*================ IOMAP ================*/
// port io map
#define IOMAP_base		0x60018

#define IOMAP_CON0			0x00
#define 	IOMAP_CON0_sd0cken		IOMAP_CON0, 0, 1	// SD0 clock enable
#define 	IOMAP_CON0_sd0dten		IOMAP_CON0, 1, 1	// SD0 data enable
#define 	IOMAP_CON0_sd1cken		IOMAP_CON0, 2, 1	// SD1 clock enable
#define 	IOMAP_CON0_sd1dten		IOMAP_CON0, 3, 1	// SD1 data enable
#define 	IOMAP_CON0_sd1ios		IOMAP_CON0, 4, 0x1	// SD1 io mapping sel
#define 	IOMAP_CON0_spi0ios		IOMAP_CON0, 5, 0x1	// SPI0 io mapping sel
#define 	IOMAP_CON0_ut0ios		IOMAP_CON0, 6, 0x3	// UART0 io mapping sel
#define 	IOMAP_CON0_irfltos		IOMAP_CON0, 8, 0x7	// IRFLT io mapping sel
#define 	IOMAP_CON0_alnkios		IOMAP_CON0, 11, 0x1	// ALNK io mapping sel
#define 	IOMAP_CON0_papwen		IOMAP_CON0, 12, 1	// PAP WR enable
#define 	IOMAP_CON0_papren		IOMAP_CON0, 13, 1	// PAP RD enable
#define 	IOMAP_CON0_papden		IOMAP_CON0, 14, 1	// PAP Dx enable
#define 	IOMAP_CON0_sd0ios		IOMAP_CON0, 15, 0x1	// SD0 io mapping sel
#define IOMAP_CON1			0x04
#define 	IOMAP_CON1_ut1ios		IOMAP_CON1, 2, 0x3	// UART1 io mapping sel
#define 	IOMAP_CON1_spi1ios		IOMAP_CON1, 4, 0x1	// SPI1 io mapping sel
#define 	IOMAP_CON1_sfcios		IOMAP_CON1, 5, 0x1	// SFC io mapping sel
#define 	IOMAP_CON1_capeds		IOMAP_CON1, 6, 0x1	// "capture side edge" [rising|falling]
#define 	IOMAP_CON1_out_ch0s		IOMAP_CON1, 8, 0x7	// ... select [ut0_tx|ut1_tx|tmr2_pwm_out|tmr3_pwm_out|irtcx32k|osc_clk|wlu_tx|pwm4_out]
#define 	IOMAP_CON1_out_ch1s		IOMAP_CON1, 11, 0x7	// ... select [ut0_tx|ut1_tx|tmr9_pwm_out|tmr1_pwm_out|rtc_osch|btosc_clk|pll_12m|ut2_tx]
#define 	IOMAP_CON1_ut2ios		IOMAP_CON1, 14, 0x3	// UART2 io mapping sel
#define 	IOMAP_CON1_spi2ios		IOMAP_CON1, 16, 0x1	// SPI2 io mapping sel
#define 	IOMAP_CON1_iicios		IOMAP_CON1, 18, 0x3	// IIC io mapping sel
#define 	IOMAP_CON1_wluios		IOMAP_CON1, 20, 0x3	// WLU (some uart?? for BT?) io mapping sel
#define IOMAP_CON2			0x08
#define 	IOMAP_CON2_wkups		IOMAP_CON2, 0, 0x3f	// Wakeup pin select
#define 	IOMAP_CON2_irflts		IOMAP_CON2, 8, 0x3f	// IRFLT pin select
#define 	IOMAP_CON2_caps			IOMAP_CON2, 16, 0x3f	// CAP pin select
#define 	IOMAP_CON2_uts			IOMAP_CON2, 24, 0x3f	// UART pin select
#define IOMAP_CON3			0x0C
#define 	IOMAP_CON3_ut0mxs		IOMAP_CON3, 0, 0x7	// UART0 mux sel [0-3=iomux]
#define 	IOMAP_CON3_ut0ioen		IOMAP_CON3, 3, 1	// UART0 io enable
#define 	IOMAP_CON3_ut1mxs		IOMAP_CON3, 4, 0x7	// UART1 mux sel
#define 	IOMAP_CON3_ut1ioen		IOMAP_CON3, 7, 1	// UART1 io enable
#define 	IOMAP_CON3_ut2mxs		IOMAP_CON3, 8, 0x7	// UART2 mux sel
#define 	IOMAP_CON3_ut2ioen		IOMAP_CON3, 11, 1	// UART2 io enable

/*================ POWER ================*/
// power stuff
#define POWER_base		0x60040

#define POWER_CON			0x00

/*================ CLOCK ================*/
// clock stuff
#define CLOCK_base		0x60044

#define CLOCK_SYS_DIV			0x00
#define 	CLOCK_SYS_DIV_orgdiv		CLOCK_SYS_DIV, 0, 0xff		// org_clk -> the rest divider (+1)
#define 	CLOCK_SYS_DIV_lsbdiv		CLOCK_SYS_DIV, 8, 0x7		// lsb_clk divider (+1)
#define CLOCK_CLK_CON0			0x04
#define 	CLOCK_CLK_CON0_rc_en		CLOCK_CLK_CON0, 0, 1		// RC osc enable
#define 	CLOCK_CLK_CON0_tssel		CLOCK_CLK_CON0, 1, 0x1		// 
#define 	CLOCK_CLK_CON0_osc_sel		CLOCK_CLK_CON0, 4, 0x3		// 
#define 	CLOCK_CLK_CON0_cksel		CLOCK_CLK_CON0, 6, 0x7		// 
#define 	CLOCK_CLK_CON0_sfr_ckmd		CLOCK_CLK_CON0, 9, 1		// 
#define 	CLOCK_CLK_CON0_pb0_sel		CLOCK_CLK_CON0, 10, 0x7		// PB0 output sel [io|lsb_clk|btosc_clk|rtosh_clk|rc_clk|hsb_clk|rtosl_clk|pll_sys_clk]
#define 	CLOCK_CLK_CON0_pb9_sel		CLOCK_CLK_CON0, 13, 0x7		// PB9 output sel [io|fm_lo_d2|pll_rfi_clk|pll_192m|bt_lo_d32|wl_clk|apc_clk|rccl_clk]
#define CLOCK_CLK_CON1			0x08
#define 	CLOCK_CLK_CON1_usb_cksel	CLOCK_CLK_CON1, 0, 0x3		// USB clock sel [pll_48m|dis|lsb_clk|dis]
#define 	CLOCK_CLK_CON1_dac_cksel	CLOCK_CLK_CON1, 2, 0x3		// DAC clock sel [pll_24m|dis|lsb_clk|dis]
#define 	CLOCK_CLK_CON1_apc_cksel	CLOCK_CLK_CON1, 6, 0x3		// API clock sel [pll_apc_clk|dis|lsb_clk|dis]
#define 	CLOCK_CLK_CON1_lcd_cksel	CLOCK_CLK_CON1, 8, 0x3		// LCD clock sel [wclk|rtosl_clk|lsb_clk|dis]
#define 	CLOCK_CLK_CON1_uart_cksel	CLOCK_CLK_CON1, 10, 0x3		// UART clock sel [osc_clk|pll_48m|lsb_clk|dis]
#define 	CLOCK_CLK_CON1_rfi_cksel	CLOCK_CLK_CON1, 12, 0x3		// RFI clock sel [pll_rfi_clk|dis|lsb_clk|dis]
#define 	CLOCK_CLK_CON1_bt_cksel		CLOCK_CLK_CON1, 14, 0x3		// BT clock sel [pll_wl_clk|dis|lsb_clk|dis]
#define 	CLOCK_CLK_CON1_mem_scke		CLOCK_CLK_CON1, 16, 1		// memory clock always on enable
#define 	CLOCK_CLK_CON1_tmsel		CLOCK_CLK_CON1, 17, 1		// don't use test_mode clock as the system clock
#define 	CLOCK_CLK_CON1_rom_pd		CLOCK_CLK_CON1, 18, 1		// ROM powerdown
#define 	CLOCK_CLK_CON1_wl_clkinv	CLOCK_CLK_CON1, 19, 1		// WL clock invert
#define 	CLOCK_CLK_CON1_sfc_dsel		CLOCK_CLK_CON1, 28, 0x3		// SFC clock delay
#define CLOCK_CLK_CON2			0x0C
#define 	CLOCK_CLK_CON2_pll_sys_sel	CLOCK_CLK_CON2, 0, 0x3		// pll_sys_clk clock sel [pll_192m|pll_480m|dis|dis]
#define 	CLOCK_CLK_CON2_pll_sys_div0	CLOCK_CLK_CON2, 2, 0x3		// pll_sys_clk divider 0 [div1|div3|div5|div7]
#define 	CLOCK_CLK_CON2_pll_sys_div1	CLOCK_CLK_CON2, 4, 0x3		// pll_sys_clk divider 1 [div1|div2|div4|div8]
#define 	CLOCK_CLK_CON2_pll_rfi_sel	CLOCK_CLK_CON2, 12, 0x3		// rfi_clk clock sel [pll_192m|pll_480m|dis|dis]
#define 	CLOCK_CLK_CON2_pll_rfi_div0	CLOCK_CLK_CON2, 14, 0x3		// rfi_clk divider 0 [div1|div3|div5|div7]
#define 	CLOCK_CLK_CON2_pll_rfi_div1	CLOCK_CLK_CON2, 16, 0x3		// rfi_clk divider 1 [div1|div2|div4|div8]
#define 	CLOCK_CLK_CON2_pll_apc_sel	CLOCK_CLK_CON2, 18, 0x3		// apc_clk clock sel [pll_192m|pll_480m|dis|dis]
#define 	CLOCK_CLK_CON2_pll_apc_div0	CLOCK_CLK_CON2, 20, 0x3		// apc_clk divider 0 [div1|div3|div5|div7]
#define 	CLOCK_CLK_CON2_pll_apc_div1	CLOCK_CLK_CON2, 22, 0x3		// apc_clk divider 1 [div1|div2|div4|div8]
#define 	CLOCK_CLK_CON2_pll_alnk_sel	CLOCK_CLK_CON2, 30, 0x1		//
#define CLOCK_PLL_CON			0x3C
#define 	CLOCK_PLL_CON_pll_en		CLOCK_PLL_CON, 0, 1		// PLL enable
#define 	CLOCK_PLL_CON_pll_rst		CLOCK_PLL_CON, 1, 1		// PLL reset
#define 	CLOCK_PLL_CON_pll_refds		CLOCK_PLL_CON, 2, 0x1f		// ref clock divider (n + 2)
#define 	CLOCK_PLL_CON_pll_refde		CLOCK_PLL_CON, 7, 1		// ref clock divider enable
#define 	CLOCK_PLL_CON_pll_rsel		CLOCK_PLL_CON, 8, 0x3		// ref clock sel [bt_osc|rt_oscl|<pll_ref_sel>|none]
#define 	CLOCK_PLL_CON_pll_divs		CLOCK_PLL_CON, 10, 1		// divider mode [integer|fractional]
#define 	CLOCK_PLL_CON_pll_dsms		CLOCK_PLL_CON, 11, 1		// fractional modulator [CIFF|MASH]
#define 	CLOCK_PLL_CON_pll_tsel		CLOCK_PLL_CON, 12, 0xf		// fractional value thing
#define 	CLOCK_PLL_CON_pll_rsel12	CLOCK_PLL_CON, 16, 1		// reference clock freq [2 MHz | 12 MHz]
#define 	CLOCK_PLL_CON_pll_ref_sel	CLOCK_PLL_CON, 17, 0x3		// ref clock sel 2 [bt_osc|rt_osch|none|pat_clk]
#define 	CLOCK_PLL_CON_pll_test		CLOCK_PLL_CON, 20, 1		// test mode

/*================ PORT =================*/
// gpio port
#define PORTA_base		0x60100
#define PORTB_base		0x60120
#define PORTC_base		0x60140
#define PORTD_base		0x60160

#define PORTx_OUT			0x00	// port output reg
#define 	PORTx_OUTn(n)			PORTx_OUT, (n), 1	// 0=low, 1=high
#define PORTx_IN			0x04	// port input reg
#define 	PORTx_INn(n)			PORTx_IN, (n), 1	// 0=low, 1=high
#define PORTx_DIR			0x08	// port direction reg
#define 	PORTx_DIRn(n)			PORTx_DIR, (n), 1	// 0=out, 1=in [[MStar OEN]]
#define PORTx_DIE			0x0C	// port digital input enable
#define 	PORTx_DIEn(n)			PORTx_DIE, (n), 1	// 0=disable, 1=enable
#define PORTx_PU			0x10	// port pullup enable
#define 	PORTx_PUn(n)			PORTx_PU, (n), 1	// 0=disable, 1=enable
#define PORTx_PD			0x14	// port pulldown enable
#define 	PORTx_PDn(n)			PORTx_PD, (n), 1	// 0=disable, 1=enable
#define PORTx_HD			0x18

/*================ TIMER ================*/
// timer
#define TIMER0_base		0x60200
#define TIMER1_base		0x60210
#define TIMER2_base		0x60220
#define TIMER3_base		0x60230

#define TIMERx_CON			0x00	// control
#define 	TIMERx_CON_mode			TIMERx_CON, 0, 0x3	// timer mode [dis|count|io rising cap|io falling cap]
#define 	TIMERx_CON_ssel			TIMERx_CON, 2, 0x3	// clock sel [bus|io|osc|rc]
#define 	TIMERx_CON_pseta		TIMERx_CON, 4, 0x3	// prescaler (1) [1|4|16|64]
#define 	TIMERx_CON_psetb		TIMERx_CON, 6, 0x3	// prescaler (2) [1|2|256|512]
#define 	TIMERx_CON_pwmen		TIMERx_CON, 8, 1	// pwm enable
#define 	TIMERx_CON_pwminv		TIMERx_CON, 9, 1	// pwm invert
#define 	TIMERx_CON_pclr			TIMERx_CON, 14, 1	// clear pending int
#define 	TIMERx_CON_pnd			TIMERx_CON, 15, 1	// int pending
#define TIMERx_CNT			0x04	// counter
#define TIMERx_PRD			0x08	// period
#define TIMERx_PWM			0x0C	// pwm duty cycle

/*================ UART =================*/
// uart interface
#define UART0_base		0x60300
#define UART1_base		0x60324
#define UART2_base		0x60348

#define UARTx_CON0			0x00	// control 0
#define 	UARTx_CON0_uten			UARTx_CON0, 0, 1	// uart enable
#define 	UARTx_CON0_m9en			UARTx_CON0, 1, 1	// frame size [8 bit|9 bit]
#define 	UARTx_CON0_txie			UARTx_CON0, 2, 1	// tx int enable
#define 	UARTx_CON0_rxie			UARTx_CON0, 3, 1	// rx int enable
#define 	UARTx_CON0_divs			UARTx_CON0, 4, 1	// baudrate predivide [/4|/3]
#define 	UARTx_CON0_ot_ie		UARTx_CON0, 5, 1	// timeout interrupt enable
#define 	UARTx_CON0_rx_mode		UARTx_CON0, 6, 1	// rx mode [pio|dma]
#define 	UARTx_CON0_rdc			UARTx_CON0, 7, 1	// received amount clear
#define 	UARTx_CON0_rb8			UARTx_CON0, 8, 1	// rx frame bit8
#define 	UARTx_CON0_tb8			UARTx_CON0, 9, 1	// tx frame bit8
#define 	UARTx_CON0_clr_otpnd		UARTx_CON0, 10, 1	// clear timeout pending
#define 	UARTx_CON0_otpnd		UARTx_CON0, 11, 1	// timeout pending
#define 	UARTx_CON0_clrrpnd		UARTx_CON0, 12, 1	// clear rx pending
#define 	UARTx_CON0_clrtpnd		UARTx_CON0, 13, 1	// clear tx pending
#define 	UARTx_CON0_rpnd			UARTx_CON0, 14, 1	// rx pending
#define 	UARTx_CON0_tpnd			UARTx_CON0, 15, 1	// tx pending
#define UARTx_CON1			0x04	// control 1
#define 	UARTx_CON1_rtse			UARTx_CON1, 0, 1	// enable rts
#define 	UARTx_CON1_rts_dmaen		UARTx_CON1, 1, 1	// rts dma enable ??
#define 	UARTx_CON1_ctse			UARTx_CON1, 2, 1	// enable cts
#define 	UARTx_CON1_ctsie		UARTx_CON1, 3, 1	// cts int enable
#define 	UARTx_CON1_baud_frac		UARTx_CON1, 4, 0x3	// baudrate fraction
#define 	UARTx_CON1_clrrts		UARTx_CON1, 13, 1	// clear rts
#define UART2_BUF			0x04	// data reg (UART2)
#define UARTx_BAUD			0x08	// baudrate
#define UARTx_BUF			0x0C	// data reg
#define UARTx_OTCNT			0x10	// timeout
#define UARTx_TXADR			0x14	// tx dma address
#define UARTx_TXCNT			0x18	// tx dma length (triggers dma)
#define UARTx_RXSADR			0x1C	// rx dma start address
#define UARTx_RXEADR			0x20	// rx dma end address
#define UARTx_RXCNT			0x24	// rx dma length
#define UARTx_HRXCNT			0x28	// rx dma received length

/*================= SPI =================*/
// spi interface
#define SPI0_base		0x60400
#define SPI1_base		0x60414
#define SPI2_base		0x60428

#define SPIx_CON			0x00	// control
#define 	SPIx_CON_spie			SPIx_CON, 0, 1		// spi enable
#define 	SPIx_CON_slave			SPIx_CON, 1, 1		// slave mode
#define 	SPIx_CON_cse			SPIx_CON, 2, 1		// enable cs
#define 	SPIx_CON_bidir			SPIx_CON, 3, 1		// full-duplex mode
#define 	SPIx_CON_se			SPIx_CON, 4, 1		// data sample [on ck rising edge|on ck falling edge]
#define 	SPIx_CON_ue			SPIx_CON, 5, 1		// data update [on ck rising edge|on ck falling edge]
#define 	SPIx_CON_ckid			SPIx_CON, 6, 1		// ck polarity [low when idle|high when idle]
#define 	SPIx_CON_csid			SPIx_CON, 7, 1		// cs polarity [low when idle|high when idle]
#define 	SPIx_CON_datw			SPIx_CON, 10, 0x3	// data width [1 bit|2 bits|4 bits]
#define 	SPIx_CON_dir			SPIx_CON, 12, 1		// transfer direction [tx|rx]
#define 	SPIx_CON_ie			SPIx_CON, 13, 1		// int enable
#define 	SPIx_CON_pclr			SPIx_CON, 14, 1		// pending int clear
#define 	SPIx_CON_pnd			SPIx_CON, 15, 1		// pending int
#define SPIx_BAUD			0x04	// baudrate
#define SPIx_BUF			0x08	// data reg
#define SPIx_ADR			0x0C	// dma address
#define SPIx_CNT			0x10	// dma length (triggers dma)

/*================= PAP =================*/
// parralel active port
#define PAP_base		0x60500

#define PAP_CON				0x00	// control
#define PAP_DAT0			0x04	// data reg 0
#define PAP_DAT1			0x08	// data reg 1
#define PAP_ADR				0x10	// dma address
#define PAP_CNT				0x14	// dma length

/*================= SD ==================*/
// sd/mmc host
#define SD0_base		0x60600
#define SD1_base		0x60614

#define SDx_CON0			0x00	// control 0
#define SDx_CON1			0x04	// control 1
#define SDx_CON2			0x08	// control 2
#define SDx_CPTR			0x0C	// command buffer address
#define SDx_DPTR			0x10	// data buffer address

/*================= IIC =================*/
// i2c interface
#define IIC_base		0x60700

// maybe it doesn't support slave mode...

#define IIC_CON0			0x00	// control 0
#define 	IIC_CON0_en			IIC_CON0, 0, 1		// enable
#define 	IIC_CON0_slave			IIC_CON0, 1, 1		// slave mode [0: master | 1: slave]
#define 	IIC_CON0_n_cfg_done		IIC_CON0, 2, 1		// fire a transfer
#define 	IIC_CON0_dat_dir		IIC_CON0, 3, 1		// data direction [0: send | 1: receive]
#define 	IIC_CON0_m_set_end		IIC_CON0, 4, 1		// send stop
#define 	IIC_CON0_m_set_rstart		IIC_CON0, 5, 1		// send (re)start
#define 	IIC_CON0_wr_ack			IIC_CON0, 6, 1		// txed ack [0: ack | 1: nak]
#define 	IIC_CON0_rd_ack			IIC_CON0, 7, 1		// rxed ack [0: ack | 1: nak]
#define 	IIC_CON0_ie			IIC_CON0, 8, 1		// interrupt enable
#define 	IIC_CON0_iic_isel		IIC_CON0, 9, 1		// input sel [0: direct | 1: filtered]
#define 	IIC_CON0_end_pnd_ie		IIC_CON0, 10, 1		// enable end interrupt
#define 	IIC_CON0_end_pnd_clr		IIC_CON0, 12, 1		// clear pending end int
#define 	IIC_CON0_end_pnd		IIC_CON0, 13, 1		// end int pending
#define 	IIC_CON0_pclr			IIC_CON0, 14, 1		// clear pending int
#define 	IIC_CON0_pnd			IIC_CON0, 15, 1		// interrupt pending
#define IIC_BUF				0x04	// data reg
#define IIC_BAUD			0x08	// baudrate / slave address

/*================= LCD =================*/
// liquid crystal display (lcd) controller
#define LCD_base		0x60800

#define LCD_CON0			0x00	// control
#define LCD_SEG_IOEN0			0x04	// segment io enable 0
#define LCD_SEG_IOEN1			0x08	// segment io enable 1

/*================ PWM4 =================*/
// dunno
#define PWM4_base		0x60900

#define PWM4_CON			0x00	// control

/*================ IRTC =================*/
// (i) rtc
#define IRTC_base		0x60904

#define IRTC_CON			0x00	// control

/*================= IR ==================*/
// ir decoder
#define IR_base			0x60908

#define IR_FLT_CON			0x00	// flt control

/*================ AUDIO ===============*/
// audio codec
#define AUDIO_base		0x60A00

#define AUDIO_DAC_LEN			0x00	// dac data length (count of samples in one buff slice, not in bytes!!)
#define AUDIO_DAC_CON			0x04	// dac control
#define 	AUDIO_DAC_CON_dacsr		AUDIO_DAC_CON, 0, 0xf		// dac sample rate [44100|48000|32000|=2|22050|24000|16000|=6|11025|12000|8000|=10|=8|=9|=10|=10]
#define 	AUDIO_DAC_CON_dacen		AUDIO_DAC_CON, 4, 1		// dac enable
#define 	AUDIO_DAC_CON_dacie		AUDIO_DAC_CON, 5, 1		// int enable
#define 	AUDIO_DAC_CON_cpnd		AUDIO_DAC_CON, 6, 1		// clear pending int
#define 	AUDIO_DAC_CON_pnd		AUDIO_DAC_CON, 7, 1		// pending int
#define 	AUDIO_DAC_CON_buff		AUDIO_DAC_CON, 8, 1		// current buffer slice
#define 	AUDIO_DAC_CON_dccs		AUDIO_DAC_CON, 12, 0xf		// dc cancelling filter setting [dis|...]
#define AUDIO_DAC_ADR			0x08	// dac data adress
#define AUDIO_DAC_TRML			0x0C	// dac trim left
#define AUDIO_DAC_TRMR			0x10 	// dac trim right
#define AUDIO_LADC_CON			0x20	// ladc control
#define 	AUDIO_LADC_CON_adcsr		AUDIO_LADC_CON, 0, 0xf		// adc sample rate [44100|48000|32000|=2|22050|24000|16000|=6|11025|12000|8000|=10|=8|=9|=10|=10]
#define 	AUDIO_LADC_CON_adcen		AUDIO_LADC_CON, 4, 1		// adc enable
#define 	AUDIO_LADC_CON_adcie		AUDIO_LADC_CON, 5, 1		// int enable
#define 	AUDIO_LADC_CON_cpnd		AUDIO_LADC_CON, 6, 1		// clear pending int
#define 	AUDIO_LADC_CON_pnd		AUDIO_LADC_CON, 7, 1		// pending int
#define 	AUDIO_LADC_CON_buff		AUDIO_LADC_CON, 8, 1		// current buffer slice
#define 	AUDIO_LADC_CON_che0		AUDIO_LADC_CON, 9, 1		// adc channel 0 en
#define 	AUDIO_LADC_CON_che1		AUDIO_LADC_CON, 10, 1		// adc channel 1 en
#define 	AUDIO_LADC_CON_che2		AUDIO_LADC_CON, 11, 1		// adc channel 2 en
#define 	AUDIO_LADC_CON_dccs		AUDIO_LADC_CON, 12, 0xf		// dc cancelling filter setting [dis|...]
#define AUDIO_LADC_ADR			0x2C	// ladc data address
#define AUDIO_LADC_LEN			0x30	// ladc data length
#define AUDIO_DAA_CON0			0x40	// daa control reg 0
#define 	AUDIO_DAA_CON0_dac_en		AUDIO_DAA_CON0, 0, 1		// DAC enable
#define 	AUDIO_DAA_CON0_dac_dtsel	AUDIO_DAA_CON0, 1, 1		// DAC clock dead time selection
#define 	AUDIO_DAA_CON0_ldo1_en		AUDIO_DAA_CON0, 2, 1		// LDO1 enable
#define 	AUDIO_DAA_CON0_ldo2_en		AUDIO_DAA_CON0, 3, 1		// LDO2 enable => for DACVDD?
#define 	AUDIO_DAA_CON0_hp_l_en		AUDIO_DAA_CON0, 4, 1		// DAC left channel enable (headphone amp left channel?)
#define 	AUDIO_DAA_CON0_hp_r_en		AUDIO_DAA_CON0, 5, 1		// DAC right channel enable (headphone amp right channel?)
#define 	AUDIO_DAA_CON0_pns_en		AUDIO_DAA_CON0, 6, 1		// enable the 50k pulldown resistor on DAC outputs
#define 	AUDIO_DAA_CON0_mute		AUDIO_DAA_CON0, 7, 1		// DAC & AMUX mute
#define 	AUDIO_DAA_CON0_pns10k_en	AUDIO_DAA_CON0, 8, 1		// enable the 10k pulldown resistor on DAC outputs
#define 	AUDIO_DAA_CON0_trim_en		AUDIO_DAA_CON0, 13, 1		// enable trimming
#define 	AUDIO_DAA_CON0_trim_sel		AUDIO_DAA_CON0, 14, 1		// trim select
#define 	AUDIO_DAA_CON0_trim_sw		AUDIO_DAA_CON0, 15, 1		// trim switch
#define AUDIO_DAA_CON1			0x44	// daa control reg 1
#define 	AUDIO_DAA_CON1_lg_sel		AUDIO_DAA_CON1, 0, 0x1f		// left channel volume
#define 	AUDIO_DAA_CON1_lr_2_l		AUDIO_DAA_CON1, 6, 1		// line-in L+R output to L
#define 	AUDIO_DAA_CON1_lr_2_r		AUDIO_DAA_CON1, 7, 1		// line-in L+R output to R
#define 	AUDIO_DAA_CON1_rg_sel		AUDIO_DAA_CON1, 8, 0x1f		// right channel volume
#define 	AUDIO_DAA_CON1_vcm_rsel		AUDIO_DAA_CON1, 13, 1		// VCM bias resistor select
#define 	AUDIO_DAA_CON1_mic_2_l		AUDIO_DAA_CON1, 14, 1		// MIC output to L
#define 	AUDIO_DAA_CON1_mic_2_r		AUDIO_DAA_CON1, 15, 1		// MIC output to R
#define AUDIO_DAA_CON2			0x48	// daa control reg 2
#define 	AUDIO_DAA_CON2_lin0l_en		AUDIO_DAA_CON2, 0, 1		// Line-in 0 left channel (AMUX0L) enable
#define 	AUDIO_DAA_CON2_lin0r_en		AUDIO_DAA_CON2, 1, 1		// Line-in 0 right channel (AMUX0R) enable
#define 	AUDIO_DAA_CON2_lin1l_en		AUDIO_DAA_CON2, 2, 1		// Line-in 1 left channel (AMUX1L) enable
#define 	AUDIO_DAA_CON2_lin1r_en		AUDIO_DAA_CON2, 3, 1		// Line-in 1 right channel (AMUX1R) enable
#define 	AUDIO_DAA_CON2_lin2l_en		AUDIO_DAA_CON2, 4, 1		// Line-in 2 left channel (AMUX2L) enable
#define 	AUDIO_DAA_CON2_lin2r_en		AUDIO_DAA_CON2, 5, 1		// Line-in 2 right channel (AMUX2R) enable
#define 	AUDIO_DAA_CON2_amux_g		AUDIO_DAA_CON2, 6, 1		// AMUX gain
#define 	AUDIO_DAA_CON2_amux_en		AUDIO_DAA_CON2, 7, 1		// AMUX enable
#define 	AUDIO_DAA_CON2_vcm_det_en	AUDIO_DAA_CON2, 8, 1		// VCM "reset enable"??
#define 	AUDIO_DAA_CON2_vcm_en		AUDIO_DAA_CON2, 9, 1		// VCOM enable
#define 	AUDIO_DAA_CON2_vcm_out_en	AUDIO_DAA_CON2, 10, 1		// enable VCOMO
#define 	AUDIO_DAA_CON2_vcm_out_pd	AUDIO_DAA_CON2, 11, 1		// enable the 1k pulldown on VCOMO
#define 	AUDIO_DAA_CON2_amux_bias_en	AUDIO_DAA_CON2, 12, 1		// AMUX bias enable
#define 	AUDIO_DAA_CON2_amux_mute	AUDIO_DAA_CON2, 13, 1		// AMUX mute
#define AUDIO_DAA_CON3			0x4C	// daa control reg 3
#define 	AUDIO_DAA_CON3_mic_g		AUDIO_DAA_CON3, 0, 0x3f		// MIC gain
#define 	AUDIO_DAA_CON3_mic_gx2		AUDIO_DAA_CON3, 6, 1		// MIC 2x gain enable
#define 	AUDIO_DAA_CON3_mic_mute		AUDIO_DAA_CON3, 7, 1		// MIC mute
#define 	AUDIO_DAA_CON3_lin0l_bias_en	AUDIO_DAA_CON3, 8, 1		// Line-in 0 left channel (AMUX0L) bias enable
#define 	AUDIO_DAA_CON3_lin0r_bias_en	AUDIO_DAA_CON3, 9, 1		// Line-in 0 right channel (AMUX0R) bias enable
#define 	AUDIO_DAA_CON3_lin1l_bias_en	AUDIO_DAA_CON3, 10, 1		// Line-in 1 left channel (AMUX1L) bias enable
#define 	AUDIO_DAA_CON3_lin1r_bias_en	AUDIO_DAA_CON3, 11, 1		// Line-in 1 right channel (AMUX1R) bias enable
#define 	AUDIO_DAA_CON3_lin2l_bias_en	AUDIO_DAA_CON3, 12, 1		// Line-in 2 left channel (AMUX2L) bias enable
#define 	AUDIO_DAA_CON3_lin2r_bias_en	AUDIO_DAA_CON3, 13, 1		// Line-in 2 right channel (AMUX2R) bias enable
#define 	AUDIO_DAA_CON3_mic_en		AUDIO_DAA_CON3, 14, 1		// MIC enable
#define 	AUDIO_DAA_CON3_trim_out		AUDIO_DAA_CON3, 15, 1		// trim output
#define AUDIO_DAA_CON4			0x50	// daa control reg 4
#define 	AUDIO_DAA_CON4_dac_isel5u	AUDIO_DAA_CON4, 0, 1		// 
#define 	AUDIO_DAA_CON4_dac_isel_third	AUDIO_DAA_CON4, 1, 1		// 
#define 	AUDIO_DAA_CON4_dac_isel_half	AUDIO_DAA_CON4, 2, 1		// 
#define 	AUDIO_DAA_CON4_mic_neg12	AUDIO_DAA_CON4, 3, 1		// MIC -12dB attenuation
#define AUDIO_DAA_CON5			0x54	// daa control reg 5
#define 	AUDIO_DAA_CON5_cken		AUDIO_DAA_CON5, 0, 1		// DAC clock output to PA4 enable
#define 	AUDIO_DAA_CON5_daten		AUDIO_DAA_CON5, 1, 1		// DAC data output to PA5(L)/PA6(R) enable
#define 	AUDIO_DAA_CON5_dac_ext		AUDIO_DAA_CON5, 2, 1		// 
#define 	AUDIO_DAA_CON5_adc_coe		AUDIO_DAA_CON5, 3, 1		// ADC clock output to PB8 enable
#define 	AUDIO_DAA_CON5_adc_doe		AUDIO_DAA_CON5, 4, 1		// ADC data output to PB3/PB2(ch1), PB5/PB4(ch2), PB7/PB6(ch3) enable
#define 	AUDIO_DAA_CON5_adc_dit		AUDIO_DAA_CON5, 5, 1		// ADC dither enable
#define AUDIO_ADA_CON0			0x80	// ada control reg 0
#define 	AUDIO_ADA_CON0_adc0_en		AUDIO_ADA_CON0, 0, 1		// ADC0 enable
#define 	AUDIO_ADA_CON0_adc0_channel_en	AUDIO_ADA_CON0, 1, 1		// ADC0 channel enable
#define 	AUDIO_ADA_CON0_adc0_ff_en	AUDIO_ADA_CON0, 2, 1		// ADC0 FF mode
#define 	AUDIO_ADA_CON0_adc0_test	AUDIO_ADA_CON0, 3, 1		// ADC0 test mode
#define 	AUDIO_ADA_CON0_adc0_s1_isel	AUDIO_ADA_CON0, 4, 0x3		// ADC0 
#define 	AUDIO_ADA_CON0_adc0_dither_cfg	AUDIO_ADA_CON0, 6, 0x3		// ADC0 dither config
#define 	AUDIO_ADA_CON0_adc0_pga_g	AUDIO_ADA_CON0, 8, 0xf		// ADC0 PGA gain
#define 	AUDIO_ADA_CON0_adc0_pga_en	AUDIO_ADA_CON0, 12, 1		// ADC0 PGA enable
#define 	AUDIO_ADA_CON0_adc0_isel	AUDIO_ADA_CON0, 13, 1		// ADC0 
#define AUDIO_ADA_CON1			0x84	// ada control reg 1
#define 	AUDIO_ADA_CON1_adc1_en		AUDIO_ADA_CON1, 0, 1		// ADC1 enable
#define 	AUDIO_ADA_CON1_adc1_channel_en	AUDIO_ADA_CON1, 1, 1		// ADC1 channel enable
#define 	AUDIO_ADA_CON1_adc1_ff_en	AUDIO_ADA_CON1, 2, 1		// ADC1 FF mode
#define 	AUDIO_ADA_CON1_adc1_test	AUDIO_ADA_CON1, 3, 1		// ADC1 test mode
#define 	AUDIO_ADA_CON1_adc1_s1_isel	AUDIO_ADA_CON1, 4, 0x3		// ADC1 
#define 	AUDIO_ADA_CON1_adc1_dither_cfg	AUDIO_ADA_CON1, 6, 0x3		// ADC1 dither config
#define 	AUDIO_ADA_CON1_adc1_pga_g	AUDIO_ADA_CON1, 8, 0xf		// ADC1 PGA gain
#define 	AUDIO_ADA_CON1_adc1_pga_en	AUDIO_ADA_CON1, 12, 1		// ADC1 PGA enable
#define AUDIO_ADA_CON2			0x88	// ada control reg 2
#define 	AUDIO_ADA_CON2_adc2_en		AUDIO_ADA_CON2, 0, 1		// ADC2 enable
#define 	AUDIO_ADA_CON2_adc2_channel_en	AUDIO_ADA_CON2, 1, 1		// ADC2 channel enable
#define 	AUDIO_ADA_CON2_adc2_ff_en	AUDIO_ADA_CON2, 2, 1		// ADC2 FF mode
#define 	AUDIO_ADA_CON2_adc2_test	AUDIO_ADA_CON2, 3, 1		// ADC2 test mode
#define 	AUDIO_ADA_CON2_adc2_s1_isel	AUDIO_ADA_CON2, 4, 0x3		// ADC2 
#define 	AUDIO_ADA_CON2_adc2_dither_cfg	AUDIO_ADA_CON2, 6, 0x3		// ADC2 dither config
#define 	AUDIO_ADA_CON2_adc2_pga_g	AUDIO_ADA_CON2, 8, 0xf		// ADC2 PGA gain
#define 	AUDIO_ADA_CON2_adc2_pga_en	AUDIO_ADA_CON2, 12, 1		// ADC2 PGA enable

/*================ ALNK =================*/
// audio link
#define ALNK_base		0x60B00

#define ALNK_CON0			0x00	// control 0
#define ALNK_CON1			0x04	// control 1
#define ALNK_CON2			0x08	// control 2
#define ALNK_CON3			0x0C	// control 3
#define ALNK_ADR0			0x10	// data address 0
#define ALNK_ADR1			0x14	// data address 1
#define ALNK_ADR2			0x18	// data address 2
#define ALNK_ADR3			0x1C	// data address 3
#define ALNK_LEN			0x20	// data length

/*================= NFC =================*/
// near field communications
#define NFC_base		0x60C00

#define NFC_CON0			0x00
#define NFC_CON1			0x04
#define NFC_CON2			0x08
#define NFC_BUF0			0x0C
#define NFC_BUF1			0x10
#define NFC_BUF2			0x14
#define NFC_BUF3			0x18

/*================= USB =================*/
// usb host/device
#define USB_base		0x60D00

#define USB_IO_CON0			0x00	// usb io control reg 0
#define 	USB_IO_CON0_DPOUT		USB_IO_CON0, 0, 1	// D+ output level
#define 	USB_IO_CON0_DMOUT		USB_IO_CON0, 1, 1	// D- output level
#define 	USB_IO_CON0_DPIE		USB_IO_CON0, 2, 1	// D+ direction [out|in]
#define 	USB_IO_CON0_DMIE		USB_IO_CON0, 3, 1	// D- direction [out|in]
#define 	USB_IO_CON0_DPPD		USB_IO_CON0, 4, 1	// D+ pulldown enable
#define 	USB_IO_CON0_DMPD		USB_IO_CON0, 5, 1	// D- pulldown enable
#define 	USB_IO_CON0_DPPU		USB_IO_CON0, 6, 1	// D+ pullup enable
#define 	USB_IO_CON0_DMPU		USB_IO_CON0, 7, 1	// D- pullup enable
#define 	USB_IO_CON0_PM			USB_IO_CON0, 8, 1	// "for test"
#define 	USB_IO_CON0_DPDIE		USB_IO_CON0, 9, 1	// D+ digital in enable
#define 	USB_IO_CON0_DMDIE		USB_IO_CON0, 10, 1	// D- digital in enable
#define 	USB_IO_CON0_USBIOMODE		USB_IO_CON0, 11, 1	// io mux mode
#define 	USB_IO_CON0_USBSR		USB_IO_CON0, 12, 1	// slew rate enable
#define USB_CON0			0x04	// usb control reg 0
#define 	USB_CON0_PHYON			USB_CON0, 0, 1		// usb phy enable
#define 	USB_CON0_IOMODE			USB_CON0, 1, 1		// usb phy gpio mode
#define 	USB_CON0_USBNRST		USB_CON0, 2, 1		// usb reset [assert|release]
#define 	USB_CON0_TM1			USB_CON0, 3, 1		// "short connect timeout"
#define 	USB_CON0_CID			USB_CON0, 4, 1		// usb mode [host|device]
#define 	USB_CON0_VBUS			USB_CON0, 5, 1		// ?? vbus control ??
#define 	USB_CON0_USBTEST		USB_CON0, 6, 1		// "usb test"
#define 	USB_CON0_LATSEL			USB_CON0, 8, 1		// "reserved"
#define 	USB_CON0_PDCHKDP		USB_CON0, 9, 1		// D+ ext pulldown check en
#define 	USB_CON0_SOFIE			USB_CON0, 10, 1		// SOF int enable
#define 	USB_CON0_SIEIE			USB_CON0, 11, 1		// int enable
#define 	USB_CON0_CLRSOFP		USB_CON0, 12, 1		// clear SOF int pending
#define 	USB_CON0_SOFPND			USB_CON0, 13, 1		// SOF int pending
#define 	USB_CON0_SIEPND			USB_CON0, 14, 1		// int pending (cleared by accessing USB)
#define 	USB_CON0_CHKDPO			USB_CON0, 15, 1		// D+ ext pulldown check status
#define 	USB_CON0_SEDM			USB_CON0, 16, 1		// single ended D- status
#define 	USB_CON0_SEDP			USB_CON0, 17, 1		// single ended D+ status
#define 	USB_CON0_LOWPMD			USB_CON0, 18, 1		// low power mode [en|dis]
#define USB_CON1			0x08	// usb control reg 1
#define 	USB_CON1_MCDAT			USB_CON1, 0, 0xff	// usb reg data
#define 	USB_CON1_MCADR			USB_CON1, 8, 0x3f	// usb reg addr
#define 	USB_CON1_MCRNW			USB_CON1, 14, 1		// usb reg access type [write|read]
#define 	USB_CON1_ACK			USB_CON1, 15, 1		// usb reg access done flag
#define USB_EP0_CNT			0x0C	// usb ep0 data len
#define USB_EP1_CNT			0x10	// usb ep1 data len
#define USB_EP2_CNT			0x14	// usb ep2 data len
#define USB_EP3_CNT			0x18	// usb ep3 data len
#define USB_EP0_ADR			0x1C	// usb ep0 buff addr
#define USB_EP1_TADR			0x20	// usb ep1 tx buff addr
#define USB_EP1_RADR			0x24	// usb ep1 rx buff addr
#define USB_EP2_TADR			0x28	// usb ep2 tx buff addr
#define USB_EP2_RADR			0x2C	// usb ep2 rx buff addr
#define USB_EP3_TADR			0x30	// usb ep3 tx buff addr
#define USB_EP3_RADR			0x34	// usb ep3 rx buff addr

// TODO the MC space regs (Spoiler: this is MUSB!! although it is altered OF COURSE)

/*================= CRC =================*/
// crc16 ccitt
#define CRC_base		0x60E00

#define CRC_FIFO			0x00	// fifo
#define CRC_REG				0x04	// shift reg

/*================= RAND ================*/
// random number generator
#define RAND_base		0x60F00

#define RAND_R64L			0x00	// low 32 bits
#define RAND_R64H			0x04	// high 32 bits

/*================= ADC =================*/
// analog-digital converter
#define ADC_base		0x61000

#define ADC_CON				0x00	// control
#define 	ADC_CON_adcbaud			ADC_CON, 0, 0x7		// clock divider [/1|/6|/12|/24|/48|/72|/96|/128] (<= 1MHz)
#define 	ADC_CON_adcae			ADC_CON, 3, 1		// analog enable
#define 	ADC_CON_adcen			ADC_CON, 4, 1		// enable
#define 	ADC_CON_adcie			ADC_CON, 5, 1		// interrupt enable
#define 	ADC_CON_cpnd			ADC_CON, 6, 1		// clear pending int
#define 	ADC_CON_pnd			ADC_CON, 7, 1		// interrupt pending
#define 	ADC_CON_chsel			ADC_CON, 8, 0xf		// channel select [adc0..14, 15=p33]
#define 	ADC_CON_waittime		ADC_CON, 12, 0xf	// startup delay (in 8x clocks)
#define ADC_RES				0x04	// result

/*================= WLA =================*/
// wl (wireless) analog stuff
#define WLA_base		0x61C00

#define WLA_CON0			0x00
#define WLA_CON1			0x04
#define WLA_CON2			0x08
#define WLA_CON3			0x0C
#define WLA_CON4			0x10
#define WLA_CON5			0x14
#define WLA_CON6			0x18
#define WLA_CON7			0x1C
#define WLA_CON8			0x20
#define WLA_CON9			0x24
#define WLA_CON10			0x28
#define WLA_CON11			0x2C
#define WLA_CON12			0x30
#define WLA_CON13			0x34
#define WLA_CON14			0x38
#define WLA_CON15			0x3C
#define WLA_CON16			0x40
#define WLA_CON17			0x44
#define WLA_CON18			0x48
#define WLA_CON19			0x4C
#define WLA_CON20			0x50
#define WLA_CON21			0x54
#define WLA_CON26			0x68
#define WLA_CON27			0x6C
#define WLA_CON28			0x70
#define WLA_CON29			0x74
#define WLA_CON30			0x78
#define WLA_CON31			0x7C
#define WLA_CON32			0x80
#define WLA_CON33			0x84
#define WLA_CON34			0x88
#define WLA_CON35			0x8C
#define WLA_CON36			0x90
#define WLA_CON37			0x94

/*================= FMA =================*/
// fm analog stuff
#define FMA_base		0x61D00

#define FMA_CON0			0x00
#define FMA_CON1			0x04
#define FMA_CON2			0x08
#define FMA_CON3			0x0C
#define FMA_CON4			0x10
#define FMA_CON5			0x14
#define FMA_CON6			0x18
#define FMA_CON7			0x1C
#define FMA_CON8			0x20
#define FMA_CON9			0x24

/*////////////////////////////////////////////////////////////////////////////*/
/* High Speed SFR */

/*================= DSP =================*/
#define DSP_base		0x70000

#define DSP_CON				0x00

/*================= NVIC ================*/
#define NVIC_base		0x70004

#define NVIC_ILAT1			0x00
#define NVIC_ILAT1_SET			0x04
#define NVIC_ILAT1_CLR			0x08
#define NVIC_ILAT0			0x0C
#define NVIC_ILAT0_SET			0x10
#define NVIC_ILAT0_CLR			0x14
#define NVIC_IPCON0			0x18
#define NVIC_IPCON1			0x1C
#define NVIC_IPCON2			0x20
#define NVIC_IPCON3			0x24

/*================= TICK ================*/
// tick timer
#define TICK_base		0x70040

#define TICK_CON			0x00	// control
#define TICK_CON_enable				TICK_CON, 0, 1	// enable
#define TICK_CON_cpnd				TICK_CON, 6, 1	// clear int pending
#define TICK_CON_pnd				TICK_CON, 7, 1	// int pending
#define TICK_CNT			0x04	// counter
#define TICK_PRD			0x08	// period

/*================ DEBUG ================*/
#define DEBUG_base		0x70100

#define DEBUG_DSP_BF_CON		0x00
#define DEBUG_WR_EN			0x04
#define DEBUG_MSG			0x08
#define DEBUG_MSG_CLR			0x0C
#define DEBUG_DSP_PC_LIMH		0x10
#define DEBUG_DSP_PC_LIML		0x14
#define DEBUG_DSP_EX_LIMH		0x18
#define DEBUG_DSP_EX_LIML		0x1C
#define DEBUG_PRP_EX_LIMH		0x20
#define DEBUG_PRP_EX_LIML		0x24
#define DEBUG_PRP_MMU_MSG		0x28
#define DEBUG_LSB_MMU_MSG_CH		0x2C
#define DEBUG_PRP_WR_LIMIT_MSG		0x30
#define DEBUG_LSB_WR_LIMIT_CH		0x34

/*================= SFC =================*/
// serial/spi flash controller
#define SFC_base		0x70200

#define SFC_CON				0x00	// control
#define 	SFC_CON_enable			SFC_CON, 0, 1		// enable
#define SFC_BAUD			0x04	// baudrate
#define SFC_CODE			0x08
#define SFC_BASE_ADR			0x0C	// map offset in flash

/*================= ENC =================*/
// encryptor
#define ENC_base		0x70300

#define ENC_CON				0x00	// control
#define 	ENC_CON_en_spi0dma		ENC_CON, 0, 1	// enable for SPI0 DMA transfers
//#define 	ENC_CON_en_konata		ENC_CON, 1, 1	// enable for something
#define 	ENC_CON_en_sd0data		ENC_CON, 2, 1	// enable for SD0 data transfers
#define 	ENC_CON_en_sfc			ENC_CON, 3, 1	// enable for SFC
#define 	ENC_CON_en_sfc_unenc		ENC_CON, 4, 1	// enable SFC's unencrypted area
#define 	ENC_CON_rstkey			ENC_CON, 7, 1	// reset key register
#define ENC_KEY				0x04	// key
#define ENC_ADR				0x08	// "address" (key ^ (ADR >> 2)), something like that
#define ENC_UNENC_ADRH			0x0C	// SFC's unencrypted area end
#define ENC_UNENC_ADRL			0x10	// SFC's unencrypted area begin

/*================= WL ==================*/
// wl (wireless)
#define WL_base			0x70400

#define WL_CON0				0x00
#define WL_CON1				0x04
#define WL_CON2				0x08
#define WL_CON3				0x0C
#define WL_LOFC_CON			0x10
#define WL_LOFC_RES			0x14

/*================= AES =================*/
// advanced encryption standard (aes) engine
#define AES_base		0x70500

#define AES_CON				0x00	// control
#define AES_DATIN			0x04	// input data
#define AES_KEY				0x08	// key
#define AES_ENCRES0			0x0C	// encryped result 0
#define AES_ENCRES1			0x10	// encryped result 1
#define AES_ENCRES2			0x14	// encryped result 2
#define AES_ENCRES3			0x18	// encryped result 3
#define AES_DECRES0			0x1C	// decryped result 0
#define AES_DECRES1			0x20	// decryped result 1
#define AES_DECRES2			0x24	// decryped result 2
#define AES_DECRES3			0x28	// decryped result 3

/*================= FFT =================*/
// fast fourier transform
#define FFT_base		0x70600

#define FFT_CON				0x00	// control
#define FFT_ADRI			0x04	// input address
#define FFT_ADRO			0x08	// output address
#define FFT_ADRW			0x0C	// window address

/*================= EQ ==================*/
// equalizer
#define EQ_base			0x70700

#define EQ_CON				0x00	// control
#define EQ_LEN				0x04	// length
#define EQ_ADRI				0x08	// input address
#define EQ_ADRO				0x0C	// output address
#define EQ_CADR				0x10	// .. address?

/*================= SRC =================*/
// sample rate converter
#define SRC_base		0x70800

#define SRC_CON0			0x00	// control 0
#define SRC_CON1			0x04	// control 1
#define SRC_CON2			0x08	// control 2
#define SRC_CON3			0x0C	// control 3
#define SRC_IDAT_ADR			0x10	// input data address
#define SRC_IDAT_LEN			0x14	// input data length
#define SRC_ODAT_ADR			0x18	// output data address
#define SRC_ODAT_LEN			0x1C	// output data length
#define SRC_FLTB_ADR			0x20	// fltb address

/*================= FMRX ================*/
// fm receiver
#define FMRX_base		0x70900

#define FMRX_CON			0x00	// control
#define FMRX_BASE			0x04	// base address
#define FMRX_ADC_CON			0x08	// adc control
#define FMRX_HF_CON0			0x0C	// hf control 0
#define FMRX_HF_CON1			0x10	// hf control 1
#define FMRX_HBT_RSSI			0x14	// hbti rssi
#define FMRX_ADCI_RSSI			0x18	// adc i rssi
#define FMRX_ADCQ_RSSI			0x1C	// adc q rssi
#define FMRX_HF_CRAM			0x20
#define FMRX_HF_DRAM			0x24
#define FMRX_LF_CON			0x28	// lf control
#define FMRX_LF_RES			0x2C	// lf res

#endif