#include <stdint.h>
#include <xprintf.h>
#include <jl_regs.h>
#include <jl_irq.h>
#include <wallclk.h>
#include <maskrom_stuff.h>

void uputc(int c) {
	reg32_write(UART0_base+UARTx_BUF, c);
	while (!reg32_rsmask(UART0_base+UARTx_CON0_tpnd));
	reg32_wsmask(UART0_base+UARTx_CON0_clrtpnd, 1);
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


void InitSFC(void) {
	reg32_wsmask(PORTD_base+PORTx_DIRn(0), 0); // PD0 out  -> SCK
	reg32_wsmask(PORTD_base+PORTx_DIRn(1), 0); // PD1 out  -> MOSI
	reg32_wsmask(PORTD_base+PORTx_DIRn(2), 1); // PD2 in   -> MISO
	reg32_wsmask(PORTD_base+PORTx_DIRn(3), 0); // PD3 out  -> CS
	reg32_wsmask(PORTD_base+PORTx_DIRn(4), 0); // PD4 out  -> power?

	reg32_wsmask(PORTD_base+PORTx_OUTn(3), 1); // PD3 high
	reg32_wsmask(PORTD_base+PORTx_OUTn(4), 1); // PD4 high

	reg32_write(SFC_base+SFC_CON, 0xf00000);
	reg32_write(SFC_base+SFC_CON, 0);

	reg32_write(SFC_base+SFC_BAUD, 255);

	reg32_wsmask(SFC_base+SFC_CON, 0, 0x6ff0f88, 0x0280280);

	reg32_write(SFC_base+SFC_BASE_ADR, 0x1000);

	reg32_write(SFCENC_base+SFCENC_CON, 0);
	reg32_write(SFCENC_base+SFCENC_KEY, 0x077a);
	reg32_write(SFCENC_base+SFCENC_UNENC_ADRH, 0x100007f);
	reg32_write(SFCENC_base+SFCENC_UNENC_ADRL, 0x1000040);
	reg32_write(SFCENC_base+SFCENC_LENC_ADRH,  0x10000bf);
	reg32_write(SFCENC_base+SFCENC_LENC_ADRL,  0x1000080);
	reg32_wsmask(SFCENC_base+SFCENC_CON_enable, 0);
	reg32_wsmask(SFCENC_base+SFCENC_CON_en_unenc, 0);

	reg32_wsmask(DSP_base+DSP_CON, 8, 1, 0); // disable sfc map
	memset((void *)0xfc000, 0x00, 0x1c00);   // clear icache tags
	memset((void *)0xf8000, 0x55, 0x4000);   // clear icache data
	reg32_wsmask(DSP_base+DSP_CON, 8, 1, 1); // enable sfc map

	reg32_wsmask(SPI0_base+SPIx_CON_spie, 0);	// disable SPI0 to not conflict with SFC
	reg32_wsmask(SFC_base+SFC_CON_enable, 1);	// enable SFC
}










#define P3_ANA_CON0                       0x00
#define P3_ANA_CON1                       0x01
#define P3_ANA_CON2                       0x02
#define P3_ANA_CON3                       0x03
#define P3_ANA_CON4                       0x04
#define P3_ANA_CON5                       0x05
#define P3_ANA_CON6                       0x06
#define P3_ANA_CON7                       0x07
#define P3_ANA_CON8                       0x08
#define P3_ANA_CON9                       0x09
#define P3_L5V_CON0                       0x0c
#define P3_L5V_CON1                       0x0d
#define P3_SDPG_CON                       0x0e
#define P3_FSPG_CON                       0x0f
#define P3_PLVD_CON                       0x10
#define P3_VLVD_CON                       0x11
#define P3_RST_SRC                        0x12
#define P3_LRC_CON0                       0x13
#define P3_LRC_CON1                       0x14
#define P3_RST_CON0                       0x15
#define P3_ANA_KEEP                       0x16
#define P3_VLD_KEEP                       0x17
#define P3_CHG_WKUP                       0x18
#define P3_CHG_READ                       0x19
#define P3_CHG_CON0                       0x1a
#define P3_CHG_CON1                       0x1b
#define P3_CHG_CON2                       0x1c
#define P3_CHG_CON3                       0x1d
#define P3_PMU_CON0                       0x30
#define P3_PMU_CON1                       0x31
#define P3_PMU_CON2                       0x32
#define P3_PMU_CON3                       0x33
#define P3_PMU_CON4                       0x34
#define P3_PMU_CON5                       0x35
#define P3_LP_PRP0                        0x36
#define P3_LP_PRP1                        0x37
#define P3_LP_STB0_STB1                   0x38
#define P3_LP_STB2_STB3                   0x39
#define P3_LP_STB4_STB5                   0x3a
#define P3_LP_STB6                        0x3b
#define P3_LP_RSC00                       0x40
#define P3_LP_RSC01                       0x41
#define P3_LP_RSC02                       0x42
#define P3_LP_RSC03                       0x43
#define P3_LP_PRD00                       0x44
#define P3_LP_PRD01                       0x45
#define P3_LP_PRD02                       0x46
#define P3_LP_PRD03                       0x47
#define P3_LP_RSC10                       0x48
#define P3_LP_RSC11                       0x49
#define P3_LP_RSC12                       0x4a
#define P3_LP_RSC13                       0x4b
#define P3_LP_PRD10                       0x4c
#define P3_LP_PRD11                       0x4d
#define P3_LP_PRD12                       0x4e
#define P3_LP_PRD13                       0x4f
#define P3_LP_TMR0_CLK                    0x54
#define P3_LP_TMR0_CON                    0x58
#define P3_LP_TMR1_CON                    0x59
#define P3_LP_CNTRD0                      0x60
#define P3_LP_CNTRD1                      0x61
#define P3_LP_CNT0                        0x62
#define P3_LP_CNT1                        0x63
#define P3_LP_CNT2                        0x64
#define P3_LP_CNT3                        0x65
#define P3_MSTM_RD                        0x70
#define P3_IVS_RD                         0x71
#define P3_IVS_SET                        0x72
#define P3_IVS_CLR                        0x73
#define P3_WLDO12_AUTO                    0x74
#define P3_WLDO06_AUTO                    0x75
#define P3_WDT_CON                        0x80
#define P3_WKUP_EN                        0x90
#define P3_WKUP_EDGE                      0x91
#define P3_WKUP_CPND                      0x92
#define P3_WKUP_PND                       0x93
#define P3_PINR_CON                       0x94
#define P3_WKUP_FLEN                      0x95
#define P3_PCNT_CON                       0x96
#define P3_PCNT_VLUE                      0x97
#define P3_PR_PWR                         0xa0
#define P3_LDO5V_CON                      0xa1
#define P3_LVCMP_CON                      0xa2
#define P3_L5DEM_CON                      0xa3
#define P3_L5DEM_FLT                      0xa4
#define P3_OSL_CON                        0xa5
#define P3_CLK_CON0                       0xa6
#define P3_WKUP_SRC                       0xa8
#define P3_WKUP_SUB                       0xa9
#define P3_PORT_FLT                       0xaa
#define P3_LVCMP                          0xac
#define P3_ANA_LAT                        0xaf
#define P3_EFUSE_CON0                     0xb0
#define P3_EFUSE_CON1                     0xb1
#define P3_EFUSE_RDAT                     0xb2
#define P3_PORT_SEL10                     0xc0
#define P3_PORT_SEL11                     0xc1
#define P3_PORT_SEL12                     0xc2
#define P3_PORT_SEL13                     0xc3
#define P3_PORT_SEL14                     0xc4
#define P3_PORT_SEL15                     0xc5
#define P3_PORT_SEL16                     0xc6
#define P3_PORT_SEL17                     0xc7




#define P33_OP_WRITE		0x00
#define P33_OP_RMW_OR		0x20
#define P33_OP_RMW_AND		0x40
#define P33_OP_RMW_XOR		0x60
#define P33_OP_READ		0x80


uint8_t p33_spi_xfer(uint8_t val) {
	/* put data to transmit */
	reg32_write(P33_base+P33_SPI_DAT, val);

	/* transfer */
	reg32_wsmask(P33_base+P33_SPI_CON, 4, 1, 1);

	/* wait for transfer to complete */
	while (reg32_rsmask(P33_base+P33_SPI_CON, 1, 1));

	/* get received data */
	return reg32_read(P33_base+P33_SPI_DAT);
}

void p33_pmu_write(uint16_t addr, uint8_t val) {
	/* target: PMU */
	reg32_wsmask(P33_base+P33_SPI_CON, 8, 1, 0);

	/* select */
	reg32_wsmask(P33_base+P33_SPI_CON, 0, 1, 1);

	/* send command */
	p33_spi_xfer((addr >> 8) | P33_OP_WRITE);
	p33_spi_xfer(addr);
	/* send data */
	p33_spi_xfer(val);

	/* deselect */
	reg32_wsmask(P33_base+P33_SPI_CON, 0, 1, 0);
}

uint8_t p33_pmu_read(uint16_t addr) {
	uint8_t val;

	/* target: PMU */
	reg32_wsmask(P33_base+P33_SPI_CON, 8, 1, 0);

	/* select */
	reg32_wsmask(P33_base+P33_SPI_CON, 0, 1, 1);

	/* send command */
	p33_spi_xfer((addr >> 8) | P33_OP_READ);
	p33_spi_xfer(addr);
	/* recv data */
	val = p33_spi_xfer(0);

	/* deselect */
	reg32_wsmask(P33_base+P33_SPI_CON, 0, 1, 0);

	return val;
}

void p33_rtc_write(uint16_t addr, uint8_t val) {
	/* target: RTC */
	reg32_wsmask(P33_base+P33_SPI_CON, 8, 1, 1);

	/* select */
	reg32_wsmask(P33_base+P33_SPI_CON, 0, 1, 1);

	/* send command */
	p33_spi_xfer((addr >> 8) | P33_OP_WRITE);
	p33_spi_xfer(addr);
	/* send data */
	p33_spi_xfer(val);

	/* deselect */
	reg32_wsmask(P33_base+P33_SPI_CON, 0, 1, 0);
}

uint8_t p33_rtc_read(uint16_t addr) {
	uint8_t val;

	/* target: RTC */
	reg32_wsmask(P33_base+P33_SPI_CON, 8, 1, 1);

	/* select */
	reg32_wsmask(P33_base+P33_SPI_CON, 0, 1, 1);

	/* send command */
	p33_spi_xfer((addr >> 8) | P33_OP_READ);
	p33_spi_xfer(addr);
	/* recv data */
	val = p33_spi_xfer(0);

	/* deselect */
	reg32_wsmask(P33_base+P33_SPI_CON, 0, 1, 0);

	return val;
}



void JieLi(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1, 10, 0x3, 0x0); // uart_clk <- pll_48m

	// init UART0 on PB5
	reg32_write(UART0_base+UARTx_CON0, 1); // 8n1, en
	reg32_write(UART0_base+UARTx_BAUD, (48000000 / 4 / 921600) - 1);
	reg32_wsmask(IOMAP_base+IOMAP_CON0_ut0ios, 0x2); // UART0 to PB5
	reg32_wsmask(IOMAP_base+IOMAP_CON3_ut0mxs, 0x0); // UART0 muxsel -> iomux
	reg32_wsmask(IOMAP_base+IOMAP_CON3_ut0ioen, 1); // UART0 I/O enable
	reg32_wsmask(PORTB_base+PORTx_DIRn(5), 0); // PB5 out

	xdev_out(uputc);
	xputs("\e[H\e[2J\e[3J"); // clear screen
	xputs("\e[1;37;41m==== JieLi AC6965A! "__DATE__" "__TIME__" ====\e[0m\n");
	xprintf("r0: <%08x>  r1: <%08x>  r2: <%08x>  r3: <%08x>\n", r0,r1,r2,r3);

	/*==================================================================*/

	InitSFC();

	xprintf("PMU_CON = %08x\n", reg32_read(P33_base+P33_PMU_CON));
	xprintf("RTC_CON = %08x\n", reg32_read(P33_base+P33_RTC_CON));
	xprintf("SPI_CON = %08x\n", reg32_read(P33_base+P33_SPI_CON));
	xprintf("SPI_DAT = %08x\n", reg32_read(P33_base+P33_SPI_DAT));

	for (int i = 0; i < 8; i++) {
		int sel1 = i >> 2, sel2 = i & 3;

		p33_pmu_write(P3_EFUSE_CON0, (sel1<<6));
		p33_pmu_write(P3_EFUSE_CON1, (sel2<<2)|(1<<7)|(1<<1));
		xprintf("%d.%d: %02x\n", sel1, sel2, p33_pmu_read(P3_EFUSE_RDAT));
		p33_pmu_write(P3_EFUSE_CON1, 0);
	}
}
