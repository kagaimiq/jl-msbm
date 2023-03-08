#include <stdint.h>
#include <stdio.h>
#include <xprintf.h>
#include <jl_regs.h>

void uputc(int c) {
	reg32_write(UART2_base+UART2_BUF, c);
	while (!reg32_rsmask(UART2_base+UARTx_CON0_tpnd));
	reg32_wsmask(UART2_base+UARTx_CON0_clrtpnd, 1);
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

	reg32_wsmask(PORTD_base+PORTx_OUTn(3), 1); // PD3 high

	reg32_write(SFC_base+SFC_CON, 0xf00000);
	reg32_write(SFC_base+SFC_CON, 0);

	reg32_write(SFC_base+SFC_BAUD, 0xff);

	/*
	 * mask -> 6ff0fbd : 0110'1111'1111'0000'1111'1011'1101
	 *         0000034   0000'0000'0000'0000'0000'0011'0100
	 *
	 * write ->6ff0f89   0110'1111'1111'0000'1111'1000'1001
	 *
	 *          200088   0000'0010'0000'0000'0000'1000'1000
	 *
	 * b0 = enable
	 * b3 = combine MOSI and MISO
	 * b7 = 
	 * b8~b11 = 
	 * b16~b19 = dummy bits count
	 * b20~b23 = 
	 * b25 = if set, then the JEDEC ID is readed out (cmd 9F)
	 *
	 * 00000011 Aaaaaaaa:aaaaaaaa:aaaaaaaa          <Data>
	 * 00001011 Aaaaaaaa:aaaaaaaa:aaaaaaaa XXXXXXXX <Data>
	 * 00111011 Aaaaaaaa:aaaaaaaa:aaaaaaaa XXXXXXXX <Dual data>
	 * 01101011 Aaaaaaaa:aaaaaaaa:aaaaaaaa XXXXXXXX <Quad data>
	 * 10111011 AA.aa.aa.aa:aa.aa.aa.aa:... XX.XX.XX.XX <Dual data>
	 * 11101011 AAAA.aaaa:aaaa.aaaa:.... XXXX.XXXX.XXXX.XXXX.XXXX.XXXX <Quad data>
	 * xxxxyzzz
	 * 
	 *
	 * SINGLE IO
	 * 0x00200088 <== [r2 != 4] && [r1 != 8]  >>>> cmd 03 - read
	 * 0x00280188 <== [r2 != 4] && [r1 == 8]  >>>> cmd 0B - read fast
	 *
	 * DUAL IO
	 * 0x00200080 <== [r2 == 2] && [r1 != 8]  >>>> cmd 03 - read
	 * 0x00280280 <== [r2 == 2] && [r1 == 4]  >>>> cmd 3B - read dual out
	 * 0x00240480 <== [r2 == 2] && [r1 == 8]  >>>> cmd BB - read dual io
	 * 0x00240680 <== [r2 == 2] && [r1 == 12] >>>> 
	 *
	 * QUAD IO
	 * 0x00200080 <== [r2 == 4] && [r1 != 8]  >>>> cmd 03 - read
	 * 0x00280380 <== [r2 == 4] && [r1 == 4]  >>>> cmd 6B - read quad out
	 * 0x00260580 <== [r2 == 4] && [r1 == 8]  >>>> cmd EB - read quad io
	 * 0x00260780 <== [r2 == 4] && [r1 == 12] >>>> 
	 */

	reg32_wsmask(SFC_base+SFC_CON, 0, 0x6ff0f88, 0x0240480);

	reg32_write(SFC_base+SFC_BASE_ADR, 0x10000);

	reg32_write(ENC_base+ENC_CON, 0);
	reg32_write(ENC_base+ENC_KEY, 0xffff);
	reg32_write(ENC_base+ENC_ADR, 0);
	reg32_write(ENC_base+ENC_UNENC_ADRH, 0x1000140);
	reg32_write(ENC_base+ENC_UNENC_ADRL, 0x10000c0);
	reg32_wsmask(ENC_base+ENC_CON_en_sfc, 0);
	reg32_wsmask(ENC_base+ENC_CON_en_sfc_unenc, 1);

	reg32_wsmask(DSP_base+DSP_CON, 8, 1, 0); // disable sfc map
	memset((void *)0x48000, 0x00, 0x1800);   // clear sfc cache tags
	reg32_wsmask(DSP_base+DSP_CON, 8, 1, 1); // enable sfc map

	reg32_wsmask(SPI0_base+SPIx_CON_spie, 0);	// disable SPI0 to not conflict with SFC
	reg32_wsmask(SFC_base+SFC_CON_enable, 1);	// enable SFC

	//hexdump((void *)0x1000000, 0x200);
}

/*====================================================================*/

int16_t dac_buffer[2][1024][2]; // 2 buffers, 1024 samples, 2 channels

__attribute__((interrupt)) void Audio_IntHandler(void) {
	if (reg32_rsmask(AUDIO_base+AUDIO_DAC_CON_pnd)) {
		reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_cpnd, 1);

		int buffi = reg32_rsmask(AUDIO_base+AUDIO_DAC_CON_buff);
		xprintf("<DAC> %d\n", buffi);

		for (int i = 0; i < 1024; i++) {
			dac_buffer[buffi][i][0] = dac_buffer[buffi][i][1] = reg32_read(RAND_base+RAND_R64L);
		}
	}

	if (reg32_rsmask(AUDIO_base+AUDIO_LADC_CON_pnd)) {
		reg32_wsmask(AUDIO_base+AUDIO_LADC_CON_cpnd, 1);

		int buffi = reg32_rsmask(AUDIO_base+AUDIO_LADC_CON_buff);
		xprintf("<LADC> %d\n", buffi);
	}

	reg32_write(NVIC_base+NVIC_ILAT0_CLR, 1<<10);
}

/*====================================================================*/

void dac_analog_init(int vcomo_en, int ldo_sel, int hp_type) {
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON4_dac_isel5u, 1);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON4_dac_isel_third, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON4_dac_isel_half, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_vcm_det_en, 1);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_mute, 1);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON1_vcm_rsel, 0); // <= dac_vcm_rsel

	// delay(100);
	for (volatile int i = 100; i; i--);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_pns_en, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_pns10k_en, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_vcm_out_pd, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON1_lg_sel, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON1_rg_sel, 0);

	switch (ldo_sel) {
	case 1: /* LDO1 */
		reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_ldo1_en, 1);
		break;
	case 2: /* LDO2 */
		reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_ldo2_en, 1);
		break;
	}

	// delay(5000);
	for (volatile int i = 500; i; i--);

	switch (hp_type) {
	case 0: /* stereo */
		reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_l_en, 1);
		reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_r_en, 1);
		break;

	case 1: /* mono (DACL) */
		reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_l_en, 1);
		reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_r_en, 0);
		break;

	case 2: /* mono (DACR) */
		reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_r_en, 1);
		reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_l_en, 0);
		break;
	}

	if (vcomo_en)
		reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_vcm_out_en, 1);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_dac_en, 1);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_vcm_en, 1);

	// delay(1000);
	for (volatile int i = 100; i; i--);

	// <=== dac_isel_cfg
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON4_dac_isel5u, 1);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON4_dac_isel_third, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON4_dac_isel_half, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_amux_en, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_trim_en, 0);
}

int dac_cmp_res(void) {
	int res0 = 0, res1 = 0;

	for (int i = 0; i <= 20; i++) {
		int out = reg32_rsmask(AUDIO_base+AUDIO_DAA_CON3_trim_out);

		res0 += !out;
		res1 += out;

		// delay(100);
		for (volatile int i = 100; i; i--);
	}

	return res1 >= res0;
}

int16_t dac_trim1(int trim_sw, int trim_sel) {
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_trim_sel, trim_sel);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_trim_sw, trim_sw);

	if (trim_sel) {
		reg32_write(AUDIO_base+AUDIO_DAC_TRMR, 127);
	} else {
		reg32_write(AUDIO_base+AUDIO_DAC_TRML, 127);
	}

	for (int i = 0; i < 4; i++) {
		while (!reg32_rsmask(AUDIO_base+AUDIO_DAC_CON_pnd));
		reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_cpnd, 1);
	}

	int res_before = dac_cmp_res();

	for (int i = 127; i >= -128; i--) {
		if (trim_sel) {
			reg32_write(AUDIO_base+AUDIO_DAC_TRMR, i);
		} else {
			reg32_write(AUDIO_base+AUDIO_DAC_TRML, i);
		}

		// delay(1000);
		for (volatile int i = 100; i; i--);

		if (dac_cmp_res() != res_before) return i;
	}

	return -128;
}

void dac_init(int vcomo_en, int ldo_sel, int hp_type) {
	reg32_wsmask(FMA_base+FMA_CON1, 12, 1, 1);

	// %2 = hp_type

	reg32_write(AUDIO_base+AUDIO_DAC_ADR, (uint32_t)dac_buffer);
	reg32_write(AUDIO_base+AUDIO_DAC_LEN, 32);

	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dccs, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dacsr, 0); // 44100 Hz
	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dccs, 14);

	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_cpnd, 1);
	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dacen, 1);

	dac_analog_init(vcomo_en, ldo_sel, hp_type);

	// dac_power_on_delay(dac_vcm_rsel);
	//........
	for (volatile int i = 100; i; i--);

	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1_dac_cksel, 0); // clk_dac = pll_24m

	reg32_write(AUDIO_base+AUDIO_DAC_TRML, 0); // reset trim (left)
	reg32_write(AUDIO_base+AUDIO_DAC_TRMR, 0); // reset trim (right)

	reg32_write(AUDIO_base+AUDIO_DAC_ADR, (uint32_t)dac_buffer);
	reg32_write(AUDIO_base+AUDIO_DAC_LEN, 32); // 32 samples

	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dacie, 0); // disable DAC ints

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_amux_en, 0); // disable AMUX

	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dacen, 1); // start DAC

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_dac_en, 1); // enable DAC

	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dacsr, 1); // 48000 Hz

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_trim_en, 1); // trim starts

	int trimL = (dac_trim1(0, 0) + dac_trim1(1, 0)) / 2;
	int trimR = (dac_trim1(0, 1) + dac_trim1(1, 1)) / 2;

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_trim_en, 0); // trim ends

	reg32_write(AUDIO_base+AUDIO_DAC_TRML, trimL);
	reg32_write(AUDIO_base+AUDIO_DAC_TRMR, trimR);

	//reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dacie, 1); // enable DAC ints
}

void dac_on(int vcomo_en, int ldo_sel, int hp_type) {
	// dac_sw = 1;
	// <- dac_vcm_rsel
	// dac_vcm_rsel = 1;

	dac_analog_init(vcomo_en, ldo_sel, hp_type);

	// -> dac_vcm_rsel

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON1_vcm_rsel, 0); // <= dac_vcm_rsel
}

void dac_off(void) {
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_mute, 1);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_pns10k_en, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_pns_en, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_vcm_out_pd, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_ldo1_en, 1);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_ldo2_en, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON1_lg_sel, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON1_rg_sel, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_vcm_en, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON4_dac_isel5u, 1);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON4_dac_isel_third, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON4_dac_isel_half, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_dac_en, 1);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_vcm_out_en, 1);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_l_en, 1);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_r_en, 1);

	// dac_power_on_delay(2);
	for (volatile int i = 100; i; i--);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_ldo1_en, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_l_en, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_r_en, 0);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_vcm_out_en, 0);
}

/*====================================================================*/

void JieLi(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1_uart_cksel, 0x1); // uart_clk <- pll_48m

	// init UART2 on PA3
	reg32_write(UART2_base+UARTx_CON0, 1); // 8n1, en
	reg32_write(UART2_base+UARTx_BAUD, (48000000 / 4 / 921600) - 1);
	reg32_wsmask(IOMAP_base+IOMAP_CON1_ut2ios, 0x0); // UART2 to TX/RX => PA3/PA4
	reg32_wsmask(IOMAP_base+IOMAP_CON3_ut2mxs, 0x0); // UART2 muxsel -> iomux
	reg32_wsmask(IOMAP_base+IOMAP_CON3_ut2ioen, 1); // UART2 I/O enable
	//reg32_wsmask(PORTA_base+PORTx_DIRn(3), 0); // PA3 out

	xdev_out(uputc);
	xputs("\e[H\e[2J\e[3J"); // clear screen
	xputs("\e[1;37;41m==== JieLi AC6908C! "__DATE__" "__TIME__" ====\e[0m\n");
	xprintf("r0: <%08x>  r1: <%08x>  r2: <%08x>  r3: <%08x>\n", r0,r1,r2,r3);

	/*==================================================================*/

	InitSFC();

	/* reset registers */
	reg32_write(AUDIO_base+AUDIO_DAA_CON0, 0);
	reg32_write(AUDIO_base+AUDIO_DAA_CON1, 0);
	reg32_write(AUDIO_base+AUDIO_DAA_CON2, 0);
	reg32_write(AUDIO_base+AUDIO_DAA_CON3, 0);
	reg32_write(AUDIO_base+AUDIO_DAA_CON4, 0);
	reg32_write(AUDIO_base+AUDIO_DAA_CON5, 0);

	reg32_write(AUDIO_base+AUDIO_ADA_CON0, 0);
	reg32_write(AUDIO_base+AUDIO_ADA_CON1, 0);
	reg32_write(AUDIO_base+AUDIO_ADA_CON2, 0);

	reg32_write(AUDIO_base+AUDIO_DAC_CON, REG_SMASK(AUDIO_DAC_CON_cpnd));
	reg32_write(AUDIO_base+AUDIO_LADC_CON, REG_SMASK(AUDIO_LADC_CON_cpnd));

	#if 0
	/* init */
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_ldo1_en, 1); // enable ldo1
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_ldo2_en, 0); // enable ldo2

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_vcm_en, 0);
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_vcm_out_en, 0); // VCOMO enable

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_hp_l_en, 1); // enable left channel

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON1_lg_sel, 0x00); // max gain!! for left ch [=w=]

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_dac_en, 1); // DAC enable
	#endif


	extern void *InterruptVectorTable[64];

	InterruptVectorTable[10] = Audio_IntHandler;

	{
		uint32_t tmp;

		asm volatile ("%0 = ie0" : "=r"(tmp));
		tmp |= (1<<10);
		asm volatile ("ie0 = %0" :: "r"(tmp));
	}

	dac_init(0, 2, 1);

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON1_lg_sel, 16); // volume: 15

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON0_mute, 0); // unmute

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_lin1l_en, 1); // AMUX1L enable
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_lin1r_en, 1); // AMUX1R enable

	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_amux_g, 1); // AMUX gain
	reg32_wsmask(AUDIO_base+AUDIO_DAA_CON2_amux_en, 1); // AMUX enable

	/*========== TEST! ==========*/

	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dacsr, 0); // 10= 8000 Hz
	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dccs, 15);
	reg32_write(AUDIO_base+AUDIO_DAC_ADR, (uint32_t)dac_buffer);
	reg32_write(AUDIO_base+AUDIO_DAC_LEN, sizeof(dac_buffer) / 2/2/2); // 2 buffers, 2 channels, 2 bytes
	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dacie, 1); // enable DAC ints

	reg32_wsmask(AUDIO_base+AUDIO_DAC_CON_dacen, 0); // start DAC
}
