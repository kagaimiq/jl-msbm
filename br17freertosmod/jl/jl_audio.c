#include "jl_audio.h"
#include "jl_irq.h"
#include "jl_br17_regs.h"


extern void usleep(int us);


struct jl_audio_initdata *initdata;

/* double-buffered, X samples, 2 channels */
int16_t dac_buffer[2][JL_AUDIO_DAC_NSAMPLES][2];

static void dac_analog_init(void) {
	JL_AUDIO->DAA_CON4 |= (1<<0);		/* DAC_ISEL5U = 1*/
	JL_AUDIO->DAA_CON4 &= ~(1<<1);		/* DAC_ISEL_THIRD = 0 */
	JL_AUDIO->DAA_CON4 &= ~(1<<2);		/* DAC_ISEL_HALF = 0 */

	JL_AUDIO->DAA_CON2 |= (1<<8);		/* VCM_DET_EN = 1 */
	JL_AUDIO->DAA_CON0 |= (1<<7);		/* MUTE = 1 */

	if (initdata->vcm_rsel)
		JL_AUDIO->DAA_CON1 |= (1<<13);		/* VCM_RSEL = 1 */
	else
		JL_AUDIO->DAA_CON1 &= ~(1<<13);		/* VCM_RSEL = 0 */

	usleep(100);

	JL_AUDIO->DAA_CON0 &= ~(1<<6);		/* PNS_EN = 0 */
	JL_AUDIO->DAA_CON0 &= ~(1<<8);		/* PNS10K_EN = 0 */

	JL_AUDIO->DAA_CON2 &= ~(1<<11);		/* VCM_OUT_PD = 0 */

	JL_AUDIO->DAA_CON1 &= ~(0x1f<<0);	/* LG_SEL = 0 */
	JL_AUDIO->DAA_CON1 &= ~(0x1f<<8);	/* RG_SEL = 0 */

	switch (initdata->ldo_sel) {
	case 1: /* LDO1 */
		JL_AUDIO->DAA_CON0 |= (1<<2);	/* LDO1_EN = 1 */
		break;

	case 2: /* LDO2 */
		JL_AUDIO->DAA_CON0 |= (1<<3);	/* LDO2_EN = 1 */
		break;
	}

	usleep(5000);

	switch (initdata->hp_type) {
	case 0: /* stereo */
		JL_AUDIO->DAA_CON0 |= (1<<4);	/* HP_L_EN = 1 */
		JL_AUDIO->DAA_CON0 |= (1<<5);	/* HP_R_EN = 1 */
		break;

	case 1: /* mono (DACL) */
		JL_AUDIO->DAA_CON0 |= (1<<4);	/* HP_L_EN = 1 */
		JL_AUDIO->DAA_CON0 &= ~(1<<5);	/* HP_R_EN = 0 */
		break;

	case 2: /* mono (DACR) */
		JL_AUDIO->DAA_CON0 &= ~(1<<4);	/* HP_L_EN = 0 */
		JL_AUDIO->DAA_CON0 |= (1<<5);	/* HP_R_EN = 1 */
		break;
	}

	if (initdata->vcomo_en)
		JL_AUDIO->DAA_CON2 |= (1<<10);	/* VCM_OUT_EN = 1 */

	JL_AUDIO->DAA_CON0 |= (1<<0);		/* DAC_EN = 1 */
	JL_AUDIO->DAA_CON2 |= (1<<9);		/* VCM_EN = 1 */

	usleep(1000);

	/* <== dac_isel_cfg */
	JL_AUDIO->DAA_CON4 |= (1<<0);		/* DAC_ISEL5U = 1 */
	JL_AUDIO->DAA_CON4 &= ~(1<<1);		/* DAC_ISEL_THIRD = 0 */
	JL_AUDIO->DAA_CON4 &= ~(1<<2);		/* DAC_ISEL_HALF = 0 */

	JL_AUDIO->DAA_CON2 &= ~(1<<7);		/* AMUX_EN = 0 */
	JL_AUDIO->DAA_CON0 &= ~(1<<13);		/* TRIM_EN = 0 */
}

static int dac_cmp_res(void) {
	int res0 = 0, res1 = 0;

	for (int i = 0; i <= 20; i++) {
		int out = (JL_AUDIO->DAA_CON3 >> 15) & 1;	/* TRIM_OUT */

		res0 += !out;
		res1 += out;

		usleep(100);
	}

	return res1 >= res0;
}

static int16_t dac_trim1(int trim_sw, int trim_sel) {
	reg_wsmask(JL_AUDIO->DAA_CON0, 15, 1, trim_sw);		/* TRIM_SW */
	reg_wsmask(JL_AUDIO->DAA_CON0, 14, 1, trim_sel);	/* TRIM_SEL */

	if (trim_sel) {
		JL_AUDIO->DAC_TRMR = 127;
	} else {
		JL_AUDIO->DAC_TRML = 127;
	}

	for (int i = 0; i < 4; i++) {
		while (!(JL_AUDIO->DAC_CON & (1<<7)));
		JL_AUDIO->DAC_CON |= (1<<6);
	}

	int res_before = dac_cmp_res();

	for (int i = 127; i > -128; i--) {
		if (trim_sel) {
			JL_AUDIO->DAC_TRMR = i;
		} else {
			JL_AUDIO->DAC_TRML = i;
		}

		usleep(1000);

		int res = dac_cmp_res();
		if (res != res_before) return i;
	}

	return -128;
}

static void dac_init(void) {
	JL_FMA->CON1 |= (1<<12);	/* FM_RADIO */

	#if 0
	JL_AUDIO->DAC_ADR = (uint32_t)dac_buffer;
	JL_AUDIO->DAC_LEN = 32;

	JL_AUDIO->DAC_CON &= ~(0xf<<12);	/* DCCS = 0 */
	JL_AUDIO->DAC_CON &= ~(0xf<<0);		/* DACSR = 0 */
	JL_AUDIO->DAC_CON |= (14<<12);		/* DCCS = 14 */

	JL_AUDIO->DAC_CON |= (1<<6);	/* CPND = 1 */
	JL_AUDIO->DAC_CON |= (1<<4);	/* DACEN = 1 */
	#endif

	dac_analog_init();

	usleep(100);

	reg_wsmask(JL_CLOCK->CLK_CON1, 2, 3, 0);	/* dac_clk = pll_24m */

	#if 0	/* Trim */
	JL_AUDIO->DAC_TRML = 0;
	JL_AUDIO->DAC_TRMR = 0;

	JL_AUDIO->DAC_ADR = (uint32_t)dac_buffer;	/* test buffer */
	JL_AUDIO->DAC_LEN = 32;				/* 32 samples */

	JL_AUDIO->DAC_CON &= ~(1<<5);	/* DACIE = 0    -disable dac ints */
	JL_AUDIO->DAA_CON2 &= ~(1<<7);	/* AMUX_EN = 0  -disable amux     */
	JL_AUDIO->DAC_CON |= (1<<4);	/* DACEN = 1    -start dac        */
	JL_AUDIO->DAA_CON0 |= (1<<0);	/* DAC_EN = 1   -enable dac       */
	JL_AUDIO->DAC_CON |= (1<<0);	/* DACSR = 1    -48000 Hz         */

	JL_AUDIO->DAA_CON0 |= (1<<13);	/* TRIM_EN = 1 */

	int trim_l = (dac_trim1(0, 0) + dac_trim1(1, 0)) / 2;
	int trim_r = (dac_trim1(0, 1) + dac_trim1(1, 1)) / 2;

	JL_AUDIO->DAA_CON0 &= ~(1<<13);	/* TRIM_EN = 0 */

	JL_AUDIO->DAC_TRML = trim_l;
	JL_AUDIO->DAC_TRMR = trim_r;

	//JL_AUDIO->DAC_CON |= (1<<5);	/* DACIE = 1    - enable dac ints */
	#endif
}

/*--------------------------------------------------------*/

__attribute__((interrupt)) void Audio_IRQ(void) {
	/* DAC interrupt */
	if (JL_AUDIO->DAC_CON & (1<<7)) {
		JL_AUDIO->DAC_CON |= (1<<6);

		if (initdata->dac_cb) {
			int buffi = (JL_AUDIO->DAC_CON >> 8) & 1;

			initdata->dac_cb((int16_t *)&dac_buffer[!buffi],
							JL_AUDIO_DAC_NSAMPLES);
		}
	}

	/* ADC interrupt */
	if (JL_AUDIO->LADC_CON & (1<<7)) {
		JL_AUDIO->LADC_CON |= (1<<6);
	}

	irq_latch_clear(irqn_AUDIO);
}

/*--------------------------------------------------------*/

void jl_audio_init(struct jl_audio_initdata *id)
{
	initdata = id;

	dac_init();

	irq_attach(irqn_AUDIO, Audio_IRQ, 0);

	JL_AUDIO->DAC_ADR = (uint32_t)dac_buffer;
	JL_AUDIO->DAC_LEN = JL_AUDIO_DAC_NSAMPLES;

	JL_AUDIO->DAC_CON |= (1<<5);		/* DACIE = 1 */

	jl_audio_mute(0);
	jl_audio_setvolume(16, 16);
}

void jl_audio_setvolume(int left, int right)
{
	reg_wsmask(JL_AUDIO->DAA_CON1, 0, 0x1f, left);	/* LG_SEL */
	reg_wsmask(JL_AUDIO->DAA_CON1, 8, 0x1f, right);	/* RG_SEL */
}

void jl_audio_mute(int en)
{
	reg_wsmask(JL_AUDIO->DAA_CON0, 7, 1, !!en);	/* MUTE */
}

void jl_audio_start(enum jl_audio_sr sr)
{
	reg_wsmask(JL_AUDIO->DAC_CON,  0, 0xf, sr);	/* DACSR */
	reg_wsmask(JL_AUDIO->DAC_CON, 12, 0xf, 14);	/* DCCS */
	JL_AUDIO->DAC_CON |= (1<<4);		/* DACEN = 1 */
}

void jl_audio_stop(void)
{
	JL_AUDIO->DAC_CON &= ~(1<<4);		/* DACEN = 0 */
}
