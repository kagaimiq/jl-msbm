#ifndef _JL_AUDIO_H
#define _JL_AUDIO_H

#include <stdint.h>

#define JL_AUDIO_DAC_NSAMPLES		256

enum jl_audio_sr {
	JL_AUDIO_SR_44100 = 0,
	JL_AUDIO_SR_48000 = 1,
	JL_AUDIO_SR_32000 = 2,
	JL_AUDIO_SR_22050 = 4,
	JL_AUDIO_SR_24000 = 5,
	JL_AUDIO_SR_16000 = 6,
	JL_AUDIO_SR_11025 = 8,
	JL_AUDIO_SR_12000 = 9,
	JL_AUDIO_SR_8000 = 10,
};

struct jl_audio_initdata {
	int vcomo_en;
	int ldo_sel;
	int hp_type;
	int vcm_rsel;
	int isel_cfg;

	void (*dac_cb)(int16_t *data, int len);
};

void jl_audio_init(struct jl_audio_initdata *id);

void jl_audio_setvolume(int left, int right);
void jl_audio_mute(int en);

void jl_audio_start(enum jl_audio_sr sr);
void jl_audio_stop(void);

#endif
