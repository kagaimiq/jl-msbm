#include <jl_br25_regs.h>
#include <jl_irq.h>
#include <xprintf.h>

struct mod_sample {
	/* header */
	char name[22+1];
	int length;
	char finetune, volume;
	int loop_st, loop_len;

	/* data */
	uint32_t data_pos;
};

struct mod_file {
	void *file;

	/* header */
	char title[20+1];
	uint8_t norders, loop_pos;
	uint8_t pat_order[128];
	uint32_t fmt_tag;

	/* samples */
	struct mod_sample sample[31];
	int nsamples;

	/* patterns */
	uint32_t pattern_pos;
	int npatterns;
	int nchannels;
};

int mod_parse(struct mod_file *mf, void *file) {
	uint8_t *fb = file;
	int off = 0;

	memset(mf, 0, sizeof *mf);
	mf->file = file;

	/* determine file format */
	mf->fmt_tag = *(uint32_t *)&fb[1080];

	switch (mf->fmt_tag) {
	case 0x2E4B2E4D: /* 'M.K.' */
	case 0x214B214D: /* 'M!K!' */
		mf->nchannels = 4;
		mf->nsamples = 31;
		break;

	default:
		/* unknown */
		return -1;
	}

	/* grab the title */
	memcpy(mf->title, &fb[off], 20);
	off += 20;

	/* grab the samples info */
	for (int s = 0; s < mf->nsamples; s++) {
		struct mod_sample *ms = &mf->sample[s];

		uint8_t *info = &fb[off];
		off += 30;

		/* grab the info */
		memcpy(ms->name, &info[0], 22);
		ms->length   = info[22] << 9 | info[23] << 1;
		ms->finetune = info[24];
		ms->volume   = info[25];
		ms->loop_st  = info[26] << 9 | info[27] << 1;
		ms->loop_len = info[28] << 9 | info[29] << 1;
	}

	/* grab other stuff */
	mf->norders  = fb[off++];
	mf->loop_pos = fb[off++];

	/* grab the pattern order */
	memcpy(mf->pat_order, &fb[off], 128);
	off += 128;

	/* count the patterns */
	for (int i = 0; i < 128; i++) {
		int p = mf->pat_order[i] + 1;
		if (p > mf->npatterns) mf->npatterns = p;
	}

	/* skip the file tag */
	off += 4;

	/* patterns offset */
	mf->pattern_pos = off;

	/* sample data offset */
	uint32_t samples_pos = mf->pattern_pos + (mf->nchannels * 4 * 64) * mf->npatterns;

	for (int s = 0; s < mf->nsamples; s++) {
		struct mod_sample *ms = &mf->sample[s];

		if (ms->length > 0) {
			ms->data_pos = samples_pos;
			samples_pos += ms->length;
		}
	}

	/* we're done */
	return 0;
}

/*-------------------------------*/

const char *note_names[12] = {"C-", "C#", "D-", "D#", "E-", "F-", "F#", "G-", "G#", "A-", "A#", "B-"};

const short mod_periods[60] = {
	/* finetune 0 */
	1712, 1616, 1525, 1440, 1357, 1281, 1209, 1141, 1077, 1017,  961,  907,
	 856,  808,  762,  720,  678,  640,  604,  570,  538,  508,  480,  453,
	 428,  404,  381,  360,  339,  320,  302,  285,  269,  254,  240,  226,
	 214,  202,  190,  180,  170,  160,  151,  143,  135,  127,  120,  113,
	 107,  101,   95,   90,   85,   80,   76,   71,   67,   64,   60,   57,
};

/*-------------------------------*/

#define CHANNERU_CNT		4

struct channeru {
	int state;
	void *data;
	int pos, len;
	int loop_st, loop_len;
	char volume, pan;
	uint32_t freq, facc;
};

struct channeru dachans[CHANNERU_CNT];

void feed_to_dac(int16_t *sdata, int cnt) {
	for (int s = 0; s < cnt; s++) {
		int16_t mixedL = 0, mixedR = 0;

		for (int ch = 0; ch < CHANNERU_CNT; ch++) {
			struct channeru *chan = &dachans[ch];

			if (chan->state > 0) {
				int8_t *data = chan->data + chan->pos;

				/* Linear interpolation */
				int mix = (data[0] * chan->volume) * chan->facc;
				mix += (data[1] * chan->volume) * (0x10000 - chan->facc);
				mix >>= 16;

				if (!((ch ^ ch >> 1) & 1)) {
					mixedL += mix;
					mixedR += mix >> 1;
				} else {
					mixedL += mix >> 1;
					mixedR += mix;
				}

				chan->facc += chan->freq;
				while (chan->facc >= 0x10000) {
					chan->facc -= 0x10000;

					chan->pos++;

					if (chan->state == 1) {
						/* Going beyond the sample length */
						if (chan->pos >= chan->len) {
							if (chan->loop_len > 2) {
								/* Loop */
								chan->state = 2;
							} else {
								/* Stop */
								chan->state = 0;
							}
						}
					} else {
						/* Going beyond the loop length */
						if (chan->pos - chan->loop_st >= chan->loop_len)
							chan->pos = chan->loop_st;
					}
				}
			}
		}

		sdata[s*2+0] = mixedL;
		sdata[s*2+1] = mixedR;
	}
}

/*-------------------------------*/

struct mod_file modfile;
uint8_t mod_row, mod_tick, mod_order;
uint8_t mod_speed = 6, mod_bpm = 125;

char mod_currsamp[4];
short mod_currper[4];

char mod_volslide[4];
short mod_freqslide[4];

void set_bpm(uint8_t bpm) {
	if (mod_bpm != bpm) {
		mod_bpm = bpm;

		JL_TIMER0->PRD = 24000000 / 256 / (bpm * 4 / 10) - 1;
		JL_TIMER0->CNT = 0;
	}
}


void __attribute__((interrupt)) Timer0_IRQ(void) {
	if (mod_tick == 0) {
		uint8_t pattern = modfile.pat_order[mod_order];
		int row = mod_row++;

		for (int ch = 0; ch < modfile.nchannels; ch++) {
			uint8_t *note = modfile.file + modfile.pattern_pos +
				        ((pattern * 64 + row) * modfile.nchannels + ch) * 4;

			int nperiod = (note[0] & 0x0f) << 8 | note[1];
			int nsample = (note[0] & 0xf0)      | (note[2] >> 4);
			int neffect = (note[2] & 0x0f) << 8 | note[3];

			/*-----------------------------------*/

			struct channeru *dachan = &dachans[ch];

			if (nsample) {
				mod_currsamp[ch] = nsample;

				struct mod_sample *msamp = &modfile.sample[nsample-1];
				dachan->data     = modfile.file + msamp->data_pos + 2;
				dachan->len      = msamp->length - 2;
				dachan->volume   = msamp->volume;
				dachan->loop_st  = msamp->loop_st;
				dachan->loop_len = msamp->loop_len;
			}

			if (nperiod) {
				mod_currper[ch] = nperiod;

				nsample = mod_currsamp[ch];
				if (nsample > 0) {
					dachan->state = 0;
					dachan->pos   = 0;
					dachan->freq  = (7093789 / (nperiod * 2)) * 0x10000 / 41661;
					dachan->state = 1;
				}
			}

				mod_volslide[ch] = 0;
				mod_freqslide[ch] = 0;

			if (neffect) {
				switch (neffect & 0xF00) {
				case 0x900: /* Sample offset */
					neffect &= 0xff;

					dachan->pos = neffect << 8;
					break;

				case 0xA00:	/* Volume slide */
					neffect &= 0xff;
					{
						char up = neffect >> 4;
						char down = neffect & 0xf;

						/* You can't slide both ways */
						if (up && down)
							break;

						mod_volslide[ch] = up ? up : -down;
					}
					break;

				case 0xB00: /* Position jump */
					neffect &= 0xff;

					if (neffect >= modfile.norders) break;
					mod_row = 0;
					mod_order = neffect;
					break;

				case 0xC00: /* Set volume */
					neffect &= 0xff;
					if (neffect > 64) neffect = 64;

					dachan->volume = neffect;
					break;

				case 0xD00: /* Pattern break */
					neffect &= 0xff;
					neffect = (neffect >> 4) * 10 + (neffect & 0xf);

					if (neffect >= 64) break;
					mod_row = neffect;
					mod_order++;
					break;

				case 0xE00: /* Extended */
					switch (neffect & 0xFF0) {
					case 0xE60: /* Loop pattern */
						neffect &= 0xf;

						
						break;

					case 0xEA0:	/* Fine volume slide up */
						neffect &= 0xf;
						{
							int vol = dachan->volume + neffect;
							if (vol > 64) vol = 64;
							dachan->volume = vol;
						}
						break;

					case 0xEB0:	/* Fine volume slide down */
						neffect &= 0xf;
						{
							int vol = dachan->volume - neffect;
							if (vol < 0) vol = 0;
							dachan->volume = vol;
						}
						break;
					}
					break;

				case 0xF00: /* Set speed */
					neffect &= 0xff;

					if (neffect >= 32) {
						set_bpm(neffect);
					} else {
						mod_speed = neffect;
					}

					break;
				}
			}
		}

		/*---------------------------*/

		/* Row exceded? */
		if (mod_row >= 64) {
			mod_row = 0;
			mod_order++;
		}

		/* Order exceeded? */
		if (mod_order >= modfile.norders) {
			if (modfile.loop_pos >= modfile.norders) {
				mod_order = 0;
			} else {
				mod_order = modfile.loop_pos;
			}
		}
	} else {
		for (int ch = 0; ch < 4; ch++) {
			struct channeru *dachan = &dachans[ch];

			/* Slide the volume */
			if (mod_volslide[ch]) {
				int vol = dachan->volume + mod_volslide[ch];
				if (vol < 0) vol = 0;
				if (vol > 64) vol = 64;
				dachan->volume = vol;
			}
		}
	}

	if (++mod_tick >= mod_speed)
		mod_tick = 0;

	JL_TIMER0->CON |= (1<<14);
}

void __attribute__((interrupt)) Timer1_IRQ(void) {
	xputs("\e[1;1H");
	//xputs("\e[H\e[2J\e[3J"); // clear screen

	uint8_t pattern = modfile.pat_order[mod_order];

	xputs("\e[31;7m===== JIELI AC690 MOD PLAYER VER5.5.5 =====\e[0m\n");

	xprintf("Title: \e[7m%-20s\e[0m\n", modfile.title);

	xprintf("Row: %02x, Order: %02x, Pattern: %02x, BPM: %3d, Speed: %2d\n",
		mod_row, mod_order, pattern, mod_bpm, mod_speed);

	xprintf("Order: %02x/%02x [", mod_order, modfile.norders);
	for (int i = -5; i <= 5; i++) {
		int ord = mod_order + i;

		if (ord < 0 || ord >= modfile.norders) {
			xputs("    ");
		} else {
			uint8_t pat = modfile.pat_order[ord];

			if (i == 0) {
				xprintf("\e[7m[%02x]\e[0m", pat);
			} else {
				xprintf(" %02x ", pat);
			}
		}
	}
	xputs("]\n");

	xputc('\n');

	for (int orow = -6; orow <= 6; orow++) {
		int row = mod_row + orow;

		if (orow == 0) xputs("\e[7m");

		if (row < 0 || row >= 64) {
			xputs("   ");

			for (int ch = 0; ch < modfile.nchannels; ch++) {
				xputs("|          ");
			}
		} else {
			xprintf(" %02x", row);

			for (int ch = 0; ch < modfile.nchannels; ch++) {
				uint8_t *note = modfile.file + modfile.pattern_pos +
						((pattern * 64 + row) * modfile.nchannels + ch) * 4;

				int nperiod = (note[0] & 0x0f) << 8 | note[1];
				int nsample = (note[0] & 0xf0)      | (note[2] >> 4);
				int neffect = (note[2] & 0x0f) << 8 | note[3];

				/*--------------------------------*/

				xputc('|');

				if (nperiod > 0) {
					int nnote = -1;

					for (int i = 0; i < 60; i++) {
						if (mod_periods[i] == nperiod) {
							nnote = i;
							break;
						}
					}

					if (nnote < 0) {
						xputs("??? ");
					} else {
						xprintf("%s%d ", note_names[nnote % 12], nnote / 12);
					}
				} else {
					xputs("... ");
				}

				if (nsample > 0) {
					xprintf("%2X ", nsample);
				} else {
					xputs(".. ");
				}

				xprintf("%03X", neffect);
			}
		}

		xputs("|\n");

		if (orow == 0) xputs("\e[0m");
	}

	for (int ch = 0; ch < modfile.nchannels; ch++) {
		struct channeru *dachan = &dachans[ch];

		xprintf("%d: %4d %2d | %8d %08x %2d\n", ch,
			mod_currper[ch], mod_currsamp[ch],
			dachan->pos, dachan->freq, dachan->volume
		);
	}

	JL_TIMER1->CON |= (1<<14);
}



void modplayerstart(void) {
	if (mod_parse(&modfile, (void *)0x1000000))
		return;

	xprintf("MOD parsed: %s\n", modfile.title);
	xprintf("format %08x, %d channels, %d orders, %d patterns\n",
		modfile.fmt_tag, modfile.nchannels, modfile.norders, modfile.npatterns);

	/* mod timer */
	JL_TIMER0->CON = (8<<4) | (2<<2);
	JL_TIMER0->CNT = 0;
	JL_TIMER0->PRD = 24000000 / 256 / (125 * 4 / 10) - 1;
	JL_TIMER0->CON |= 1;

	/* display timer */
	#if 0
	JL_TIMER1->CON = (8<<4) | (2<<2);
	JL_TIMER1->CNT = 0;
	JL_TIMER1->PRD = 24000000 / 256 / 25 - 1;
	JL_TIMER1->CON |= 1;
	#endif

	irq_attach(irqn_TMR0, Timer0_IRQ, 1);
	irq_attach(irqn_TMR1, Timer1_IRQ, 0);

	/* enable dac! */
}
