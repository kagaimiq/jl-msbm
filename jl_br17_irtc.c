#include "jl_irtc.h"
#include "jl_br17_regs.h"

/*
 * IRTC_CON
 *
 * b11 = MISO
 * b10 = MOSI
 * b9  = SCK
 * b8  = CS
 */

uint8_t irtc_xfer(uint8_t val) {
	for (int i = 0; i < 8; i++) {
		/* Set a bit to send out */
		if (val & 0x80) JL_IRTC->CON |= (1<<10);
		else            JL_IRTC->CON &= ~(1<<10);

		/* Clock this bit out */
		JL_IRTC->CON |= (1<<9);
		JL_IRTC->CON &= ~(1<<9);

		val <<= 1;

		/* Fetch a bit */
		if (JL_IRTC->CON & (1<<11))
			val |= 1;
	}

	return val;
}

uint8_t irtc_read(char addr) {
	uint8_t val;

	JL_IRTC->CON |= (1<<8);
	irtc_xfer(IRTC_OP_RDREG|(addr&0x1f));
	val = irtc_xfer(0);
	JL_IRTC->CON &= ~(1<<8);

	return val;
}

void irtc_write(char addr, uint8_t val) {
	JL_IRTC->CON |= (1<<8);
	irtc_xfer(IRTC_OP_WRREG|(addr&0x1f));
	irtc_xfer(val);
	JL_IRTC->CON &= ~(1<<8);
}

void irtc_wsmask(char addr, char shift, uint8_t mask, uint8_t val) {
	irtc_write(addr, (irtc_read(addr) & ~(mask << shift)) | ((val & mask) << shift));
}

uint8_t irtc_rsmask(char addr, char shift, uint8_t mask) {
	return (irtc_read(addr) >> shift) & mask;
}
