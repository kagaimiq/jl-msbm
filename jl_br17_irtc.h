#ifndef _JL_IRTC_H
#define _JL_IRTC_H

#include <stdint.h>

/* operations */
#define IRTC_OP_WRALM		0x10	/* Write alarm */
#define IRTC_OP_WRREG		0x20	/* Write register */
#define IRTC_OP_WRRTC		0x40	/* Write time */
#define IRTC_OP_RDALM		0x90	/* Read alarm */
#define IRTC_OP_RDREG		0xA0	/* Read register */
#define IRTC_OP_RDRTC		0xC0	/* Read time */

/* registers */
#define IRTC_RTC_CON		0x00
#define IRTC_PORTR_IN		0x01
#define IRTC_PORTR_OUT		0x02
#define IRTC_PORTR_PU		0x03
#define IRTC_WKUP_EN		0x04
#define IRTC_WKUP_PND		0x05
#define IRTC_OSC_CON		0x06
#define IRTC_LDO_CON		0x07
#define IRTC_PORTR_DIE		0x08

uint8_t irtc_xfer(uint8_t val);

uint8_t irtc_read(char addr);
void irtc_write(char addr, uint8_t val);
void irtc_wsmask(char addr, char shift, uint8_t mask, uint8_t val);
uint8_t irtc_rsmask(char addr, char shift, uint8_t mask);

#endif
