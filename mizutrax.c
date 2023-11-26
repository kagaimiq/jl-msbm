#include "mizutrax.h"
#include <string.h>
#include <stddef.h>

/*==========================================================================*/

#if defined(__JIELI_BR17__)
#define HEADER_ADDR		0x1E100
#define BUFFER_SIZE		0x2000
#define BUFFER_ADDR		(0x1A000 - BUFFER_SIZE)
#elif defined(__JIELI_BR21__)
#define HEADER_ADDR		0x1BF00
#define BUFFER_SIZE		0x2000
#define BUFFER_ADDR		(0x1BF00 - BUFFER_SIZE)
#elif defined(__JIELI_BR25__)
#define HEADER_ADDR		0x32500
#define BUFFER_SIZE		0x2000
#define BUFFER_ADDR		(0x30000 - BUFFER_SIZE)
#else
#error Undefined header address / buffer size
#endif

#define FIXEDBUFFER

/*==========================================================================*/

struct mizutrax_hdr {
	char		magic[8];	/* "MiZUTraX" */
	//uint16_t	revid;		/* 0x68AF */
	//uint16_t	hdr_size;	/* sizeof (struct mizutrax_hdr) */

	uint8_t		*buff_ptr;
	size_t		buff_size;
	size_t		nbytes;
};

static struct mizutrax_hdr * const mzthdr = (void *)HEADER_ADDR;

#ifndef FIXEDBUFFER
static uint8_t mztbuffer[BUFFER_SIZE];
#endif

/*==========================================================================*/

void mzt_init(void) {
	/* Clear out the header contents */
	memset(mzthdr, 0, sizeof *mzthdr);

	/* Fill the x data 1 */
	memcpy(mzthdr->magic, "MiZUTraX", 8);
	//mzthdr->revid = 0x68AF;
	//mzthdr->hdr_size = sizeof *mzthdr;

	/* Fill out the reset */
#ifndef FIXEDBUFFER
	mzthdr->buff_ptr = mztbuffer;
	mzthdr->buff_size = sizeof mztbuffer;
#else
	mzthdr->buff_ptr = (void *)BUFFER_ADDR;
	mzthdr->buff_size = BUFFER_SIZE;
#endif
}

void mzt_putbuff(uint8_t val) {
	uint8_t *buff = mzthdr->buff_ptr;
	size_t off = (mzthdr->nbytes ++) % mzthdr->buff_size;
	buff[off] = val;
}

int mzt_putc(int c) {
	mzt_putbuff(c);
	return 0;
}
