#include "jl_p33.h"
#include "jl_br25_regs.h"

uint8_t p33_spi_xfer(uint8_t val) {
	JL_P33->SPI_DAT = val;
	JL_P33->SPI_CON |= (1<<4);
	while (JL_P33->SPI_CON & (1<<1));
	return JL_P33->SPI_DAT;
}

uint8_t p33_xfer(int sel, uint8_t op, uint16_t addr, uint8_t val) {
	if (sel)
		JL_P33->SPI_CON |= (1<<8);	/* access RTC */
	else
		JL_P33->SPI_CON &= ~(1<<8);	/* access PMU */

	addr = (op << 8) | (addr & 0x1fff);

	/* select */
	JL_P33->SPI_CON |= (1<<0);
	/* opcode, address */
	p33_spi_xfer(addr >> 8);
	p33_spi_xfer(addr & 0xff);
	/* data */
	val = p33_spi_xfer(val);
	/* deselect */
	JL_P33->SPI_CON &= ~(1<<0);

	return val;
}
