#ifndef _JL_P33_H
#define _JL_P33_H

#include <stdint.h>

#define P33_OP_WRITE		0x00
#define P33_OP_RMW_OR		0x20
#define P33_OP_RMW_AND		0x40
#define P33_OP_RMW_XOR		0x60
#define P33_OP_READ		0x80

uint8_t p33_spi_xfer(uint8_t val);
uint8_t p33_xfer(int sel, uint8_t op, uint16_t addr, uint8_t val);

#endif
