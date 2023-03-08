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

void JieLi(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
	reg32_wsmask(CLOCK_base+CLOCK_CLK_CON1, 10, 0x3, 0x0); // uart_clk <- pll_48m?

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

	// === For MSTAR ISP TOOL POSTAL2 POSTAL3 POSTAL4 POSTAL5 MINIPROG TL866 ASFSA === //
	// PB6 = IIC_SCL / UART1_TX -----> UART_TX / I2CS_SCL
	// PB7 = IIC_SDA / UART1_RX -----> UART_RX / I2CS_SDA

	reg32_wsmask(IOMAP_base+IOMAP_CON1_iicios, 'C'-'A'); // IIC map -> PB6/PB7 = SCL/SDA

	reg32_wsmask(PORTB_base+PORTx_DIEn(6), 1); // PB6 digital in enable
	reg32_wsmask(PORTB_base+PORTx_DIEn(7), 1); // PB7 digital in enable
	reg32_wsmask(PORTB_base+PORTx_DIRn(6), 1); // PB6 input
	reg32_wsmask(PORTB_base+PORTx_DIRn(7), 1); // PB7 input
	reg32_wsmask(PORTB_base+PORTx_PUn(6), 1); // PB6 pull up
	reg32_wsmask(PORTB_base+PORTx_PUn(7), 1); // PB7 pull up
	reg32_wsmask(PORTB_base+PORTx_PDn(6), 0); // PB6 NO pull down
	reg32_wsmask(PORTB_base+PORTx_PDn(7), 0); // PB7 NO pull down

	// clear regs, clear ints
	reg32_write(IIC_base+IIC_CON0, REG_SMASK(IIC_CON0_pclr)|REG_SMASK(IIC_CON0_end_pnd_clr));
	reg32_write(IIC_base+IIC_CON1, REG_SMASK(IIC_CON1_spnd_clr));

	reg32_write(IIC_base+IIC_BAUD, 255); // 100 kHz clock

	reg32_wsmask(IIC_base+IIC_CON0_iic_isel, 0); // filtered
	reg32_wsmask(IIC_base+IIC_CON0_en, 1); // enable

	xputs("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	for (int i = 0; i < 0x80; i += 0x10) {
		xprintf("%02x: ", i);
		for (int j = 0; j < 0x10; j++) {
			int a = i+j;
			if (a < 0x08 || a >= 0x78) {
				xputs("   ");
			} else {
				reg32_wsmask(IIC_base+IIC_CON0_m_set_rstart, 1); // send start
				reg32_wsmask(IIC_base+IIC_CON0_dat_dir, 0);      // send
				reg32_write(IIC_base+IIC_BUF, a << 1);           // data
				reg32_wsmask(IIC_base+IIC_CON0_n_cfg_done, 1);   // start!

				for (volatile int i = 50000; i && reg32_rsmask(IIC_base+IIC_CON0_pnd); i--);    // wait
				reg32_wsmask(IIC_base+IIC_CON0_pclr, 1);         // clr

				if (reg32_rsmask(IIC_base+IIC_CON0_rd_ack))
					xputs("-- ");
				else
					xprintf("%02x ", a);

				reg32_wsmask(IIC_base+IIC_CON0_m_set_end, 1);     // send end
				reg32_wsmask(IIC_base+IIC_CON0_n_cfg_done, 1);    // start!
				for (volatile int i = 50000; i && reg32_rsmask(IIC_base+IIC_CON0_end_pnd); i--);    // wait
				reg32_wsmask(IIC_base+IIC_CON0_end_pnd_clr, 1);   // clr

				wdt_clr();
			}
		}
		xputc('\n');
	}
}
