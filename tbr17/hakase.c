#include <jl_br17_regs.h>
#include <jl_irq.h>
#include <jl_irtc.h>
#include <xprintf.h>

void uputc(int c) {
	while (!(JL_UART2->CON & (1<<15)));
	JL_UART2->BUF = c;
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

/*=========================================================================*/

const char *dbgmsgbits[] = {
	"reserved",
	"reserved",
	"prp_ex_limit_err",
	"sdr_wr_limit_err",

	"reserved",
	"reserved",
	"reserved",
	"reserved",

	"reserved",
	"reserved",
	"reserved",
	"reserved",

	"reserved",
	"reserved",
	"reserved",
	"reserved",

	"dsp_pc_limit_err",
	"dsp_ex_limit_err",
	"dsp_illegal",
	"dsp_misaligned",

	"reserved",
	"reserved",
	"reserved",
	"reserved",

	"dsp_if_bus_inv",
	"dsp_of_bus_inv",
	"dsp_ex_bus_inv",
	"prp_bus_inv",

	"reserved",
	"reserved",
	"reserved",
	"reserved",
};

void debugmsg_print(void) {
	uint32_t debug_msg = JL_DEBUG->MSG;

	xputs("Debug msg:\n");

	if (debug_msg) {
		for (int i = 0; i < 32; i++) {
			if (debug_msg & (1<<i)) {
				xprintf(" - %d: %s\n", i, dbgmsgbits[i]);
			}
		}
	} else {
		xputs(" - None\n");
	}

	xprintf("msg: %08x\n", JL_DEBUG->MSG);
	xprintf("prp mmu msg: %08x\n", JL_DEBUG->PRP_MMU_MSG);
	xprintf("lsb mmu msg: %08x\n", JL_DEBUG->LSB_MMU_MSG_CH);
	xprintf("prp wr limit msg: %08x\n", JL_DEBUG->PRP_WR_LIMIT_MSG);
	xprintf("lsb wr limit msg: %08x\n", JL_DEBUG->LSB_WR_LIMIT_CH);
}




struct JLExcFrame {
	uint32_t gpr[16];
	uint32_t maccl, macch;
	uint32_t reti, rets, psr;
	uint32_t ie1, ie0, icfg;
	uint32_t ssp, usp;
};

void TheExceptionHandler(struct JLExcFrame *ef) {
	uint32_t tmp;

	xputs("\n\n<><><><> Oh noooo.... Jieli crashed!!! <><><><>\n");

	xprintf("r0:  <%08x>  r1:  <%08x>  r2:  <%08x>  r3:  <%08x>\n", ef->gpr[ 0], ef->gpr[ 1], ef->gpr[ 2], ef->gpr[ 3]);
	xprintf("r4:  <%08x>  r5:  <%08x>  r6:  <%08x>  r7:  <%08x>\n", ef->gpr[ 4], ef->gpr[ 5], ef->gpr[ 6], ef->gpr[ 7]);
	xprintf("r8:  <%08x>  r9:  <%08x>  r10: <%08x>  r11: <%08x>\n", ef->gpr[ 8], ef->gpr[ 9], ef->gpr[10], ef->gpr[11]);
	xprintf("r12: <%08x>  r13: <%08x>  r14: <%08x>  sp:  <%08x>\n", ef->gpr[12], ef->gpr[13], ef->gpr[14], ef->gpr[15]);
	xprintf("maccl: [%08x]  macch: [%08x]\n", ef->maccl, ef->macch);

	xprintf("[reti]: %08x\n", ef->reti);
	xprintf("[rets]: %08x\n", ef->rets);
	xprintf("[psr]: %08x\n", ef->psr);

	/*xprintf("msg: %08x\n", JL_DEBUG->MSG);
	xprintf("prp mmu msg: %08x\n", JL_DEBUG->PRP_MMU_MSG);
	xprintf("lsb mmu msg: %08x\n", JL_DEBUG->LSB_MMU_MSG_CH);
	xprintf("prp wr limit msg: %08x\n", JL_DEBUG->PRP_WR_LIMIT_MSG);
	xprintf("lsb wr limit msg: %08x\n", JL_DEBUG->LSB_WR_LIMIT_CH);*/

	debugmsg_print();

	xputs("System halted.\n");

	JL_POWER->CON |= (1<<4);	/* soft reset */
}

void __attribute__((naked)) ExceptionHandler(void) {
	asm (
		//"[--sp] = {usp, pc, icfg, ie0, ssp, ie1, sfr9, sfr8, psr, rets, macch, maccl, sfr3, sfr2, sfr1, reti}\n"
		//"[--sp] = {usp, icfg, ie0, ssp, ie1, psr, rets, macch, maccl, reti}\n"
		"[--sp] = {usp, ssp}\n"
		"[--sp] = {icfg, ie0, ie1}\n"
		"[--sp] = {psr, rets, reti}\n"
		"[--sp] = {macch, maccl}\n"
		"[--sp] = {usp}\n"
		"[--sp] = {r14-r0}\n"
		"r0 = sp\n"
		"call TheExceptionHandler\n"
		"1: idle\n"
		"goto 1b\n"
	);
}


volatile unsigned int tickcntr;

void __attribute__((interrupt)) TickTimerHandler(void) {
	static char switz;

	switz = !switz;
	xputs(switz ? "tik..." : "tok...\n");

	/* toggle PR2 */
	irtc_write(IRTC_PORTR_OUT, irtc_read(IRTC_PORTR_OUT) ^ (1<<2));

	/* increment tick counter */
	tickcntr++;

	/* clear wdt */
	JL_SYSTEM->WDT_CON |= (1<<6);

	/* clear interrupt */
	JL_TICK->CON |= (1<<6);
	irq_latch_clear(1);
}

void __attribute__((interrupt)) SomeIntHandler(void) {
	uint32_t tmp;

	xputs("\n\n------- MY Xpopa Interruptz --------\n\n");

	asm ("%0 = ie0" : "=r"(tmp));
	xprintf("ie0: %08x\n", tmp);

	asm ("%0 = ie1" : "=r"(tmp));
	xprintf("ie1: %08x\n", tmp);

	asm ("%0 = icfg" : "=r"(tmp));
	xprintf("icfg: %08x\n", tmp);

	xprintf("ILAT0: %08x\n", JL_NVIC->ILAT0);
	xprintf("ILAT1: %08x\n", JL_NVIC->ILAT1);

	irq_latch_clear(63);
}

/*=========================================================================*/

/* Common USB registers */
#define MC_FAddr		0x00	/* 00 */
#define MC_Power		0x01	/* 01 */
#define 	MC_Power_EnableSuspendM		(1<<0)
#define 	MC_Power_SuspendMode		(1<<1)
#define 	MC_Power_Resume			(1<<2)
#define 	MC_Power_Reset			(1<<3)
#define 	MC_Power_HSMode			(1<<4)
#define 	MC_Power_HSEnab			(1<<5)
#define 	MC_Power_SoftConn		(1<<6)
#define 	MC_Power_ISOUpdate		(1<<7)
#define MC_IntrTxL		0x02	/* 02 */
#define MC_IntrTxH		0x03	/* 03 */
#define MC_IntrRxL		0x04	/* 04 */
#define MC_IntrRxH		0x05	/* 05 */
#define MC_IntrTxEL		0x07	/* 06 */
#define MC_IntrTxEH		0x08	/* 07 */
#define MC_IntrRxEL		0x09	/* 08 */
#define MC_IntrRxEH		0x0A	/* 09 */
#define MC_IntrUSB		0x06	/* 0A */
#define 	MC_IntrUSB_Suspend		(1<<0)
#define 	MC_IntrUSB_Resume		(1<<1)
#define 	MC_IntrUSB_Reset		(1<<2)	/* peri */
#define 	MC_IntrUSB_Babble		(1<<2)	/* host */
#define 	MC_IntrUSB_SOF			(1<<3)
#define 	MC_IntrUSB_Conn			(1<<4)
#define 	MC_IntrUSB_Discon		(1<<5)
#define 	MC_IntrUSB_SessReq		(1<<6)
#define 	MC_IntrUSB_VBusError		(1<<7)
#define MC_IntrUSBE		0x0B	/* 0B */
#define MC_FrameL		0x0C	/* 0C */
#define MC_FrameH		0x0D	/* 0D */
#define MC_Index		0x0E	/* 0E */
					/* 0F */
/* Indexed endpoint regs */
#define MC_TxMaxP		0x10	/* 10 */
					/* 11 */
#define MC_CSR0			0x11	/* 12 */
#define 	MC_CSR0_RxPktRdy		(1<<0)
#define 	MC_CSR0_TxPktRdy		(1<<1)
#define 	MC_CSR0_SentStall		(1<<2)	/* peri */
#define 	MC_CSR0_RxStall			(1<<2)	/* host */
#define 	MC_CSR0_DataEnd			(1<<3)	/* peri */
#define 	MC_CSR0_SetupPkt		(1<<3)	/* host */
#define 	MC_CSR0_SetupEnd		(1<<4)	/* peri */
#define 	MC_CSR0_Error			(1<<4)	/* host */
#define 	MC_CSR0_SendStall		(1<<5)	/* peri */
#define 	MC_CSR0_ReqPkt			(1<<5)	/* host */
#define 	MC_CSR0_ServicedRxPktRdy	(1<<6)	/* peri */
#define 	MC_CSR0_StatusPkt		(1<<6)	/* host */
#define 	MC_CSR0_ServicedSetupEnd	(1<<7)	/* peri */
#define 	MC_CSR0_NAKTimeout		(1<<7)	/* host */
#define MC_TxCSRL		0x11	/* 12 */
#define 	MC_TxCSRL_TxPktRdy		(1<<0)
#define 	MC_TxCSRL_FIFONotEmpty		(1<<1)
#define 	MC_TxCSRL_UnderRun		(1<<2)	/* peri */
#define 	MC_TxCSRL_Error			(1<<2)	/* host */
#define 	MC_TxCSRL_FlushFIFO		(1<<3)
#define 	MC_TxCSRL_SendStall		(1<<4)	/* peri */
#define 	MC_TxCSRL_SetupPkt		(1<<4)	/* host */
#define 	MC_TxCSRL_SentStall		(1<<5)	/* peri */
#define 	MC_TxCSRL_RxStall		(1<<5)	/* host */
#define 	MC_TxCSRL_ClrDataTog		(1<<6)
#define 	MC_TxCSRL_IncompTx		(1<<7)
#define 	MC_TxCSRL_NAKTimeout		(1<<7)	/* host */
#define MC_TxCSRH		0x12	/* 13 */
#define 	MC_TxCSRH_DataToggle		(1<<0)	/* host */
#define 	MC_TxCSRH_DataToggleWrEnable	(1<<1)	/* host */
#define 	MC_TxCSRH_DMAReqMode		(1<<2)
#define 	MC_TxCSRH_FrcDataTog		(1<<3)
#define 	MC_TxCSRH_DMAReqEnab		(1<<4)
#define 	MC_TxCSRH_Mode			(1<<5)
#define 	MC_TxCSRH_ISO			(1<<6)	/* peri */
#define 	MC_TxCSRH_AutoSet		(1<<7)
#define MC_RxMaxP		0x13	/* 14 */
					/* 15 */
#define MC_RxCSRL		0x14	/* 16 */
#define 	MC_RxCSRL_RxPktRdy		(1<<0)
#define 	MC_RxCSRL_FIFOFull		(1<<1)
#define 	MC_RxCSRL_OverRun		(1<<2)	/* peri */
#define 	MC_RxCSRL_Error			(1<<2)	/* host */
#define 	MC_RxCSRL_DataError		(1<<3)
#define 	MC_RxCSRL_NAKTimeout		(1<<3)	/* host */
#define 	MC_RxCSRL_FlushFIFO		(1<<4)
#define 	MC_RxCSRL_SendStall		(1<<5)	/* peri */
#define 	MC_RxCSRL_ReqPkt		(1<<5)	/* host */
#define 	MC_RxCSRL_SentStall		(1<<6)	/* peri */
#define 	MC_RxCSRL_RxStall		(1<<6)	/* host */
#define 	MC_RxCSRL_ClrDataTog		(1<<7)
#define MC_RxCSRH		0x15	/* 17 */
#define 	MC_RxCSRL_IncompRx		(1<<0)
#define 	MC_RxCSRL_DataToggle		(1<<1)	/* host */
#define 	MC_RxCSRL_DataToggleWrEnable	(1<<2)	/* host */
#define 	MC_RxCSRL_DMAReqMode		(1<<3)
#define 	MC_RxCSRL_DisNyet		(1<<4)	/* peri */
#define 	MC_RxCSRL_PIDError		(1<<4)
#define 	MC_RxCSRL_DMAReqEnab		(1<<5)
#define 	MC_RxCSRL_ISO			(1<<6)	/* peri */
#define 	MC_RxCSRL_AutoReq		(1<<6)	/* host */
#define 	MC_RxCSRL_AutoClear		(1<<7)
#define MC_Count0		0x16	/* 18 */
#define MC_RxCountL		0x16	/* 18 */
#define MC_RxCountH		0x17	/* 19 */
#define MC_TxType		0x18	/* 1A */
#define MC_TxInterval		0x19	/* 1B */
#define MC_RxType		0x1A	/* 1C */
#define MC_RxInterval		0x1B	/* 1D */
					/* 1E */
					/* 1F */
/* Additional control&config regs */
#define MC_DevCtl		0x0F	/* 60 */


uint8_t usb_mc_read(char addr) {
	JL_USB->CON1 = (addr << 8) | (1<<14);
	while (!(JL_USB->CON1 & (1<<15)));
	return JL_USB->CON1;
}

void usb_mc_write(char addr, uint8_t val) {
	JL_USB->CON1 = val | (addr << 8) | (0<<14);
	while (!(JL_USB->CON1 & (1<<15)));
}


/* Serial Interface Engine interrupt */
void __attribute__((interrupt)) USB_SIE_IRQHandler(void) {
	irq_latch_clear(7);

	xputs("----- USB SIE -----\n");
	xprintf("%08x\n", JL_USB->CON0);

	uint8_t intr_usb = usb_mc_read(MC_IntrUSB);
	uint16_t intr_tx = (usb_mc_read(MC_IntrTxH) << 8) | usb_mc_read(MC_IntrTxL);
	uint16_t intr_rx = (usb_mc_read(MC_IntrRxH) << 8) | usb_mc_read(MC_IntrRxL);

	xprintf("%02x | %04x | %04x\n", intr_usb, intr_tx, intr_rx);
}

/*=========================================================================*/

void JieLi(uint32_t args[4]) {
	reg_wsmask(JL_CLOCK->CLK_CON1, 10, 0x3, 0x1);	/* uart_clk = pll_48m */

	JL_UART2->CON = 1;
	JL_UART2->BAUD = (48000000 / 4 / 2000000) - 1;

	reg_wsmask(JL_IOMAP->CON1, 14, 0x3, 'A'-'A');
	reg_wsmask(JL_IOMAP->CON3, 8, 0x7, 0x0);
	JL_IOMAP->CON3 |= (1<<11);

	JL_PORTA->DIR &= ~(1<<3);	/* PA3 out */

	xdev_out(uputc);
	xputs("\n\nhello br17\n");

	/*---------------------------------------------------*/

	xprintf("Chip ID: %08x\n", JL_SYSTEM->CHIP_ID);

	asm ("ie0 = %0\nie1 = %0\n" :: "r"(0));

	irq_set_handler(irqn_EXCEPTION, ExceptionHandler);

	/* Don't get outside the 16k icache sram!!! */
	/*JL_DEBUG->DSP_PC_LIMH = 0x1dfff;
	JL_DEBUG->DSP_PC_LIML = 0x1a000;*/

	irtc_wsmask(IRTC_PORTR_DIE, 6, 1, 0);	/* no long press reset */

	irtc_wsmask(IRTC_PORTR_OUT, 4+2, 1, 0);
	irtc_wsmask(IRTC_PORTR_OUT, 0+2, 1, 1);

	JL_TICK->CON = 0;
	JL_TICK->CNT = 0;
	JL_TICK->PRD = 48000000 / 2;
	//JL_TICK->CON = 1;

	irq_attach(irqn_TICK_TMR, TickTimerHandler, 3);
	irq_attach(63, SomeIntHandler, 3);

#if 0
	asm ("swi 63\n");

	uint16_t val = 0;

	for (int i = 0; i < 16; i++) {
		JL_SYSTEM->EFUSE_CON = i | (1<<4);
		JL_SYSTEM->EFUSE_CON |= (1<<9);
		for (volatile int i = 10000; i; i--);
		if (!(JL_SYSTEM->EFUSE_CON & (1<<15))) val |= (1 << i);
		JL_SYSTEM->EFUSE_CON &= ~(1<<9);
	}

	xprintf("%04x\n", val);

	//irq_attach(irqn_FUSB_CTL, USB_SIE_IRQHandler, 0);

	xprintf("dsp_bf_con: %08x\n", JL_DEBUG->DSP_BF_CON);
	xprintf("wr_en: %08x\n", JL_DEBUG->WR_EN);

	/*xprintf("msg: %08x\n", JL_DEBUG->MSG);
	xprintf("prp mmu msg: %08x\n", JL_DEBUG->PRP_MMU_MSG);
	xprintf("lsb mmu msg: %08x\n", JL_DEBUG->LSB_MMU_MSG_CH);
	xprintf("prp wr limit msg: %08x\n", JL_DEBUG->PRP_WR_LIMIT_MSG);
	xprintf("lsb wr limit msg: %08x\n", JL_DEBUG->LSB_WR_LIMIT_CH);*/

	debugmsg_print();

	void    *mmu_area = (void *)0x80000;
	uint8_t *mmu_tag0 = (void *)0x49800;

	//asm ("r0.h = 14\nr0.l = 2\nr1 = 0xdead\n[r0] = r1\n");

	JL_DEBUG->DSP_PC_LIMH = 0x1dfff;
	JL_DEBUG->DSP_PC_LIML = 0x1a000;

	mmu_tag0[0] = 0x2000 >> 9;
	memcpy(mmu_area, (void *)0x50000, 16);

	hexdump(mmu_tag0, 0x100);
	hexdump(mmu_area, 0x100);

	debugmsg_print();
#endif

	JL_TIMER2->CON = (1<<14)|(2<<2);	/* OSC clk */
	JL_TIMER2->PRD = 250 * 2;	/* 2x RC clk */
	JL_TIMER2->CNT = 0;

	JL_TIMER3->CON = (1<<14)|(3<<2);	/* RC clk */
	JL_TIMER3->PRD = -1;
	JL_TIMER3->CNT = 0;

	JL_TIMER2->CON |= (1<<0);
	JL_TIMER3->CON |= (1<<0);

	while (!(JL_TIMER2->CON & (1<<15)));

	JL_TIMER2->CON = (1<<14);
	JL_TIMER3->CON = (1<<14);

	xprintf("%d/%d :: osc frequency: ~%d kHz\n",
		JL_TIMER2->CNT, JL_TIMER3->CNT,
		(JL_TIMER2->CNT + 250 * 2) * 250 / JL_TIMER3->CNT
	);

	xputs("\n___________ bye! ___________\n");

	/*for (;;) {
		asm ("idle");
	}*/
}
