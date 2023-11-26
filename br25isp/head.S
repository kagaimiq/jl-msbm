	.section .text.startup
	.global _start

	/* Set stack pointers */
	sp = __ssp
	usp = __sp

	/* Switch to HRC (16 MHz) */
	r4 = 0x1e000c
	[r4 + 0] |= 1
	[r4 + 0] |= 2
	nop
	nop
	nop
	[r4 + 0] &= ~256

	/* Enter user mode */
	reti = _start
	rti

_start:
	/* clear bss */
	r4 = _sbss
	r5 = _ebss
	r6 = 0
1:	if (r4 >= r5) goto 2f
	[r4 ++= 4] = r6
	goto 1b
2:

	/* call main */
	call JieLi

	/* halt */
1:	idle
	goto 1b
