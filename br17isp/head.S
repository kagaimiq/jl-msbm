	.section .text.startup

	/* Where did we came from? */
	r0 = pc
	r1 = rets
	r2 = ssp
	r3 = usp

	/* Supervisor stack pointer */
	sp.h = __ssp
	sp.l = __ssp
	/* User stack pointer */
	usp.h = __sp
	usp.l = __sp

	/* Enter user mode */
	reti.l = _start
	reti.h = _start
	rti

	.global _start
_start:
	/* clear bss */
	r4.l = _sbss
	r4.h = _sbss
	r5.l = _ebss
	r5.h = _ebss
	r6 = 0
1:	cmp r4, r5
	if (nzcv >= 0) goto 2f
	[r4 ++= 4] = r6
	goto 1b
2:

	/* call main */
	call JieLi

	/* halt */
1:	idle
	goto 1b