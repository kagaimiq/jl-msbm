	.section .text.startup
	.global _start
_start:
	/* save return address and caller regs */
	[--sp] = {rets, r15-r4}

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

	/* restore caller regs and return */
	{pc, r15-r4} = [sp++]
