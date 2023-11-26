	.section .text.startup

	.global _start
_start:
	/* save caller's registers */
	[--sp] = {rets, r14-r4}

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
	call loader_main

	/* return back */
	{pc, r14-r4} = [sp++]

