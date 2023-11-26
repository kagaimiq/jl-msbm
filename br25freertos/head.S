	.section .text.startup
	.global _start
_start:
	/* clear bss */
	r4 = _sbss
	r5 = _ebss
	r6 = 0
1:	if (r4 >= r5) goto 2f
	[r4 ++= 4] = r6
	goto 1b
2:

	/* enter main */
	goto JieLi
