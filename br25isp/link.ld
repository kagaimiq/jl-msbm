MEMORY {
	RAM (rwx): ORIGIN = 0x10000, LENGTH = 128k
}

ENTRY(_start);

SECTIONS {
	.text : {
		*(.text.startup)
		*(.text)
		*(.text.*)
	} >RAM

	.rodata : {
		*(.rodata)
		*(.rodata.*)
	} >RAM

	.data : {
		*(.data)
		*(.data.*)
	} >RAM

	.bss : {
		_sbss = .;
		*(.bss)
		*(.bss.*)
		*(COMMON)
		_ebss = .;
	} >RAM

	.stack : {
		/* User mode stack */
		. = . + 1024;
		. = ALIGN(8);
		__sp = .;

		/* Supervisor mode stack */
		. = . + 1024;
		. = ALIGN(8);
		__ssp = .;
	} >RAM

	end = .;
};
