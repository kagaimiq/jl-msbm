MEMORY {
	RAM (rwx): ORIGIN = 0x8000, LENGTH = 104k - 0x8000
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
		. = . + 1024;
		. = ALIGN(8);
		__sp = .;

		. = . + 1024;
		. = ALIGN(8);
		__ssp = .;
	} >RAM

	end = .;
};
