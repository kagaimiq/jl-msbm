MEMORY {
	RAM (rwx): ORIGIN = 0x00000 + 0x2000, LENGTH = 104k - 0x2000
	/* RAM (rwx): ORIGIN = 0x1a000, LENGTH = 16k */
	/* RAM (rwx): ORIGIN = 0x40000 + 0x2000, LENGTH = 24k - 0x2000 */
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

	end = .;
};
