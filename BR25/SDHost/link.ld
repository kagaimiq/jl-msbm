MEMORY {
	RAM (rwx): ORIGIN = 0x10000+0x2000, LENGTH = 136k-0x2000-0x100
}

ENTRY(_start);

InterruptVectorTable = 0x32000 - 0x100;

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
		*(.sdata)
		*(.sdata.*)
	} >RAM

	.bss : {
		_sbss = .;
		*(.sbss)
		*(.sbss.*)
		*(.scommon)
		*(.bss)
		*(.bss.*)
		*(COMMON)
		_ebss = .;
	} >RAM

	/*.stack : {
		. = . + 4096;
		. = ALIGN(8);
		__usp = .;
	} >RAM*/

	end = .;
};
