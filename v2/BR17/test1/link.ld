MEMORY {
	RAM (rwx): ORIGIN = 0x00000+0x2000, LENGTH = 104k+16k-0x2000
}

ENTRY(_start);

InterruptVectorTable = 0x1e000;

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
