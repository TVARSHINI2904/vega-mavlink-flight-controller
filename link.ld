ENTRY(_start)
SECTIONS {
    . = 0x80000000;
    .text : { *(.text.start) *(.text*) }
    .data : { *(.data*) }
    .bss  : { *(.bss*) }
    . = . + 0x1000;
    _stack_top = .;
}

