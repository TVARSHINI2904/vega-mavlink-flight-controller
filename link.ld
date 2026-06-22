ENTRY(_start)
SECTIONS {
    . = 0x80000000;
    .text : { *(.text.start) *(.text*) }
    .data : { *(.data*) }
    .bss  : { *(.bss*) }

    /* simulated non-volatile memory for mission persistence */
    .nvm (NOLOAD) : {
        _nvm_start = .;
        . = . + 0x400;              /* 1KB of NVM */
        _nvm_end = .;
    }

    . = . + 0x1000;
    _stack_top = .;
}