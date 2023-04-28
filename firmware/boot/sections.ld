OUTPUT_ARCH( "riscv" )

ENTRY(_start)

MEMORY
{
    ROM (xrx)     : ORIGIN = 0x30000000, LENGTH = 0x400 /* entire ROM, 1KB */
    RAM (xrw)     : ORIGIN = 0x80000000, LENGTH = 0x20000 /* 64KB */
}

SECTIONS {
    /* The program code and other data goes into ROM */
    .text :
    {
        . = ALIGN(4);
        *(.text)           /* .text sections (code) */
        *(.text*)          /* .text* sections (code) */
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        *(.srodata)        /* .rodata sections (constants, strings, etc.) */
        *(.srodata*)       /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
        _etext = .;        /* define a global symbol at end of code */
        _sidata = _etext;  /* This is used by the startup in order to initialize the .data secion */
    } > ROM
}

