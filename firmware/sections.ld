OUTPUT_ARCH( "riscv" )

_STACK_SIZE = DEFINED(_STACK_SIZE) ? _STACK_SIZE : 0x2000;
_HEAP_SIZE = DEFINED(_HEAP_SIZE) ? _HEAP_SIZE : 0x2000;

ENTRY(_start)

MEMORY
{
    ROM (rx)      : ORIGIN = 0x80000000, LENGTH = 0x20000 /* entire ROM, 10000->64KB, 20000->128KB */
    RAM (xrw)     : ORIGIN = 0x80080000, LENGTH = 0x20000 /* entire RAM, 10000->64KB, 20000->128KB */
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


    /* This is the initialized data section
    The program executes knowing that the data is in the RAM
    but the loader puts the initial values in the ROM (inidata).
    It is one task of the startup to copy the initial values from ROM to RAM. */
    .data :AT ( _sidata )
    {
        . = ALIGN(4);
        _sdata = .;        /* create a global symbol at data start; used by startup code in order to initialise the .data section in RAM */
        _ram_start = .;    /* create a global symbol at ram start for garbage collector */
        . = ALIGN(4);
        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */
        *(.sdata)           /* .sdata sections */
        *(.sdata*)          /* .sdata* sections */
        . = ALIGN(4);
        _edata = .;        /* define a global symbol at data end; used by startup code in order to initialise the .data section in RAM */
    } > RAM

    /* Uninitialized data section */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;         /* define a global symbol at bss start; used by startup code */
        *(.bss)
        *(.bss*)
        *(.sbss)
        *(.sbss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;         /* define a global symbol at bss end; used by startup code */
    } > RAM

    /* this is to define the start of the heap, and make sure we have a minimum size */
    /* .heap :
    {
       . = ALIGN(4);
       _heap = .;
       _heap_start = .;
       . += _HEAP_SIZE;
       _heap_end = .;
    } > RAM */

    .stack ORIGIN(RAM) + LENGTH(RAM) - _STACK_SIZE :
    {
       . = ALIGN(4);
       _stack_end = .;
       . += _STACK_SIZE;
       . = ALIGN(4);
       _stack = .;
       __stack = _stack;
    } > RAM
}

