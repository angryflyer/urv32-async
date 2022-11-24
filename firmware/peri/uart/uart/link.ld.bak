/* See LICENSE.SiFive for license details. */
OUTPUT_ARCH( "riscv" )
ENTRY( _start )
SECTIONS
{
    . = 0x20000;
    .text :
    {
        *(.text)
    }
    . += 256;
    _end = .;
}
