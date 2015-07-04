/*
 * generic gcc linker script for LPC812
 * Kamal Mostafa <kamal@whence.com>.
 *
 * License: Public Domain
 */

MEMORY
{
/* From https://github.com/sebseb7/lpc8xx/blob/master/core/lpc812.ld */
    /* NXP LPC810 */
    FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x4000 /* 16k */
    RAM (rwx) :  ORIGIN = 0x10000000, LENGTH = 0x1000 /* 4k */
}

SECTIONS
{
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        KEEP(*(.after_vectors))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .data : AT (_etext)
    {
        _data = .;
        *(vtable)
        *(.data*)
        _edata = .;
    } > RAM

    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM
}
