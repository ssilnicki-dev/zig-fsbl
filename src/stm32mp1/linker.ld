OUTPUT_ARCH(arm)
ENTRY(_start)

MEMORY
{
  SYSRAM (xrw)      : ORIGIN = 0x2FFC0000, LENGTH = 256K
}

SECTIONS
{
    .text : ALIGN(4) {
        . = 0x2FFC2500;
        KEEP(*(.text._start))
        *(.text)
    } > SYSRAM

    .rodata : ALIGN(4) {
        *(.rodata)
    } > SYSRAM

    .data : ALIGN(4) {
        *(.data)
    } > SYSRAM

    .bss : ALIGN(4) {
        *(COMMON)
        *(.bss)
    } > SYSRAM

    /DISCARD/ : { *(.ARM.exidx*) }
}

PROVIDE(stack_bottom_addr = ORIGIN(SYSRAM) + LENGTH(SYSRAM));
