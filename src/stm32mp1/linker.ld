OUTPUT_ARCH(arm)
ENTRY(_start)


BOOTROM_RESERVED_LENGTH = 0x2400;
STM32_HEADER_LENGTH = 0x100;
RESERVED_SYSRAM_LENGTH = BOOTROM_RESERVED_LENGTH + STM32_HEADER_LENGTH;

MEMORY
{
  SYSRAM (xrw)      : ORIGIN = 0x2FFC0000 + RESERVED_SYSRAM_LENGTH, LENGTH = 256K - RESERVED_SYSRAM_LENGTH
}

SECTIONS
{
    .text : ALIGN(4) {
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
