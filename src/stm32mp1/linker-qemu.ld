OUTPUT_ARCH(arm)
ENTRY(_start)


BOOTROM_RESERVED_LENGTH = 0x2400;
STM32_HEADER_LENGTH = 0x100;
RESERVED_SYSRAM_LENGTH = BOOTROM_RESERVED_LENGTH + STM32_HEADER_LENGTH;

MEMORY
{
  SYSRAM (xrw)      : ORIGIN = 0x0, LENGTH = 256K
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
        PROVIDE(rw_data_start = .);
        *(.data)
        PROVIDE(rw_data_size = . - rw_data_start);
    } > SYSRAM

    .bss : ALIGN(4) {
        PROVIDE(bss_data_start = .);
        *(COMMON)
        *(.bss)
        PROVIDE(bss_data_size = . - rw_data_start);
    } > SYSRAM

    /DISCARD/ : { *(.ARM.exidx*) }
}

PROVIDE(stack_bottom_addr = ORIGIN(SYSRAM) + LENGTH(SYSRAM));
