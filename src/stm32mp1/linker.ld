OUTPUT_ARCH(arm)

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
        KEEP(*(.text.ExceptionVectors))
        *(.text)
    } > SYSRAM

    .rodata : ALIGN(4) {
        *(.rodata)
        *(.rodata.*)
    } > SYSRAM

    .data : ALIGN(4) {
        PROVIDE(rw_data_start = .);
        *(.data)
        PROVIDE(rw_data_size = . - rw_data_start);
    } > SYSRAM

    .stack (NOLOAD) : ALIGN(8) {
        KEEP(*(.stack));
    } > SYSRAM

    .bss : ALIGN(4) {
        PROVIDE(bss_data_start = .);
        *(COMMON)
        *(.bss)
        PROVIDE(bss_data_size = . - bss_data_start);
    } > SYSRAM

    /DISCARD/ : { *(.ARM.exidx*) }
}

PROVIDE(stack_bottom_addr = ORIGIN(SYSRAM) + LENGTH(SYSRAM));
