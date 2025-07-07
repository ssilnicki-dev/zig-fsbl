OUTPUT_ARCH(arm)

MEMORY
{
  SYSRAM (xrw)      : ORIGIN = 0x0, LENGTH = 256K
}

SECTIONS
{
    .text : ALIGN(4) {
        KEEP(*(.text.ExceptionVectors))
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
