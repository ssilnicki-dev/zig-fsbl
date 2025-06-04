OUTPUT_ARCH(arm64)
ENTRY(_start)

MEMORY
{
  OCRAM (xrw) : ORIGIN = 0x900000, LENGTH = 2M
}
SECTIONS
{
    .text : ALIGN(4) {
        *(.text)
    } > OCRAM

    .rodata : ALIGN(4) {
        *(.rodata)
    } > OCRAM

    .data : ALIGN(4) {
        *(.data)
    } > OCRAM

    .bss : ALIGN(4) {
        *(COMMON)
        *(.bss)
    } > OCRAM
}

PROVIDE(stack_bottom_addr = ORIGIN(OCRAM) + LENGTH(OCRAM));
