OUTPUT_ARCH(arm)
ENTRY(_start)

MEMORY
{
  DDR (xrw)      : ORIGIN = 0xC0000000, LENGTH = 512M
}

SECTIONS
{
    .text : ALIGN(4) {
        *(.text)
    } >DDR

    .rodata : ALIGN(4) {
        *(.rodata)
    } >DDR

    .data : ALIGN(4) {
        *(.data)
    } >DDR

    .bss : ALIGN(4) {
        *(COMMON)
        *(.bss)
    } >DDR
}

PROVIDE(stack_bottom_addr = ORIGIN(DDR) + LENGTH(DDR));
