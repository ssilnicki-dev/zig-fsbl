OUTPUT_ARCH(arm)
ENTRY(_start)

SECTIONS
{
  . = 0x2FFC2500;

    .text : ALIGN(4) {
        *(.text)
    }

    .rodata : ALIGN(4) {
        *(.rodata)
    }

    .data : ALIGN(4) {
        *(.data)
    }

    .bss : ALIGN(4) {
        *(COMMON)
        *(.bss)
    }


}
