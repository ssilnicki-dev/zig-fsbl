.section .text
.global _start

_start:
    ldr sp, =0xE0000000
    bl ddr_main
    b .

.align 8
.section .bss
    // Define a block of uninitialized data (BSS section)
    // .lcomm stack, 4096
