.section .text
.global _start

_start:
    ldr sp, =0x30000000
    bl _init
    bl main
    b .

_init:
    // enable C10-C11 coprocessors - VFP&NEON
    ldr r1, =0xF00000
    mcr p15, 0, r1, c1, c0, 2

    // enable simd in NSACR
    ldr r0, =0xC00
    mcr p15, 0, r0, c1, c1, 2

    // enable VFP instructions
    ldr r0, =0x40000000
    vmsr FPEXC, r0

    bx lr

.align 8
.section .bss
    // Define a block of uninitialized data (BSS section)
    // .lcomm stack, 4096
