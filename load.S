.section .text
.global _start

_start:
    // Initialize stack pointer
    ldr sp, =0xE0000000

    // Call main function
    bl main

    // If main returns, enter an infinite loop
    b .

.section .bss
    // Define a block of uninitialized data (BSS section)
    // .lcomm stack, 4096
