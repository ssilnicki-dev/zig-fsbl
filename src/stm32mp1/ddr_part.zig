const qsmp = @import("qsmp.zig");

extern const stack_bottom_addr: u32;
export fn _start() callconv(.naked) void {
    asm volatile ("mov sp, %[sp_value]"
        :
        : [sp_value] "r" (&stack_bottom_addr),
    );
    asm volatile (
    // enable C10-C11 coprocessors - VFP&NEON
        \\ ldr r1, =0xF00000
        \\ mcr p15, 0, r1, c1, c0, 2

        // enable simd in NSACR
        \\ ldr r0, =0xC00
        \\ mcr p15, 0, r0, c1, c1, 2

        // enable VFP instructions
        \\ ldr r0, =0x40000000
        \\ vmsr FPEXC, r0
    );

    asm volatile ("bl ddr_main");
    asm volatile ("nop");
    asm volatile ("b .-2");
}

export fn ddr_main() void {
    qsmp.peripheryInit();
}
