const qsmp = @import("qsmp.zig");
const arch = @import("arch.zig");

extern const stack_bottom_addr: u32;

export fn _start() callconv(.naked) void {
    asm volatile ("b Reset_Handler");
    asm volatile ("b Undefined_Handler");
    asm volatile ("b SWI_Handler");
    asm volatile ("b Prefetch_Handler");
    asm volatile ("b Data_Handler");
    asm volatile ("nop"); //reserved vector
    asm volatile ("b IRQ_Handler");
    // FIQ handler fall through
    arch.EndlessLoop(); // stub
}

export fn Reset_Handler() callconv(.naked) void {
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

    arch.EndlessLoop();
}

export fn Undefined_Handler() callconv(.naked) void {
    arch.EndlessLoop();
}
export fn SWI_Handler() callconv(.naked) void {
    arch.EndlessLoop();
}
export fn Prefetch_Handler() callconv(.naked) void {
    arch.EndlessLoop();
}
export fn Data_Handler() callconv(.naked) void {
    arch.EndlessLoop();
}
export fn IRQ_Handler() callconv(.naked) void {
    arch.EndlessLoop();
}
