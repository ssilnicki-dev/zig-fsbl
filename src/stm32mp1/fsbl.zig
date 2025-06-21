const qsmp = @import("qsmp.zig");

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
    asm volatile ("b loop_stub");
}

export fn Reset_Handler() callconv(.naked) void {
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

    asm volatile ("bl main");
    asm volatile ("nop");
    asm volatile ("b .-2");
}

export fn Undefined_Handler() callconv(.naked) void {
    asm volatile ("b loop_stub");
}
export fn SWI_Handler() callconv(.naked) void {
    asm volatile ("b loop_stub");
}
export fn Prefetch_Handler() callconv(.naked) void {
    asm volatile ("b loop_stub");
}
export fn Data_Handler() callconv(.naked) void {
    asm volatile ("b loop_stub");
}
export fn IRQ_Handler() callconv(.naked) void {
    asm volatile ("b loop_stub");
}
export fn loop_stub() callconv(.naked) void {
    asm volatile ("nop");
    asm volatile ("b .-2");
}

export fn main() void {
    qsmp.rccMpuAxiDdrInit();
    qsmp.peripheryInit();
}
