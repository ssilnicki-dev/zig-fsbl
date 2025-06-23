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
    arch.CPACR.CP10.Select(.Enabled); // SIMD
    arch.CPACR.CP11.Select(.Enabled); // FPE
    arch.NSACR.AllCPAccessInNonSecureState.Select(.Disabled);
    arch.NSACR.CP10.Select(.AccessFromAnySecureState); // SIMD
    arch.NSACR.CP11.Select(.AccessFromAnySecureState); // FPE
    arch.FPEXC.EN.Select(.Enabled); // Enable SIMD & FPE Extensions
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
