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
    const SCTLR = arch.SCTLR;
    arch.SetValue(.r0, SCTLR.RES1 |
        SCTLR.NTWE.asU32(.NotTrapped) |
        SCTLR.NTWI.asU32(.NotTrapped) |
        SCTLR.CP15BEN.asU32(.Enabled) |
        SCTLR.EE.asU32(.LittleEndian) |
        SCTLR.TE.asU32(.ARM) |
        SCTLR.V.asU32(.LowVectors) |
        SCTLR.DSSBS.asU32(.DisableMitigation));
    SCTLR.writeFrom(.r0);
    asm volatile ("isb");

    arch.SetMode(.Monitor);

    arch.LoadAddr(.r0, &_start);
    arch.VBAR.writeFrom(.r0);
    arch.MVBAR.writeFrom(.r0);
    asm volatile ("isb");

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
