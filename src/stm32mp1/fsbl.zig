const qsmp = @import("qsmp.zig");
const arch = @import("arch.zig");

extern const stack_bottom_addr: u32;

export fn _start() callconv(.naked) void {
    arch.goto(Reset_Handler);
    arch.goto(Undefined_Handler);
    arch.goto(SWI_Handler);
    arch.goto(Prefetch_Handler);
    arch.goto(Data_Handler);
    arch.goto(Reserved_Handler); //reserved vector
    arch.goto(IRQ_Handler);
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

    arch.SetMode(.Monitor);

    arch.LoadAddr(.r0, &_start);
    arch.VBAR.writeFrom(.r0);
    arch.MVBAR.writeFrom(.r0);

    arch.SCTLR.I.Select(.Enabled);
    arch.SCTLR.A.Select(.Enabled);

    arch.SetValue(.r0, arch.SCR.ResetValue);
    arch.SCR.writeFrom(.r0);
    arch.SCR.SIF.Select(.Enabled);

    asm volatile ("cpsie a");
    asm volatile ("isb");
    const NSACR = arch.NSACR;
    arch.SetValue(.r0, NSACR.AllCPAccessInNonSecureState.asU32(.Disabled) |
        NSACR.CP10.asU32(.AccessFromAnySecureState) |
        NSACR.CP11.asU32(.AccessFromAnySecureState));
    NSACR.writeFrom(.r0);
    arch.ID_DFR0.CopTrc.ifEqual(.Implemented, arch.DisableNSAccessToTraceRegisters);
    const CPACR = arch.CPACR;
    arch.SetValue(.r0, CPACR.ResetValue |
        CPACR.CP10.asU32(.Enabled) |
        CPACR.CP11.asU32(.Enabled) |
        CPACR.TRCDIS.asU32(.Disabled));
    CPACR.writeFrom(.r0);
    arch.SetValue(.r0, arch.FPEXC.ResetValue | arch.FPEXC.EN.asU32(.Enabled));
    arch.FPEXC.writeFrom(.r0);

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
export fn Reserved_Handler() callconv(.naked) void {
    arch.EndlessLoop();
}
