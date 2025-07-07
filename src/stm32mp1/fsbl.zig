const qsmp = @import("qsmp.zig");
const arch = @import("arch.zig");

extern const stack_bottom_addr: u32;

const goto = arch.goto;

export fn ExceptionVectors() callconv(.naked) void {
    goto(Reset_Handler);
    goto(Undefined_Handler);
    goto(SWI_Handler);
    goto(Prefetch_Handler);
    goto(Data_Handler);
    goto(Reserved_Handler); //reserved vector
    goto(IRQ_Handler);
    // FIQ handler fall through
    arch.EndlessLoop(); // stub
}

export fn Reset_Handler() callconv(.naked) void {
    arch.DebugMode();

    arch.InitializeSystemControlRegister();
    arch.EnableSMP();
    arch.SetMode(.Monitor);
    arch.InitializeExceptionVectorsTable(&ExceptionVectors);
    arch.InitializeInstructionCache(.Enabled);
    arch.InitializeAlignmentFaultChecking(.Enabled);
    arch.InitializeSecureConfigurationRegister();
    arch.InitializeException(.Abort, .Enabled);
    arch.InitializeCoprocessors();
    arch.InitializePerformanceMonitorControlRegister();
    arch.InitializeCurrentProgramStatusRegister();

    arch.PassOnlyPrimaryCpu();

    arch.ResetMemory();

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
