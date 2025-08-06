const arch = @import("arch.zig");
const plat = @import("plat.zig");

const goto = arch.goto;
const call = arch.call;

export fn EntryPoint() callconv(.naked) void {
    goto(Reset_Handler);
}

export fn ExceptionVectors() align(4096) callconv(.naked) void {
    goto(Reset_Handler);
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
    arch.InitializeStacks();

    arch.PassOnlyPrimaryCpu();

    call(arch.ResetMemory);

    call(arch.InitializeMMU);
    call(plat.Initialize);
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
