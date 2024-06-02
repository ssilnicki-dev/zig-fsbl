const qsmp = @import("qsmp.zig");

export fn main() void {
    qsmp.rccMpuAxiDdrInit();
    qsmp.peripheryInit();
}
