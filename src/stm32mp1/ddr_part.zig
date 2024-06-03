const qsmp = @import("qsmp.zig");

export fn ddr_main() void {
    qsmp.peripheryInit();
}
