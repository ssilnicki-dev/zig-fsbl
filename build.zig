const std = @import("std");
const Target = @import("std").Target;
const CrossTarget = std.zig.CrossTarget;
const Feature = @import("std").Target.Cpu.Feature;

const stm32mp1 = @import("build-stm32mp1.zig");

pub fn build(b: *std.Build) void {
    stm32mp1.build(b);
}
