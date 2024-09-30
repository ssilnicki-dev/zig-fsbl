const std = @import("std");
const stm32mp1 = @import("build-stm32mp1.zig");

pub fn build(b: *std.Build) void {
    _ = b.standardOptimizeOption(.{});
    stm32mp1.build(b);
}
