const std = @import("std");
const stm32mp1 = @import("build-stm32mp1.zig");
const imx8m = @import("build-imx8m.zig");

pub fn build(b: *std.Build) void {
    _ = b.standardOptimizeOption(.{});
    stm32mp1.build(b);
    imx8m.build(b);
}
