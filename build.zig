const std = @import("std");
const cv1800b = @import("build-cv1800b.zig");

pub fn build(b: *std.Build) void {
    _ = b.standardOptimizeOption(.{});
    cv1800b.build(b);
}
