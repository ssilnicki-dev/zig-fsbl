const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{ .preferred_optimize_mode = .ReleaseFast });

    const exe = b.addExecutable(.{
        .name = "bare",
        .root_source_file = .{ .path = "src/main.zig" },
        .target = .{ .abi = .eabihf, .cpu_arch = .arm, .os_tag = .freestanding },
        .optimize = optimize,
    });
    exe.addAssemblyFile(.{.path = "load.S"});
    exe.setLinkerScript(.{ .path = "linker.ld" });
    b.installArtifact(exe);

    // library unit tests
    const lib_unit_tests = b.addTest(.{
        .root_source_file = .{ .path = "src/root.zig" },
        .target = target,
        .optimize = optimize,
    });
    const run_lib_unit_tests = b.addRunArtifact(lib_unit_tests);
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_lib_unit_tests.step);
}
