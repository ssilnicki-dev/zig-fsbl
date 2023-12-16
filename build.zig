const std = @import("std");
const Target = @import("std").Target;
const CrossTarget = std.zig.CrossTarget;
const Feature = @import("std").Target.Cpu.Feature;

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{ .preferred_optimize_mode = .ReleaseSmall });

    const features = Target.arm.Feature;
    var enabled_features = Feature.Set.empty;
    enabled_features.addFeature(@intFromEnum(features.v7a));
    enabled_features.addFeature(@intFromEnum(features.vldn_align));
    enabled_features.addFeature(@intFromEnum(features.neon));
    enabled_features.addFeature(@intFromEnum(features.vfp3d16));

    const default_target = CrossTarget{
        .cpu_arch = .arm,
        .os_tag = .freestanding,
        .cpu_model = .{
            .explicit = &.{
                .name = "cortex_a7",
                .llvm_name = "cortex-a7",
                .features = .{ .ints = .{ 0, 0, 0, 0, 0 } }, // empty
            },
        },
        .abi = .eabihf,
        .cpu_features_add = enabled_features,
    };

    const exe = b.addExecutable(.{
        .name = "kernel",
        .root_source_file = .{ .path = "src/main.zig" },
        .target = default_target,
        .optimize = optimize,
    });

    exe.addAssemblyFile(.{ .path = "load.S" });
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
