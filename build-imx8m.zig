const std = @import("std");
const Target = @import("std").Target;
const Feature = @import("std").Target.Cpu.Feature;

pub fn build(b: *std.Build, optimize: std.builtin.OptimizeMode) void {
    const armv8a_features = Target.arm.Feature;
    var enabled_features = Feature.Set.empty;

    enabled_features.addFeature(@intFromEnum(armv8a_features.v8a));
    enabled_features.addFeature(@intFromEnum(armv8a_features.vldn_align));
    enabled_features.addFeature(@intFromEnum(armv8a_features.neon));
    enabled_features.addFeature(@intFromEnum(armv8a_features.vfp4d16));

    const armv8a_target = std.Target.Query{
        .cpu_arch = .aarch64,
        .os_tag = .freestanding,
        .cpu_model = .{
            .explicit = &.{
                .name = "cortex_a53",
                .llvm_name = "cortex-a53",
                .features = .{ .ints = .{ 0, 0, 0, 0, 0 } }, // empty
            },
        },
        .cpu_features_add = enabled_features,
    };
    const resolver_target = b.resolveTargetQuery(armv8a_target);

    const fsbl_elf = b.addExecutable(.{
        .name = "qs8m-fsbl",
        .root_source_file = .{ .src_path = .{ .owner = b, .sub_path = "src/imx8m/ocram_part.zig" } },
        .target = resolver_target,
        .optimize = optimize,
    });
    fsbl_elf.setLinkerScript(.{ .src_path = .{ .owner = b, .sub_path = "src/imx8m/ocram_part.ld" } });

    const copy_elf = b.addInstallArtifact(fsbl_elf, .{});
    const bin = b.addObjCopy(fsbl_elf.getEmittedBin(), .{ .format = .bin });
    const copy_bin = b.addInstallBinFile(bin.getOutput(), "qs8m-fsbl.bin");

    bin.step.dependOn(&copy_elf.step);
    copy_bin.step.dependOn(&bin.step);

    b.default_step.dependOn(&copy_bin.step);
}
