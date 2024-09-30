const std = @import("std");
const Target = @import("std").Target;
const Feature = @import("std").Target.Cpu.Feature;

pub fn build(b: *std.Build) void {
    const armv8a_features = Target.arm.Feature;
    var enabled_features = Feature.Set.empty;

    enabled_features.addFeature(@intFromEnum(armv8a_features.v8a));
    enabled_features.addFeature(@intFromEnum(armv8a_features.vldn_align));
    enabled_features.addFeature(@intFromEnum(armv8a_features.neon));
    enabled_features.addFeature(@intFromEnum(armv8a_features.vfp4d16));

    const armv8a_target = std.Target.Query{
        .cpu_arch = .arm,
        .os_tag = .freestanding,
        .cpu_model = .{
            .explicit = &.{
                .name = "cortex_a8",
                .llvm_name = "cortex-a8",
                .features = .{ .ints = .{ 0, 0, 0, 0, 0 } }, // empty
            },
        },
        .abi = .eabihf,
        .cpu_features_add = enabled_features,
    };
    const resolver_target = b.resolveTargetQuery(armv8a_target);

    const fsbl_elf = b.addExecutable(.{
        .name = "qs8m-fsbl",
        .target = resolver_target,
    });
    fsbl_elf.addAssemblyFile(.{ .src_path = .{ .owner = b, .sub_path = "src/imx8m/load.S" } });
    fsbl_elf.setLinkerScript(.{ .src_path = .{ .owner = b, .sub_path = "src/imx8m/linker.ld" } });

    const copy_elf = b.addInstallArtifact(fsbl_elf, .{});
    const bin = b.addObjCopy(fsbl_elf.getEmittedBin(), .{ .format = .bin });
    const copy_bin = b.addInstallBinFile(bin.getOutput(), "qs8m-fsbl.bin");

    bin.step.dependOn(&copy_elf.step);
    copy_bin.step.dependOn(&bin.step);

    b.default_step.dependOn(&copy_bin.step);
}
