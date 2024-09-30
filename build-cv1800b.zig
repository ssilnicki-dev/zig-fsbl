const std = @import("std");
const Feature = @import("std").Target.Cpu.Feature;

pub fn build(b: *std.Build) void {
    const bl33_elf = b.addExecutable(.{
        .name = "milk-v-duo-bl33",
        .target = b.resolveTargetQuery(std.Target.Query{
            .cpu_arch = .riscv64,
            .os_tag = .freestanding,
            .cpu_model = .baseline,
            .cpu_features_add = Feature.Set.empty,
        }),
    });

    bl33_elf.addAssemblyFile(.{ .src_path = .{ .owner = b, .sub_path = "src/cv1800b/milk-v-duo-bl33.S" } });
    bl33_elf.setLinkerScript(.{ .src_path = .{ .owner = b, .sub_path = "src/cv1800b/linker.ld" } });

    const copy_elf = b.addInstallArtifact(bl33_elf, .{});
    const bin = b.addObjCopy(bl33_elf.getEmittedBin(), .{ .format = .bin });
    const copy_bin = b.addInstallBinFile(bin.getOutput(), "milk-v-duo-bl33.bin");

    bin.step.dependOn(&copy_elf.step);
    copy_bin.step.dependOn(&bin.step);

    b.default_step.dependOn(&copy_bin.step);
}
