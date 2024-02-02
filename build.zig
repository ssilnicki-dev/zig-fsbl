const std = @import("std");
const Target = @import("std").Target;
const CrossTarget = std.zig.CrossTarget;
const Feature = @import("std").Target.Cpu.Feature;

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{ .preferred_optimize_mode = .ReleaseSmall });
    const standard_target = b.standardTargetOptions(.{});

    const armv7a_features = Target.arm.Feature;
    var enabled_features = Feature.Set.empty;
    enabled_features.addFeature(@intFromEnum(armv7a_features.v7a));
    enabled_features.addFeature(@intFromEnum(armv7a_features.vldn_align));
    enabled_features.addFeature(@intFromEnum(armv7a_features.neon));
    enabled_features.addFeature(@intFromEnum(armv7a_features.vfp3d16));

    const armv7a_target = CrossTarget{
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
    const resolver_target = b.resolveTargetQuery(armv7a_target);

    const fsbl_elf = b.addExecutable(.{
        .name = "fsbl",
        .root_source_file = .{ .path = "src/main.zig" },
        .target = resolver_target,
        .optimize = optimize,
    });

    fsbl_elf.addAssemblyFile(.{ .path = "src/load.S" });
    fsbl_elf.setLinkerScript(.{ .path = "src/linker.ld" });

    const stm32header_elf = b.addExecutable(.{
        .name = "stm32header",
        .root_source_file = .{ .path = "src/stm32header.zig" },
        .target = standard_target,
        .optimize = optimize,
    });
    stm32header_elf.linkSystemLibrary("c");

    const copy_elf = b.addInstallArtifact(fsbl_elf, .{});
    const bin = b.addObjCopy(fsbl_elf.getEmittedBin(), .{ .format = .bin });
    const copy_bin = b.addInstallBinFile(bin.getOutput(), "fsbl.bin");
    const elf2_run_step = b.addRunArtifact(stm32header_elf);

    bin.step.dependOn(&copy_elf.step);
    copy_bin.step.dependOn(&bin.step);
    elf2_run_step.step.dependOn(&copy_bin.step);
    elf2_run_step.addFileArg(fsbl_elf.getEmittedBin()); // elf
    elf2_run_step.addFileArg(copy_bin.source); // bin
    elf2_run_step.addFileArg(.{ .path = copy_bin.dest_builder.exe_dir }); // install path
    b.default_step.dependOn(&elf2_run_step.step);

    const regmap_unit_tests = b.addTest(.{
        .root_source_file = .{ .path = "src/regmap.zig" },
        .target = standard_target,
        .optimize = optimize,
    });

    const run_regmap_unit_tests = b.addRunArtifact(regmap_unit_tests);
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_regmap_unit_tests.step);
}
