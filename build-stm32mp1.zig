const std = @import("std");
const Target = @import("std").Target;
const Feature = @import("std").Target.Cpu.Feature;

pub fn build(b: *std.Build, optimize: std.builtin.OptimizeMode) void {
    const standard_target = b.standardTargetOptions(.{});

    const armv7a_features = Target.arm.Feature;
    var enabled_features = Feature.Set.empty;
    enabled_features.addFeature(@intFromEnum(armv7a_features.v7a));
    enabled_features.addFeature(@intFromEnum(armv7a_features.vldn_align));
    enabled_features.addFeature(@intFromEnum(armv7a_features.neon));
    enabled_features.addFeature(@intFromEnum(armv7a_features.vfp3d16));

    const armv7a_target = std.Target.Query{
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
        .name = "qsmp-fsbl",
        .root_source_file = .{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/fsbl.zig" } },
        .target = resolver_target,
        .optimize = optimize,
        .strip = false,
        .unwind_tables = .none,
    });
    fsbl_elf.entry = .{.symbol_name = "EntryPoint"};
    fsbl_elf.setLinkerScript(.{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/linker.ld" } });
    fsbl_elf.link_gc_sections = true;
    fsbl_elf.link_function_sections = true;
    fsbl_elf.link_data_sections = true;
    // fsbl_elf.want_lto = true;

    const fsbl_elf_qemu = b.addExecutable(.{
        .name = "qsmp-fsbl-qemu",
        .root_source_file = .{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/fsbl.zig" } },
        .target = resolver_target,
        .optimize = optimize,
        .strip = false,
        .unwind_tables = .none,
    });
    fsbl_elf_qemu.entry = .{.symbol_name = "Reset_Handler"};
    fsbl_elf_qemu.setLinkerScript(.{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/linker-qemu.ld" } });
    fsbl_elf_qemu.link_gc_sections = true;
    fsbl_elf_qemu.link_function_sections = true;
    fsbl_elf_qemu.link_data_sections = true;
    // fsbl_elf.want_lto = true;


    const sysram_part_elf = b.addExecutable(.{
        .name = "sysram-part",
        .root_source_file = .{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/sysram_part.zig" } },
        .target = resolver_target,
        .optimize = optimize,
        .strip = false,
    });
    sysram_part_elf.addAssemblyFile(.{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/sysram_part.S" } });
    sysram_part_elf.setLinkerScript(.{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/sysram_part.ld" } });

    const ddr_part_elf = b.addExecutable(.{
        .name = "ddr-part",
        .root_source_file = .{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/ddr_part.zig" } },
        .target = resolver_target,
        .optimize = optimize,
        .strip = false,
    });
    ddr_part_elf.setLinkerScript(.{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/ddr_part.ld" } });

    const stm32header_elf = b.addExecutable(.{
        .name = "stm32header",
        .root_source_file = .{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/stm32header.zig" } },
        .target = standard_target,
        .optimize = .ReleaseFast,
        .strip = true,
    });
    stm32header_elf.linkSystemLibrary("c");

    const copy_sysram_part_elf = b.addInstallArtifact(sysram_part_elf, .{});
    const sysram_bin = b.addObjCopy(sysram_part_elf.getEmittedBin(), .{ .format = .bin });
    const copy_sysram_bin = b.addInstallBinFile(sysram_bin.getOutput(), "sysram-part.bin");
    const copy_ddr_part_elf = b.addInstallArtifact(ddr_part_elf, .{});
    const sysram_elf2_run_step = b.addRunArtifact(stm32header_elf);

    sysram_elf2_run_step.addFileArg(sysram_part_elf.getEmittedBin()); // elf
    sysram_elf2_run_step.addFileArg(copy_sysram_bin.source); // bin
    sysram_elf2_run_step.addFileArg(.{ .src_path = .{ .owner = b, .sub_path = b.getInstallPath(copy_sysram_bin.dir, "") } }); // install path
    sysram_elf2_run_step.addArg("sysram-part"); // output filename

    copy_sysram_bin.step.dependOn(&copy_sysram_part_elf.step);
    sysram_elf2_run_step.step.dependOn(&copy_sysram_bin.step);

    const copy_elf = b.addInstallArtifact(fsbl_elf, .{});
    const bin = b.addObjCopy(fsbl_elf.getEmittedBin(), .{ .format = .bin });
    const copy_bin = b.addInstallBinFile(bin.getOutput(), "qsmp-fsbl.bin");
    const elf2_run_step = b.addRunArtifact(stm32header_elf);
    const copy_elf_qemu = b.addInstallArtifact(fsbl_elf_qemu, .{});

    copy_elf.step.dependOn(&copy_elf_qemu.step);
    bin.step.dependOn(&copy_elf.step);
    bin.step.dependOn(&sysram_elf2_run_step.step);
    bin.step.dependOn(&copy_ddr_part_elf.step);
    copy_bin.step.dependOn(&bin.step);
    elf2_run_step.step.dependOn(&copy_bin.step);
    elf2_run_step.addFileArg(fsbl_elf.getEmittedBin()); // elf
    elf2_run_step.addFileArg(copy_bin.source); // bin
    elf2_run_step.addFileArg(.{ .src_path = .{ .owner = b, .sub_path = b.getInstallPath(copy_bin.dir, "") } }); // install path
    elf2_run_step.addArg("qsmp-fsbl"); // output filename

    b.default_step.dependOn(&elf2_run_step.step);

    const regmap_unit_tests = b.addTest(.{
        .root_source_file = .{ .src_path = .{ .owner = b, .sub_path = "src/stm32mp1/stm32mp157c.zig" } },
        .target = standard_target,
        .optimize = .ReleaseSafe,
    });

    const run_regmap_unit_tests = b.addRunArtifact(regmap_unit_tests);
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_regmap_unit_tests.step);
}
