const c = @cImport({
    @cInclude("elf.h");
});

const std = @import("std");

const Stm32Header = extern struct {
    // Zig regulat structs do not gurtantee memory layout... :(
    // only extern structs said to do
    magic: u32 = 0x324D5453, // 'S', 'T', 'M', 0x32
    ecdsa_sig: [16]u32 = std.mem.zeroes([16]u32),
    checksum: u32 = 0x0,
    header_ver: u32 = 0x10000,
    img_length: u32 = 0x0,
    entry_p: u32 = 0x0,
    reserved1: u32 = 0x0,
    load_addr: u32 = 0x2FFC0000,
    reserved2: u32 = 0x0,
    version_nr: u32 = 0x0,
    option_flags: u32 = 0x1, // no sig. verification
    ecdsa_alg: u32 = 0x1, // 1: P-256 NIST ; 2: brainpool 256
    ecdsa_pub_k: [16]u32 = std.mem.zeroes([16]u32), // undefined,
    padding: [83]u8 = std.mem.zeroes([83]u8),
    bin_type: u8 = 0x10, //0x10-0x1F: FSBL ; 0x30: Copro
};

pub fn main() !u8 {
    var args = std.process.args();
    _ = args.skip(); // binary path

    var elf_header: c.Elf32_Ehdr = undefined;
    const elf_file_path = args.next().?;
    const bin_file_path = args.next().?;
    const install_path = args.next().?;
    const output_filename = args.next().?;
    var header: Stm32Header = .{};
    var bin_size: usize = 0;
    var bin_sum: u64 = 0;

    // read ELF32 header
    const elf_input = try std.fs.openFileAbsolute(elf_file_path, .{});
    defer elf_input.close();
    var read_bytes = elf_input.read(@as([*]u8, @ptrCast(&elf_header))[0..@sizeOf(@TypeOf(elf_header))]) catch unreachable;
    if (read_bytes != @sizeOf(@TypeOf(elf_header))) {
        std.debug.print("ERROR reading ELF32 header.\n", .{});
        return 1;
    }

    // prepare output binary
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    var allocator = gpa.allocator();
    const stm32_output_path = std.fmt.allocPrint(allocator, "{s}/{s}.stm32", .{ install_path, output_filename }) catch unreachable;
    defer allocator.free(stm32_output_path);
    const output = try std.fs.createFileAbsolute(stm32_output_path, .{});
    // write empty STM32 header
    _ = try output.writeAll(@as([*]u8, @ptrCast(&header))[0..@sizeOf(Stm32Header)]);

    // read input binary
    var read_buffer: [64]u8 = undefined;
    const bin_input = try std.fs.openFileAbsolute(bin_file_path, .{});
    defer bin_input.close();
    while (true) {
        read_bytes = try bin_input.read(&read_buffer);
        if (read_bytes == 0)
            break;

        bin_size += read_bytes;
        for (read_buffer[0..read_bytes]) |value| {
            bin_sum += value;
        }
        _ = try output.writeAll(read_buffer[0..read_bytes]);
    }

    header.entry_p = elf_header.e_entry;
    header.checksum = @intCast(bin_sum & 0xFFFFFFFF);
    header.img_length = @intCast(bin_size);
    _ = try output.seekTo(0);
    _ = try output.writeAll(@as([*]u8, @ptrCast(&header))[0..@sizeOf(Stm32Header)]);
    std.debug.print("{s} ELF binary enrty point @ 0x{X}, binary length w/0 STM32 header = {d}, binary checksum = 0x{X}.\n", .{ output_filename, elf_header.e_entry, header.img_length, header.checksum });

    return 0;
}
