const std = @import("std");

const NoType = struct {};


pub fn printf(comptime fmt: []const u8, args: anytype) !void {
    const uart_writer = writer(.{});
    nosuspend uart_writer.print(fmt, args) catch return;
}

fn stm32mp1_uart4_writer(nt: NoType, bytes: []const u8) std.os.WriteError!usize {
    _ = nt;

    const stat: *volatile u32 = @ptrFromInt(0x4001001C);
    const tx:  *volatile u32 = @ptrFromInt(0x40010028);
    const tx_rdy: u32 = 1<<7;

    for (bytes) |byte| {
        while((stat.* & tx_rdy) == 0 )
            continue;
        tx.* = byte;
    }
    return bytes.len;
}

const Writer = std.io.Writer(NoType, std.os.WriteError, stm32mp1_uart4_writer);
fn writer(nt: NoType) Writer {
    return .{ .context = nt };
}
