const std = @import("std");
const keep = @import("std").mem.doNotOptimizeAway;

pub fn printf(comptime fmt: []const u8, args: anytype) void {
    nosuspend semihostingStdoutwriter.print(fmt, args) catch return;
}

const Error = error{Undexpected};
var SemihostingStdoutHandler: i32 = undefined;
const semihostingStdoutwriter = writer(&SemihostingStdoutHandler);
fn writer(context: *const i32) std.io.Writer(*const i32, Error, write) {
    return .{ .context = context };
}

fn write(context: *const i32, data: []const u8) Error!usize {
    if (context.* == -1)
        return Error.Undexpected;

    const arg: struct {
        handle: u32,
        data_addr: u32,
        data_len: u32,
    } = .{ .handle = @intCast(context.*), .data_addr = @intFromPtr(data.ptr), .data_len = data.len };
    var ret: u32 = undefined;
    asm volatile (
        \\ mov r1, %[arg_addr]
        \\ mov r0, #0x5
        \\ svc 0x123456
        \\ mov %[ret_code], r0
        : [ret_code] "=r" (ret),
        : [arg_addr] "r" (&arg),
        : "r0", "r1", "memory"
    );

    switch (ret) {
        0 => return data.len,
        else => return Error.Undexpected,
    }
}

pub fn init() void {
    SemihostingStdoutHandler = getSemihostingHandler(":tt");
}

fn getSemihostingHandler(filename: [:0]const u8) i32 {
    const arg: struct {
        addr: u32,
        mode: u32,
        filename_len: u32,
    } = .{ .addr = @intFromPtr(filename.ptr), .mode = 4, .filename_len = filename.len };
    var handle: i32 = 0;
    asm volatile (
        \\ mov r1, %[arg_addr]
        \\ mov r0, #0x1
        \\ svc 0x123456
        \\ mov %[handle], r0
        : [handle] "=r" (handle),
        : [arg_addr] "r" (&arg),
        : "r0", "r1", "memory"
    );

    return handle;
}
