const root = @import("root.zig");

export fn _start() void {
    asm volatile ("ldr sp, =0xE0000000"); // stack pointer at the end of available DDR
    asm volatile ("bl main");
    asm volatile ("b .");
}

export fn main() u8 {
    return @intCast(root.add(2,3));
}
