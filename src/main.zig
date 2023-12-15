const root = @import("root.zig");
const uart = @import("uart.zig");
const printf = uart.printf;

export  fn main() u8 {
    try printf("{s}", .{"Hello, world!\r\n"});
    return @intCast(root.add(2,3));
}
