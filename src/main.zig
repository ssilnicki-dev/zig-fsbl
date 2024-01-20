const Mem = @import("regmap.zig");

const console = Mem.Bus.APB1._().UART4._().api;

export fn main() u8 {
    // RCC init

    _ = console.write("Hello, world!\r\n");

    return 0;
}
