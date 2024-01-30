const Mem = @import("regmap.zig");

const console = Mem.Bus.APB1._().UART4._().api;
const RCC = Mem.Bus.AHB4._().RCC._().api;

export fn main() u8 {
    // RCC init
    RCC.LSE.init();

    _ = console.write("Hello, world!\r\n");

    return 0;
}
