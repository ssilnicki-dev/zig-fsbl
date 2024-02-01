const Mem = @import("regmap.zig");

const console = Mem.Bus.APB1.ports().UART4.api();
const RCC = Mem.Bus.AHB4.ports().RCC.api();

export fn main() u8 {
    // RCC init
    RCC.LSE.init();

    _ = console.write("Hello, world!\r\n");

    return 0;
}
