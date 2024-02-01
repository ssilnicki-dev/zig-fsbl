const Mem = @import("regmap.zig");

const console = Mem.Bus.APB1.ports().UART4.api();
const RCC = Mem.Bus.AHB4.ports().RCC.api();
const LED = Mem.Bus.AHB4.ports().GPIOA.api().pin(13);

export fn main() u8 {
    // LED
    LED.configure(LED.MODE.Output, LED.OTYPE.OpenDrain, LED.OSPEED.High, LED.PUPD.Disabled, 0);
    LED.reset();

    // RCC init
    RCC.LSE.init();

    _ = console.write("Hello, world!\r\n");

    return 0;
}
