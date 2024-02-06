const Mem = @import("regmap.zig");

const console = Mem.Bus.APB1.ports().UART4.api();
const RCC = Mem.Bus.AHB4.ports().RCC.api();
const MUX = RCC.MUX;
const PLL = RCC.PLL;
const LED = Mem.Bus.AHB4.ports().GPIOA.api().pin(13);

export fn main() u8 {
    // RCC init
    RCC.LSE.init(RCC.EXT_CLOCK_MODE.Crystal);
    RCC.HSE.init(RCC.EXT_CLOCK_MODE.Crystal);

    // MPU clock source
    MUX.PLL12.setSource(MUX.source(MUX.PLL12).HSE); // Prepare MPU clock source
    PLL.PLL1.setDividers(2, 80, 2048, 0, 1, 1); // -> 650 MHz on DIVP PLL1 port
    PLL.PLL1.enableOutput(PLL.OUTPUT.P); // Enable MPU source clock output
    MUX.MPU.setSource(MUX.source(MUX.MPU).PLL1); // Switch to new MPU clock source

    // UART pins SoC dependant
    const uart_rx_pin = Mem.Bus.AHB4.ports().GPIOB.api().pin(2);
    const uart_tx_pin = Mem.Bus.AHB4.ports().GPIOG.api().pin(11);
    uart_rx_pin.enableGPIOclocks();
    uart_tx_pin.enableGPIOclocks();
    uart_rx_pin.configure(uart_rx_pin.MODE.AltFunc, uart_rx_pin.OTYPE.PushPull, uart_rx_pin.OSPEED.High, uart_rx_pin.PUPD.PullUp, 8);
    uart_tx_pin.configure(uart_tx_pin.MODE.AltFunc, uart_tx_pin.OTYPE.PushPull, uart_tx_pin.OSPEED.High, uart_tx_pin.PUPD.PullUp, 6);
    console.init();

    // LED
    LED.configure(LED.MODE.Output, LED.OTYPE.OpenDrain, LED.OSPEED.High, LED.PUPD.Disabled, 0);
    LED.reset();

    _ = console.write("Hello, world!\r\n");

    return 0;
}
