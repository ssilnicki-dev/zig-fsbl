const Mem = @import("regmap.zig");

const console = Mem.Bus.APB1.ports().UART4.api();
const RCC = Mem.Bus.AHB4.ports().RCC.api();
const MUX = RCC.MUX;
const PLL = RCC.PLL;
const LED = Mem.Bus.AHB4.ports().GPIOA.api().pin(13);
const DDR = @import("regmap.zig").Bus.APB4.ports().DDR.api();
const TZC = Mem.Bus.APB5.ports().TZC.api();

export fn main() u8 {
    // RCC init
    RCC.LSE.init(RCC.EXT_CLOCK_MODE.Crystal);
    RCC.HSE.init(RCC.EXT_CLOCK_MODE.Crystal);

    // MPU clock source
    MUX.PLL12.setSource(MUX.source(MUX.PLL12).HSE); // Prepare MPU clock source
    PLL.PLL1.setDividers(2, 80, 2048, 0, 1, 1); // -> 650 MHz on DIVP PLL1 port
    PLL.PLL1.enableOutput(PLL.OUTPUT.P); // Enable MPU source clock output
    MUX.MPU.setSource(MUX.source(MUX.MPU).PLL1); // Switch to new MPU clock source

    // AXI, DDR
    PLL.PLL2.setDividers(2, 65, 5120, 1, 0, 0); // -> 533 MHz on DIVR PPL2 port
    PLL.PLL2.enableOutput(PLL.OUTPUT.R); // Enable DDR source clock output
    PLL.PLL2.enableOutput(PLL.OUTPUT.P); // Enable AXI source clock output
    MUX.AXI.setSource(MUX.source(MUX.AXI).PLL2); // Switch to new AXI clock source
    RCC.AXI.setDividers(0, 1, 2); // see actual div. values RM0436 Rev 6 pp.662-665
    TZC.initSecureDDRAccess();
    _ = DDR.init(ddr_regs_values);
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

const REGS = DDR.regs();
const REG_VALUE = DDR.REG_VALUE;
const GRV = DDR.generateRegValue;
const ddr_regs_values = DDR.REGS_VALUES{
    .ctrl = .{
        .map = &[_]*const REG_VALUE{
            GRV(REGS.ADDRMAP1, 0x00070707),
            GRV(REGS.ADDRMAP2, 0x00000000),
            GRV(REGS.ADDRMAP3, 0x1F000000),
            GRV(REGS.ADDRMAP4, 0x00001F1F),
            GRV(REGS.ADDRMAP5, 0x06060606),
            GRV(REGS.ADDRMAP6, 0x0F060606),
            GRV(REGS.ADDRMAP9, 0x00000000),
            GRV(REGS.ADDRMAP10, 0x00000000),
            GRV(REGS.ADDRMAP11, 0x00000000),
        },
        .timing = &[_]*const REG_VALUE{
            GRV(REGS.RFSHTMG, 0x0081008B),
            GRV(REGS.DRAMTMG0, 0x121B2414),
            GRV(REGS.DRAMTMG1, 0x000A041C),
            GRV(REGS.DRAMTMG2, 0x0608090F),
            GRV(REGS.DRAMTMG3, 0x0050400C),
            GRV(REGS.DRAMTMG4, 0x08040608),
            GRV(REGS.DRAMTMG5, 0x06060403),
            GRV(REGS.DRAMTMG6, 0x02020002),
            GRV(REGS.DRAMTMG7, 0x00000202),
            GRV(REGS.DRAMTMG8, 0x00001005),
            GRV(REGS.DRAMTMG14, 0x000000A0),
            GRV(REGS.ODTCFG, 0x06000600),
        },
        .perf = &[_]*const REG_VALUE{
            GRV(REGS.SCHED, 0x00000C01),
            GRV(REGS.SCHED1, 0x00000000),
            GRV(REGS.PERFHPR1, 0x01000001),
            GRV(REGS.PERFLPR1, 0x08000200),
            GRV(REGS.PERFWR1, 0x08000400),
            GRV(REGS.PCFGR_0, 0x00010000),
            GRV(REGS.PCFGW_0, 0x00000000),
            GRV(REGS.PCFGQOS0_0, 0x02100C03),
            GRV(REGS.PCFGQOS1_0, 0x00800100),
            GRV(REGS.PCFGWQOS0_0, 0x01100C03),
            GRV(REGS.PCFGWQOS1_0, 0x01000200),
            GRV(REGS.PCFGR_1, 0x00010000),
            GRV(REGS.PCFGW_1, 0x00000000),
            GRV(REGS.PCFGQOS0_1, 0x02100C03),
            GRV(REGS.PCFGQOS1_1, 0x00800040),
            GRV(REGS.PCFGWQOS0_1, 0x01100C03),
            GRV(REGS.PCFGWQOS1_1, 0x01000200),
        },
        .reg = &[_]*const REG_VALUE{
            GRV(REGS.MSTR, 0x00041401),
            GRV(REGS.MRCTRL0, 0x00000010),
            GRV(REGS.MRCTRL1, 0x00000000),
            GRV(REGS.DERATEEN, 0x00000000),
            GRV(REGS.DERATEINT, 0x00800000),
            GRV(REGS.PWRCTL, 0x00000000),
            GRV(REGS.PWRTMG, 0x00400010),
            GRV(REGS.HWLPCTL, 0x00000000),
            GRV(REGS.RFSHCTL0, 0x00210000),
            GRV(REGS.RFSHCTL3, 0x00000000),
            GRV(REGS.CRCPARCTL0, 0x00000000),
            GRV(REGS.ZQCTL0, 0xC2000040),
            GRV(REGS.DFITMG0, 0x02060105),
            GRV(REGS.DFITMG1, 0x00000202),
            GRV(REGS.DFILPCFG0, 0x07000000),
            GRV(REGS.DFIUPD0, 0xC0400003),
            GRV(REGS.DFIUPD1, 0x00000000),
            GRV(REGS.DFIUPD2, 0x00000000),
            GRV(REGS.DFIPHYMSTR, 0x00000000),
            GRV(REGS.ODTMAP, 0x00000001),
            GRV(REGS.DBG0, 0x00000000),
            GRV(REGS.DBG1, 0x00000000),
            GRV(REGS.DBGCMD, 0x00000000),
            GRV(REGS.POISONCFG, 0x00000000),
            GRV(REGS.PCCFG, 0x00000010),
        },
    },
    .phy = .{
        .reg = &[_]*const REG_VALUE{
            GRV(REGS.PGCR, 0x01442E02),
            GRV(REGS.ACIOCR, 0x10400812),
            GRV(REGS.DXCCR, 0x00000C40),
            GRV(REGS.DSGCR, 0xF200011F),
            GRV(REGS.DCR, 0x0000000B),
            GRV(REGS.ODTCR, 0x00010000),
            GRV(REGS.ZQ0CR1, 0x00000038),
            GRV(REGS.DX0GCR, 0x0000CE81),
            GRV(REGS.DX1GCR, 0x0000CE81),
            GRV(REGS.DX2GCR, 0x0000CE80),
            GRV(REGS.DX3GCR, 0x0000CE80),
        },
        .timing = &[_]*const REG_VALUE{
            GRV(REGS.PTR0, 0x0022AA5B),
            GRV(REGS.PTR1, 0x04841104),
            GRV(REGS.PTR2, 0x042DA068),
            GRV(REGS.DTPR0, 0x38D488D0),
            GRV(REGS.DTPR1, 0x098B00D8),
            GRV(REGS.DTPR2, 0x10023600),
            GRV(REGS.MR2, 0x00000208),
            GRV(REGS.MR3, 0x00000000),
            GRV(REGS.MR1, 0x00000000),
            GRV(REGS.MR0, 0x00000840),
        },
    },
};
