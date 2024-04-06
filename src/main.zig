const bus = @import("regmap.zig");
const std = @import("std");
const led = bus.gpioa.pin(13);

const uart_writer = writer(void{});
pub fn printf(comptime fmt: []const u8, args: anytype) void {
    nosuspend uart_writer.print(fmt, args) catch return;
}

pub fn stm32mp1_uart4_writer(nt: @TypeOf(void{}), bytes: []const u8) error{}!usize {
    _ = nt;
    return bus.uart4.write(bytes);
}

const Writer = std.io.Writer(@TypeOf(void{}), error{}, stm32mp1_uart4_writer);
fn writer(nt: @TypeOf(void{})) Writer {
    return .{ .context = nt };
}

export fn main() u8 {
    // RCC init
    bus.rcc.enableHSE(.Crystal, 24_000_000);

    // MPU clock source
    bus.pll1.configure(.HSE, 2, 80, 2048, 0, 1, 1); // 650 MHz for MPU
    bus.mpu.configure(.PLL1, null); // Switch to new MPU clock source

    // AXI, DDR
    bus.pll2.configure(null, 2, 65, 5120, 1, 0, 0); // 533 MHz for DDR
    bus.axi.configure(.PLL2, 0, 1, 2);
    bus.tzc.initSecureDDRAccess();
    _ = bus.ddr.configure(ddr_reg_values);

    // UART pins SoC dependant
    bus.gpiob.pin(2).configure(.AltFunc, .PushPull, .High, .PullUp, 8);
    bus.gpiog.pin(11).configure(.AltFunc, .PushPull, .High, .PullUp, 6);
    bus.uart4.configure(.HSI, .B115200, .EightDataBits, .NoParity, .OneStopBit);
    led.configure(.Output, .OpenDrain, .High, .Disabled, 0);

    // SDMMC1 - SD
    bus.gpioc.pin(8).configure(.AltFunc, .PushPull, .Medium, .PullUp, 12); // D0
    bus.gpioc.pin(9).configure(.AltFunc, .PushPull, .Medium, .PullUp, 12); // D1
    bus.gpioc.pin(10).configure(.AltFunc, .PushPull, .Medium, .PullUp, 12); // D2
    bus.gpioc.pin(11).configure(.AltFunc, .PushPull, .Medium, .Disabled, 12); // D3
    bus.gpioc.pin(12).configure(.AltFunc, .PushPull, .High, .Disabled, 12); // CK
    bus.gpiod.pin(2).configure(.AltFunc, .PushPull, .Medium, .PullUp, 12); // CMD
    bus.gpiob.pin(7).configure(.Input, .PushPull, .Medium, .PullUp, 0); // CD

    bus.pll4.configure(.HSE, 1, 49, 0, 2, 6, 7); // 200 MHz for SDMMC 1 & 2
    bus.sdmmc1.setClockSource(.PLL4);
    const sd1_media_type = bus.sdmmc1.getMediaType(.Performance);

    switch (sd1_media_type) {
        .SDHC => {
            switch (bus.sdmmc1.getCard()) {
                .card => |*card| {
                    if (card.BlockSize > 0 and card.Blocks512 > 0)
                        led.reset();
                    bus.mpu.udelay(400000);
                },
                else => {},
            }
        },
        else => {},
    }

    led.assert();

    return 0;
}

const ddr_reg_values = @TypeOf(bus.ddr).RegValues{
    .ctrl = .{
        .map = &[_]*const @TypeOf(bus.ddr).RegValues.CtrlRegValues.Pair{
            &.{ .reg = .ADDRMAP1, .value = 0x00070707 },
            &.{ .reg = .ADDRMAP2, .value = 0x00000000 },
            &.{ .reg = .ADDRMAP3, .value = 0x1F000000 },
            &.{ .reg = .ADDRMAP4, .value = 0x00001F1F },
            &.{ .reg = .ADDRMAP5, .value = 0x06060606 },
            &.{ .reg = .ADDRMAP6, .value = 0x0F060606 },
            &.{ .reg = .ADDRMAP9, .value = 0x00000000 },
            &.{ .reg = .ADDRMAP10, .value = 0x00000000 },
            &.{ .reg = .ADDRMAP11, .value = 0x00000000 },
        },
        .timing = &[_]*const @TypeOf(bus.ddr).RegValues.CtrlRegValues.Pair{
            &.{ .reg = .RFSHTMG, .value = 0x0081008B },
            &.{ .reg = .DRAMTMG0, .value = 0x121B2414 },
            &.{ .reg = .DRAMTMG1, .value = 0x000A041C },
            &.{ .reg = .DRAMTMG2, .value = 0x0608090F },
            &.{ .reg = .DRAMTMG3, .value = 0x0050400C },
            &.{ .reg = .DRAMTMG4, .value = 0x08040608 },
            &.{ .reg = .DRAMTMG5, .value = 0x06060403 },
            &.{ .reg = .DRAMTMG6, .value = 0x02020002 },
            &.{ .reg = .DRAMTMG7, .value = 0x00000202 },
            &.{ .reg = .DRAMTMG8, .value = 0x00001005 },
            &.{ .reg = .DRAMTMG14, .value = 0x000000A0 },
            &.{ .reg = .ODTCFG, .value = 0x06000600 },
        },
        .perf = &[_]*const @TypeOf(bus.ddr).RegValues.CtrlRegValues.Pair{
            &.{ .reg = .SCHED, .value = 0x00000C01 },
            &.{ .reg = .SCHED1, .value = 0x00000000 },
            &.{ .reg = .PERFHPR1, .value = 0x01000001 },
            &.{ .reg = .PERFLPR1, .value = 0x08000200 },
            &.{ .reg = .PERFWR1, .value = 0x08000400 },
            &.{ .reg = .PCFGR_0, .value = 0x00010000 },
            &.{ .reg = .PCFGW_0, .value = 0x00000000 },
            &.{ .reg = .PCFGQOS0_0, .value = 0x02100C03 },
            &.{ .reg = .PCFGQOS1_0, .value = 0x00800100 },
            &.{ .reg = .PCFGWQOS0_0, .value = 0x01100C03 },
            &.{ .reg = .PCFGWQOS1_0, .value = 0x01000200 },
            &.{ .reg = .PCFGR_1, .value = 0x00010000 },
            &.{ .reg = .PCFGW_1, .value = 0x00000000 },
            &.{ .reg = .PCFGQOS0_1, .value = 0x02100C03 },
            &.{ .reg = .PCFGQOS1_1, .value = 0x00800040 },
            &.{ .reg = .PCFGWQOS0_1, .value = 0x01100C03 },
            &.{ .reg = .PCFGWQOS1_1, .value = 0x01000200 },
        },
        .reg = &[_]*const @TypeOf(bus.ddr).RegValues.CtrlRegValues.Pair{
            &.{ .reg = .MSTR, .value = 0x00041401 },
            &.{ .reg = .MRCTRL0, .value = 0x00000010 },
            &.{ .reg = .MRCTRL1, .value = 0x00000000 },
            &.{ .reg = .DERATEEN, .value = 0x00000000 },
            &.{ .reg = .DERATEINT, .value = 0x00800000 },
            &.{ .reg = .PWRCTL, .value = 0x00000000 },
            &.{ .reg = .PWRTMG, .value = 0x00400010 },
            &.{ .reg = .HWLPCTL, .value = 0x00000000 },
            &.{ .reg = .RFSHCTL0, .value = 0x00210000 },
            &.{ .reg = .RFSHCTL3, .value = 0x00000000 },
            &.{ .reg = .CRCPARCTL0, .value = 0x00000000 },
            &.{ .reg = .ZQCTL0, .value = 0xC2000040 },
            &.{ .reg = .DFITMG0, .value = 0x02060105 },
            &.{ .reg = .DFITMG1, .value = 0x00000202 },
            &.{ .reg = .DFILPCFG0, .value = 0x07000000 },
            &.{ .reg = .DFIUPD0, .value = 0xC0400003 },
            &.{ .reg = .DFIUPD1, .value = 0x00000000 },
            &.{ .reg = .DFIUPD2, .value = 0x00000000 },
            &.{ .reg = .DFIPHYMSTR, .value = 0x00000000 },
            &.{ .reg = .ODTMAP, .value = 0x00000001 },
            &.{ .reg = .DBG0, .value = 0x00000000 },
            &.{ .reg = .DBG1, .value = 0x00000000 },
            &.{ .reg = .DBGCMD, .value = 0x00000000 },
            &.{ .reg = .POISONCFG, .value = 0x00000000 },
            &.{ .reg = .PCCFG, .value = 0x00000010 },
        },
    },
    .phyc = .{
        .reg = &[_]*const @TypeOf(bus.ddr).RegValues.PhycRegValues.Pair{
            &.{ .reg = .PGCR, .value = 0x01442E02 },
            &.{ .reg = .ACIOCR, .value = 0x10400812 },
            &.{ .reg = .DXCCR, .value = 0x00000C40 },
            &.{ .reg = .DSGCR, .value = 0xF200011F },
            &.{ .reg = .DCR, .value = 0x0000000B },
            &.{ .reg = .ODTCR, .value = 0x00010000 },
            &.{ .reg = .ZQ0CR1, .value = 0x00000038 },
            &.{ .reg = .DX0GCR, .value = 0x0000CE81 },
            &.{ .reg = .DX1GCR, .value = 0x0000CE81 },
            &.{ .reg = .DX2GCR, .value = 0x0000CE80 },
            &.{ .reg = .DX3GCR, .value = 0x0000CE80 },
        },
        .timing = &[_]*const @TypeOf(bus.ddr).RegValues.PhycRegValues.Pair{
            &.{ .reg = .PTR0, .value = 0x0022AA5B },
            &.{ .reg = .PTR1, .value = 0x04841104 },
            &.{ .reg = .PTR2, .value = 0x042DA068 },
            &.{ .reg = .DTPR0, .value = 0x38D488D0 },
            &.{ .reg = .DTPR1, .value = 0x098B00D8 },
            &.{ .reg = .DTPR2, .value = 0x10023600 },
            &.{ .reg = .MR2, .value = 0x00000208 },
            &.{ .reg = .MR3, .value = 0x00000000 },
            &.{ .reg = .MR1, .value = 0x00000000 },
            &.{ .reg = .MR0, .value = 0x00000840 },
        },
    },
};
