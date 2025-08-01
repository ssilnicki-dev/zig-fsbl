const arch = @import("arch.zig");
const stm32mp157c = @import("stm32mp157c.zig");
const RCC = stm32mp157c.RCC;
const PWR = stm32mp157c.PWR;

var pwr: PWR = undefined;
var rcc: RCC = undefined;

fn mapPeriphery() void {
    pwr = PWR{ .port = arch.mapPeriphery(0x50001000, 1024) catch |e| plat_panic(@src().line, e) };
    rcc = RCC{ .port = arch.mapPeriphery(0x50000000, 4096) catch |e| plat_panic(@src().line, e) };
}

pub export fn Initialize() void {
    mapPeriphery();
    pwr.backupDomainWriteProtection(.Disable);
    if (rcc.getRTCSource() == .NoClock) { // Coldboot
        rcc.resetVSwitchDomain();
    }
    // TODO: setup clocks/dividers/plls/muxers -> mostly refactoring
    rcc.enableCSI(); // required for automatic IO compensation
    asm volatile ("nop");
    asm volatile ("nop");
}

noinline fn plat_panic(line: u32, err: anyerror) noreturn {
    _ = arch.LOAD(.Word, .r4, arch.SET(.r5, &line));
    switch (err) {
        error.NotAligned => _ = arch.SET(.r5, @src().line),
        error.NotAvailable => _ = arch.SET(.r5, @src().line),
        else => _ = arch.SET(.r5, @src().line),
    }
    asm volatile ("b .");
    unreachable;
}
