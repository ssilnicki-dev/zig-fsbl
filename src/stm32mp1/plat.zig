const arch = @import("arch.zig");
const stm32mp157c = @import("stm32mp157c.zig");
const PWR = stm32mp157c.PWR;
var pwr: PWR = undefined;

fn mapPeriphery() void {
    pwr = PWR{ .port = arch.mapPeriphery(0x50001000, 1024) catch |e| panic(@src().line, e) };
}

pub export fn Initialize() void {
    mapPeriphery();
    pwr.backupDomainWriteProtection(.Disable);
}

noinline fn panic(line: u32, err: anyerror) noreturn {
    _ = arch.LOAD(.Word, .r4, arch.SET(.r5, &line));
    switch (err) {
        error.NotAligned => _ = arch.SET(.r5, @src().line),
        error.NotAvailable => _ = arch.SET(.r5, @src().line),
        else => _ = arch.SET(.r5, @src().line),
    }
    asm volatile ("b .");
    unreachable;
}
