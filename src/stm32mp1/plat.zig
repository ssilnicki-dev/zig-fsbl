const arch = @import("arch.zig");

fn mapPeriphery() void {
}

pub export fn Initialize() void {
    mapPeriphery();
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
