const Mem = @import("regmap.zig");

pub fn init() void {
    initLSE();
}
fn initLSE() void {
    Mem.Bus.AHB4._().PWR._().disableBackupDomainWriteProtection();
    Mem.Bus.AHB4._().PWR._().enableBackupDomainWriteProtection();
}
