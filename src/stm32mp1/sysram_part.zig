const bus = @import("stm32mp157c.zig");
const led = bus.gpioa.pin(13);

export fn sysram_main() void {
    @import("qsmp.zig").rccMpuAxiDdrInit();
    led.configure(.Output, .OpenDrain, .High, .Disabled, 0);
    led.reset();
    bus.mpu.udelay(100_000);
    led.assert();
    bus.mpu.udelay(300_000);
    led.reset();
    bus.mpu.udelay(100_000);
    led.assert();
}
