// TODO: stick below typedefs to platform specifics
const bus_type: type = u32;
const enum_type: type = @TypeOf(enum {});

const HSI_FRIQUENCY = 64000000;
const HSE_FRIQUENCY = 24000000; // SoM dependant
var system_clock_hz: u32 = HSI_FRIQUENCY;
var cycle_counter_enabled: bool = false;

const Field = struct {
    pub const shift_type = switch (bus_type) {
        u32 => u5,
        u64 => u6,
        else => unreachable, // >64bit arch??? not so fast....
    };
    reg: bus_type,
    width: type,
    rw: enum {
        ReadOnly,
        WriteOnly,
        ReadWrite,
    },
    shift: shift_type,
    values: enum_type,

    fn setZeros(comptime self: Field) void {
        self.setValueImpl(0);
    }

    fn intToEnum(comptime self: Field, comptime value: bus_type) self.values {
        comptime if (@typeInfo(self.values).Enum.fields.len == 0)
            unreachable;
        return @as(self.values, @enumFromInt((value & self.getMask()) >> self.shift));
    }

    fn getShift(comptime self: Field) shift_type {
        return self.shift;
    }

    fn enumToInt(comptime self: Field, comptime value: anytype) bus_type {
        comptime if (@typeInfo(self.values).Enum.fields.len == 0)
            unreachable;
        return @as(bus_type, @intFromEnum(@as(self.values, value))) << self.shift;
    }

    inline fn getMask(comptime self: Field) bus_type {
        return @as(bus_type, (1 << @bitSizeOf(self.width)) - 1) << self.shift;
    }

    inline fn getResetMask(comptime self: Field) bus_type {
        return ~self.getMask();
    }

    pub fn set(comptime self: Field, value: anytype) void {
        switch (@TypeOf(value)) {
            self.values => self.setEnumValue(value),
            else => self.setIntValue(value),
        }
    }

    inline fn setValueImpl(comptime self: Field, value: bus_type) void {
        const addr: *volatile bus_type = @ptrFromInt(self.reg);
        if (self.rw == .WriteOnly) {
            addr.* = value << self.shift;
            return;
        }
        addr.* = addr.* & self.getResetMask() | ((value << self.shift) & self.getMask());
    }

    inline fn setEnumValue(comptime self: Field, value: self.values) void {
        comptime if (@typeInfo(self.values).Enum.fields.len == 0 or self.rw == .ReadOnly)
            unreachable;
        self.setValueImpl(@intFromEnum(value));
    }

    inline fn setIntValue(comptime self: Field, value: bus_type) void {
        comptime if (@typeInfo(self.values).Enum.fields.len > 0 or self.rw == .ReadOnly)
            unreachable;
        self.setValueImpl(value);
    }

    pub fn getIntValue(comptime self: Field) bus_type {
        comptime if (self.rw == .WriteOnly)
            unreachable;
        return (@as(*volatile bus_type, @ptrFromInt(self.reg)).* >> self.shift) & ((1 << @bitSizeOf(self.width)) - 1);
    }

    pub fn getEnumValue(comptime self: Field) self.values {
        comptime if (@typeInfo(self.values).Enum.fields.len == 0)
            unreachable;

        return @as(self.values, @enumFromInt(self.getIntValue()));
    }
};

fn API(comptime port: anytype) enum_type {
    const gpioa = @intFromEnum(Bus.AHB4.ports().GPIOA);
    const gpiob = @intFromEnum(Bus.AHB4.ports().GPIOB);
    const gpiog = @intFromEnum(Bus.AHB4.ports().GPIOG);
    const rcc = @intFromEnum(Bus.AHB4.ports().RCC);
    const pwr = @intFromEnum(Bus.AHB4.ports().PWR);
    const uart4 = @intFromEnum(Bus.APB1.ports().UART4);
    const ddr = @intFromEnum(Bus.APB4.ports().DDR);
    const tzc = @intFromEnum(Bus.APB5.ports().TZC);

    const current_port = @intFromEnum(port);

    return comptime switch (current_port) {
        tzc => enum(bus_type) {
            pub fn initSecureDDRAccess() void {
                Bus.AHB4.ports().RCC.regs().MP_APB5ENSETR.fields().TZC1EN.setEnumValue(.Set);
                Bus.AHB4.ports().RCC.regs().MP_APB5ENSETR.fields().TZC2EN.setEnumValue(.Set);
                const GATE_KEEPER = port.regs().GATE_KEEPER.fields();
                GATE_KEEPER.OPENREQ_FLT0.setEnumValue(.Opened);
                GATE_KEEPER.OPENREQ_FLT1.setEnumValue(.Opened);
                port.regs().ID_ACCESS0.fields().NSAID_WR_EN.set(0xFFFF);
                port.regs().ID_ACCESS0.fields().NSAID_RD_EN.set(0xFFFF);
                port.regs().ATTRIBUTE0.fields().S_WR_EN.setEnumValue(.Permitted);
                port.regs().ATTRIBUTE0.fields().S_RD_EN.setEnumValue(.Permitted);
                GATE_KEEPER.OPENREQ_FLT0.setEnumValue(.Closed);
                GATE_KEEPER.OPENREQ_FLT1.setEnumValue(.Closed);
                Bus.AHB4.ports().RCC.regs().MP_APB5ENSETR.fields().TZPCEN.setEnumValue(.Set);
                port.regs().SPECULATION_CTRL.fields().WRITESPEC_DISABLE.setEnumValue(.Asserted);
                port.regs().SPECULATION_CTRL.fields().READSPEC_DISABLE.setEnumValue(.Asserted);
            }
        },
        ddr => enum(bus_type) {
            pub fn regs() enum_type {
                return port.regs();
            }

            pub fn init(comptime reg_values: REGS_VALUES) bool {
                const udelay = Bus.AHB4.ports().RCC.api().udelay;
                const DDRITFCR = Bus.AHB4.ports().RCC.regs().DDRITFCR.fields();

                DDRITFCR.AXIDCGEN.setEnumValue(.Disabled);

                DDRITFCR.DDRCAPBRST.setEnumValue(.Asserted);
                DDRITFCR.DDRCAXIRST.setEnumValue(.Asserted);
                DDRITFCR.DDRCORERST.setEnumValue(.Asserted);
                DDRITFCR.DPHYAPBRST.setEnumValue(.Asserted);
                DDRITFCR.DPHYRST.setEnumValue(.Asserted);
                DDRITFCR.DPHYCTLRST.setEnumValue(.Asserted);

                DDRITFCR.DDRC1EN.setEnumValue(.Enabled);
                DDRITFCR.DDRC2EN.setEnumValue(.Enabled);
                DDRITFCR.DDRCAPBEN.setEnumValue(.Enabled);
                DDRITFCR.DDRPHYCAPBEN.setEnumValue(.Enabled);
                DDRITFCR.DDRPHYCEN.setEnumValue(.Enabled);

                DDRITFCR.DPHYRST.setEnumValue(.Cleared);
                DDRITFCR.DPHYCTLRST.setEnumValue(.Cleared);
                DDRITFCR.DDRCAPBRST.setEnumValue(.Cleared);

                udelay(2);

                regs().DFIMISC.fields().DFI_INIT_COMPLETE_EN.setEnumValue(.Cleared);
                setRegs(reg_values.ctrl.reg);
                setRegs(reg_values.ctrl.timing);
                setRegs(reg_values.ctrl.map);
                regs().INIT0.fields().SKIP_DRAM_INIT.setEnumValue(.InitSkippedNormal);
                setRegs(reg_values.ctrl.perf);

                DDRITFCR.DDRCORERST.setEnumValue(.Cleared);
                DDRITFCR.DDRCAXIRST.setEnumValue(.Cleared);
                DDRITFCR.DPHYAPBRST.setEnumValue(.Cleared);

                setRegs(reg_values.phy.reg);
                setRegs(reg_values.phy.timing);

                if (!ddrphyInitWait())
                    return false;

                const PIR = regs().PIR.fields();
                var pir = PIR.DLLSRST.enumToInt(.Asserted);
                pir |= PIR.DLLLOCK.enumToInt(.Asserted);
                pir |= PIR.ZCAL.enumToInt(.Asserted);
                pir |= PIR.ITMSRST.enumToInt(.Asserted);
                pir |= PIR.DRAMINIT.enumToInt(.Asserted);
                pir |= PIR.ICPC.enumToInt(.PhyAndPubl);
                pir |= PIR.INIT.enumToInt(.Asserted);
                if (regs().MSTR.fields().DDR3.getEnumValue() == .DDR3) {
                    pir |= PIR.DRAMRST.enumToInt(.Asserted);
                }
                regs().PIR.ptr().* = pir;
                udelay(10);
                if (!ddrphyInitWait())
                    return false;

                startSwDone();
                regs().DFIMISC.fields().DFI_INIT_COMPLETE_EN.setEnumValue(.Asserted);
                waitSwDone();

                normalOpModeWait();

                startSwDone();
                regs().RFSHCTL3.fields().DIS_AUTO_REFRESH.setEnumValue(.Asserted);
                regs().PWRCTL.fields().SELFREF_EN.setEnumValue(.Disabled);
                regs().PWRCTL.fields().POWERDOWN_EN.setEnumValue(.Disabled);
                regs().DFIMISC.fields().DFI_INIT_COMPLETE_EN.setEnumValue(.Cleared);
                waitSwDone();

                pir = PIR.QSTRN.enumToInt(.Asserted);
                pir |= PIR.INIT.enumToInt(.Asserted);
                if (regs().MSTR.fields().DDR3.getEnumValue() != .DDR3) {
                    pir |= PIR.RVTRN.enumToInt(.Asserted);
                }
                regs().PIR.ptr().* = pir;
                udelay(10);

                if (!ddrphyInitWait())
                    return false;

                startSwDone();
                if (regs().RFSHCTL3.fields().DIS_AUTO_REFRESH.intToEnum(findRegValue(reg_values.ctrl.reg, regs().RFSHCTL3)) == .Cleared)
                    regs().RFSHCTL3.fields().DIS_AUTO_REFRESH.setEnumValue(.Cleared);
                if (regs().PWRCTL.fields().POWERDOWN_EN.intToEnum(findRegValue(reg_values.ctrl.reg, regs().PWRCTL)) == .Enabled)
                    regs().PWRCTL.fields().POWERDOWN_EN.setEnumValue(.Enabled);
                if (regs().PWRCTL.fields().SELFREF_EN.intToEnum(findRegValue(reg_values.ctrl.reg, regs().PWRCTL)) == .Enabled)
                    regs().PWRCTL.fields().SELFREF_EN.setEnumValue(.Enabled);
                regs().DFIMISC.fields().DFI_INIT_COMPLETE_EN.setEnumValue(.Asserted);
                waitSwDone();

                if (regs().PWRCTL.fields().EN_DFI_DRAM_CLK_DISABLE.intToEnum(findRegValue(reg_values.ctrl.reg, regs().PWRCTL)) != .Cleared) {
                    DDRITFCR.DDRCKMOD.setEnumValue(.AutoSelfRef);
                    startSwDone();
                    regs().HWLPCTL.fields().HW_LP_EN.setEnumValue(.Enabled);
                    regs().PWRTMG.fields().POWERDOWN_TO_X32.set(0x10);
                    regs().PWRTMG.fields().SELFREF_TO_X32.set(0x1);
                    regs().PWRCTL.fields().EN_DFI_DRAM_CLK_DISABLE.setEnumValue(.Asserted);
                    if (regs().PWRCTL.fields().SELFREF_EN.intToEnum(findRegValue(reg_values.ctrl.reg, regs().PWRCTL)) == .Enabled) {
                        regs().PWRCTL.fields().SELFREF_EN.setEnumValue(.Enabled);
                    }
                    regs().DFIMISC.fields().DFI_INIT_COMPLETE_EN.setEnumValue(.Asserted);
                    waitSwDone();
                }

                regs().PCTRL_0.fields().PORT_EN.setEnumValue(.Enabled);
                regs().PCTRL_1.fields().PORT_EN.setEnumValue(.Enabled);

                DDRITFCR.AXIDCGEN.setEnumValue(.Enabled);

                return true;
            }

            fn findRegValue(comptime reg_values: anytype, comptime reg: anytype) bus_type {
                for (reg_values) |reg_value| {
                    if (@intFromPtr(reg_value.ptr) == @intFromPtr(reg.ptr())) {
                        return reg_value.value;
                    }
                }
                return 0;
            }

            inline fn startSwDone() void {
                regs().SWCTL.fields().SW_DONE.setEnumValue(.Enable);
            }
            inline fn waitSwDone() void {
                regs().SWCTL.fields().SW_DONE.setEnumValue(.Disable);
                while (regs().SWSTAT.fields().SW_DONE_ACK.getEnumValue() != .Done) {}
            }

            inline fn normalOpModeWait() void {
                while (true) {
                    if (regs().STAT.fields().OPERATING_MODE.getEnumValue() == .Normal)
                        return;
                    if (regs().STAT.fields().OPERATING_MODE.getEnumValue() == .SelfRefresh and regs().STAT.fields().SELFREF_TYPE.getEnumValue() == .AutoSelfRefresh)
                        return;
                }
            }

            inline fn ddrphyInitWait() bool {
                while (true) {
                    if (regs().PGSR.fields().DTERR.getEnumValue() == .Error)
                        return false;
                    if (regs().PGSR.fields().DTIERR.getEnumValue() == .Error)
                        return false;
                    if (regs().PGSR.fields().DFTERR.getEnumValue() == .Error)
                        return false;
                    if (regs().PGSR.fields().RVERR.getEnumValue() == .Error)
                        return false;
                    if (regs().PGSR.fields().RVIERR.getEnumValue() == .Error)
                        return false;
                    if (regs().PGSR.fields().IDONE.getEnumValue() == .Done)
                        return true;
                }
            }

            inline fn setRegs(comptime reg_values: anytype) void {
                for (reg_values) |reg_value| {
                    reg_value.ptr.* = reg_value.value;
                }
            }
            pub const REG_VALUE = struct {
                ptr: *volatile bus_type,
                value: bus_type,
            };
            pub const REGS_VALUES = struct {
                ctrl: struct {
                    reg: []const *const REG_VALUE,
                    map: []const *const REG_VALUE,
                    timing: []const *const REG_VALUE,
                    perf: []const *const REG_VALUE,
                },
                phy: struct {
                    reg: []const *const REG_VALUE,
                    timing: []const *const REG_VALUE,
                },
            };
            pub fn generateRegValue(reg: port.regs(), value: u32) *const REG_VALUE {
                return comptime &REG_VALUE{ .ptr = reg.ptr(), .value = value };
            }
        },

        rcc => enum(bus_type) {
            inline fn resetCycleCounter() void {
                var value: u32 = undefined;
                asm volatile (
                    \\ mrc p15, 0, %[value], c9, c12, 0
                    : [value] "=r" (value),
                ); // read
                value |= (1 << 2) | 1;
                asm volatile (
                    \\ mcr p15, 0, %[value], c9, c12, 0
                    :
                    : [value] "r" (value),
                    : "memory"
                ); // write modified
            }
            inline fn readCycleCounter() u32 {
                var value: u32 = undefined;
                asm volatile (
                    \\ mrc p15, 0, %[value], c9, c13, 0
                    : [value] "=r" (value),
                );
                return value;
            }
            pub fn udelay(usec: u32) void {
                if (!cycle_counter_enabled)
                    enableCycleCounter();
                resetCycleCounter();
                const delay = getSystemClockHz() / 1_000_000 * usec;
                while (readCycleCounter() < delay) {}
            }

            inline fn enableCycleCounter() void {
                var value: u32 = undefined;
                asm volatile (
                    \\ mrc p15, 0, %[result], c9, c12, 1
                    : [result] "=r" (value),
                );
                value |= 0x80000000;
                asm volatile (
                    \\ mcr p15, 0, %[reg], c9, c12, 1
                    :
                    : [reg] "r" (value),
                    : "memory"
                );
                cycle_counter_enabled = true;
            }
            pub const EXT_CLOCK_MODE = enum { Crystal };
            pub const LSE = struct {
                pub fn init(comptime mode: EXT_CLOCK_MODE) void { // RM0436 Rev 6, p.531
                    _ = mode;
                    const PWR = API(Bus.AHB4.ports().PWR);
                    const BDCR = port.regs().BDCR.fields();
                    PWR.disableBackupDomainWriteProtection();
                    BDCR.LSEON.set(BDCR.LSEON.values.Off);
                    while (BDCR.LSERDY.getEnumValue() == BDCR.LSERDY.values.Ready) {}
                    BDCR.LSEBYP.set(BDCR.LSEBYP.values.NotBypassed);
                    BDCR.LSEON.set(BDCR.LSEON.values.On);
                    while (BDCR.LSERDY.getEnumValue() != BDCR.LSERDY.values.Ready) {}
                    PWR.enableBackupDomainWriteProtection();
                }
            };
            pub const HSE = struct {
                pub fn init(comptime mode: EXT_CLOCK_MODE) void { // EM0436 Rev 6, p.526
                    _ = mode;
                    const OCENCLRR = port.regs().OCENCLRR.fields();
                    const HSEON = port.regs().OCENSETR.fields().HSEON;
                    const HSERDY = port.regs().OCRDYR.fields().HSERDY;
                    OCENCLRR.HSEON.set(OCENCLRR.HSEON.values.Clear);
                    while (HSERDY.getEnumValue() == HSERDY.values.Ready) {}
                    OCENCLRR.HSEBYP.set(OCENCLRR.HSEBYP.values.Clear);
                    HSEON.set(HSEON.values.Set);
                    while (HSERDY.getEnumValue() != HSERDY.values.Ready) {}
                }
            };
            pub const HSI = enum {
                pub fn disable() void {
                    const OCENCLRR = port.regs().OCENCLRR.fields();
                    OCENCLRR.HSION.set(OCENCLRR.HSION.values.Clear);
                }
            };
            pub const AXI = enum {
                pub fn setDividers(comptime axi: u3, comptime apb4: u3, comptime apb5: u3) void {
                    const AXIDIVR = port.regs().AXIDIVR.fields();
                    const APB4DIVR = port.regs().APB4DIVR.fields();
                    const APB5DIVR = port.regs().APB5DIVR.fields();

                    AXIDIVR.AXIDIV.set(axi);
                    while (AXIDIVR.AXIDIVRDY.getEnumValue() != AXIDIVR.AXIDIVRDY.values.Ready) {}

                    APB4DIVR.APB4DIV.set(apb4);
                    while (APB4DIVR.APB4DIVRDY.getEnumValue() != APB4DIVR.APB4DIVRDY.values.Ready) {}

                    APB5DIVR.APB5DIV.set(apb5);
                    while (APB5DIVR.APB5DIVRDY.getEnumValue() != APB5DIVR.APB5DIVRDY.values.Ready) {}
                }
            };
            pub const MUX = enum {
                PLL12,
                PLL3,
                PLL4,
                MPU,
                AXI,
                MCU,
                pub fn source(comptime mux: MUX) enum_type {
                    return comptime switch (mux) {
                        .PLL12 => enum(u2) { HSI = 0, HSE = 1, Off },
                        .PLL3 => enum(u2) { HSI = 0, HSE = 1, CSI = 2, Off = 3 },
                        .PLL4 => enum(u2) { HSI = 0, HSE = 1, CSI = 2, EXT_I2S = 3 },
                        .MPU => enum(u2) { HSI = 0, HSE = 1, PLL1 = 2, PLL1DIV = 3 },
                        .AXI => enum(u2) { HSI = 0, HSE = 1, PLL2 = 2 },
                        .MCU => enum(u2) { HSI = 0, HSE = 1, CSI = 2, PLL3 = 3 },
                    };
                }
                pub fn setSource(comptime self: MUX, comptime src: anytype) void {
                    const SRC = comptime switch (self) {
                        .PLL12 => port.regs().RCK12SELR.fields().PLL12SRC,
                        .PLL3 => port.regs().RCK3SELR.fields().PLL3SRC,
                        .PLL4 => port.regs().RCK4SELR.fields().PLL4SRC,
                        .MPU => port.regs().MPCKSELR.fields().MPUSRC,
                        .AXI => port.regs().ASSCKSELR.fields().AXISSRC,
                        .MCU => port.regs().MSSCKSELR.fields().MCUSSRC,
                    };
                    const RDY = comptime switch (self) {
                        .PLL12 => port.regs().RCK12SELR.fields().PLL12SRCRDY,
                        .PLL3 => port.regs().RCK3SELR.fields().PLL3SRCRDY,
                        .PLL4 => port.regs().RCK4SELR.fields().PLL4SRCRDY,
                        .MPU => port.regs().MPCKSELR.fields().MPUSRCRDY,
                        .AXI => port.regs().ASSCKSELR.fields().AXISSRCRDY,
                        .MCU => port.regs().MSSCKSELR.fields().MCUSSRCRDY,
                    };
                    SRC.set(src);
                    while (RDY.getEnumValue() != RDY.values.Ready) {}

                    switch (self) {
                        .PLL12, .MPU => {
                            system_clock_hz = calculateSystemClockHz();
                        },
                        else => {},
                    }
                }
            };
            inline fn getSystemClockHz() u32 {
                return system_clock_hz;
            }
            fn calculateSystemClockHz() u32 {
                const PLL12SRC = port.regs().RCK12SELR.fields().PLL12SRC;
                const PLL12SRCRDY = port.regs().RCK12SELR.fields().PLL12SRCRDY;
                const HSIDIV = port.regs().HSICFGR.fields().HSIDIV;
                const MPUSRC = port.regs().MPCKSELR.fields().MPUSRC;
                const MPUSRCRDY = port.regs().MPCKSELR.fields().MPUSRCRDY;
                while (MPUSRCRDY.getEnumValue() != MPUSRCRDY.values.Ready) {}
                const MPUMUX = MPUSRC.getEnumValue();
                switch (MPUMUX) {
                    .HSI => {
                        return HSI_FRIQUENCY;
                    },
                    .HSE => {
                        return HSE_FRIQUENCY;
                    },
                    .PLL1, .PLL1DIV => {
                        while (PLL12SRCRDY.getEnumValue() != PLL12SRCRDY.values.Ready) {}
                        const PLLCFG1R = port.regs().PLL1CFGR1.fields();
                        const MPUDIVR = port.regs().MPCKDIVR.fields();
                        const divm = PLLCFG1R.DIVM.getIntValue();
                        const divn = PLLCFG1R.DIVN.getIntValue();
                        const frac = port.regs().PLL1FRACR.fields().FRACV.getIntValue();
                        const divp = port.regs().PLL1CFGR2.fields().DIVP.getIntValue();
                        const ref: u32 = switch (PLL12SRC.getEnumValue()) {
                            .HSE => HSE_FRIQUENCY,
                            .HSI => @as(u32, HSI_FRIQUENCY) >> @truncate(HSIDIV.getIntValue()),
                            .Off => return 0,
                        } / (divm + 1) * 2;

                        const result: u32 = ((ref / 1_000 * frac / 8192) * 1_000 + ref * (divn + 1)) / 2 / (divp + 1);

                        switch (MPUMUX) {
                            .PLL1DIV => {
                                switch (MPUDIVR.MPUDIV.getEnumValue()) {
                                    .Off => return 0,
                                    else => return result >> @truncate(MPUDIVR.MPUDIV.getIntValue()),
                                }
                            },
                            else => return result,
                        }
                    },
                }
            }

            pub const PLL = enum {
                PLL1,
                PLL2,
                PLL3,
                PLL4,
                pub const OUTPUT = enum { P, Q, R };
                // as per RM0436 Rev 6, p.621
                pub fn enableOutput(comptime pll: @This(), comptime divout: OUTPUT) void {
                    const DIVEN = comptime switch (divout) {
                        .P => getCR(pll).DIVPEN,
                        .Q => getCR(pll).DIVQEN,
                        .R => getCR(pll).DIVREN,
                    };
                    DIVEN.set(DIVEN.values.Enabled);
                }
                // as per RM0436 Rev 6, p.621
                pub fn setDividers(comptime pll: PLL, comptime m: u6, comptime n: u9, comptime frac: u16, comptime p: u7, comptime q: u7, comptime r: u7) void {
                    const PLLCFG1R = comptime switch (pll) {
                        .PLL1 => port.regs().PLL1CFGR1.fields(),
                        .PLL2 => port.regs().PLL2CFGR1.fields(),
                        .PLL3 => port.regs().PLL3CFGR1.fields(),
                        .PLL4 => port.regs().PLL4CFGR1.fields(),
                    };
                    const PLLCFG2R = comptime switch (pll) {
                        .PLL1 => port.regs().PLL1CFGR2.fields(),
                        .PLL2 => port.regs().PLL2CFGR2.fields(),
                        .PLL3 => port.regs().PLL3CFGR2.fields(),
                        .PLL4 => port.regs().PLL4CFGR2.fields(),
                    };
                    const PLLFRACR = comptime switch (pll) {
                        .PLL1 => port.regs().PLL1FRACR.fields(),
                        .PLL2 => port.regs().PLL2FRACR.fields(),
                        .PLL3 => port.regs().PLL3FRACR.fields(),
                        .PLL4 => port.regs().PLL4FRACR.fields(),
                    };
                    PLLCFG1R.DIVM.set(m);
                    PLLFRACR.FRACLE.set(0);
                    PLLFRACR.FRACV.set(frac);
                    PLLFRACR.FRACLE.set(1);
                    PLLCFG1R.DIVN.set(n);
                    PLLCFG2R.DIVP.set(p);
                    PLLCFG2R.DIVQ.set(q);
                    PLLCFG2R.DIVR.set(r);
                    const PLLCR = getCR(pll);
                    PLLCR.PLLON.set(PLLCR.PLLON.values.On);
                    while (PLLCR.PLLRDY.getEnumValue() != PLLCR.PLLRDY.values.Locked) {}
                }
                fn getCR(comptime pll: @This()) enum_type {
                    return comptime switch (pll) {
                        .PLL1 => port.regs().PLL1CR.fields(),
                        .PLL2 => port.regs().PLL2CR.fields(),
                        .PLL3 => port.regs().PLL3CR.fields(),
                        .PLL4 => port.regs().PLL4CR.fields(),
                    };
                }
            };
        },

        pwr => enum(bus_type) {
            const DBP = port.regs().CR1.fields().DBP;
            pub fn disableBackupDomainWriteProtection() void {
                DBP.set(DBP.values.Enabled);
            }
            pub fn enableBackupDomainWriteProtection() void {
                DBP.set(DBP.values.Disabled);
            }
        },

        gpioa, gpiob, gpiog => enum(bus_type) {
            pub fn pin(comptime PIN: u4) enum_type {
                return comptime enum {
                    pub const MODE = enum(u2) { Input = 0, Output = 1, AltFunc = 2, Analog = 3 };
                    pub const OTYPE = enum(u1) { PushPull = 0, OpenDrain = 1 };
                    pub const OSPEED = enum(u2) { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };
                    pub const PUPD = enum(u2) { Disabled = 0, PullUp = 1, PullDown = 2, Reserved = 3 };
                    pub fn configure(comptime mode: MODE, comptime otype: OTYPE, comptime ospeed: OSPEED, comptime pupd: PUPD, comptime af: u4) void {
                        MODER.set(mode);
                        OTYPER.set(otype);
                        OSPEEDR.set(ospeed);
                        PUPDR.set(pupd);
                        if (mode == .AltFunc)
                            AFR.set(af);
                    }
                    pub fn set() void {
                        BSR.set(1);
                    }
                    pub fn reset() void {
                        BRR.set(1);
                    }
                    // private
                    const Pin: bus_type = PIN;
                    const MODER = Field{ .reg = Reg(0x0, port), .rw = .ReadWrite, .shift = Pin * 2, .width = u2, .values = MODE };
                    const OTYPER = Field{ .reg = Reg(0x4, port), .rw = .ReadWrite, .shift = Pin, .width = u1, .values = OTYPE };
                    const OSPEEDR = Field{ .reg = Reg(0x8, port), .rw = .ReadWrite, .shift = Pin * 2, .width = u2, .values = OSPEED };
                    const PUPDR = Field{ .reg = Reg(0xC, port), .rw = .ReadWrite, .shift = Pin * 2, .width = u2, .values = PUPD };
                    const AFR = Field{ .reg = Reg(0x20 + (Pin / 8) * 0x4, port), .rw = .ReadWrite, .shift = (Pin % 8) * 4, .width = u4, .values = enum {} };
                    const BSR = Field{ .reg = Reg(0x18, port), .rw = .WriteOnly, .shift = Pin, .width = u1, .values = enum {} };
                    const BRR = Field{ .reg = Reg(0x18, port), .rw = .WriteOnly, .shift = (Pin + 0x10), .width = u1, .values = enum {} };

                    pub fn enableGPIOclocks() void {
                        port.api().CLOCKS.enable();
                    }

                    pub fn disableGPIOclocks() void {
                        port.api().CLOCKS.disable();
                    }
                };
            }
            pub const CLOCKS = struct {
                // TODO: manage MCU as weel as MPU
                pub fn enable() void {
                    const en_reg = Bus.AHB4.ports().RCC.regs().MP_AHB4ENSETR.fields();
                    const en = comptime switch (current_port) {
                        gpiob => en_reg.GPIOBEN,
                        gpiog => en_reg.GPIOGEN,
                        else => unreachable,
                    };
                    en.set(en.values.Set);
                }
                pub fn disable() void {
                    const dis_reg = Bus.AHB4.ports().RCC.regs().MP_AHB4ENCLRR.fields();
                    const dis = comptime switch (current_port) {
                        gpiob => dis_reg.GPIOBEN,
                        gpiog => dis_reg.GPIOGEN,
                        else => unreachable,
                    };
                    dis.set(dis.values.Set);
                }
            };
        },

        uart4 => enum(bus_type) {
            pub fn init() void {
                // TODO: parametrise as per baud rate, parity, bits, stop bits, clock source, etc.
                // currently configuring as HSI, 115200, 8N1, FIFO
                const HSIKERON = Bus.AHB4.ports().RCC.regs().OCENSETR.fields().HSIKERON;
                const HSIDIVRDY = Bus.AHB4.ports().RCC.regs().OCRDYR.fields().HSIDIVRDY;
                const HSIDIV = Bus.AHB4.ports().RCC.regs().HSICFGR.fields().HSIDIV;
                const UARTSRC = switch (port) {
                    .UART4 => Bus.AHB4.ports().RCC.regs().UART24CKSELR.fields().UART24SRC,
                };
                const UARTEN = switch (port) {
                    .UART4 => Bus.AHB4.ports().RCC.regs().MP_APB1ENSETR.fields().UART4EN,
                };

                const CR1 = port.regs().CR1.fields();
                const CR2 = port.regs().CR2.fields();
                const PRESCALER = port.regs().PRESC.fields().PRESCALER;
                const BRR = port.regs().BRR;

                HSIKERON.set(HSIKERON.values.Set); // keep HSI running in LP mode
                UARTSRC.set(UARTSRC.values.HSI); // clock source for UART
                UARTEN.set(UARTEN.values.Set); // enable clock delivery to periph.

                CR1.UE.set(CR1.UE.values.DisabledLPmode);
                CR1.M0.set(CR1.M0.values.DataBits7or8);
                CR1.M1.set(CR1.M1.values.DataBits8or9);
                CR1.OVER8.set(CR1.OVER8.values.Oversampling8);
                CR1.PCE.set(CR1.PCE.values.Disabled);
                CR1.TE.set(CR1.TE.values.Enabled);
                CR1.RE.set(CR1.RE.values.Enabled);
                CR1.FIFOEN.set(CR1.FIFOEN.values.Enabled);
                CR2.STOP.set(CR2.STOP.values.One);
                while (HSIDIVRDY.getEnumValue() != HSIDIVRDY.values.Ready) {}

                const baudrate: u32 = 115200;
                const hsi_clock_fq: u32 = switch (HSIDIV.getEnumValue()) {
                    HSIDIV.values.One => HSI_FRIQUENCY,
                    HSIDIV.values.Two => HSI_FRIQUENCY / 2,
                    HSIDIV.values.Four => HSI_FRIQUENCY / 4,
                    HSIDIV.values.Eight => HSI_FRIQUENCY / 8,
                };
                const prescaler: u32 = 16;
                const brr: u32 = ((2 * hsi_clock_fq / prescaler) + (baudrate / 2)) / baudrate;
                BRR.fields().BRR3_0.set((brr & 0xF) >> 1);
                BRR.fields().BRR15_4.set((brr & 0xFFF0) >> 4);
                PRESCALER.set(PRESCALER.values.Div16);

                CR1.UE.set(CR1.UE.values.Enabled);
            }

            pub fn write(bytes: []const u8) usize {
                const TXFNF = port.regs().ISR.fields().TXFNF;
                const TDR = port.regs().TDR.fields().TDR;
                for (bytes) |byte| {
                    while (TXFNF.getEnumValue() == TXFNF.values.Full) {}
                    TDR.set(byte);
                }
                return bytes.len;
            }
        },
        else => unreachable,
    };
}

pub const Bus = enum(bus_type) {
    APB5 = 0x5C000000,
    APB4 = 0x5A000000,
    AHB4 = 0x50000000,
    APB1 = 0x40000000,

    pub fn ports(bus: @This()) enum_type {
        return comptime switch (bus) {
            .APB4 => enum(bus_type) {
                pub fn api(port: @This()) enum_type {
                    return API(port);
                }
                DDRCTRL = Port(0x3000, bus),
                DDRPHYC = Port(0x4000, bus),
                DDR = Port(0xFFFFFF, bus),

                fn regs(port: @This()) enum_type {
                    const CTRL = bus.ports().DDRCTRL;
                    const PHY = bus.ports().DDRPHYC;
                    return comptime switch (port) {
                        .DDR => enum(bus_type) {
                            fn ptr(comptime reg: @This()) *volatile u32 {
                                return @ptrFromInt(@intFromEnum(reg));
                            }
                            MSTR = Reg(0x0, CTRL),
                            STAT = Reg(0x4, CTRL),
                            MRCTRL0 = Reg(0x10, CTRL),
                            MRCTRL1 = Reg(0x14, CTRL),
                            DERATEEN = Reg(0x20, CTRL),
                            DERATEINT = Reg(0x24, CTRL),
                            PWRCTL = Reg(0x30, CTRL),
                            PWRTMG = Reg(0x34, CTRL),
                            HWLPCTL = Reg(0x38, CTRL),
                            RFSHCTL0 = Reg(0x50, CTRL),
                            RFSHCTL3 = Reg(0x60, CTRL),
                            RFSHTMG = Reg(0x64, CTRL),
                            CRCPARCTL0 = Reg(0xC0, CTRL),
                            INIT0 = Reg(0xD0, CTRL),
                            DRAMTMG0 = Reg(0x100, CTRL),
                            DRAMTMG1 = Reg(0x104, CTRL),
                            DRAMTMG2 = Reg(0x108, CTRL),
                            DRAMTMG3 = Reg(0x10C, CTRL),
                            DRAMTMG4 = Reg(0x110, CTRL),
                            DRAMTMG5 = Reg(0x114, CTRL),
                            DRAMTMG6 = Reg(0x118, CTRL),
                            DRAMTMG7 = Reg(0x11C, CTRL),
                            DRAMTMG8 = Reg(0x120, CTRL),
                            DRAMTMG14 = Reg(0x138, CTRL),
                            ZQCTL0 = Reg(0x180, CTRL),
                            DFITMG0 = Reg(0x190, CTRL),
                            DFITMG1 = Reg(0x194, CTRL),
                            DFILPCFG0 = Reg(0x198, CTRL),
                            DFIUPD0 = Reg(0x1A0, CTRL),
                            DFIUPD1 = Reg(0x1A4, CTRL),
                            DFIUPD2 = Reg(0x1A8, CTRL),
                            DFIMISC = Reg(0x1B0, CTRL),
                            DFIPHYMSTR = Reg(0x1C4, CTRL),
                            ADDRMAP1 = Reg(0x204, CTRL),
                            ADDRMAP2 = Reg(0x208, CTRL),
                            ADDRMAP3 = Reg(0x20C, CTRL),
                            ADDRMAP4 = Reg(0x210, CTRL),
                            ADDRMAP5 = Reg(0x214, CTRL),
                            ADDRMAP6 = Reg(0x218, CTRL),
                            ADDRMAP9 = Reg(0x224, CTRL),
                            ADDRMAP10 = Reg(0x228, CTRL),
                            ADDRMAP11 = Reg(0x22C, CTRL),
                            ODTCFG = Reg(0x240, CTRL),
                            ODTMAP = Reg(0x244, CTRL),
                            SCHED = Reg(0x250, CTRL),
                            SCHED1 = Reg(0x254, CTRL),
                            PERFHPR1 = Reg(0x25C, CTRL),
                            PERFLPR1 = Reg(0x264, CTRL),
                            PERFWR1 = Reg(0x26C, CTRL),
                            DBG0 = Reg(0x300, CTRL),
                            DBG1 = Reg(0x304, CTRL),
                            DBGCMD = Reg(0x30C, CTRL),
                            SWCTL = Reg(0x320, CTRL),
                            SWSTAT = Reg(0x324, CTRL),
                            POISONCFG = Reg(0x36C, CTRL),
                            PCCFG = Reg(0x400, CTRL),
                            PCFGR_0 = Reg(0x404, CTRL),
                            PCFGW_0 = Reg(0x408, CTRL),
                            PCTRL_0 = Reg(0x490, CTRL),
                            PCFGQOS0_0 = Reg(0x494, CTRL),
                            PCFGQOS1_0 = Reg(0x498, CTRL),
                            PCFGWQOS0_0 = Reg(0x49C, CTRL),
                            PCFGWQOS1_0 = Reg(0x4A0, CTRL),
                            PCFGR_1 = Reg(0x404 + 0xB0, CTRL),
                            PCFGW_1 = Reg(0x408 + 0xB0, CTRL),
                            PCTRL_1 = Reg(0x490 + 0xB0, CTRL),
                            PCFGQOS0_1 = Reg(0x494 + 0xB0, CTRL),
                            PCFGQOS1_1 = Reg(0x498 + 0xB0, CTRL),
                            PCFGWQOS0_1 = Reg(0x49C + 0xB0, CTRL),
                            PCFGWQOS1_1 = Reg(0x4A0 + 0xB0, CTRL),

                            PIR = Reg(0x4, PHY),
                            PGCR = Reg(0x8, PHY),
                            PGSR = Reg(0xC, PHY),
                            PTR0 = Reg(0x18, PHY),
                            PTR1 = Reg(0x1C, PHY),
                            PTR2 = Reg(0x20, PHY),
                            ACIOCR = Reg(0x24, PHY),
                            DXCCR = Reg(0x28, PHY),
                            DSGCR = Reg(0x2C, PHY),
                            DCR = Reg(0x30, PHY),
                            DTPR0 = Reg(0x34, PHY),
                            DTPR1 = Reg(0x38, PHY),
                            DTPR2 = Reg(0x3C, PHY),
                            MR0 = Reg(0x40, PHY),
                            MR1 = Reg(0x44, PHY),
                            MR2 = Reg(0x48, PHY),
                            MR3 = Reg(0x4C, PHY),
                            ODTCR = Reg(0x50, PHY),
                            ZQ0CR1 = Reg(0x184, PHY),
                            DX0GCR = Reg(0x1C0, PHY),
                            DX1GCR = Reg(0x200, PHY),
                            DX2GCR = Reg(0x240, PHY),
                            DX3GCR = Reg(0x280, PHY),
                            fn fields(reg: @This()) enum_type {
                                const addr = @intFromEnum(reg);
                                return comptime switch (reg) {
                                    .PCTRL_0, .PCTRL_1 => enum {
                                        const PORT_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                    },
                                    .PWRTMG => enum {
                                        const POWERDOWN_TO_X32 = Field{ .rw = .ReadWrite, .width = u5, .shift = 0, .reg = addr, .values = enum {} };
                                        const SELFREF_TO_X32 = Field{ .rw = .ReadWrite, .width = u8, .shift = 16, .reg = addr, .values = enum {} };
                                    },
                                    .HWLPCTL => enum {
                                        const HW_LP_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                    },
                                    .PWRCTL => enum {
                                        const EN_DFI_DRAM_CLK_DISABLE = Field{ .rw = .ReadWrite, .width = u1, .shift = 3, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const POWERDOWN_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const SELFREF_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                    },
                                    .RFSHCTL3 => enum {
                                        const DIS_AUTO_REFRESH = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                    },
                                    .STAT => enum {
                                        const OPERATING_MODE = Field{ .rw = .ReadOnly, .width = u3, .shift = 0, .reg = addr, .values = enum(u3) {
                                            Init = 0,
                                            Normal = 1,
                                            PowerDown = 2,
                                            SelfRefresh = 3,
                                        } };
                                        const SELFREF_TYPE = Field{ .rw = .ReadOnly, .width = u2, .shift = 4, .reg = addr, .values = enum(u2) {
                                            NonSelfRefresh = 0,
                                            PhySelfRefresh = 1,
                                            SelfRefresh = 2,
                                            AutoSelfRefresh = 3,
                                        } };
                                    },
                                    .SWSTAT => enum {
                                        const SW_DONE_ACK = Field{ .rw = .ReadOnly, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            InProgress = 0,
                                            Done = 1,
                                        } };
                                    },
                                    .SWCTL => enum {
                                        const SW_DONE = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Enable = 0,
                                            Disable = 1,
                                        } };
                                    },
                                    .MSTR => enum {
                                        const DDR3 = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            NonDDR3 = 0,
                                            DDR3 = 1,
                                        } };
                                    },
                                    .PIR => enum {
                                        const ICPC = Field{ .rw = .WriteOnly, .width = u1, .shift = 16, .reg = addr, .values = enum(u1) {
                                            PhyOnly = 0,
                                            PhyAndPubl = 1,
                                        } };
                                        const RVTRN = Field{ .rw = .WriteOnly, .width = u1, .shift = 8, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const QSTRN = Field{ .rw = .WriteOnly, .width = u1, .shift = 7, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const DRAMINIT = Field{ .rw = .WriteOnly, .width = u1, .shift = 6, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const DRAMRST = Field{ .rw = .WriteOnly, .width = u1, .shift = 5, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const ITMSRST = Field{ .rw = .WriteOnly, .width = u1, .shift = 4, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const ZCAL = Field{ .rw = .WriteOnly, .width = u1, .shift = 3, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const DLLLOCK = Field{ .rw = .WriteOnly, .width = u1, .shift = 2, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const DLLSRST = Field{ .rw = .WriteOnly, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const INIT = Field{ .rw = .WriteOnly, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                    },
                                    .PGSR => enum {
                                        const IDONE = Field{ .rw = .ReadOnly, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Done = 1,
                                        } };
                                        const DTERR = Field{ .rw = .ReadOnly, .width = u1, .shift = 5, .reg = addr, .values = enum(u1) {
                                            Ok = 0,
                                            Error = 1,
                                        } };
                                        const DTIERR = Field{ .rw = .ReadOnly, .width = u1, .shift = 6, .reg = addr, .values = enum(u1) {
                                            Ok = 0,
                                            Error = 1,
                                        } };
                                        const DFTERR = Field{ .rw = .ReadOnly, .width = u1, .shift = 7, .reg = addr, .values = enum(u1) {
                                            Ok = 0,
                                            Error = 1,
                                        } };
                                        const RVERR = Field{ .rw = .ReadOnly, .width = u1, .shift = 8, .reg = addr, .values = enum(u1) {
                                            Ok = 0,
                                            Error = 1,
                                        } };
                                        const RVIERR = Field{ .rw = .ReadOnly, .width = u1, .shift = 9, .reg = addr, .values = enum(u1) {
                                            Ok = 0,
                                            Error = 1,
                                        } };
                                    },
                                    .INIT0 => enum {
                                        const SKIP_DRAM_INIT = Field{ .rw = .ReadWrite, .width = u2, .shift = 30, .reg = addr, .values = enum(u2) {
                                            InitAfterPowerUp = 0,
                                            InitSkippedNormal = 1,
                                            InitSkippedSelfRefresh = 3,
                                        } };
                                    },
                                    .DFIMISC => enum {
                                        const DFI_INIT_COMPLETE_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                    },
                                    else => unreachable,
                                };
                            }
                        },
                        else => unreachable,
                    };
                }
            },
            .APB5 => enum(bus_type) {
                pub fn api(port: @This()) enum_type {
                    return API(port);
                }

                RTC = Port(0x4000, bus),
                TZC = Port(0x6000, bus),
                fn regs(port: @This()) enum_type {
                    return comptime switch (port) {
                        .RTC => enum(bus_type) {},
                        .TZC => enum(bus_type) {
                            GATE_KEEPER = Reg(0x8, port),
                            SPECULATION_CTRL = Reg(0xC, port),
                            ID_ACCESS0 = Reg(0x114, port),
                            ID_ACCESS1 = Reg(0x114 + 0x20, port),
                            ID_ACCESS2 = Reg(0x114 + 0x20 * 2, port),
                            ID_ACCESS3 = Reg(0x114 + 0x20 * 3, port),
                            ID_ACCESS4 = Reg(0x114 + 0x20 * 4, port),
                            ID_ACCESS5 = Reg(0x114 + 0x20 * 5, port),
                            ID_ACCESS6 = Reg(0x114 + 0x20 * 6, port),
                            ID_ACCESS7 = Reg(0x114 + 0x20 * 7, port),
                            ID_ACCESS8 = Reg(0x114 + 0x20 * 8, port),
                            ATTRIBUTE0 = Reg(0x110, port),
                            ATTRIBUTE1 = Reg(0x130, port),
                            ATTRIBUTE2 = Reg(0x130 + 0x20, port),
                            ATTRIBUTE3 = Reg(0x130 + 0x20 * 2, port),
                            ATTRIBUTE4 = Reg(0x130 + 0x20 * 3, port),
                            ATTRIBUTE5 = Reg(0x130 + 0x20 * 4, port),
                            ATTRIBUTE6 = Reg(0x130 + 0x20 * 5, port),
                            ATTRIBUTE7 = Reg(0x130 + 0x20 * 6, port),
                            ATTRIBUTE8 = Reg(0x130 + 0x20 * 7, port),

                            fn fields(reg: @This()) enum_type {
                                const addr = @intFromEnum(reg);
                                return comptime switch (reg) {
                                    .ATTRIBUTE1, .ATTRIBUTE2, .ATTRIBUTE3, .ATTRIBUTE4, .ATTRIBUTE5, .ATTRIBUTE6, .ATTRIBUTE7, .ATTRIBUTE8 => enum {
                                        const S_WR_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotAllowed = 0,
                                            Permitted = 1,
                                        } };
                                        const S_RD_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 30, .reg = addr, .values = enum(u1) {
                                            NotAllowed = 0,
                                            Permitted = 1,
                                        } };
                                        const FILTER0_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const FILTER1_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                    },
                                    .ATTRIBUTE0 => enum {
                                        const S_WR_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotAllowed = 0,
                                            Permitted = 1,
                                        } };
                                        const S_RD_EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 30, .reg = addr, .values = enum(u1) {
                                            NotAllowed = 0,
                                            Permitted = 1,
                                        } };
                                        const FILTER0_EN = Field{ .rw = .ReadOnly, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const FILTER1_EN = Field{ .rw = .ReadOnly, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                    },
                                    .ID_ACCESS0, .ID_ACCESS1, .ID_ACCESS2, .ID_ACCESS3, .ID_ACCESS4, .ID_ACCESS5, .ID_ACCESS6, .ID_ACCESS7, .ID_ACCESS8 => enum {
                                        const NSAID_WR_EN = Field{ .rw = .ReadWrite, .width = u16, .shift = 16, .reg = addr, .values = enum {} };
                                        const NSAID_RD_EN = Field{ .rw = .ReadWrite, .width = u16, .shift = 0, .reg = addr, .values = enum {} };
                                    },
                                    .SPECULATION_CTRL => enum {
                                        const WRITESPEC_DISABLE = Field{ .rw = .ReadWrite, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const READSPEC_DISABLE = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                    },
                                    .GATE_KEEPER => enum {
                                        const OPENSTAT_FLT0 = Field{ .rw = .ReadOnly, .width = u1, .shift = 16, .reg = addr, .values = enum(u1) {
                                            Opened = 0,
                                            Closed = 1,
                                        } };
                                        const OPENSTAT_FLT1 = Field{ .rw = .ReadOnly, .width = u1, .shift = 17, .reg = addr, .values = enum(u1) {
                                            Opened = 0,
                                            Closed = 1,
                                        } };
                                        const OPENREQ_FLT0 = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Opened = 0,
                                            Closed = 1,
                                        } };
                                        const OPENREQ_FLT1 = Field{ .rw = .ReadWrite, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Opened = 0,
                                            Closed = 1,
                                        } };
                                    },
                                };
                            }
                        },
                    };
                }
            },
            .APB1 => enum(bus_type) {
                pub fn api(port: @This()) enum_type {
                    return API(port);
                }

                UART4 = Port(0x10000, bus),
                fn regs(port: @This()) enum_type {
                    return comptime switch (port) {
                        .UART4 => enum(bus_type) {
                            CR1 = Reg(0x0, port), // USART control register 1 (USART_CR1)
                            CR2 = Reg(0x4, port), // USART control register 2 (USART_CR2)
                            BRR = Reg(0xC, port), // USART baud rate register (USART_BRR)
                            ISR = Reg(0x1C, port), // USART interrupt and status register (USART_ISR)
                            TDR = Reg(0x28, port), // USART transmit data register (USART_TDR)
                            PRESC = Reg(0x2C, port), // USART prescaler register (USART_PRESC)
                            fn fields(reg: @This()) enum_type {
                                const addr = @intFromEnum(reg);
                                return comptime switch (reg) {
                                    .BRR => enum {
                                        const BRR3_0 = Field{ .rw = .ReadWrite, .width = u4, .shift = 0, .reg = addr, .values = enum {} };
                                        const BRR15_4 = Field{ .rw = .ReadWrite, .width = u12, .shift = 4, .reg = addr, .values = enum {} };
                                    },
                                    .PRESC => enum {
                                        const PRESCALER = Field{ .rw = .ReadWrite, .width = u4, .shift = 0, .reg = addr, .values = enum(u4) {
                                            NoDiv = 0,
                                            Div2 = 1,
                                            Div4 = 2,
                                            Div6 = 3,
                                            Div8 = 4,
                                            Div10 = 5,
                                            Div12 = 6,
                                            Div16 = 7,
                                            Div32 = 8,
                                            Div64 = 9,
                                            Div128 = 10,
                                            Div256 = 11,
                                        } };
                                    },
                                    .CR2 => enum {
                                        const STOP = Field{ .rw = .ReadWrite, .width = u2, .shift = 12, .reg = addr, .values = enum(u2) {
                                            One = 0,
                                            Half = 1,
                                            Two = 2,
                                            OneHalf = 3,
                                        } };
                                    },
                                    .CR1 => enum {
                                        const UE = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            DisabledLPmode = 0,
                                            Enabled = 1,
                                        } };
                                        const RE = Field{ .rw = .ReadWrite, .width = u1, .shift = 2, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const TE = Field{ .rw = .ReadWrite, .width = u1, .shift = 3, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const PCE = Field{ .rw = .ReadWrite, .width = u1, .shift = 10, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const M0 = Field{ .rw = .ReadWrite, .width = u1, .shift = 12, .reg = addr, .values = enum(u1) {
                                            DataBits7or8 = 0,
                                            DataBits9 = 1,
                                        } };
                                        const M1 = Field{ .rw = .ReadWrite, .width = u1, .shift = 28, .reg = addr, .values = enum(u1) {
                                            DataBits8or9 = 0,
                                            DataBits7 = 1,
                                        } };
                                        const OVER8 = Field{ .rw = .ReadWrite, .width = u1, .shift = 15, .reg = addr, .values = enum(u1) {
                                            Oversampling16 = 0,
                                            Oversampling8 = 1,
                                        } };
                                        const FIFOEN = Field{ .rw = .ReadWrite, .width = u1, .shift = 29, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                    },
                                    .ISR => enum {
                                        const TXFNF = Field{ .rw = .ReadOnly, .width = u1, .shift = 7, .reg = addr, .values = enum(u1) {
                                            Full = 0,
                                            NotFull = 1,
                                        } };
                                    },
                                    .TDR => enum {
                                        const TDR = Field{ .rw = .WriteOnly, .width = u8, .shift = 0, .reg = addr, .values = enum {} };
                                    },
                                };
                            }
                        },
                    };
                }
            },
            .AHB4 => enum(bus_type) {
                pub fn api(port: @This()) enum_type {
                    return API(port);
                }

                RCC = Port(0x0, bus),
                PWR = Port(0x1000, bus),
                GPIOA = Port(0x2000, bus),
                GPIOB = Port(0x3000, bus),
                GPIOG = Port(0x8000, bus),

                fn regs(port: @This()) enum_type {
                    return comptime switch (port) {
                        .RCC => enum(bus_type) {
                            OCENSETR = Reg(0x0C, port), // RCC oscillator clock enable set register (RCC_OCENSETR)
                            OCENCLRR = Reg(0x10, port), // RCC oscillator clock enable clear register (RCC_OCENCLRR)
                            HSICFGR = Reg(0x18, port), // RCC HSI configuration register (RCC_HSICFGR)
                            BDCR = Reg(0x140, port), // RCC backup domain control register (RCC_BDCR)
                            OCRDYR = Reg(0x808, port), // RCC oscillator clock ready register (RCC_OCRDYR)
                            UART24CKSELR = Reg(0x8E8, port), // RCC UART2,4 kernel clock selection register (RCC_UART24CKSELR)
                            MP_APB1ENSETR = Reg(0xA00, port), // RCC APB1 peripheral enable for MPU set register (RCC_MP_APB1ENSETR)
                            MP_AHB4ENSETR = Reg(0xA28, port), // RCC AHB4 peripheral enable for MPU set register (RCC_MP_AHB4ENSETR)
                            MP_AHB4ENCLRR = Reg(0xA2C, port), // RCC AHB4 peripheral enable for MPU clear register (RCC_MP_AHB4ENCLRR)
                            MPCKSELR = Reg(0x20, port), // RCC MPU clock selection register (RCC_MPCKSELR)
                            ASSCKSELR = Reg(0x24, port), // RCC AXI sub-system clock selection register (RCC_ASSCKSELR)
                            MSSCKSELR = Reg(0x48, port), // RCC MCU sub-system clock selection register (RCC_MSSCKSELR)
                            MPCKDIVR = Reg(0x2C, port), // RCC MPU clock divider register (RCC_MPCKDIVR)
                            RCK12SELR = Reg(0x28, port), // RCC PLL 1 and 2 reference clock selection register (RCC_RCK12SELR)
                            RCK3SELR = Reg(0x820, port), // RCC PLL 3 reference clock selection register (RCC_RCK3SELR)
                            RCK4SELR = Reg(0x824, port), // RCC PLL 3 reference clock selection register (RCC_RCK3SELR)
                            PLL1CR = Reg(0x80, port), // RCC PLL1 control register (RCC_PLL1CR)
                            PLL1CFGR1 = Reg(0x84, port), // RCC PLL1 configuration register 1 (RCC_PLL1CFGR1)
                            PLL1CFGR2 = Reg(0x88, port), // RCC PLL1 configuration register 2 (RCC_PLL1CFGR2)
                            PLL1FRACR = Reg(0x8C, port), // RCC PLL1 fractional register (RCC_PLL1FRACR)
                            PLL1CSGR = Reg(0x90, port), // RCC PLL1 clock spreading generator register (RCC_PLL1CSGR)
                            PLL2CR = Reg(0x94, port), // RCC PLL2 control register (RCC_PLL2CR)
                            PLL2CFGR1 = Reg(0x98, port), // RCC PLL2 configuration register 1 (RCC_PLL2CFGR1)
                            PLL2CFGR2 = Reg(0x9C, port), // RCC PLL2 configuration register 2 (RCC_PLL2CFGR2)
                            PLL2FRACR = Reg(0xA0, port), // RCC PLL2 fractional register (RCC_PLL2FRACR)
                            PLL2CSGR = Reg(0xA4, port), // RCC PLL2 clock spreading generator register (RCC_PLL2CSGR)
                            PLL3CR = Reg(0x880, port), // RCC PLL3 control register (RCC_PLL3CR)
                            PLL3CFGR1 = Reg(0x884, port), // RCC PLL3 configuration register 1 (RCC_PLL3CFGR1)
                            PLL3CFGR2 = Reg(0x888, port), // RCC PLL3 configuration register 2 (RCC_PLL3CFGR2)
                            PLL3FRACR = Reg(0x88C, port), // RCC PLL3 fractional register (RCC_PLL3FRACR)
                            PLL4CR = Reg(0x894, port), // RCC PLL4 control register (RCC_PLL4CR)
                            PLL4CFGR1 = Reg(0x898, port), // RCC PLL4 configuration register 1 (RCC_PLL4CFGR1)
                            PLL4CFGR2 = Reg(0x89C, port), // RCC PLL4 configuration register 2 (RCC_PLL4CFGR2)
                            PLL4FRACR = Reg(0x8A0, port), // RCC PLL4 fractional register (RCC_PLL4FRACR)
                            AXIDIVR = Reg(0x30, port), // RCC AXI clock divider register (RCC_AXIDIVR)
                            APB4DIVR = Reg(0x3C, port), // RCC APB4 clock divider register (RCC_APB4DIVR)
                            APB5DIVR = Reg(0x40, port), // RCC APB5 clock divider register (RCC_APB5DIVR)
                            DDRITFCR = Reg(0xD8, port), // RCC DDR interface control register (RCC_DDRITFCR)
                            MP_APB5ENSETR = Reg(0x208, port), // RCC APB5 peripheral enable for MPU set register
                            MP_APB5ENCLRR = Reg(0x20C, port), // RCC APB5 peripheral enable for MPU clear register
                            fn fields(reg: @This()) enum_type {
                                const addr = @intFromEnum(reg);
                                return comptime switch (reg) {
                                    .MP_APB5ENSETR => enum {
                                        const TZC1EN = Field{ .rw = .WriteOnly, .width = u1, .shift = 11, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                        const TZC2EN = Field{ .rw = .WriteOnly, .width = u1, .shift = 12, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                        const TZPCEN = Field{ .rw = .WriteOnly, .width = u1, .shift = 13, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                    },
                                    .MP_APB5ENCLRR => enum {
                                        const TZC1EN = Field{ .rw = .WriteOnly, .width = u1, .shift = 11, .reg = addr, .values = enum(u1) {
                                            Clear = 1,
                                        } };
                                        const TZC2EN = Field{ .rw = .WriteOnly, .width = u1, .shift = 12, .reg = addr, .values = enum(u1) {
                                            Clear = 1,
                                        } };
                                        const TZPCEN = Field{ .rw = .WriteOnly, .width = u1, .shift = 13, .reg = addr, .values = enum(u1) {
                                            Clear = 1,
                                        } };
                                    },
                                    .DDRITFCR => enum {
                                        const DDRCKMOD = Field{ .rw = .ReadWrite, .width = u3, .shift = 20, .reg = addr, .values = enum(u3) {
                                            Normal = 0,
                                            AutoSelfRef = 1,
                                            HardwareSelfRef = 2,
                                            FullAutoSelfRef = 5,
                                            FullHardwareSelfRef = 6,
                                        } };
                                        const DPHYCTLRST = Field{ .rw = .ReadWrite, .width = u1, .shift = 19, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const DPHYRST = Field{ .rw = .ReadWrite, .width = u1, .shift = 18, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const DPHYAPBRST = Field{ .rw = .ReadWrite, .width = u1, .shift = 17, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const DDRCORERST = Field{ .rw = .ReadWrite, .width = u1, .shift = 16, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const DDRCAXIRST = Field{ .rw = .ReadWrite, .width = u1, .shift = 15, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const DDRCAPBRST = Field{ .rw = .ReadWrite, .width = u1, .shift = 14, .reg = addr, .values = enum(u1) {
                                            Cleared = 0,
                                            Asserted = 1,
                                        } };
                                        const DDRPHYCAPBEN = Field{ .rw = .ReadWrite, .width = u1, .shift = 9, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const AXIDCGEN = Field{ .rw = .ReadWrite, .width = u1, .shift = 8, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const DDRCAPBEN = Field{ .rw = .ReadWrite, .width = u1, .shift = 6, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const DDRPHYCEN = Field{ .rw = .ReadWrite, .width = u1, .shift = 4, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const DDRC2EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 2, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const DDRC1EN = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                    },
                                    .APB5DIVR => enum {
                                        const APB5DIVRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const APB5DIV = Field{ .rw = .ReadWrite, .width = u3, .shift = 0, .reg = addr, .values = enum {} };
                                    },
                                    .APB4DIVR => enum {
                                        const APB4DIVRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const APB4DIV = Field{ .rw = .ReadWrite, .width = u3, .shift = 0, .reg = addr, .values = enum {} };
                                    },
                                    .AXIDIVR => enum {
                                        const AXIDIVRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const AXIDIV = Field{ .rw = .ReadWrite, .width = u3, .shift = 0, .reg = addr, .values = enum {} };
                                    },
                                    .MPCKDIVR => enum {
                                        const MPUDIVRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const MPUDIV = Field{ .rw = .ReadWrite, .width = u2, .shift = 0, .reg = addr, .values = enum(u2) {
                                            Off = 0,
                                            Two = 1,
                                            Four = 2,
                                            Eight = 3,
                                        } };
                                    },
                                    .ASSCKSELR => enum {
                                        const AXISSRCRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const AXISSRC = Field{ .rw = .ReadWrite, .width = u3, .shift = 0, .reg = addr, .values = api(port).MUX.source(api(port).MUX.AXI) };
                                    },
                                    .MSSCKSELR => enum {
                                        const MCUSSRCRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const MCUSSRC = Field{ .rw = .ReadWrite, .width = u2, .shift = 0, .reg = addr, .values = api(port).MUX.source(api(port).MUX.MCU) };
                                    },
                                    .MPCKSELR => enum {
                                        const MPUSRCRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const MPUSRC = Field{ .rw = .ReadWrite, .width = u2, .shift = 0, .reg = addr, .values = api(port).MUX.source(api(port).MUX.MPU) };
                                    },
                                    .PLL1CR, .PLL2CR, .PLL3CR, .PLL4CR => enum {
                                        const DIVREN = Field{ .rw = .ReadWrite, .width = u1, .shift = 6, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const DIVQEN = Field{ .rw = .ReadWrite, .width = u1, .shift = 5, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const DIVPEN = Field{ .rw = .ReadWrite, .width = u1, .shift = 4, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const SSCG_CTRL = Field{ .rw = .ReadWrite, .width = u1, .shift = 2, .reg = addr, .values = enum(u1) {
                                            Disabled = 0,
                                            Enabled = 1,
                                        } };
                                        const PLLRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Unlocked = 0,
                                            Locked = 1,
                                        } };
                                        const PLLON = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Off = 0,
                                            On = 1,
                                        } };
                                    },
                                    .PLL1CFGR1, .PLL2CFGR1 => enum {
                                        const DIVM = Field{ .rw = .ReadWrite, .width = u6, .shift = 16, .reg = addr, .values = enum {} };
                                        const DIVN = Field{ .rw = .ReadWrite, .width = u9, .shift = 0, .reg = addr, .values = enum {} };
                                    },
                                    .PLL3CFGR1, .PLL4CFGR1 => enum {
                                        const IFRGE = Field{ .rw = .ReadWrite, .width = u2, .shift = 24, .reg = addr, .values = enum {} };
                                        const DIVM = Field{ .rw = .ReadWrite, .width = u6, .shift = 16, .reg = addr, .values = enum {} };
                                        const DIVN = Field{ .rw = .ReadWrite, .width = u9, .shift = 0, .reg = addr, .values = enum {} };
                                    },
                                    .PLL1CFGR2, .PLL2CFGR2, .PLL3CFGR2, .PLL4CFGR2 => enum {
                                        const DIVR = Field{ .rw = .ReadWrite, .width = u7, .shift = 16, .reg = addr, .values = enum {} };
                                        const DIVQ = Field{ .rw = .ReadWrite, .width = u7, .shift = 8, .reg = addr, .values = enum {} };
                                        const DIVP = Field{ .rw = .ReadWrite, .width = u7, .shift = 0, .reg = addr, .values = enum {} };
                                    },
                                    .PLL1FRACR, .PLL2FRACR, .PLL3FRACR, .PLL4FRACR => enum {
                                        const FRACLE = Field{ .rw = .ReadWrite, .width = u1, .shift = 16, .reg = addr, .values = enum {} };
                                        const FRACV = Field{ .rw = .ReadWrite, .width = u13, .shift = 3, .reg = addr, .values = enum {} };
                                    },
                                    .PLL1CSGR, .PLL2CSGR => enum {
                                        const INC_STEP = Field{ .rw = .ReadWrite, .width = u15, .shift = 16, .reg = addr, .values = enum {} };
                                        const SSCG_MODE = Field{ .rw = .ReadWrite, .width = u1, .shift = 15, .reg = addr, .values = enum(u1) {
                                            CenterSpread = 0,
                                            DownSpread = 1,
                                        } };
                                        const RPDFN_DIS = Field{ .rw = .ReadWrite, .width = u1, .shift = 14, .reg = addr, .values = enum(u1) {
                                            InjectionEnabled = 0,
                                            InhjectionDisabled = 1,
                                        } };
                                        const TPDFN_DIS = Field{ .rw = .ReadWrite, .width = u1, .shift = 13, .reg = addr, .values = enum(u1) {
                                            InjectionEnabled = 0,
                                            InhjectionDisabled = 1,
                                        } };
                                        const MOD_PER = Field{ .rw = .ReadWrite, .width = u13, .shift = 0, .reg = addr, .values = enum {} };
                                    },
                                    .RCK4SELR => enum {
                                        const PLL4SRCRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const PLL4SRC = Field{ .rw = .ReadWrite, .width = u2, .shift = 0, .reg = addr, .values = api(port).MUX.source(api(port).MUX.PLL4) };
                                    },
                                    .RCK3SELR => enum {
                                        const PLL3SRCRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const PLL3SRC = Field{ .rw = .ReadWrite, .width = u2, .shift = 0, .reg = addr, .values = api(port).MUX.source(api(port).MUX.PLL3) };
                                    },
                                    .RCK12SELR => enum {
                                        const PLL12SRCRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 31, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const PLL12SRC = Field{ .rw = .ReadWrite, .width = u2, .shift = 0, .reg = addr, .values = api(port).MUX.source(api(port).MUX.PLL12) };
                                    },
                                    .HSICFGR => enum {
                                        const HSIDIV = Field{ .rw = .ReadWrite, .width = u2, .shift = 0, .reg = addr, .values = enum(u2) {
                                            One = 0,
                                            Two = 1,
                                            Four = 2,
                                            Eight = 3,
                                        } };
                                    },
                                    .UART24CKSELR => enum {
                                        const UART24SRC = Field{ .rw = .ReadWrite, .width = u3, .shift = 0, .reg = addr, .values = enum(u3) {
                                            PCLK1 = 0,
                                            PLL4Q = 1,
                                            HSI = 2,
                                            CSI = 3,
                                            HSE = 4,
                                        } };
                                    },
                                    .MP_AHB4ENCLRR => enum {
                                        const GPIOAEN = Field{ .rw = .WriteOnly, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                        const GPIOBEN = Field{ .rw = .WriteOnly, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                        const GPIOGEN = Field{ .rw = .WriteOnly, .width = u1, .shift = 6, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                    },
                                    .MP_AHB4ENSETR => enum {
                                        const GPIOAEN = Field{ .rw = .WriteOnly, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                        const GPIOBEN = Field{ .rw = .WriteOnly, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                        const GPIOGEN = Field{ .rw = .WriteOnly, .width = u1, .shift = 6, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                    },
                                    .MP_APB1ENSETR => enum {
                                        const UART4EN = Field{ .rw = .WriteOnly, .width = u1, .shift = 16, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                    },
                                    .OCENSETR => enum {
                                        const HSION = Field{ .rw = .WriteOnly, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                        const HSEON = Field{ .rw = .WriteOnly, .width = u1, .shift = 8, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                        const HSIKERON = Field{ .rw = .WriteOnly, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                    },
                                    .OCENCLRR => enum {
                                        const HSION = Field{ .rw = .WriteOnly, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Clear = 1,
                                        } };
                                        const HSEON = Field{ .rw = .WriteOnly, .width = u1, .shift = 8, .reg = addr, .values = enum(u1) {
                                            Clear = 1,
                                        } };
                                        const HSEBYP = Field{ .rw = .WriteOnly, .width = u1, .shift = 10, .reg = addr, .values = enum(u1) {
                                            Clear = 1,
                                        } };
                                    },
                                    .BDCR => enum {
                                        const LSEON = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = addr, .values = enum(u1) {
                                            Off = 0,
                                            On = 1,
                                        } };
                                        const LSEBYP = Field{ .rw = .ReadWrite, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            NotBypassed = 0,
                                            Bypassed = 1,
                                        } };
                                        const LSERDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 2, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                    },
                                    .OCRDYR => enum {
                                        const HSERDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 8, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                        const HSIDIVRDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 2, .reg = addr, .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                    },
                                };
                            }
                        },
                        .PWR => enum(bus_type) {
                            CR1 = Reg(0x0, port), // PWR control register 1 (PWR_CR1)
                            fn fields(reg: @This()) enum_type {
                                const addr = @intFromEnum(reg);
                                return comptime switch (reg) {
                                    .CR1 => enum {
                                        const DBP = Field{ .rw = .ReadWrite, .width = u1, .shift = 8, .reg = addr, .values = enum(u1) {
                                            Enabled = 1,
                                            Disabled = 0,
                                        } };
                                    },
                                };
                            }
                        },
                        else => unreachable,
                    };
                }
            },
        };
    }
};

// private

fn Reg(comptime offset: bus_type, comptime port: anytype) bus_type {
    return offset + @intFromEnum(port);
}
fn Port(comptime offset: bus_type, comptime bus: anytype) bus_type {
    return offset + @intFromEnum(bus);
}

// tests
const testing = @import("std").testing;

test "regular field test" {}
