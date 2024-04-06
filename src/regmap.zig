// platform specifics
const BusType: type = u32;
const FieldShiftType = switch (BusType) {
    u32 => u5,
    u64 => u6,
    else => unreachable,
};
const FieldWidthType = FieldShiftType;
const hsi_fq_hz: u32 = 64000000;
const csi_fq_hz: u32 = 4000000;
var hse_fq_hz: u32 = 0;

// main peripheral instantiation
const bus: struct {
    axi: AXI = .{ .mux = .{ .src = .{ .reg = .ASSCKSELR, .shift = 0, .width = 3 }, .rdy = .{ .reg = .ASSCKSELR, .shift = 31 } } },
    mpu: MPU = .{ .mux = .{ .src = .{ .reg = .MPCKSELR, .shift = 0, .width = 2 }, .rdy = .{ .reg = .MPCKSELR, .shift = 31 } } },
    pll1: PLL = .{ .cfg1r = .PLL1CFGR1, .cfg2r = .PLL1CFGR2, .fracr = .PLL1FRACR, .cr = .PLL1CR, .mux = .{ .rdy = .{ .reg = .RCK12SELR, .shift = 31 }, .src = .{ .reg = .RCK12SELR, .shift = 0, .width = 2 } } },
    pll2: PLL = .{ .cfg1r = .PLL2CFGR1, .cfg2r = .PLL2CFGR2, .fracr = .PLL2FRACR, .cr = .PLL2CR, .mux = .{ .rdy = .{ .reg = .RCK12SELR, .shift = 31 }, .src = .{ .reg = .RCK12SELR, .shift = 0, .width = 2 } } },
    pll3: PLL = .{ .cfg1r = .PLL3CFGR1, .cfg2r = .PLL3CFGR2, .fracr = .PLL3FRACR, .cr = .PLL3CR, .mux = .{ .rdy = .{ .reg = .RCK3SELR, .shift = 31 }, .src = .{ .reg = .RCK3SELR, .shift = 0, .width = 2 } } },
    pll4: PLL = .{ .cfg1r = .PLL4CFGR1, .cfg2r = .PLL4CFGR2, .fracr = .PLL4FRACR, .cr = .PLL4CR, .mux = .{ .rdy = .{ .reg = .RCK4SELR, .shift = 31 }, .src = .{ .reg = .RCK4SELR, .shift = 0, .width = 2 } } },
    ahb4: struct {
        const base: BusType = 0x50000000;
        rcc: RCC = .{ .port = 0x0 + base },
        gpioa: GPIO = .{ .port = 0x2000 + base, .rcc_switch = .{ .set_reg = .MP_AHB4ENSETR, .clr_reg = .MP_AHB4ENCLRR, .shift = 0 } },
        gpiob: GPIO = .{ .port = 0x3000 + base, .rcc_switch = .{ .set_reg = .MP_AHB4ENSETR, .clr_reg = .MP_AHB4ENCLRR, .shift = 1 } },
        gpioc: GPIO = .{ .port = 0x4000 + base, .rcc_switch = .{ .set_reg = .MP_AHB4ENSETR, .clr_reg = .MP_AHB4ENCLRR, .shift = 2 } },
        gpiod: GPIO = .{ .port = 0x5000 + base, .rcc_switch = .{ .set_reg = .MP_AHB4ENSETR, .clr_reg = .MP_AHB4ENCLRR, .shift = 3 } },
        gpioe: GPIO = .{ .port = 0x6000 + base, .rcc_switch = .{ .set_reg = .MP_AHB4ENSETR, .clr_reg = .MP_AHB4ENCLRR, .shift = 4 } },
        gpiog: GPIO = .{ .port = 0x8000 + base, .rcc_switch = .{ .set_reg = .MP_AHB4ENSETR, .clr_reg = .MP_AHB4ENCLRR, .shift = 6 } },
    } = .{},
    ahb6: struct {
        const base: BusType = 0x58000000;
        sdmmc1: SDMMC = .{
            .port = 0x5000 + base,
            .mux = .{ .src = .{ .reg = .SDMMC12CKSELR, .shift = 0, .width = 3 } },
            .rcc_switch = .{ .set_reg = .MP_AHB6ENSETR, .clr_reg = .MP_AHB6ENCLRR, .shift = 16 },
        },
        sdmmc2: SDMMC = .{
            .port = 0x7000 + base,
            .mux = .{ .src = .{ .reg = .SDMMC12CKSELR, .shift = 0, .width = 3 } },
            .rcc_switch = .{ .set_reg = .MP_AHB6ENSETR, .clr_reg = .MP_AHB6ENCLRR, .shift = 17 },
        },
    } = .{},
    apb1: struct {
        const base: BusType = 0x40000000;
        uart4: UART = .{
            .port = 0x10000 + base,
            .idx = 4,
            .mux = .{ .src = .{ .reg = .UART24CKSELR, .shift = 0, .width = 3 } },
            .rcc_switch = .{ .set_reg = .MP_APB1ENSETR, .clr_reg = null, .shift = 16 },
        },
    } = .{},
    apb4: struct {
        const base: BusType = 0x5A000000;
        ddr: DDR = .{ .ctrl_port = 0x3000 + base, .phyc_port = 0x4000 + base },
    } = .{},
    apb5: struct {
        const base: BusType = 0x5C000000;
        tzc: TZC = .{ .port = 0x6000 + base },
    } = .{},
} = .{};

// peripheries public aliasing
pub const gpioa = bus.ahb4.gpioa;
pub const gpiob = bus.ahb4.gpiob;
pub const gpioc = bus.ahb4.gpioc;
pub const gpiod = bus.ahb4.gpiod;
pub const gpioe = bus.ahb4.gpioe;
pub const gpiog = bus.ahb4.gpiog;
pub const uart4 = bus.apb1.uart4;
pub const rcc = bus.ahb4.rcc;
pub const pll1 = bus.pll1;
pub const pll2 = bus.pll2;
pub const pll3 = bus.pll3;
pub const pll4 = bus.pll4;
pub const mpu = bus.mpu;
pub const axi = bus.axi;
pub const tzc = bus.apb5.tzc;
pub const ddr = bus.apb4.ddr;
pub const sdmmc1 = bus.ahb6.sdmmc1;
pub const sdmmc2 = bus.ahb6.sdmmc2;

// pripheries private aliasing

// private basic types
const Field = struct {
    pub const RwType = enum {
        ReadOnly,
        WriteOnly,
        ReadWrite,
    };
    reg: BusType,
    width: FieldWidthType,
    rw: RwType = .ReadWrite,
    shift: FieldShiftType,

    fn set(self: Field, value: BusType) void {
        const addr: *volatile BusType = @ptrFromInt(self.reg);
        if (self.rw == .ReadOnly)
            return;

        if (self.rw == .WriteOnly) {
            addr.* = value << self.shift;
            return;
        }
        addr.* = addr.* & self.getResetMask() | ((value << self.shift) & self.getMask());
    }
    fn get(self: Field) BusType {
        const addr: *volatile BusType = @ptrFromInt(self.reg);
        return ((addr.* & self.getMask()) >> self.shift);
    }

    fn isAsserted(self: Field) bool {
        return self.get() != 0;
    }
    fn isCleared(self: Field) bool {
        return self.get() == 0;
    }

    fn getMask(self: Field) BusType {
        return ((@as(BusType, 1) << self.width) - 1) << self.shift;
    }

    fn getResetMask(self: Field) BusType {
        return ~self.getMask();
    }
};

const Register = struct {
    addr: BusType,
    inline fn reset(self: Register) void {
        self.set(0x0);
    }
    fn set(self: Register, value: BusType) void {
        @as(*volatile BusType, @ptrFromInt(self.addr)).* = value;
    }
    fn get(self: Register) BusType {
        return @as(*volatile BusType, @ptrFromInt(self.addr)).*;
    }
};
// private peripheries implementation

const SDMMC = struct {
    port: BusType,
    mux: RCC.ClockMuxer,
    rcc_switch: RCC.PeripherySwitch,

    fn getReg(self: SDMMC, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    const ClockSource = enum(u3) { HCLK6 = 0, PLL3 = 1, PLL4 = 2, HSI = 3, Off }; // FIXME: current support for sdmmc1&2 only. sdmmc3 has other options
    pub const MediaType = enum {
        MediaError,
        BusError,
        NoMedia,
        SDSC,
        SDHC,
        eMMC,
        Unsupported,
    };

    pub fn setClockSource(self: SDMMC, clock_source: ClockSource) void {
        rcc.setMuxerValue(self.mux, @intFromEnum(clock_source));
        switch (clock_source) {
            .PLL3 => pll3.enable(.R),
            .PLL4 => pll4.enable(.P),
            else => {},
        }
    }

    fn getRefClockHz(self: SDMMC) BusType {
        return switch (@as(ClockSource, @enumFromInt(rcc.getMuxerValue(self.mux)))) {
            .Off => 0,
            .HCLK6 => axi.getOutputFrequency(.HCLK6),
            .PLL3 => pll3.getOutputFrequency(.R),
            .PLL4 => pll4.getOutputFrequency(.P),
            .HSI => rcc.getOutputFrequency(.HSI),
        };
    }
    fn getPowerCtrl(self: SDMMC) Field {
        return Field{ .reg = self.getReg(.POWER), .shift = 0, .width = 2 };
    }
    fn powerCycle(self: SDMMC) void {
        self.getPowerCtrl().set(2); // PowerCycle
        mpu.udelay(2000);
        self.getPowerCtrl().set(0); // PowerOff
        mpu.udelay(2000);
    }
    fn powerOn(self: SDMMC) void {
        self.getPowerCtrl().set(3); // PowerOn
        mpu.udelay(2000);
    }

    const BusClockMode = enum(u1) { SDR = 0, DDR = 1 };
    const BusSpeed = enum(u1) { UpTo50MHz = 0, Above50MHz = 1 };
    const BusRXClockSource = enum(u2) { Internal = 0, External = 1, InternalDelay = 2 };
    const BusWidth = enum(u2) { Default1Bit = 0, Wide4Bit = 1, Wide8Bit = 2 };
    const BusPowerSave = enum(u1) { PowerSaveOff = 0, PowerSaveOn = 1 };

    const Command = enum(u6) {
        GO_IDLE_STATE = 0,
        SEND_OP_COND = 1,
        ALL_SEND_CID = 2,
        SEND_RELATIVE_ADDR = 3,
        SEND_IF_COND = 8,
        SD_SEND_OP_COND = 41,
        APP_CMD = 55,
        fn getStuff(self: Command) struct { timeout: BusType, ret: RetType } {
            return switch (self) {
                .GO_IDLE_STATE => .{ .timeout = 0, .ret = .empty },
                .SEND_OP_COND => .{ .timeout = 0, .ret = .{ .r3 = undefined } },
                .ALL_SEND_CID => .{ .timeout = 0, .ret = .{ .r2 = undefined } },
                .SEND_RELATIVE_ADDR => .{ .timeout = 0, .ret = .{ .r6 = undefined } },
                .SEND_IF_COND => .{ .timeout = 0, .ret = .{ .r7 = undefined } },
                .SD_SEND_OP_COND => .{ .timeout = 0, .ret = .{ .r3 = undefined } },
                .APP_CMD => .{ .timeout = 0, .ret = .{ .r1 = undefined } },
            };
        }
        const RespType = enum(u2) {
            NoResponse = 0,
            ShortWithCRC = 1,
            Short = 2,
            LongWithCRC = 3,
        };
        const RetType = union(enum) {
            r1: struct { cmd_idx: u6, card_status: u32 },
            r1b: struct { cmd_idx: u6, card_status: u32 },
            r2: u128,
            r3: u32,
            r6: struct { cmd_idx: u6, rsa: u16, card_status: u16 },
            r7: u32, // TODO: redefine response fields
            err: enum {
                Busy,
                Timeout,
                BusTimeout,
                CRCError,
            },
            empty: void,
            fn getRespType(self: RetType) RespType {
                return switch (self) {
                    .r1, .r6, .r7 => .ShortWithCRC,
                    .r2 => .LongWithCRC,
                    .r3 => .Short,
                    else => .NoResponse,
                };
            }
        };
    };

    fn getCommandResponse(self: SDMMC, cmd: Command, arg: u32) Command.RetType {
        const cmdr = self.getReg(.CMDR);
        const cpsmen = Field{ .reg = cmdr, .shift = 12, .width = 1 };

        if (cpsmen.isAsserted())
            (Register{ .addr = cmdr }).reset();
        (Register{ .addr = self.getReg(.DCTRL) }).reset();
        var stuff = cmd.getStuff();
        const resp_tpye = stuff.ret.getRespType();
        (Field{ .reg = cmdr, .shift = 8, .width = 2 }).set(@intFromEnum(resp_tpye));
        (Register{ .addr = self.getReg(.DTIMER) }).set(stuff.timeout);
        self.clearInterrupts();
        (Register{ .addr = self.getReg(.ARGR) }).set(arg);
        (Field{ .reg = cmdr, .shift = 0, .width = 6 }).set(@intFromEnum(cmd));
        cpsmen.set(1);
        const star = self.getReg(.STAR);
        const ctimeout = Field{ .reg = star, .shift = 2, .width = 1, .rw = .ReadOnly };
        const ccrfail = Field{ .reg = star, .shift = 0, .width = 1, .rw = .ReadOnly };
        const cmdrend = Field{ .reg = star, .shift = 6, .width = 1, .rw = .ReadOnly };
        const cmdsent = Field{ .reg = star, .shift = 7, .width = 1, .rw = .ReadOnly };

        var timeout_us: u32 = 10_000;
        while (true) {
            mpu.udelay(1);
            timeout_us -= 1;
            if (timeout_us == 0)
                return .{ .err = .Timeout };
            if (ctimeout.isAsserted())
                return .{ .err = .BusTimeout };
            if (resp_tpye != .NoResponse) {
                if ((resp_tpye == .ShortWithCRC or resp_tpye == .LongWithCRC) and ccrfail.isAsserted())
                    return .{ .err = .CRCError };
                if (cmdrend.isAsserted())
                    break;
            } else if (cmdsent.isAsserted())
                break;
        }

        const respr1r = Register{ .addr = self.getReg(.RESP1R) };
        const respr2r = Register{ .addr = self.getReg(.RESP2R) };
        const respr3r = Register{ .addr = self.getReg(.RESP3R) };
        const respr4r = Register{ .addr = self.getReg(.RESP4R) };
        const respcmdr = (Field{ .reg = self.getReg(.RESPCMDR), .shift = 0, .width = 6, .rw = .ReadOnly });
        switch (stuff.ret) {
            .empty => {},
            .r7, .r3 => |*value| value.* = respr1r.get(),
            .r6 => |*value| value.* = .{ .cmd_idx = @truncate(respcmdr.get()), .rsa = @truncate(respr1r.get() >> 16), .card_status = @truncate(respr1r.get()) },
            .r2 => |*value| value.* = (@as(u128, respr4r.get()) << 0) + (@as(u128, respr3r.get()) << 32) + (@as(u128, respr2r.get()) << 64) + (@as(u128, respr1r.get()) << 96),
            .r1 => |*value| value.* = .{ .cmd_idx = @truncate(respcmdr.get()), .card_status = respr1r.get() },
            else => unreachable,
        }
        return stuff.ret;
    }

    fn clearInterrupts(self: SDMMC) void {
        const clear_mask: BusType = 0x1FE00FFF; // all bits
        (@as(*volatile BusType, @ptrFromInt(self.getReg(.ICR)))).* |= clear_mask;
    }

    pub fn getMediaType(self: SDMMC, power_mode: enum(BusType) { PowerSave = 0, Performance = 1 << 28 }) MediaType {
        self.configure(400_000, .SDR, .Default1Bit, .PowerSaveOn);

        if (self.getCommandResponse(.GO_IDLE_STATE, 0) != .empty)
            return .BusError;

        // try detect SD Card
        switch (self.getCommandResponse(.SEND_IF_COND, 0x1AA)) {
            .r7 => |*value| {
                if ((value.* & 0x1FF) != 0x1AA) {
                    return .NoMedia;
                } // TODO: shall we support v.1 ????
            },
            else => {
                // try detect eMMC
                switch (self.getCommandResponse(.SEND_OP_COND, 0x0)) {
                    .r3 => return .eMMC,
                    else => return .NoMedia,
                }
            },
        }
        // try setup ~3v SDHC
        var cntr: u16 = 1_000;
        while (cntr != 0) {
            switch (self.getCommandResponse(.APP_CMD, 0)) {
                .r1 => {}, // ready to receive application command
                else => return .MediaError,
            }

            switch (self.getCommandResponse(.SD_SEND_OP_COND, 0x403E0000 | @intFromEnum(power_mode))) { // HCS, 2.9-3.4 V
                .r3 => |*value| {
                    const v = value.*;
                    if ((v & (1 << 31)) == (1 << 31)) { // only when Busy bit is set
                        if (v & 0x3E0000 == 0x0)
                            return .Unsupported;
                        if (v & 0x40000000 != 0x0)
                            return .SDHC;
                        return .SDSC;
                    }
                },
                else => return .MediaError,
            }
            cntr -= 1;
            mpu.udelay(1000);
        }
        return .NoMedia;
    }

    pub fn getSDCardAddr(self: SDMMC) u16 {
        switch (self.getCommandResponse(.ALL_SEND_CID, 0x0)) {
            .r2 => {}, // FIXME: unused CID result
            else => return 0,
        }
        switch (self.getCommandResponse(.SEND_RELATIVE_ADDR, 0x0)) {
            .r6 => |*value| {
                return value.rsa;
            },
            else => return 0,
        }
        return 0;
    }

    fn configure(self: SDMMC, bus_clock_hz: BusType, mode: BusClockMode, width: BusWidth, power_save: BusPowerSave) void {
        rcc.enablePeriphery(self.rcc_switch);
        mpu.udelay(100);
        const ref_clock_hz = self.getRefClockHz();
        if (ref_clock_hz == 0)
            return;
        self.powerCycle();
        const clkcr = self.getReg(.CLKCR);
        (Field{ .reg = clkcr, .shift = 0, .width = 10 }).set(ref_clock_hz / (bus_clock_hz * 2)); // CLKDIV
        (Field{ .reg = clkcr, .shift = 18, .width = 1 }).set(@intFromEnum(mode));
        if (bus_clock_hz <= 50_000_000) {
            (Field{ .reg = clkcr, .shift = 19, .width = 1 }).set(@intFromEnum(BusSpeed.UpTo50MHz));
        } else (Field{ .reg = clkcr, .shift = 19, .width = 1 }).set(@intFromEnum(BusSpeed.Above50MHz));

        // TODO: review RX clock source and HW flow control settings
        (Field{ .reg = clkcr, .shift = 20, .width = 2 }).set(@intFromEnum(BusRXClockSource.Internal));
        (Field{ .reg = clkcr, .shift = 17, .width = 1 }).set(0); // HWFC_EN

        (Field{ .reg = clkcr, .shift = 14, .width = 2 }).set(@intFromEnum(width));
        (Field{ .reg = clkcr, .shift = 12, .width = 1 }).set(@intFromEnum(power_save));
        self.powerOn();
    }

    const Reg = enum(BusType) {
        POWER = 0x0, // SDMMC power control register (SDMMC_POWER)
        CLKCR = 0x4, // SDMMC clock control register (SDMMC_CLKCR)
        ARGR = 0x8, // SDMMC argument register (SDMMC_ARGR)
        CMDR = 0xC, // SDMMC command register (SDMMC_CMDR)
        RESPCMDR = 0x10, // SDMMC command response register (SDMMC_RESPCMDR)
        RESP1R = 0x14, // SDMMC response x register (SDMMC_RESPxR)
        RESP2R = 0x18, // SDMMC response x register (SDMMC_RESPxR)
        RESP3R = 0x1C, // SDMMC response x register (SDMMC_RESPxR)
        RESP4R = 0x20, // SDMMC response x register (SDMMC_RESPxR)
        DTIMER = 0x24, // SDMMC data length register (SDMMC_DLENR)
        DLENR = 0x28, // SDMMC data timer register (SDMMC_DTIMER)
        DCTRL = 0x2C, // SDMMC data control register (SDMMC_DCTRL)
        DCNTR = 0x30, // SDMMC data counter register (SDMMC_DCNTR)
        STAR = 0x34, // SDMMC status register (SDMMC_STAR)
        ICR = 0x38, // SDMMC interrupt clear register (SDMMC_ICR)
        MASKR = 0x3C, // SDMMC mask register (SDMMC_MASKR)
        ACKTIMER = 0x40, // SDMMC acknowledgment timer register (SDMMC_ACKTIMER)
        IDMACTRLR = 0x50, // SDMMC DMA control register (SDMMC_IDMACTRLR)
        IDMABSIZER = 0x54, // SDMMC IDMA buffer size register (SDMMC_IDMABSIZER)
        IDMABASER = 0x58, // SDMMC IDMA buffer base address register
        IDMALAR = 0x64, // SDMMC IDMA linked list address register (SDMMC_IDMALAR)
        IDMABAR = 0x68, // SDMMC IDMA linked list memory base register (SDMMC_IDMABAR)
    };
};

const UART = struct {
    port: BusType,
    idx: comptime_int,
    mux: RCC.ClockMuxer,
    rcc_switch: RCC.PeripherySwitch,

    fn getReg(self: UART, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    const ClockSource = enum { HSI }; // TODO: add support for other clock sources
    const Reg = enum(BusType) {
        CR1 = 0x0, // USART control register 1 (USART_CR1)
        CR2 = 0x4, // USART control register 2 (USART_CR2)
        BRR = 0xC, // USART baud rate register (USART_BRR)
        ISR = 0x1C, // USART interrupt and status register (USART_ISR)
        TDR = 0x28, // USART transmit data register (USART_TDR)
        PRESC = 0x2C, // USART prescaler register (USART_PRESC)
    };

    const DataBits = enum { SevenDataBits, EightDataBits, NineDataBits };
    const Oversamling = enum(u1) { Oversampling8 = 1, Oversampling16 = 0 };
    const Parity = enum(u1) { WithParity = 1, NoParity = 0 };
    const StopBits = enum(u2) { OneStopBit = 0, HalfStopBit = 1, TwoStopBits = 2, OneHalfStopBits = 3 };
    const BaudRate = enum(u32) { B115200 = 115200 };
    const Prescaler = enum(u8) { NoPrescale = 0, Prescale2 = 1, Prescale4 = 2, Prescale6 = 3, Prescale8 = 4, Prescale10 = 5, Prescale12 = 6, Prescale16 = 7, Prescale32 = 8, Prescale64 = 9, Prescale128 = 10, Prescale256 = 11 };

    fn getPeriphEnablerField(self: UART) Field {
        return .{ .reg = self.getReg(.CR1), .shift = 0, .width = 1 };
    }

    fn setDataBits(self: UART, bits: DataBits) void {
        const m0 = Field{ .reg = self.getReg(.CR1), .shift = 12, .width = 1 };
        const m1 = Field{ .reg = self.getReg(.CR1), .shift = 28, .width = 1 };
        switch (bits) {
            .SevenDataBits => {
                m0.set(0);
                m1.set(1);
            },
            .EightDataBits => {
                m0.set(0);
                m1.set(0);
            },
            .NineDataBits => {
                m0.set(1);
                m1.set(0);
            },
        }
    }

    pub fn write(self: UART, bytes: []const u8) usize {
        const txfnf = Field{ .reg = self.getReg(.ISR), .shift = 7, .width = 1 };
        const fifo_full = 0;
        const tdr = Field{ .reg = self.getReg(.TDR), .shift = 0, .width = 8 };
        for (bytes) |byte| {
            while (txfnf.get() == fifo_full) {}
            tdr.set(byte);
        }
        return bytes.len;
    }

    pub fn configure(self: UART, clock_source: ClockSource, baud_rate: BaudRate, data_bits: DataBits, parity: Parity, stop_bits: StopBits) void {
        rcc.setMuxerValue(self.mux, self.resolveClockSource(clock_source));
        rcc.enablePeriphery(self.rcc_switch);

        self.getPeriphEnablerField().set(0);
        self.setDataBits(data_bits);
        const cr1 = self.getReg(.CR1);
        (Field{ .reg = cr1, .shift = 15, .width = 1 }).set(@intFromEnum(Oversamling.Oversampling8));
        (Field{ .reg = cr1, .shift = 10, .width = 1 }).set(@intFromEnum(parity));
        (Field{ .reg = cr1, .shift = 2, .width = 1 }).set(1); // enable RX
        (Field{ .reg = cr1, .shift = 3, .width = 1 }).set(1); // enable TX
        (Field{ .reg = cr1, .shift = 29, .width = 1 }).set(1); // enable FIFO
        (Field{ .reg = self.getReg(.CR2), .shift = 12, .width = 2 }).set(@intFromEnum(stop_bits));
        const prescaler: u32 = 16;
        (Field{ .reg = self.getReg(.PRESC), .shift = 0, .width = 4 }).set(@intFromEnum(Prescaler.Prescale16));
        const src_clock_fq = rcc.getOutputFrequency(translateClockSource(clock_source));
        const baud = @intFromEnum(baud_rate);
        const brr: u32 = ((2 * src_clock_fq / prescaler) + (baud / 2)) / baud;
        (Field{ .reg = self.getReg(.BRR), .shift = 0, .width = 16 }).set((brr & 0xFFF0) + ((brr & 0x0F) >> 1));
        self.getPeriphEnablerField().set(1);
    }

    fn translateClockSource(own: ClockSource) RCC.ClockSource {
        return switch (own) {
            .HSI => .HSI,
        };
    }

    fn resolveClockSource(self: UART, clock_source: ClockSource) u3 {
        return switch (self.idx) {
            4 => switch (clock_source) {
                .HSI => 2,
            },
            else => unreachable,
        };
    }
};

const DDR = struct {
    ctrl_port: BusType,
    phyc_port: BusType,
    fn getCtrlReg(self: DDR, comptime reg: CtrlReg) BusType {
        return @intFromEnum(reg) + self.ctrl_port;
    }
    fn getPhycReg(self: DDR, comptime reg: PhycReg) BusType {
        return @intFromEnum(reg) + self.phyc_port;
    }
    fn setCtrlRegs(self: DDR, pairs: []const *const RegValues.CtrlRegValues.Pair) void {
        for (pairs) |reg_value| {
            @as(*volatile BusType, @ptrFromInt(self.ctrl_port + @intFromEnum(reg_value.reg))).* = reg_value.value;
        }
    }
    fn setPhycRegs(self: DDR, pairs: []const *const RegValues.PhycRegValues.Pair) void {
        for (pairs) |reg_value| {
            @as(*volatile BusType, @ptrFromInt(self.phyc_port + @intFromEnum(reg_value.reg))).* = reg_value.value;
        }
    }

    pub fn configure(self: DDR, comptime reg_values: RegValues) bool {
        pll2.enable(.R);
        const ddritfcr = rcc.getReg(.DDRITFCR);
        (Field{ .reg = ddritfcr, .shift = 8, .width = 1 }).set(0); // AXIDCGEN

        (Field{ .reg = ddritfcr, .shift = 14, .width = 1 }).set(1); // DDRCAPBRST
        (Field{ .reg = ddritfcr, .shift = 15, .width = 1 }).set(1); // DDRCAXIRST
        (Field{ .reg = ddritfcr, .shift = 16, .width = 1 }).set(1); // DDRCORERST
        (Field{ .reg = ddritfcr, .shift = 17, .width = 1 }).set(1); // DPHYAPBRST
        (Field{ .reg = ddritfcr, .shift = 18, .width = 1 }).set(1); // DPHYRST
        (Field{ .reg = ddritfcr, .shift = 19, .width = 1 }).set(1); // DPHYCTLRST

        (Field{ .reg = ddritfcr, .shift = 0, .width = 1 }).set(1); // DDRC1EN
        (Field{ .reg = ddritfcr, .shift = 2, .width = 1 }).set(1); // DDRC2EN
        (Field{ .reg = ddritfcr, .shift = 6, .width = 1 }).set(1); // DDRCAPBEN
        (Field{ .reg = ddritfcr, .shift = 9, .width = 1 }).set(1); // DDRPHYCAPBEN
        (Field{ .reg = ddritfcr, .shift = 4, .width = 1 }).set(1); // DDRPHYCEN

        (Field{ .reg = ddritfcr, .shift = 18, .width = 1 }).set(0); // DPHYRST
        (Field{ .reg = ddritfcr, .shift = 19, .width = 1 }).set(0); // DPHYCTLRST
        (Field{ .reg = ddritfcr, .shift = 14, .width = 1 }).set(0); // DDRCAPBRST
        mpu.udelay(2);

        (Field{ .reg = self.getCtrlReg(.DFIMISC), .shift = 0, .width = 1 }).set(0); // DFI_INIT_COMPLETE_EN
        self.setCtrlRegs(reg_values.ctrl.reg);
        self.setCtrlRegs(reg_values.ctrl.timing);
        self.setCtrlRegs(reg_values.ctrl.map);
        (Field{ .reg = self.getCtrlReg(.INIT0), .shift = 30, .width = 2 }).set(1); // SKIP_DRAM_INIT -> InitSkippedNormal
        self.setCtrlRegs(reg_values.ctrl.perf);

        (Field{ .reg = ddritfcr, .shift = 16, .width = 1 }).set(0); // DDRCORERST
        (Field{ .reg = ddritfcr, .shift = 15, .width = 1 }).set(0); // DDRCAXIRST
        (Field{ .reg = ddritfcr, .shift = 17, .width = 1 }).set(0); // DPHYAPBRST

        self.setPhycRegs(reg_values.phyc.reg);
        self.setPhycRegs(reg_values.phyc.timing);

        if (!self.phyInitWait())
            return false;

        const pir_dllsrst_asserted: BusType = 1 << 1;
        const pir_dlllock_asserted: BusType = 1 << 2;
        const pir_zcal_asserted: BusType = 1 << 3;
        const pir_itmsrst_asserted: BusType = 1 << 4;
        const pir_draminit_asserted: BusType = 1 << 6;
        const pir_icpc_asserted: BusType = 1 << 16;
        const pir_init_asserted: BusType = 1 << 0;
        var pir_reg_value = pir_dllsrst_asserted | pir_dlllock_asserted | pir_zcal_asserted | pir_itmsrst_asserted | pir_draminit_asserted | pir_icpc_asserted | pir_init_asserted;
        const is_ddr3: bool = (Field{ .reg = self.getCtrlReg(.MSTR), .shift = 0, .width = 1 }).isAsserted(); // DDR3?
        if (is_ddr3) {
            const pir_dramrst_asserted: BusType = 1 << 5;
            pir_reg_value |= pir_dramrst_asserted;
        }
        @as(*volatile BusType, @ptrFromInt(self.getPhycReg(PhycReg.PIR))).* = pir_reg_value;
        mpu.udelay(10);
        if (!self.phyInitWait())
            return false;

        const dfi_init_complete_en = Field{ .reg = self.getCtrlReg(.DFIMISC), .shift = 0, .width = 1 };
        self.startSwDone();
        dfi_init_complete_en.set(1);
        self.waitSwDone();

        self.normalOpModeWait();
        const dis_auto_refresh = Field{ .reg = self.getCtrlReg(.RFSHCTL3), .shift = 0, .width = 1 };
        const selfref_en = Field{ .reg = self.getCtrlReg(.PWRCTL), .shift = 0, .width = 1 };
        const powerdown_en = Field{ .reg = self.getCtrlReg(.PWRCTL), .shift = 1, .width = 1 };

        const dis_auto_refresh_value = dis_auto_refresh.get();
        const selfref_en_value = selfref_en.get();
        const powerdown_en_value = powerdown_en.get();
        const en_dfi_dram_clk_disable_value = (Field{ .reg = self.getCtrlReg(.PWRCTL), .shift = 3, .width = 1 }).get();

        self.startSwDone();
        dis_auto_refresh.set(1);
        selfref_en.set(0);
        powerdown_en.set(0);
        dfi_init_complete_en.set(0);
        self.waitSwDone();

        const pir_qstrn_asserted: BusType = 1 << 7;
        pir_reg_value = pir_qstrn_asserted | pir_init_asserted;
        if (is_ddr3) {
            const pir_rvtrn_asserted: BusType = 1 << 8;
            pir_reg_value |= pir_rvtrn_asserted;
        }
        @as(*volatile BusType, @ptrFromInt(self.getPhycReg(PhycReg.PIR))).* = pir_reg_value;
        mpu.udelay(10);
        if (!self.phyInitWait())
            return false;

        self.startSwDone();
        dis_auto_refresh.set(dis_auto_refresh_value);
        selfref_en.set(selfref_en_value);
        powerdown_en.set(powerdown_en_value);
        dfi_init_complete_en.set(1);
        self.waitSwDone();

        if (en_dfi_dram_clk_disable_value == 0) {
            (Field{ .reg = ddritfcr, .shift = 20, .width = 3 }).set(1); // DDRCKMOD = AutoSelfRef
            self.startSwDone();
            (Field{ .reg = self.getCtrlReg(.HWLPCTL), .shift = 0, .width = 1 }).set(1); // HW_LP_EN
            (Field{ .reg = self.getCtrlReg(.PWRTMG), .shift = 0, .width = 5 }).set(0x10); // POWERDOWN_TO_X32
            (Field{ .reg = self.getCtrlReg(.PWRTMG), .shift = 16, .width = 8 }).set(0x1); // SELFREF_TO_X32
            (Field{ .reg = self.getCtrlReg(.PWRCTL), .shift = 3, .width = 1 }).set(1); // EN_DFI_DRAM_CLK_DISABLE
            if (selfref_en_value == 1)
                selfref_en.set(1);
            dfi_init_complete_en.set(1);
            self.waitSwDone();
        }

        (Field{ .reg = self.getCtrlReg(.PCTRL_0), .shift = 0, .width = 1 }).set(1); // PORT_EN
        (Field{ .reg = self.getCtrlReg(.PCTRL_1), .shift = 0, .width = 1 }).set(1); // PORT_EN

        (Field{ .reg = ddritfcr, .shift = 8, .width = 1 }).set(1); // AXIDCGEN

        return true;
    }

    fn normalOpModeWait(self: DDR) void {
        const operating_mode = Field{ .reg = self.getCtrlReg(.STAT), .rw = .ReadOnly, .shift = 0, .width = 3 };
        const selfref_type = Field{ .reg = self.getCtrlReg(.STAT), .rw = .ReadOnly, .shift = 4, .width = 2 };
        while (true) {
            if (operating_mode.get() == 1) // Normal
                return;
            if (operating_mode.get() == 3 and selfref_type.get() == 3) // SelfRefresh & AutoSelfRefresh
                return;
        }
    }

    fn startSwDone(self: DDR) void {
        (Field{ .reg = self.getCtrlReg(.SWCTL), .shift = 0, .width = 1 }).set(0); // SW_DONE
    }
    fn waitSwDone(self: DDR) void {
        (Field{ .reg = self.getCtrlReg(.SWCTL), .shift = 0, .width = 1 }).set(1); // SW_DONE
        const sw_done_ack = Field{ .reg = self.getCtrlReg(.SWSTAT), .rw = .ReadOnly, .shift = 0, .width = 1 };
        while (sw_done_ack.isCleared()) {}
    }

    fn phyInitWait(self: DDR) bool {
        const pgsr = self.getPhycReg(PhycReg.PGSR);
        const dterr = Field{ .reg = pgsr, .shift = 5, .width = 1 };
        const dtierr = Field{ .reg = pgsr, .shift = 6, .width = 1 };
        const dfterr = Field{ .reg = pgsr, .shift = 7, .width = 1 };
        const dverr = Field{ .reg = pgsr, .shift = 8, .width = 1 };
        const dvierr = Field{ .reg = pgsr, .shift = 9, .width = 1 };
        const idone = Field{ .reg = pgsr, .shift = 0, .width = 1 };
        while (true) {
            if (dterr.isAsserted() or dtierr.isAsserted() or dfterr.isAsserted() or dverr.isAsserted() or dvierr.isAsserted())
                return false;
            if (idone.isAsserted())
                return true;
        }
    }

    pub const RegValues = struct {
        ctrl: CtrlRegValues,
        phyc: PhycRegValues,

        pub const CtrlRegValues = struct {
            pub const Pair = struct { reg: CtrlReg, value: BusType };
            map: []const *const Pair,
            timing: []const *const Pair,
            perf: []const *const Pair,
            reg: []const *const Pair,
        };
        pub const PhycRegValues = struct {
            pub const Pair = struct { reg: PhycReg, value: BusType };
            reg: []const *const Pair,
            timing: []const *const Pair,
        };
    };
    const CtrlReg = enum(BusType) {
        MSTR = 0x0,
        STAT = 0x4,
        MRCTRL0 = 0x10,
        MRCTRL1 = 0x14,
        DERATEEN = 0x20,
        DERATEINT = 0x24,
        PWRCTL = 0x30,
        PWRTMG = 0x34,
        HWLPCTL = 0x38,
        RFSHCTL0 = 0x50,
        RFSHCTL3 = 0x60,
        RFSHTMG = 0x64,
        CRCPARCTL0 = 0xC0,
        INIT0 = 0xD0,
        DRAMTMG0 = 0x100,
        DRAMTMG1 = 0x104,
        DRAMTMG2 = 0x108,
        DRAMTMG3 = 0x10C,
        DRAMTMG4 = 0x110,
        DRAMTMG5 = 0x114,
        DRAMTMG6 = 0x118,
        DRAMTMG7 = 0x11C,
        DRAMTMG8 = 0x120,
        DRAMTMG14 = 0x138,
        ZQCTL0 = 0x180,
        DFITMG0 = 0x190,
        DFITMG1 = 0x194,
        DFILPCFG0 = 0x198,
        DFIUPD0 = 0x1A0,
        DFIUPD1 = 0x1A4,
        DFIUPD2 = 0x1A8,
        DFIMISC = 0x1B0,
        DFIPHYMSTR = 0x1C4,
        ADDRMAP1 = 0x204,
        ADDRMAP2 = 0x208,
        ADDRMAP3 = 0x20C,
        ADDRMAP4 = 0x210,
        ADDRMAP5 = 0x214,
        ADDRMAP6 = 0x218,
        ADDRMAP9 = 0x224,
        ADDRMAP10 = 0x228,
        ADDRMAP11 = 0x22C,
        ODTCFG = 0x240,
        ODTMAP = 0x244,
        SCHED = 0x250,
        SCHED1 = 0x254,
        PERFHPR1 = 0x25C,
        PERFLPR1 = 0x264,
        PERFWR1 = 0x26C,
        DBG0 = 0x300,
        DBG1 = 0x304,
        DBGCMD = 0x30C,
        SWCTL = 0x320,
        SWSTAT = 0x324,
        POISONCFG = 0x36C,
        PCCFG = 0x400,
        PCFGR_0 = 0x404,
        PCFGW_0 = 0x408,
        PCTRL_0 = 0x490,
        PCFGQOS0_0 = 0x494,
        PCFGQOS1_0 = 0x498,
        PCFGWQOS0_0 = 0x49C,
        PCFGWQOS1_0 = 0x4A0,
        PCFGR_1 = 0x404 + 0xB0,
        PCFGW_1 = 0x408 + 0xB0,
        PCTRL_1 = 0x490 + 0xB0,
        PCFGQOS0_1 = 0x494 + 0xB0,
        PCFGQOS1_1 = 0x498 + 0xB0,
        PCFGWQOS0_1 = 0x49C + 0xB0,
        PCFGWQOS1_1 = 0x4A0 + 0xB0,
    };
    const PhycReg = enum(BusType) {
        PIR = 0x4,
        PGCR = 0x8,
        PGSR = 0xC,
        PTR0 = 0x18,
        PTR1 = 0x1C,
        PTR2 = 0x20,
        ACIOCR = 0x24,
        DXCCR = 0x28,
        DSGCR = 0x2C,
        DCR = 0x30,
        DTPR0 = 0x34,
        DTPR1 = 0x38,
        DTPR2 = 0x3C,
        MR0 = 0x40,
        MR1 = 0x44,
        MR2 = 0x48,
        MR3 = 0x4C,
        ODTCR = 0x50,
        ZQ0CR1 = 0x184,
        DX0GCR = 0x1C0,
        DX1GCR = 0x200,
        DX2GCR = 0x240,
        DX3GCR = 0x280,
    };
};

const MPU = struct {
    mux: RCC.ClockMuxer,
    const ClockSource = enum(u2) { HSI = 0, HSE = 1, PLL1 = 2, PLL1DIV = 3 };
    const PLL1Divider = enum(u3) { Div1 = 0, Div2 = 1, Div4 = 2, Div8 = 3, Div16 = 4 };

    pub fn configure(self: MPU, clock_source: ClockSource, pll1_div: ?PLL1Divider) void {
        rcc.setMuxerValue(self.mux, @intFromEnum(clock_source));
        if (clock_source == .PLL1DIV or clock_source == .PLL1)
            pll1.enable(.P);
        if (clock_source == .PLL1DIV) {
            const divr = rcc.getReg(.MPCKDIVR);
            (Field{ .reg = divr, .shift = 0, .width = 3 }).set(@intFromEnum(pll1_div orelse .Div1));
            const rdy = Field{ .reg = divr, .shift = 31, .width = 1 };
            while (rdy.isCleared()) {}
        }
    }

    inline fn readCycleCounter() u32 {
        var value: u32 = undefined;
        asm volatile ("mrc p15, 0, %[value], c9, c13, 0"
            : [value] "=r" (value),
        );
        return value;
    }
    pub fn udelay(self: MPU, usec: u32) void {
        asm volatile ("mrc p15, 0, r0, c9, c12, 0; orr r0, r0, #5; mcr p15, 0, r0, c9, c12, 0" ::: "r0"); // reset cycle counter
        asm volatile ("mrc p15, 0, r0, c9, c12, 1; orr r0, r0, #0x80000000; mcr p15, 0, r0, c9, c12, 1;" ::: "r0"); // enable cycle counter
        const delay = self.getSystemClockHz() / 1_000_000 * usec;
        while (readCycleCounter() < delay) {}
    }
    pub fn resetUsCounter(self: MPU) void {
        _ = self;
        asm volatile ("mrc p15, 0, r0, c9, c12, 0; orr r0, r0, #5; mcr p15, 0, r0, c9, c12, 0" ::: "r0"); // reset cycle counter
    }
    pub fn readUsCounter(self: MPU) u32 {
        _ = self;
        return readCycleCounter();
    }

    pub fn getSystemClockHz(self: MPU) BusType {
        return switch (@as(ClockSource, @enumFromInt(rcc.getMuxerValue(self.mux)))) {
            .HSI => rcc.getOutputFrequency(.HSI),
            .HSE => rcc.getOutputFrequency(.HSE),
            .PLL1 => pll1.getOutputFrequency(.P),
            .PLL1DIV => pll1.getOutputFrequency(.P) >> @truncate((Field{ .reg = rcc.getReg(.MPCKDIVR), .shift = 0, .width = 3 }).get()),
        };
    }
};

const AXI = struct {
    mux: RCC.ClockMuxer,
    const ClockSource = enum(u2) { HSI = 0, HSE = 1, PLL2 = 2, Gated };
    const Output = enum { ACLK, HCLK5, HCLK6, PCLK4, PCLK5 };
    pub fn configure(self: AXI, clock_source: ClockSource, prescaler: u3, apb4div: u3, apb5div: u3) void {
        rcc.setMuxerValue(self.mux, @intFromEnum(clock_source));
        if (clock_source == .PLL2)
            pll2.enable(.P);
        (Field{ .reg = rcc.getReg(.AXIDIVR), .shift = 0, .width = 3 }).set(prescaler);
        while ((Field{ .reg = rcc.getReg(.AXIDIVR), .shift = 31, .width = 1 }).isCleared()) {}
        (Field{ .reg = rcc.getReg(.APB4DIVR), .shift = 0, .width = 3 }).set(apb4div);
        while ((Field{ .reg = rcc.getReg(.APB4DIVR), .shift = 31, .width = 1 }).isCleared()) {}
        (Field{ .reg = rcc.getReg(.APB5DIVR), .shift = 0, .width = 3 }).set(apb5div);
        while ((Field{ .reg = rcc.getReg(.APB5DIVR), .shift = 31, .width = 1 }).isCleared()) {}
    }
    pub fn getOutputFrequency(self: AXI, output: Output) BusType {
        var ref_clock_hz: BusType = switch (@as(ClockSource, @enumFromInt(rcc.getMuxerValue(self.mux)))) {
            .HSI => rcc.getOutputFrequency(.HSI),
            .HSE => rcc.getOutputFrequency(.HSE),
            .PLL2 => pll2.getOutputFrequency(.P),
            .Gated => return 0,
        };

        while ((Field{ .reg = rcc.getReg(.AXIDIVR), .shift = 31, .width = 1 }).isCleared()) {}
        var prescaler = (Field{ .reg = rcc.getReg(.AXIDIVR), .shift = 0, .width = 3 }).get();
        if (prescaler > 3)
            prescaler = 3;
        ref_clock_hz /= (prescaler + 1);

        switch (output) {
            .ACLK, .HCLK5, .HCLK6 => return ref_clock_hz,
            .PCLK4, .PCLK5 => {
                const reg = switch (output) {
                    .PCLK4 => RCC.Reg.APB4DIVR,
                    .PCLK5 => RCC.Reg.APB5DIVR,
                    else => unreachable,
                };
                while ((Field{ .reg = rcc.getReg(reg), .shift = 31, .width = 1 }).isCleared()) {}
                var div = (Field{ .reg = rcc.getReg(reg), .shift = 0, .width = 3 }).get();
                if (div > 4)
                    div = 4;
                return ref_clock_hz / (@as(BusType, 1) << @truncate(div));
            },
        }
    }
};

const PLL = struct {
    cfg1r: RCC.Reg,
    cfg2r: RCC.Reg,
    fracr: RCC.Reg,
    cr: RCC.Reg,
    mux: RCC.ClockMuxer,
    const ClockSource = enum(u2) { HSI = 0, HSE = 1, CSI = 2 }; // TODO: support enum I2S_CKIN = 3
    const Output = enum(FieldShiftType) { P = 4, Q = 5, R = 6 };
    pub fn configure(self: PLL, clock_source: ?ClockSource, m: u6, n: u9, fracv: u13, p: u7, q: u7, r: u7) void {
        if (clock_source) |src| {
            rcc.setMuxerValue(self.mux, @intFromEnum(src));
        }
        const cfg1r = rcc.getReg(self.cfg1r);
        const cfg2r = rcc.getReg(self.cfg2r);
        const fracr = rcc.getReg(self.fracr);
        const fracle = Field{ .reg = fracr, .shift = 16, .width = 1 };
        const cr = rcc.getReg(self.cr);
        const pllrdy = Field{ .reg = cr, .shift = 1, .width = 1, .rw = .ReadOnly };
        (Field{ .reg = cfg1r, .shift = 16, .width = 6 }).set(m);
        (Field{ .reg = cfg1r, .shift = 0, .width = 9 }).set(n);
        fracle.set(0);
        (Field{ .reg = fracr, .shift = 3, .width = 13 }).set(fracv);
        fracle.set(1);
        (Field{ .reg = cfg2r, .shift = 0, .width = 7 }).set(p);
        (Field{ .reg = cfg2r, .shift = 8, .width = 7 }).set(q);
        (Field{ .reg = cfg2r, .shift = 16, .width = 7 }).set(r);
        (Field{ .reg = cr, .shift = 0, .width = 1 }).set(1); // PLLON
        while (pllrdy.isCleared()) {}
    }
    pub fn enable(self: PLL, output: Output) void {
        (Field{ .reg = rcc.getReg(self.cr), .shift = @intFromEnum(output), .width = 1 }).set(1);
    }
    fn isOutputEnabled(self: PLL, output: Output) bool {
        return (Field{ .reg = rcc.getReg(self.cr), .shift = @intFromEnum(output), .width = 1 }).isAsserted();
    }
    pub fn getOutputFrequency(self: PLL, output: Output) BusType {
        if (!self.isOutputEnabled(output))
            return 0;
        const divm = (Field{ .reg = rcc.getReg(self.cfg1r), .shift = 16, .width = 6 }).get();
        const divn = (Field{ .reg = rcc.getReg(self.cfg1r), .shift = 0, .width = 9 }).get();
        const frac = (Field{ .reg = rcc.getReg(self.fracr), .shift = 3, .width = 13 }).get();
        const div_out = switch (output) {
            .P => (Field{ .reg = rcc.getReg(self.cfg2r), .shift = 0, .width = 7 }).get(),
            .Q => (Field{ .reg = rcc.getReg(self.cfg2r), .shift = 8, .width = 7 }).get(),
            .R => (Field{ .reg = rcc.getReg(self.cfg2r), .shift = 16, .width = 7 }).get(),
        };
        const ref_fq_hz = switch (@as(ClockSource, @enumFromInt(rcc.getMuxerValue(self.mux)))) {
            .HSI => rcc.getOutputFrequency(.HSI),
            .HSE => rcc.getOutputFrequency(.HSE),
            .CSI => rcc.getOutputFrequency(.CSI),
        } / (divm + 1) * 2;
        return ((ref_fq_hz / 1_000 * frac / 8192) * 1_000 + ref_fq_hz * (divn + 1)) / 2 / (div_out + 1);
    }
};

const TZC = struct {
    port: BusType,
    const Reg = enum(BusType) {
        GATE_KEEPER = 0x8,
        SPECULATION_CTRL = 0xC,
        ATTRIBUTE0 = 0x110,
        ID_ACCESS0 = 0x114,
    };

    fn getReg(self: TZC, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    pub fn initSecureDDRAccess(self: TZC) void {
        rcc.enablePeriphery(.{ .set_reg = .MP_APB5ENSETR, .shift = 11 }); // TZC1EN
        rcc.enablePeriphery(.{ .set_reg = .MP_APB5ENSETR, .shift = 12 }); // TZC2EN
        const openreq_flt0 = Field{ .reg = self.getReg(.GATE_KEEPER), .shift = 0, .width = 1 };
        const openreq_flt1 = Field{ .reg = self.getReg(.GATE_KEEPER), .shift = 1, .width = 1 };
        openreq_flt0.set(0); // Open
        openreq_flt1.set(0); // Open
        (Field{ .reg = self.getReg(.ID_ACCESS0), .shift = 0, .width = 16 }).set(0xFFFF); // NSAID_WR_EN
        (Field{ .reg = self.getReg(.ID_ACCESS0), .shift = 16, .width = 16 }).set(0xFFFF); // NSAID_RD_EN
        (Field{ .reg = self.getReg(.ATTRIBUTE0), .shift = 31, .width = 1 }).set(1); // S_WR_EN
        (Field{ .reg = self.getReg(.ATTRIBUTE0), .shift = 30, .width = 1 }).set(1); // S_RD_EN
        openreq_flt0.set(1); // Close
        openreq_flt1.set(1); // Close
        rcc.enablePeriphery(.{ .set_reg = .MP_APB5ENSETR, .shift = 12 }); // TZPCEN
        (Field{ .reg = self.getReg(.SPECULATION_CTRL), .shift = 0, .width = 1 }).set(1); // READSPEC_DISABLE
        (Field{ .reg = self.getReg(.SPECULATION_CTRL), .shift = 1, .width = 1 }).set(1); // WRITESPEC_DISABLE
    }
};

const RCC = struct {
    port: BusType,
    fn getReg(self: RCC, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    const ClockSource = enum { HSI, HSE, CSI };
    const ClockMuxer = struct {
        src: FieldDesc,
        rdy: ?FieldDesc = null,
        const FieldDesc = struct {
            reg: Reg,
            shift: FieldShiftType,
            width: FieldWidthType = 1,
        };
    };
    const PeripherySwitch = struct {
        set_reg: Reg,
        clr_reg: ?Reg = null,
        shift: FieldShiftType,
    };

    const HSEMode = enum { Crystal };

    pub fn enableHSE(self: RCC, mode: HSEMode, fq: u32) void {
        hse_fq_hz = fq;
        switch (mode) {
            .Crystal => {
                const ocenclrr = self.getReg(.OCENCLRR);
                (Field{ .reg = ocenclrr, .rw = .WriteOnly, .shift = 8, .width = 1 }).set(1); // HSE -> Off
                const hserdy = Field{ .reg = self.getReg(.OCRDYR), .rw = .ReadOnly, .shift = 8, .width = 1 };
                while (hserdy.isAsserted()) {}
                (Field{ .reg = ocenclrr, .rw = .WriteOnly, .shift = 10, .width = 1 }).set(1); // HSEBYP -> Off
                (Field{ .reg = self.getReg(.OCENSETR), .rw = .WriteOnly, .shift = 8, .width = 1 }).set(1); // HSE -> On
                while (hserdy.isCleared()) {}
            },
        }
    }

    fn getOutputFrequency(self: RCC, clock: ClockSource) u32 {
        const ocrdyr = self.getReg(.OCRDYR);
        const ocensetr = self.getReg(.OCENSETR);
        switch (clock) {
            .HSI => {
                if ((Field{ .reg = ocensetr, .shift = 0, .width = 1 }).isCleared())
                    return 0;
                const hsirdy = Field{ .reg = ocrdyr, .rw = .ReadOnly, .shift = 0, .width = 1 };
                const hsidivrdy = Field{ .reg = ocrdyr, .rw = .ReadOnly, .shift = 2, .width = 1 };
                while (hsirdy.isCleared()) {}
                while (hsidivrdy.isCleared()) {}
                const hsi_div: u2 = @truncate((Field{ .reg = self.getReg(.HSICFGR), .shift = 0, .width = 2 }).get());
                return hsi_fq_hz / (@as(u8, 1) << hsi_div);
            },
            .HSE => {
                if ((Field{ .reg = ocensetr, .shift = 8, .width = 1 }).isCleared())
                    return 0;
                const hserdy = Field{ .reg = ocrdyr, .rw = .ReadOnly, .shift = 8, .width = 1 };
                while (hserdy.isCleared()) {}
                return hse_fq_hz;
            },
            .CSI => {
                if ((Field{ .reg = ocensetr, .shift = 4, .width = 1 }).isCleared())
                    return 0;
                const csirdy = Field{ .reg = ocrdyr, .rw = .ReadOnly, .shift = 4, .width = 1 };
                while (csirdy.isCleared()) {}
                return csi_fq_hz;
            },
        }
    }

    fn setMuxerValue(self: RCC, mux: ClockMuxer, value: BusType) void {
        (Field{ .reg = self.getReg(mux.src.reg), .shift = mux.src.shift, .width = mux.src.width }).set(value);
        if (mux.rdy) |rdy| {
            while ((Field{ .reg = self.getReg(rdy.reg), .shift = rdy.shift, .width = 1 }).isCleared()) {}
        }
    }
    fn getMuxerValue(self: RCC, mux: ClockMuxer) BusType {
        if (mux.rdy) |rdy| {
            while ((Field{ .reg = self.getReg(rdy.reg), .shift = rdy.shift, .width = 1 }).isCleared()) {}
        }
        return (Field{ .reg = self.getReg(mux.src.reg), .shift = mux.src.shift, .width = mux.src.width }).get();
    }

    fn enablePeriphery(self: RCC, periph_switch: PeripherySwitch) void {
        (Field{ .reg = self.getReg(periph_switch.set_reg), .rw = .WriteOnly, .shift = periph_switch.shift, .width = 1 }).set(1);
    }

    fn disablePeriphery(self: RCC, periph_switch: PeripherySwitch) void {
        if (periph_switch.clr_reg) |clr_reg| {
            (Field{ .reg = self.getReg(clr_reg), .rw = .WriteOnly, .shift = periph_switch.shift, .width = 1 }).set(1);
        }
    }

    const Reg = enum(BusType) {
        OCENSETR = 0x0C, // RCC oscillator clock enable set register (RCC_OCENSETR)
        OCENCLRR = 0x10, // RCC oscillator clock enable clear register (RCC_OCENCLRR)
        HSICFGR = 0x18, // RCC HSI configuration register (RCC_HSICFGR)
        MPCKSELR = 0x20, // RCC MPU clock selection register (RCC_MPCKSELR)
        ASSCKSELR = 0x24, // RCC AXI sub-system clock selection register (RCC_ASSCKSELR)
        MPCKDIVR = 0x2C, // RCC MPU clock divider register (RCC_MPCKDIVR)
        RCK12SELR = 0x28, // RCC PLL 1 and 2 reference clock selection register (RCC_RCK12SELR)
        AXIDIVR = 0x30, // RCC AXI clock divider register (RCC_AXIDIVR)
        APB4DIVR = 0x3C, // RCC APB4 clock divider register (RCC_APB4DIVR)
        APB5DIVR = 0x40, // RCC APB5 clock divider register (RCC_APB5DIVR)
        MSSCKSELR = 0x48, // RCC MCU sub-system clock selection register (RCC_MSSCKSELR)
        PLL1CR = 0x80, // RCC PLL1 control register (RCC_PLL1CR)
        PLL1CFGR1 = 0x84, // RCC PLL1 configuration register 1 (RCC_PLL1CFGR1)
        PLL1CFGR2 = 0x88, // RCC PLL1 configuration register 2 (RCC_PLL1CFGR2)
        PLL1FRACR = 0x8C, // RCC PLL1 fractional register (RCC_PLL1FRACR)
        PLL1CSGR = 0x90, // RCC PLL1 clock spreading generator register (RCC_PLL1CSGR)
        PLL2CR = 0x94, // RCC PLL2 control register (RCC_PLL2CR)
        PLL2CFGR1 = 0x98, // RCC PLL2 configuration register 1 (RCC_PLL2CFGR1)
        PLL2CFGR2 = 0x9C, // RCC PLL2 configuration register 2 (RCC_PLL2CFGR2)
        PLL2FRACR = 0xA0, // RCC PLL2 fractional register (RCC_PLL2FRACR)
        PLL2CSGR = 0xA4, // RCC PLL2 clock spreading generator register (RCC_PLL2CSGR)
        DDRITFCR = 0xD8, // RCC DDR interface control register (RCC_DDRITFCR)
        BDCR = 0x140, // RCC backup domain control register (RCC_BDCR)
        AHB6RSTSETR = 0x198, // RCC AHB6 peripheral reset set register (RCC_AHB6RSTSETR)
        AHB6RSTCLRR = 0x19C, // RCC AHB6 peripheral reset set register (RCC_AHB6RSTSETR)
        MP_APB5ENSETR = 0x208, // RCC APB5 peripheral enable for MPU set register
        MP_APB5ENCLRR = 0x20C, // RCC APB5 peripheral enable for MPU clear register
        MP_AHB6ENSETR = 0x218, // RCC AHB6 peripheral enable for MPU set register
        MP_AHB6ENCLRR = 0x21C, // RCC AHB6 peripheral enable for MPU clear register
        OCRDYR = 0x808, // RCC oscillator clock ready register (RCC_OCRDYR)
        RCK3SELR = 0x820, // RCC PLL 3 reference clock selection register (RCC_RCK3SELR)
        RCK4SELR = 0x824, // RCC PLL 3 reference clock selection register (RCC_RCK3SELR)
        PLL3CR = 0x880, // RCC PLL3 control register (RCC_PLL3CR)
        PLL3CFGR1 = 0x884, // RCC PLL3 configuration register 1 (RCC_PLL3CFGR1)
        PLL3CFGR2 = 0x888, // RCC PLL3 configuration register 2 (RCC_PLL3CFGR2)
        PLL3FRACR = 0x88C, // RCC PLL3 fractional register (RCC_PLL3FRACR)
        PLL4CR = 0x894, // RCC PLL4 control register (RCC_PLL4CR)
        PLL4CFGR1 = 0x898, // RCC PLL4 configuration register 1 (RCC_PLL4CFGR1)
        PLL4CFGR2 = 0x89C, // RCC PLL4 configuration register 2 (RCC_PLL4CFGR2)
        PLL4FRACR = 0x8A0, // RCC PLL4 fractional register (RCC_PLL4FRACR)
        UART24CKSELR = 0x8E8, // RCC UART2,4 kernel clock selection register (RCC_UART24CKSELR)
        SDMMC12CKSELR = 0x8F4, // RCC SDMMC1 and 2 kernel clock selection register
        MP_APB1ENSETR = 0xA00, // RCC APB1 peripheral enable for MPU set register (RCC_MP_APB1ENSETR)
        MP_AHB4ENSETR = 0xA28, // RCC AHB4 peripheral enable for MPU set register (RCC_MP_AHB4ENSETR)
        MP_AHB4ENCLRR = 0xA2C, // RCC AHB4 peripheral enable for MPU clear register (RCC_MP_AHB4ENCLRR)
    };
};

const GPIO = struct {
    port: BusType,
    rcc_switch: RCC.PeripherySwitch,
    fn getReg(self: GPIO, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    const Reg = enum(BusType) {
        MODER = 0x0,
        OTYPER = 0x4,
        OSPEEDR = 0x8,
        PUPDR = 0xC,
        IDR = 0x10,
        BRSR = 0x18,
        AFR = 0x20,
    };
    const MODE = enum(u2) { Input = 0, Output = 1, AltFunc = 2, Analog = 3 };
    const OTYPE = enum(u1) { PushPull = 0, OpenDrain = 1 };
    const OSPEED = enum(u2) { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };
    const PUPD = enum(u2) { Disabled = 0, PullUp = 1, PullDown = 2, Reserved = 3 };

    pub fn pin(self: GPIO, nr: u4) Pin {
        return .{ .gpio = self, .pin = nr };
    }

    const Pin = struct {
        gpio: GPIO,
        pin: u4,

        pub fn configure(self: Pin, mode: MODE, otype: OTYPE, ospeed: OSPEED, pupd: PUPD, af: u4) void {
            if (mode == .AltFunc)
                rcc.enablePeriphery(self.gpio.rcc_switch);
            self.getMODER().set(@intFromEnum(mode));
            self.getOTYPER().set(@intFromEnum(otype));
            self.getOSPEEDR().set(@intFromEnum(ospeed));
            self.getPUPDR().set(@intFromEnum(pupd));
            if (mode == .AltFunc)
                self.getAFR().set(af);
        }
        pub fn isAsserted(self: Pin) bool {
            return !(self.getIDR().get() == 0);
        }
        pub fn assert(self: Pin) void {
            self.getBSR().set(1);
        }
        pub fn reset(self: Pin) void {
            self.getBRR().set(1);
        }
        fn getMODER(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.MODER), .rw = .ReadWrite, .shift = @as(FieldShiftType, self.pin) * 2, .width = 2 };
        }
        fn getOTYPER(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.OTYPER), .rw = .ReadWrite, .shift = @as(FieldShiftType, self.pin), .width = 1 };
        }
        fn getOSPEEDR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.OSPEEDR), .rw = .ReadWrite, .shift = @as(FieldShiftType, self.pin) * 2, .width = 2 };
        }
        fn getPUPDR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.PUPDR), .rw = .ReadWrite, .shift = @as(FieldShiftType, self.pin) * 2, .width = 2 };
        }
        fn getAFR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.AFR) + (self.pin / 8) * 4, .rw = .ReadWrite, .shift = (@as(FieldShiftType, self.pin) % 8) * 4, .width = 4 };
        }
        fn getBSR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.BRSR), .rw = .WriteOnly, .shift = @as(FieldShiftType, self.pin), .width = 1 };
        }
        fn getBRR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.BRSR), .rw = .WriteOnly, .shift = 0x10 + @as(FieldShiftType, self.pin), .width = 1 };
        }
        fn getIDR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.IDR), .rw = .ReadOnly, .shift = @as(FieldShiftType, self.pin), .width = 1 };
        }
    };
};
