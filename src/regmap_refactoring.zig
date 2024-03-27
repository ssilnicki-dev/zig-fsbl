// platform specifics
const BusType: type = u32;
const FieldShiftType = switch (BusType) {
    u32 => u5,
    u64 => u6,
    else => unreachable,
};
const FieldWidthType = FieldShiftType;
const hsi_fq_hz: u32 = 64000000;

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
    apb1: struct {
        const base: BusType = 0x40000000;
        uart4: UART = .{
            .port = 0x10000 + base,
            .idx = 4,
            .mux = .{ .src = .{ .reg = .UART24CKSELR, .shift = 0, .width = 3 } },
            .rcc_switch = .{ .set_reg = .MP_APB1ENSETR, .clr_reg = null, .shift = 16 },
        },
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

// private peripheries implementation
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
        const src_clock_fq = rcc.getClockFrequency(translateClockSource(clock_source));
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

const MPU = struct {
    mux: RCC.ClockMuxer,
    const ClockSource = enum(u2) { HSI = 0, HSE = 1, PLL1 = 2, PLL1DIV = 3 };
    pub fn configure(self: MPU, clock_source: ClockSource) void {
        rcc.setMuxerValue(self.mux, @intFromEnum(clock_source));
    }
};

const AXI = struct {
    mux: RCC.ClockMuxer,
    const ClockSource = enum(u2) { HSI = 0, HSE = 1, PLL2 = 2 };
    pub fn configure(self: AXI, clock_source: ClockSource, prescaler: u3, apb4div: u3, apb5div: u3) void {
        rcc.setMuxerValue(self.mux, @intFromEnum(clock_source));
        (Field{ .reg = rcc.getReg(.AXIDIVR), .shift = 0, .width = 3 }).set(prescaler);
        while ((Field{ .reg = rcc.getReg(.AXIDIVR), .shift = 31, .width = 1 }).isCleared()) {}
        (Field{ .reg = rcc.getReg(.APB4DIVR), .shift = 0, .width = 3 }).set(apb4div);
        while ((Field{ .reg = rcc.getReg(.APB4DIVR), .shift = 31, .width = 1 }).isCleared()) {}
        (Field{ .reg = rcc.getReg(.APB5DIVR), .shift = 0, .width = 3 }).set(apb5div);
        while ((Field{ .reg = rcc.getReg(.APB5DIVR), .shift = 31, .width = 1 }).isCleared()) {}
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
};
const RCC = struct {
    port: BusType,
    hse_fq_hz: HSEfqHz = .Zero,
    const HSEfqHz = enum(BusType) { Zero = 0, Crystal24MHz = 24_000_000 };
    fn getReg(self: RCC, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    const ClockSource = enum { HSI };
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

    pub fn enableHSE(self: RCC, mode: HSEMode, fq: HSEfqHz) void {
        @constCast(&self).hse_fq_hz = fq;
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

    fn getClockFrequency(self: RCC, clock: ClockSource) u32 {
        const ocrdyr = self.getReg(.OCRDYR);
        switch (clock) {
            .HSI => {
                const hsirdy = Field{ .reg = ocrdyr, .rw = .ReadOnly, .shift = 0, .width = 1 };
                const hsidivrdy = Field{ .reg = ocrdyr, .rw = .ReadOnly, .shift = 2, .width = 1 };
                while (!hsirdy.isAsserted()) {}
                while (!hsidivrdy.isAsserted()) {}
                const hsi_div: u2 = @truncate((Field{ .reg = self.getReg(.HSICFGR), .shift = 0, .width = 2 }).get());
                return hsi_fq_hz / (@as(u8, 1) << hsi_div);
            },
        }
    }

    fn setMuxerValue(self: RCC, mux: ClockMuxer, value: BusType) void {
        (Field{ .reg = self.getReg(mux.src.reg), .shift = mux.src.shift, .width = mux.src.width }).set(value);
        if (mux.rdy) |rdy| {
            while ((Field{ .reg = self.getReg(rdy.reg), .shift = rdy.shift, .width = 1 }).isCleared()) {}
        }
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