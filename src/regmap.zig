// TODO: stick below typedefs to platform specifics
const bus_type: type = u32;
const enum_type: type = @TypeOf(enum {});

const HSI_FRIQUENCY = 64000000;

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

    inline fn getMask(comptime self: Field) bus_type {
        return @as(bus_type, (1 << @bitSizeOf(self.width)) - 1);
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
        var v = addr.*;
        const mask = self.getMask();

        v &= ~(mask << self.shift);
        v |= (value & mask) << self.shift;
        addr.* = v;
    }

    inline fn setEnumValue(comptime self: Field, value: self.values) void {
        comptime if (@typeInfo(self.values).Enum.fields.len == 0)
            unreachable;
        self.setValueImpl(@intFromEnum(value));
    }

    inline fn setIntValue(comptime self: Field, value: bus_type) void {
        comptime if (@typeInfo(self.values).Enum.fields.len > 0)
            unreachable;
        comptime if (self.rw == .ReadOnly)
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

    const current_port = @intFromEnum(port);

    return comptime switch (current_port) {
        rcc => enum(bus_type) {
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
    AHB4 = 0x50000000,
    APB1 = 0x40000000,

    pub fn ports(bus: @This()) enum_type {
        return comptime switch (bus) {
            .APB5 => enum(bus_type) {
                pub fn api(port: @This()) enum_type {
                    return API(port);
                }

                RTC = Port(0x4000, bus),
                fn regs(port: @This()) enum_type {
                    return comptime switch (port) {
                        .RTC => enum(bus_type) {},
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
                            fn fields(reg: @This()) enum_type {
                                const addr = @intFromEnum(reg);
                                return comptime switch (reg) {
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
                                        const HSEON = Field{ .rw = .WriteOnly, .width = u1, .shift = 8, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                        const HSIKERON = Field{ .rw = .WriteOnly, .width = u1, .shift = 1, .reg = addr, .values = enum(u1) {
                                            Set = 1,
                                        } };
                                    },
                                    .OCENCLRR => enum {
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
