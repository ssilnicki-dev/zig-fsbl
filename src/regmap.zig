// TODO: stick below typedefs to platform specifics
const bus_type: type = u32;
const enum_type: type = @TypeOf(enum {});

pub const reg_type = bus_type;

const Field = struct {
    pub const shift_type = switch (reg_type) {
        u32 => u5,
        u64 => u6,
        else => unreachable, // >64bit arch??? not so fast....
    };
    reg: reg_type,
    width: type,
    rw: enum {
        ReadOnly,
        WriteOnly,
        ReadWrite,
    },
    shift: shift_type,
    values: enum_type,

    inline fn getMask(comptime self: Field) reg_type {
        return @as(reg_type, (1 << @bitSizeOf(self.width)) - 1);
    }

    pub fn set(comptime self: Field, value: anytype) void {
        switch (@TypeOf(value)) {
            self.values => self.setEnumValue(value),
            else => self.setIntValue(value),
        }
    }

    inline fn setValueImpl(comptime self: Field, value: reg_type) void {
        const addr: *volatile reg_type = @ptrFromInt(self.reg);
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

    inline fn setIntValue(comptime self: Field, value: reg_type) void {
        comptime if (@typeInfo(self.values).Enum.fields.len > 0)
            unreachable;
        comptime if (self.rw == .ReadOnly)
            unreachable;
        self.setValueImpl(value);
    }

    pub fn getIntValue(comptime self: Field) reg_type {
        comptime if (self.rw == .WriteOnly)
            unreachable;
        return (@as(*volatile reg_type, @ptrFromInt(self.reg)).* >> self.shift) & ((1 << @bitSizeOf(self.width)) - 1);
    }

    pub fn getEnumValue(comptime self: Field) self.values {
        comptime if (@typeInfo(self.values).Enum.fields.len == 0)
            unreachable;

        return @as(self.values, @enumFromInt(self.getIntValue()));
    }
};
pub const Bus = enum(reg_type) {
    APB5 = 0x5C000000,
    AHB4 = 0x50000000,
    APB1 = 0x40000000,

    pub fn ports(bus: @This()) enum_type {
        return comptime switch (bus) {
            @This().APB5 => enum(reg_type) {
                RTC = Port(0x4000, bus),
                pub fn api(port: @This()) enum_type {
                    _ = port;
                }
                fn regs(port: @This()) enum_type {
                    return comptime switch (port) {
                        @This().RTC => enum(reg_type) {},
                    };
                }
            },
            @This().APB1 => enum(reg_type) {
                UART4 = Port(0x10000, bus),
                pub fn api(port: @This()) enum_type {
                    return comptime switch (port) {
                        @This().UART4 => enum(reg_type) {
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
                    };
                }

                fn regs(port: @This()) enum_type {
                    return comptime switch (port) {
                        @This().UART4 => enum(reg_type) {
                            ISR = Reg(0x1C, port), // USART interrupt and status register (USART_ISR)
                            TDR = Reg(0x28, port), // USART transmit data register (USART_TDR)
                            fn fields(reg: @This()) enum_type {
                                return comptime switch (reg) {
                                    @This().ISR => enum {
                                        const TXFNF = Field{ .rw = .ReadOnly, .width = u1, .shift = 7, .reg = @intFromEnum(reg), .values = enum(u1) {
                                            Full = 0,
                                            NotFull = 1,
                                        } };
                                    },
                                    @This().TDR => enum {
                                        const TDR = Field{ .rw = .WriteOnly, .width = u8, .shift = 0, .reg = @intFromEnum(reg), .values = enum {} };
                                    },
                                };
                            }
                        },
                    };
                }
            },
            @This().AHB4 => enum(reg_type) {
                RCC = Port(0x0, bus),
                PWR = Port(0x1000, bus),
                GPIOA = Port(0x2000, bus),
                GPIOB = Port(0x3000, bus),
                GPIOC = Port(0x4000, bus),
                GPIOD = Port(0x5000, bus),
                GPIOE = Port(0x6000, bus),
                GPIOF = Port(0x7000, bus),
                GPIOG = Port(0x8000, bus),
                GPIOH = Port(0x9000, bus),
                GPIOI = Port(0xA000, bus),
                GPIOJ = Port(0xB000, bus),
                GPIOK = Port(0xC000, bus),

                pub fn api(port: @This()) enum_type {
                    return comptime switch (port) {
                        @This().RCC => enum(reg_type) {
                            pub const LSE = struct {
                                pub fn init() void { // RM0436 Rev 6, p.531
                                    const PWR = bus.ports().PWR.api();
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
                        },
                        @This().PWR => enum(reg_type) {
                            const DBP = port.regs().CR1.fields().DBP;
                            pub fn disableBackupDomainWriteProtection() void {
                                DBP.set(DBP.values.Disabled);
                            }
                            pub fn enableBackupDomainWriteProtection() void {
                                DBP.set(DBP.values.Enabled);
                            }
                        },
                        @This().GPIOA, @This().GPIOB, @This().GPIOC, @This().GPIOD, @This().GPIOE, @This().GPIOF, @This().GPIOG, @This().GPIOH, @This().GPIOI, @This().GPIOJ, @This().GPIOK => enum {
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
                                };
                            }
                        },
                    };
                }

                fn regs(port: @This()) enum_type {
                    return comptime switch (port) {
                        @This().RCC => enum(reg_type) {
                            BDCR = Reg(0x140, port), // RCC backup domain control register (RCC_BDCR)
                            fn fields(reg: @This()) enum_type {
                                return comptime switch (reg) {
                                    @This().BDCR => enum {
                                        const LSEON = Field{ .rw = .ReadWrite, .width = u1, .shift = 0, .reg = @intFromEnum(reg), .values = enum(u1) {
                                            Off = 0,
                                            On = 1,
                                        } };
                                        const LSEBYP = Field{ .rw = .ReadWrite, .width = u1, .shift = 1, .reg = @intFromEnum(reg), .values = enum(u1) {
                                            NotBypassed = 0,
                                            Bypassed = 1,
                                        } };
                                        const LSERDY = Field{ .rw = .ReadOnly, .width = u1, .shift = 2, .reg = @intFromEnum(reg), .values = enum(u1) {
                                            NotReady = 0,
                                            Ready = 1,
                                        } };
                                    },
                                };
                            }
                        },
                        @This().PWR => enum(reg_type) {
                            CR1 = Reg(0x0, port), // PWR control register 1 (PWR_CR1)
                            fn fields(reg: @This()) enum_type {
                                return comptime switch (reg) {
                                    @This().CR1 => enum {
                                        const DBP = Field{ .rw = .ReadWrite, .width = u1, .shift = 8, .reg = @intFromEnum(reg), .values = enum(u1) {
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

fn Reg(comptime offset: reg_type, comptime port: anytype) reg_type {
    return offset + @intFromEnum(port);
}
fn Port(comptime offset: reg_type, comptime bus: anytype) reg_type {
    return offset + @intFromEnum(bus);
}

// tests
const testing = @import("std").testing;

test "regular field test" {}
