const print = @import("std").fmt.comptimePrint;
const armv7_general_register = enum(u4) { r0 = 0, r1 = 1 };

pub inline fn goto(comptime func: anytype) void {
    asm volatile ("b %[addr]"
        :
        : [addr] "i" (func),
    );
}

pub inline fn SetValue(comptime reg: armv7_general_register, comptime value: u32) void {
    asm volatile (print("movw r{d}, #:lower16:{d}", .{ @intFromEnum(reg), value }));
    asm volatile (print("movt r{d}, #:upper16:{d}", .{ @intFromEnum(reg), value }));
}

pub inline fn LoadAddr(comptime reg: armv7_general_register, address: anytype) void {
    asm volatile (print("ldr r{d}, %[addr]", .{@intFromEnum(reg)})
        :
        : [addr] "i" (&address),
    );
}

const Mode = enum(u5) { Monitor = 0x16 };

pub const SCR = struct {
    const self = CP15Reg(0, 1, 1, 0, .ReadWrite);
    pub const SIF = self.Bit(9);
    pub const ResetValue = 0;

    pub const writeFrom = self.writeFrom;
};

pub const VBAR = struct {
    const self = CP15Reg(0, 12, 0, 0, .ReadWrite);
    pub const writeFrom = self.writeFrom;
};

pub const MVBAR = struct {
    const self = CP15Reg(0, 12, 0, 1, .ReadWrite);
    pub const writeFrom = self.writeFrom;
};

pub const SCTLR = struct {
    const self = CP15Reg(0, 1, 0, 0, .ReadWrite);
    pub const DSSBS = self.Field(31, 1, enum(u1) { DisableMitigation = 0, EnableMitigation = 1 });
    pub const TE = self.Field(30, 1, enum(u1) { ARM = 0, Thumb = 1 });
    pub const EE = self.Field(25, 1, enum(u1) { LittleEndian = 0, BigEndian = 1 });
    pub const V = self.Field(13, 1, enum(u1) { LowVectors = 0, HiVectors = 1 });
    pub const NTWE = self.Field(18, 1, enum(u1) { Trapped = 0, NotTrapped = 1 });
    pub const NTWI = self.Field(16, 1, enum(u1) { Trapped = 0, NotTrapped = 1 });
    pub const CP15BEN = self.Bit(5);
    pub const A = self.Bit(1);
    pub const I = self.Bit(12);
    const ReservedBit23 = self.ReservedBit(23, 1); // aka SPAN
    const ReservedBit22 = self.ReservedBit(22, 1); // RES1
    const ReservedBit04 = self.ReservedBit(4, 1); // aka LSMAOE
    const ReservedBit03 = self.ReservedBit(3, 1); // aka nTLSMD
    pub const RES1 = ReservedBit23.asU32() | ReservedBit22.asU32() | ReservedBit04.asU32() | ReservedBit03.asU32();

    pub const writeFrom = self.writeFrom;
};

pub const CPACR = struct {
    const self = CP15Reg(0, 1, 0, 2, .ReadWrite);
    pub const CP10 = self.Field(20, 2, enum(u2) { Disabled = 0b0, PL1Only = 0b1, Enabled = 0b11 });
    pub const CP11 = self.Field(22, 2, enum(u2) { Disabled = 0b0, PL1Only = 0b1, Enabled = 0b11 });
};

pub const NSACR = struct {
    const self = CP15Reg(0, 1, 1, 2, .ReadWrite);
    pub const AllCPAccessInNonSecureState = self.Field(0, 14, enum(u1) { Disabled = 0 });
    pub const CP10 = self.Field(10, 1, enum(u1) { SecureAccessOnly = 0, AccessFromAnySecureState = 1 });
    pub const CP11 = self.Field(11, 1, enum(u1) { SecureAccessOnly = 0, AccessFromAnySecureState = 1 });

    pub const writeFrom = self.writeFrom;
};

pub const FPEXC = struct {
    const self = FPReg(.fpexc);
    pub const EN = self.Bit(30);
};

// TODO: add support for 64 bit width registers
fn CP15Reg(comptime op1: u3, comptime crn: u4, comptime crm: u4, comptime op2: u3, comptime rw: enum { ReadOnly, ReadWrite }) type {
    return struct {
        usingnamespace GenericAccessors(@This());
        inline fn readTo(comptime access_reg: armv7_general_register) void {
            access(access_reg, .mrc);
        }
        inline fn writeFrom(comptime access_reg: armv7_general_register) void {
            switch (rw) {
                .ReadWrite => {
                    access(access_reg, .mcr);
                    asm volatile ("isb");
                },
                .ReadOnly => @compileError("Register is ReadOnly"),
            }
        }

        inline fn access(comptime access_reg: armv7_general_register, comptime instruction: @TypeOf(.@"enum")) void {
            asm volatile (print("{s} p15, {d}, r{d}, c{d}, c{d}, {d}", .{ @tagName(instruction), op1, @intFromEnum(access_reg), crn, crm, op2 }));
        }
    };
}

fn FPReg(comptime register: @TypeOf(.@"enum")) type {
    return struct {
        usingnamespace GenericAccessors(@This());
        inline fn readTo(comptime access_reg: armv7_general_register) void {
            asm volatile (print("vmrs r{d}, {s}", .{ @intFromEnum(access_reg), @tagName(register) }));
        }
        inline fn writeFrom(comptime access_reg: armv7_general_register) void {
            asm volatile (print("vmsr {s}, r{d}", .{ @tagName(register), @intFromEnum(access_reg) }));
        }
    };
}

fn GenericAccessors(self: type) type {
    return struct {
        const data = armv7_general_register.r0;

        fn ReservedBit(comptime shift: u5, comptime value: u1) type {
            return ReservedField(shift, 1, value);
        }

        fn ReservedField(comptime shift: u5, comptime width: u5, comptime value: u32) type {
            comptime if ((32 - @clz(value)) > width) {
                @compileError(print("Field value {d} does not fit {d} bit field", .{ value, width }));
            };
            return struct {
                pub inline fn asU32() u32 {
                    return value << shift;
                }
            };
        }

        fn Bit(comptime bit: u5) type {
            return Field(bit, 1, enum(u1) { Disabled = 0, Enabled = 1 });
        }

        fn Field(comptime shift: u5, comptime width: u5, comptime values: @TypeOf(enum {})) type {
            _ = comptime (shift + (width - 1)); // check Field declaration sanity
            return struct {
                pub inline fn Select(comptime v: values) void {
                    Set(@intFromEnum(v));
                }
                pub inline fn Set(comptime v: u31) void {
                    if (v == 0) {
                        Clear();
                    } else {
                        self.readTo(data);
                        asm volatile (print("bfc r{d}, #{d}, #{d}", .{ @intFromEnum(data), shift, width }));
                        asm volatile (print("orr r{d}, r{d}, #{d}", .{ @intFromEnum(data), @intFromEnum(data), v << shift }));
                        self.writeFrom(data);
                    }
                }
                pub inline fn Clear() void {
                    self.readTo(data);
                    asm volatile (print("bfc r{d}, #{d}, #{d} ", .{ @intFromEnum(data), shift, width }));
                    self.writeFrom(data);
                }
                pub inline fn readTo(comptime reg: armv7_general_register) void {
                    self.readTo(reg);
                    asm volatile (print("ubfx r{d}, r{d}, {d}, {d}", .{ @intFromEnum(reg), @intFromEnum(reg), shift, width }));
                }
                pub inline fn asU32(comptime value: values) u32 {
                    return @as(u32, @intFromEnum(value)) << shift;
                }
                pub inline fn ifEqual(comptime cmp_value: anytype, proc: anytype) void {
                    readTo(.r0);
                    switch (@TypeOf(cmp_value)) {
                        @TypeOf(.@"enum") => SetValue(.r1, @intFromEnum(@as(values, cmp_value))),
                        comptime_int => SetValue(.r1, cmp_value),
                        else => @compileError("unsupported value type"),
                    }
                    asm volatile ("cmp r0, r1");
                    asm volatile ("bne 1f");
                    proc();
                    asm volatile ("1:");
                }
            };
        }
    };
}

pub inline fn EndlessLoop() void {
    asm volatile (
        \\ nop
        \\ b . -2
    );
}
pub inline fn SetMode(comptime mode: Mode) void {
    asm volatile (print("cps {d}", .{@intFromEnum(mode)}));
    asm volatile ("isb");
}
