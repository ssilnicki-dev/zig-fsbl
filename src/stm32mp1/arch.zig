const print = @import("std").fmt.comptimePrint;
const armv7_general_register = enum(u4) { r0 = 0 };

pub const CPACR = struct {
    const self = CP15Reg(0, 1, 0, 2);
    pub const CP10 = self.Field(20, 2, enum(u2) { Disabled = 0b0, PL1Only = 0b1, Enabled = 0b11 });
    pub const CP11 = self.Field(22, 2, enum(u2) { Disabled = 0b0, PL1Only = 0b1, Enabled = 0b11 });
};

pub const NSACR = struct {
    const self = CP15Reg(0, 1, 1, 2);
    pub const AllCPAccessInNonSecureState = self.Field(0, 14, enum(u1) { Disabled = 0 });
    pub const CP10 = self.Field(0, 10, enum(u1) { SecureAccessOnly = 0, AccessFromAnySecureState = 1 });
    pub const CP11 = self.Field(0, 11, enum(u1) { SecureAccessOnly = 0, AccessFromAnySecureState = 1 });
};

pub const FPEXC = struct {
    const self = FPReg(.fpexc);
    pub const EN = self.Bit(30);
};

// TODO: add support for 64 bit width registers
fn CP15Reg(comptime op1: u3, comptime crn: u4, comptime crm: u4, comptime op2: u3) type {
    return struct {
        usingnamespace GenericAccessors(@This());
        inline fn readTo(comptime access_reg: armv7_general_register) void {
            access(access_reg, .mrc);
        }
        inline fn writeFrom(comptime access_reg: armv7_general_register) void {
            access(access_reg, .mcr);
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
