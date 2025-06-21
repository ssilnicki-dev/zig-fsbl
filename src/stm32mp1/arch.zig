const print = @import("std").fmt.comptimePrint;
const armv7_general_register = enum(u4) { r0 = 0 };

// TODO: add support for 64 bit width registers
fn CP15Reg(comptime op1: u3, comptime crn: u4, comptime crm: u4, comptime op2: u3) type {
    return struct {
        const Self = @This();
        const r = armv7_general_register.r0;
        pub inline fn readTo(comptime access_reg: armv7_general_register) void {
            access(access_reg, .mrc);
        }
        pub inline fn writeFrom(comptime access_reg: armv7_general_register) void {
            access(access_reg, .mcr);
        }
        inline fn access(comptime access_reg: armv7_general_register, comptime instruction: @TypeOf(.@"enum")) void {
            asm volatile (print("{s} p15, {d}, r{d}, c{d}, c{d}, {d}", .{ @tagName(instruction), op1, @intFromEnum(access_reg), crn, crm, op2 }));
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
                pub inline fn Set(comptime v: u32) void {
                    comptime if (@bitSizeOf(@TypeOf(v)) - @clz(v) > width) {
                        @compileError(print("Value {d} (0x{x}) can not fit into Field with length of {d} bits", .{ v, v, width }));
                    };
                    if (v == 0) {
                        Clear();
                    } else {
                        Self.readTo(r);
                        asm volatile (print("and r{d}, r{d}, 0x{x}", .{ @intFromEnum(r), @intFromEnum(r), ~(((@as(u32, 1) << width) - 1) << shift) }));
                        asm volatile (print("orr r{d}, r{d}, 0x{x}", .{ @intFromEnum(r), @intFromEnum(r), v << shift }));
                        Self.writeFrom(r);
                    }
                }
                pub inline fn Clear() void {
                    Self.readTo(r);
                    asm volatile (print("and r{d}, r{d}, 0x{x}", .{ @intFromEnum(r), @intFromEnum(r), ~(((@as(u32, 1) << width) - 1) << shift) }));
                    Self.writeFrom(r);
                }
                pub inline fn readTo(comptime reg: armv7_general_register) void {
                    Self.readTo(reg);
                    asm volatile (print("ubfx r{d}, r{d}, {d}, {d}", .{ @intFromEnum(reg), @intFromEnum(reg), shift, width }));
                }
            };
        }
    };
}
