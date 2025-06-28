// Reference documentation: ARM DDI 0487 L.b
const print = @import("std").fmt.comptimePrint;
const armv7_general_register = enum(u4) { r0 = 0, r1 = 1 };

const Mode = enum(u5) { Monitor = 0x16 };

const CPSR = struct { // Current Program Status Register: G8-11861
    usingnamespace SysReg(.cpsr, .mrs, .msr);
    pub const DIT = CPSR.Bit(21); // Data Independent Timing
};

const ID_PFR0 = struct { // Processor Feature Register 0: G8-12095
    usingnamespace CP15Reg(0, 0, 1, 0, .ReadOnly);
    pub const DIT = ID_PFR0.Field(24, 4, enum(u4) { Implemented = 0b001 }); // Data Independent Timing
};

const PMCR = struct {
    usingnamespace CP15Reg(0, 9, 12, 0, .ReadWrite);
    pub const ResetValue = 0;
    pub const DP = PMCR.Bit(5);
};

const SCR = struct {
    usingnamespace CP15Reg(0, 1, 1, 0, .ReadWrite);
    pub const SIF = SCR.Bit(9);
    pub const ResetValue = 0;
};

const VBAR = struct {
    usingnamespace CP15Reg(0, 12, 0, 0, .ReadWrite);
};

const MVBAR = struct {
    usingnamespace CP15Reg(0, 12, 0, 1, .ReadWrite);
};

const SCTLR = struct {
    usingnamespace CP15Reg(0, 1, 0, 0, .ReadWrite);
    pub const DSSBS = SCTLR.Field(31, 1, enum(u1) { DisableMitigation = 0, EnableMitigation = 1 });
    pub const TE = SCTLR.Field(30, 1, enum(u1) { ARM = 0, Thumb = 1 });
    pub const EE = SCTLR.Field(25, 1, enum(u1) { LittleEndian = 0, BigEndian = 1 });
    pub const V = SCTLR.Field(13, 1, enum(u1) { LowVectors = 0, HiVectors = 1 });
    pub const NTWE = SCTLR.Field(18, 1, enum(u1) { Trapped = 0, NotTrapped = 1 });
    pub const NTWI = SCTLR.Field(16, 1, enum(u1) { Trapped = 0, NotTrapped = 1 });
    pub const CP15BEN = SCTLR.Bit(5);
    pub const A = SCTLR.Bit(1);
    pub const I = SCTLR.Bit(12);
    const ReservedBit23 = SCTLR.ReservedBit(23, 1); // aka SPAN
    const ReservedBit22 = SCTLR.ReservedBit(22, 1); // RES1
    const ReservedBit04 = SCTLR.ReservedBit(4, 1); // aka LSMAOE
    const ReservedBit03 = SCTLR.ReservedBit(3, 1); // aka nTLSMD
    pub const RES1 = ReservedBit23.asU32() | ReservedBit22.asU32() | ReservedBit04.asU32() | ReservedBit03.asU32();
};

const CPACR = struct {
    usingnamespace CP15Reg(0, 1, 0, 2, .ReadWrite);
    pub const CP10 = CPACR.Field(20, 2, enum(u2) { Disabled = 0b0, PL1Only = 0b1, Enabled = 0b11 });
    pub const CP11 = CPACR.Field(22, 2, enum(u2) { Disabled = 0b0, PL1Only = 0b1, Enabled = 0b11 });
    pub const TRCDIS = CPACR.Bit(28);
    pub const ResetValue = 0;
};

const NSACR = struct {
    usingnamespace CP15Reg(0, 1, 1, 2, .ReadWrite);
    pub const AllCPAccessInNonSecureState = NSACR.Field(0, 14, enum(u1) { Disabled = 0 });
    pub const CP10 = NSACR.Field(10, 1, enum(u1) { SecureAccessOnly = 0, AccessFromAnySecureState = 1 });
    pub const CP11 = NSACR.Field(11, 1, enum(u1) { SecureAccessOnly = 0, AccessFromAnySecureState = 1 });
    const NSTRCDIS = NSACR.Bit(20);
};

const ID_DFR0 = struct {
    usingnamespace CP15Reg(0, 0, 1, 2, .ReadOnly);
    pub const CopTrc = ID_DFR0.Field(12, 4, enum(u4) { Implemented = 0b0001 });
};

const FPEXC = struct {
    usingnamespace SysReg(.fpexc, .vmrs, .vmsr);
    pub const VECITR = FPEXC.ReservedField(8, 3, 0b111);
    pub const EN = FPEXC.Bit(30);
    pub const ResetValue = VECITR.asU32();
};

// TODO: add support for 64 bit width registers
fn CP15Reg(comptime op1: u3, comptime crn: u4, comptime crm: u4, comptime op2: u3, comptime rw: enum { ReadOnly, ReadWrite }) type {
    return struct {
        pub usingnamespace GenericAccessors(@This());
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

fn SysReg(comptime register: @TypeOf(.@"enum"), comptime read_instruction: @TypeOf(.@"enum"), comptime write_instruction: @TypeOf(.@"enum")) type {
    return struct {
        usingnamespace GenericAccessors(@This());
        inline fn readTo(comptime access_reg: armv7_general_register) void {
            asm volatile (print("{s} r{d}, {s}", .{ @tagName(read_instruction), @intFromEnum(access_reg), @tagName(register) }));
        }
        inline fn writeFrom(comptime access_reg: armv7_general_register) void {
            asm volatile (print("{s} {s}, r{d}", .{ @tagName(write_instruction), @tagName(register), @intFromEnum(access_reg) }));
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

        pub fn Bit(comptime bit: u5) type {
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
                pub inline fn ifEqual(comptime cmp_value: anytype, proc: anytype, comptime args: anytype) void {
                    readTo(.r0);
                    switch (@TypeOf(cmp_value)) {
                        @TypeOf(.@"enum") => SetValue(.r1, @intFromEnum(@as(values, cmp_value))),
                        comptime_int => SetValue(.r1, cmp_value),
                        else => @compileError("unsupported value type"),
                    }
                    asm volatile ("cmp r0, r1");
                    asm volatile ("bne 1f");
                    @call(.always_inline, proc, args);
                    asm volatile ("1:");
                }
            };
        }
    };
}

inline fn SetValue(comptime reg: armv7_general_register, comptime value: u32) void {
    asm volatile (print("movw r{d}, #:lower16:{d}", .{ @intFromEnum(reg), value }));
    asm volatile (print("movt r{d}, #:upper16:{d}", .{ @intFromEnum(reg), value }));
}

inline fn LoadAddr(comptime reg: armv7_general_register, address: anytype) void {
    asm volatile (print("ldr r{d}, %[addr]", .{@intFromEnum(reg)})
        :
        : [addr] "i" (&address),
    );
}

pub inline fn EndlessLoop() void {
    asm volatile ("nop");
    asm volatile ("b . -2");
}

pub inline fn goto(comptime func: anytype) void {
    asm volatile ("b %[addr]"
        :
        : [addr] "i" (func),
    );
}

pub inline fn SetMode(comptime mode: Mode) void {
    asm volatile (print("cps {d}", .{@intFromEnum(mode)}));
    asm volatile ("isb");
}

pub inline fn InitializeSystemControlRegister() void {
    SetValue(.r0, SCTLR.RES1 |
        SCTLR.NTWE.asU32(.NotTrapped) |
        SCTLR.NTWI.asU32(.NotTrapped) |
        SCTLR.CP15BEN.asU32(.Enabled) |
        SCTLR.EE.asU32(.LittleEndian) |
        SCTLR.TE.asU32(.ARM) |
        SCTLR.V.asU32(.LowVectors) |
        SCTLR.DSSBS.asU32(.DisableMitigation));
    SCTLR.writeFrom(.r0);
}

pub inline fn InitializeExceptionVectorsTable(comptime addr: anytype) void {
    LoadAddr(.r0, addr);
    VBAR.writeFrom(.r0);
    MVBAR.writeFrom(.r0);
}

pub inline fn InitializeInstructionCache(comptime state: @TypeOf(.@"enum")) void {
    SCTLR.I.Select(state);
}

pub inline fn InitializeAlignmentFaultChecking(comptime state: @TypeOf(.@"enum")) void {
    SCTLR.A.Select(state);
}

pub inline fn InitializeSecureConfigurationRegister() void {
    SetValue(.r0, SCR.ResetValue);
    SCR.writeFrom(.r0);
    SCR.SIF.Select(.Enabled);
}

pub inline fn InitializeException(comptime exception: enum { Abort, IRQ, FIQ }, comptime state: enum { Enabled, Disabled }) void {
    asm volatile (print("cpsi{s} {s}", .{
            switch (state) {
                .Enabled => "e",
                .Disabled => "d",
            },
            switch (exception) {
                .Abort => "a",
                .FIQ => "f",
                .IRQ => "i",
            },
        }));
    asm volatile ("isb");
}

pub inline fn InitializeCoprocessors() void {
    SetValue(.r0, NSACR.AllCPAccessInNonSecureState.asU32(.Disabled) |
        NSACR.CP10.asU32(.AccessFromAnySecureState) |
        NSACR.CP11.asU32(.AccessFromAnySecureState));
    NSACR.writeFrom(.r0);
    ID_DFR0.CopTrc.ifEqual(.Implemented, NSACR.NSTRCDIS.Select, .{.Enabled});

    SetValue(.r0, CPACR.ResetValue |
        CPACR.CP10.asU32(.Enabled) |
        CPACR.CP11.asU32(.Enabled) |
        CPACR.TRCDIS.asU32(.Disabled));
    CPACR.writeFrom(.r0);

    SetValue(.r0, FPEXC.ResetValue | FPEXC.EN.asU32(.Enabled));
    FPEXC.writeFrom(.r0);
}

pub inline fn InitializePerformanceMonitorControlRegister() void {
    SetValue(.r0, PMCR.ResetValue | PMCR.DP.asU32(.Enabled));
    PMCR.writeFrom(.r0);
}

pub inline fn InitializeCurrentProgramStatusRegister() void {
    ID_PFR0.DIT.ifEqual(.Implemented, CPSR.DIT.Select, .{.Enabled});
}
