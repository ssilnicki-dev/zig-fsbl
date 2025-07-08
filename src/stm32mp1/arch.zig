// Reference documentation: ARM DDI 0487 L.b [#1]
//                          ARM DDI 0464 E   [#2]
const print = @import("std").fmt.comptimePrint;
const armv7_general_register = enum { r0, r1, r2, r3, r4, r5 };
const cpu_word_size = 4;

const Mode = enum(u5) {
    Supervisor = 0x13,
    Monitor = 0x16,
};

const ACTLR = struct { // Auxilary Control Register: G8-11795[1], 4-59[2]
    usingnamespace CP15Reg(0, 1, 0, 1, .ReadWrite);
    pub const SMP = ACTLR.Bit(6); // Coherent requests to the processor
};

const CTR = struct { // Cache Type Register: G8-11868[1]
    usingnamespace CP15Reg(0, 0, 0, 1, .ReadOnly);
    pub const DminLine = CTR.Field(16, 4, enum {}); // Log2 of the number of words in the smallest cache line

};

const DCIMVAC = struct { // Data Cache line Invalidate by VA to Point of Coherency: G8-11883[1]
    usingnamespace CP15Reg(0, 7, 6, 1, .WriteOnly);
};

const MPIDR = struct { // Multiprocessor Affinity Register: G8-12140[1]
    usingnamespace CP15Reg(0, 0, 0, 5, .ReadOnly);
    pub const Aff1 = MPIDR.Field(8, 8, enum(u8) { CLUSTER0 = 0 });
    pub const Aff0 = MPIDR.Field(0, 8, enum(u8) { CPU0 = 0 });
};

const CPSR = struct { // Current Program Status Register: G8-11861[1]
    usingnamespace SysReg(.cpsr, .mrs, .msr);
    pub const DIT = CPSR.Bit(21); // Data Independent Timing
};

const ID_PFR0 = struct { // Processor Feature Register 0: G8-12095[1]
    usingnamespace CP15Reg(0, 0, 1, 0, .ReadOnly);
    pub const DIT = ID_PFR0.Field(24, 4, enum(u4) { Implemented = 0b001 }); // Data Independent Timing
};

const PMCR = struct { // Performance Monitor Control Register: G8-12525[1]
    usingnamespace CP15Reg(0, 9, 12, 0, .ReadWrite);
    pub const DP = PMCR.Bit(5); // Disable cycle counter when event counting Prohibited
    pub const ResetValue = 0;
};

const SCR = struct { // Secure Configuration Register: G8-12190[1]
    usingnamespace CP15Reg(0, 1, 1, 0, .ReadWrite);
    pub const SIF = SCR.Bit(9); // Secure Instruction Fetch
    pub const ResetValue = 0;
};

const VBAR = struct { // Vector Base Address Register: G8-12321[1]
    usingnamespace CP15Reg(0, 12, 0, 0, .ReadWrite);
};

const MVBAR = struct { // Monitor Vector Base Register: G8-12143[1]
    usingnamespace CP15Reg(0, 12, 0, 1, .ReadWrite);
};

const SCTLR = struct { // System Control Register: G8-12196[1]
    usingnamespace CP15Reg(0, 1, 0, 0, .ReadWrite);
    pub const DSSBS = SCTLR.Field(31, 1, enum(u1) { DisableMitigation = 0, EnableMitigation = 1 }); // Default Speculative Store Bypass Safe value on exception
    pub const TE = SCTLR.Field(30, 1, enum(u1) { ARM = 0, Thumb = 1 }); // T32 Exception Enable
    pub const EE = SCTLR.Field(25, 1, enum(u1) { LittleEndian = 0, BigEndian = 1 }); // Endianess on Exception
    pub const NTWE = SCTLR.Field(18, 1, enum(u1) { Trapped = 0, NotTrapped = 1 }); // Trap execution of WFE at EL0
    pub const NTWI = SCTLR.Field(16, 1, enum(u1) { Trapped = 0, NotTrapped = 1 }); // Trap execution of WFI at EL0
    pub const V = SCTLR.Field(13, 1, enum(u1) { LowVectors = 0, HiVectors = 1 }); // Vectors bit
    pub const I = SCTLR.Bit(12); // Instruction access Cacheability control, for accesses from EL1 and EL0
    pub const CP15BEN = SCTLR.Bit(5); // System instruction memory barrier enable
    pub const A = SCTLR.Bit(1); // Alignment ckeck enable
    const ReservedBit23 = SCTLR.ReservedBit(23, 1); // aka SPAN
    const ReservedBit22 = SCTLR.ReservedBit(22, 1); // RES1
    const ReservedBit04 = SCTLR.ReservedBit(4, 1); // aka LSMAOE
    const ReservedBit03 = SCTLR.ReservedBit(3, 1); // aka nTLSMD
    pub const ResetValue = ReservedBit23.asU32() | ReservedBit22.asU32() | ReservedBit04.asU32() | ReservedBit03.asU32();
};

const CPACR = struct { // Architectural Feature Access Control Register: G8-11852[1]
    usingnamespace CP15Reg(0, 1, 0, 2, .ReadWrite);
    pub const CP10 = CPACR.Field(20, 2, enum(u2) { Disabled = 0b0, PL1Only = 0b1, Enabled = 0b11 });
    pub const CP11 = CPACR.Field(22, 2, enum(u2) { Disabled = 0b0, PL1Only = 0b1, Enabled = 0b11 });
    pub const TRCDIS = CPACR.Bit(28); // Traps PL0 and PL1 access to all trace registers to Undefined Mode
    pub const ResetValue = 0;
};

const NSACR = struct { // Non-Secure Access Control Register: G8-12162[1]
    usingnamespace CP15Reg(0, 1, 1, 2, .ReadWrite);
    pub const AllCPAccessInNonSecureState = NSACR.Field(0, 14, enum(u1) { Disabled = 0 });
    pub const CP10 = NSACR.Field(10, 1, enum(u1) { SecureAccessOnly = 0, AccessFromAnySecureState = 1 }); //Defines access rights for SIMD and FP functionality
    pub const CP11 = NSACR.Field(11, 1, enum(u1) { SecureAccessOnly = 0, AccessFromAnySecureState = 1 }); // unused, must follow CP10
    const NSTRCDIS = NSACR.Bit(20);
};

const ID_DFR0 = struct { // Debug Feature Register: G8-12037[1]
    usingnamespace CP15Reg(0, 0, 1, 2, .ReadOnly);
    pub const CopTrc = ID_DFR0.Field(12, 4, enum(u4) { Implemented = 0b0001 }); // Support for System registers-based model, using registers in the coproc == 0b1110 encoding space
};

const FPEXC = struct { // Floating Point Excepion Register: G8-11910[1]
    usingnamespace SysReg(.fpexc, .vmrs, .vmsr);
    pub const VECITR = FPEXC.ReservedField(8, 3, 0b111); // Vector Iteration count
    pub const EN = FPEXC.Bit(30); // Enable Access to SIMD and FP functionality
    pub const ResetValue = VECITR.asU32();
};

const BSEC_DENABLE = struct {
    usingnamespace PlatformReg(0x5C005014);
};

// TODO: add support for 64 bit width registers
fn CP15Reg(comptime op1: u3, comptime crn: u4, comptime crm: u4, comptime op2: u3, comptime rw: enum { ReadOnly, ReadWrite, WriteOnly }) type {
    return struct {
        pub usingnamespace GenericAccessors(@This());
        inline fn readTo(comptime access_reg: armv7_general_register) void {
            switch (rw) {
                .WriteOnly => {
                    @compileError("Register is ReadOnly");
                },
                else => {
                    access(access_reg, .mrc);
                },
            }
        }
        inline fn writeFrom(access_reg: armv7_general_register) void {
            switch (rw) {
                .ReadOnly => @compileError("Register is ReadOnly"),
                else => {
                    access(access_reg, .mcr);
                    asm volatile ("isb");
                },
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
            asm volatile (print("{s} {s}, {s}", .{ @tagName(read_instruction), @tagName(access_reg), @tagName(register) }));
        }
        inline fn writeFrom(access_reg: armv7_general_register) void {
            asm volatile (print("{s} {s}, {s}", .{ @tagName(write_instruction), @tagName(register), @tagName(access_reg) }));
        }
    };
}

fn PlatformReg(addr: comptime_int) type {
    return struct {
        pub usingnamespace GenericAccessors(@This());
        inline fn writeFrom(reg: armv7_general_register) void {
            const tmp = switch (reg) {
                .r0 => .r1,
                else => .r0,
            };
            SAVE(.Word, reg, SET(tmp, addr));
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
                pub inline fn readTo(comptime reg: armv7_general_register) armv7_general_register {
                    self.readTo(reg);
                    asm volatile (print("ubfx r{d}, r{d}, {d}, {d}", .{ @intFromEnum(reg), @intFromEnum(reg), shift, width }));
                    return reg;
                }
                pub inline fn asU32(comptime value: values) u32 {
                    return @as(u32, @intFromEnum(value)) << shift;
                }
                pub inline fn If(comptime condition: anytype, comptime cmp_value: anytype, action: anytype, comptime action_argument: anytype) void {
                    const reg = readTo(.r0);
                    IF(reg, condition, switch (@TypeOf(cmp_value)) {
                        @TypeOf(.@"enum") => @intFromEnum(@as(values, cmp_value)),
                        else => cmp_value,
                    }, action, action_argument);
                }
            };
        }
    };
}

inline fn SET(reg: armv7_general_register, value: anytype) armv7_general_register {
    switch (@typeInfo(@TypeOf(value))) {
        .pointer => asm volatile (print("ldr {s}, %[addr]", .{@tagName(reg)})
            :
            : [addr] "i" (&value),
        ),
        .int, .comptime_int => {
            asm volatile (print("movw {s}, #:lower16:{d}", .{ @tagName(reg), value }));
            asm volatile (print("movt {s}, #:upper16:{d}", .{ @tagName(reg), value }));
        },
        else => if (@TypeOf(value) == armv7_general_register) {
            if (value != reg) {
                asm volatile (print("mov {s}, {s}", .{ @tagName(reg), @tagName(value) }));
            }
        } else {
            @compileError("Unsupported value type");
        },
    }
    return reg;
}

inline fn SAVE(comptime size: enum { Byte, HalfWord, Word }, comptime reg: armv7_general_register, address: armv7_general_register) void {
    asm volatile (print("{s} {s}, [{s}]", .{ switch (size) {
            .Byte => "strb",
            .HalfWord => "strh",
            .Word => "str",
        }, @tagName(reg), @tagName(address) }));
}

inline fn DO(comptime op: enum { Add, Sub, LeftShift, ClearBits, Mul }, reg: armv7_general_register, arg: anytype) void {
    const opcode = switch (op) {
        .Add => "add",
        .Sub => "sub",
        .LeftShift => "lsl",
        .ClearBits => "bic",
        .Mul => "mul",
    };
    switch (@TypeOf(arg)) {
        @Type(.enum_literal), armv7_general_register => asm volatile (print("{s} {s}, {s}, {s}", .{ opcode, @tagName(reg), @tagName(reg), @tagName(arg) })),
        comptime_int => asm volatile (print("{s} {s}, {s}, #{d}", .{ opcode, @tagName(reg), @tagName(reg), arg })),
        else => @compileError("Unsupported type or argument"),
    }
}

inline fn ADD(reg: armv7_general_register, arg: anytype) armv7_general_register {
    DO(.Add, reg, arg);
    return reg;
}
inline fn SUB(reg: armv7_general_register, arg: anytype) armv7_general_register {
    DO(.Sub, reg, arg);
    return reg;
}
inline fn MUL(reg: armv7_general_register, arg: anytype) armv7_general_register {
    DO(.Mul, reg, arg);
    return reg;
}

inline fn LABEL(comptime name: comptime_int) void {
    asm volatile (print("{d}:", .{name}));
}

inline fn IF(comptime reg: armv7_general_register, comptime condition: enum { Equal, NotEqual, LowerUnsigned }, comptime cmp_value: anytype, comptime action: anytype, comptime action_argument: anytype) void {
    const tmp_reg = switch (@TypeOf(cmp_value)) {
        @TypeOf(reg) => switch (cmp_value) {
            reg => @compileError("Comparing register with ifself makes no sence"),
            else => cmp_value,
        },
        else => switch (reg) {
            .r0 => SET(.r1, cmp_value),
            else => SET(.r0, cmp_value),
        },
    };
    asm volatile (print("cmp {s}, {s}", .{ @tagName(reg), @tagName(tmp_reg) }));

    const exit = 1 << 31;

    asm volatile (print("b{s} {d}f", .{
            switch (condition) {
                .Equal => "ne",
                .NotEqual => "eq",
                .LowerUnsigned => "hs", // higher or same
            },
            exit,
        }));

    switch (@typeInfo(@TypeOf(action))) {
        .comptime_int => asm volatile (print("b {d}{s}", .{ action, @tagName(action_argument) })),
        .@"fn" => _ = @call(.always_inline, action, action_argument),
        else => {},
    }
    LABEL(exit);
}

pub inline fn EndlessLoop() void {
    asm volatile ("bl parkingLoop");
    asm volatile ("nop");
}

export fn parkingLoop() callconv(.naked) void {
    asm volatile ("nop");
    asm volatile ("b . -2");
}

pub inline fn goto(comptime func: anytype) void {
    asm volatile ("b %[addr]"
        :
        : [addr] "i" (func),
    );
}

pub inline fn SetMode(mode: Mode) void {
    switch (mode) {
        .Supervisor => {
            asm volatile ("adr r0, 1f");
            asm volatile ("msr spsr_cxsf, #0xD3");
            asm volatile ("mov lr, r0");
            asm volatile ("isb");
            asm volatile ("movs pc, lr");
            asm volatile ("nop");
            asm volatile ("nop");
            asm volatile ("1:");
        },
        else => {
            asm volatile (print("cps {d}", .{@intFromEnum(mode)}));
            asm volatile ("isb");
        },
    }
}

pub inline fn InitializeSystemControlRegister() void {
    SCTLR.writeFrom(SET(.r0, SCTLR.ResetValue |
        SCTLR.NTWE.asU32(.NotTrapped) |
        SCTLR.NTWI.asU32(.NotTrapped) |
        SCTLR.CP15BEN.asU32(.Enabled) |
        SCTLR.EE.asU32(.LittleEndian) |
        SCTLR.TE.asU32(.ARM) |
        SCTLR.V.asU32(.LowVectors) |
        SCTLR.DSSBS.asU32(.DisableMitigation)));
}

pub inline fn InitializeExceptionVectorsTable(comptime addr: anytype) void {
    const reg = SET(.r0, addr);
    VBAR.writeFrom(reg);
    MVBAR.writeFrom(reg);
}

pub inline fn InitializeInstructionCache(comptime state: @TypeOf(.enum_internal)) void {
    SCTLR.I.Select(state);
}

pub inline fn InitializeAlignmentFaultChecking(comptime state: @TypeOf(.enum_internal)) void {
    SCTLR.A.Select(state);
}

pub inline fn InitializeSecureConfigurationRegister() void {
    SCR.writeFrom(SET(.r0, SCR.ResetValue));
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
    NSACR.writeFrom(SET(.r0, NSACR.AllCPAccessInNonSecureState.asU32(.Disabled) |
        NSACR.CP10.asU32(.AccessFromAnySecureState) |
        NSACR.CP11.asU32(.AccessFromAnySecureState)));

    ID_DFR0.CopTrc.If(.Equal, .Implemented, NSACR.NSTRCDIS.Select, .{.Enabled});

    CPACR.writeFrom(SET(.r0, CPACR.ResetValue |
        CPACR.CP10.asU32(.Enabled) |
        CPACR.CP11.asU32(.Enabled) |
        CPACR.TRCDIS.asU32(.Disabled)));

    FPEXC.writeFrom(SET(.r0, FPEXC.ResetValue | FPEXC.EN.asU32(.Enabled)));
}

pub inline fn InitializePerformanceMonitorControlRegister() void {
    PMCR.writeFrom(SET(.r0, PMCR.ResetValue | PMCR.DP.asU32(.Enabled)));
}

pub inline fn InitializeCurrentProgramStatusRegister() void {
    ID_PFR0.DIT.If(.Equal, .Implemented, CPSR.DIT.Select, .{.Enabled});
}

inline fn SecondaryCpuColdBoot() void {
    EndlessLoop();
}

pub inline fn EnableSMP() void {
    ACTLR.SMP.Select(.Enabled);
}

pub inline fn PassOnlyPrimaryCpu() void {
    MPIDR.Aff1.If(.NotEqual, .CLUSTER0, SecondaryCpuColdBoot, .{});
    MPIDR.Aff0.If(.NotEqual, .CPU0, SecondaryCpuColdBoot, .{});
}

extern const rw_data_start: u32;
extern const rw_data_size: u32;
extern const bss_data_start: u32;
extern const bss_data_size: u32;

inline fn UpdateDataCache(comptime action: enum { Invalidate }, comptime base: anytype, comptime size: anytype) void {
    const loop = 1;
    const exit = 2;
    const sys_reg = switch (action) {
        .Invalidate => DCIMVAC,
    };
    const mem_size = SET(.r2, size);
    IF(mem_size, .Equal, 0, exit, .f);
    const mem_addr = SET(.r3, base);
    DO(.Add, mem_size, mem_addr);
    const mem_end = mem_size;
    const data_cache_line_size = CTR.DminLine.readTo(.r4);
    DO(.LeftShift, data_cache_line_size, cpu_word_size);
    const mask = SET(.r5, data_cache_line_size);
    DO(.Sub, mask, 1);
    DO(.ClearBits, mem_addr, mask);
    LABEL(loop);
    sys_reg.writeFrom(mem_addr);
    DO(.Add, mem_addr, data_cache_line_size);
    IF(mem_addr, .LowerUnsigned, mem_end, loop, .b);
    asm volatile ("dsb");
    LABEL(exit);
}

inline fn ZeroMemory(comptime base: anytype, comptime size: anytype) void {
    const loop = 1;
    const exit = 2;
    const mem_size = SET(.r2, size);
    IF(mem_size, .Equal, 0, exit, .f);
    const mem_addr = SET(.r3, base);
    const mem_end = mem_size;
    DO(.Add, mem_end, mem_addr);
    const value = SET(.r4, 0);
    LABEL(loop);
    SAVE(.Byte, value, mem_addr);
    DO(.Add, mem_addr, 1);
    IF(mem_addr, .LowerUnsigned, mem_end, loop, .b);
    LABEL(exit);
}

pub inline fn ResetMemory() void {
    UpdateDataCache(.Invalidate, &rw_data_start, &rw_data_size);
    ZeroMemory(&bss_data_start, &bss_data_size);
}

pub inline fn DebugMode() void {
    BSEC_DENABLE.writeFrom(SET(.r0, 0x47f)); // enable full debug access
}
