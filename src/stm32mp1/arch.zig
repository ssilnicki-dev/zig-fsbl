// Reference documentation: ARM DDI 0487 L.b [#1]
//                          ARM DDI 0464 E   [#2]
const print = @import("std").fmt.comptimePrint;
const armv7_general_register = enum { r0, r1, r2, r3, r4, r5, sp };
const cpu_word_size = 4;
const platform_clusters = 1;
const platform_cpus_per_cluster = 2;

const Mode = enum(u5) {
    Fiq = 0x11,
    Irq = 0x12,
    Supervisor = 0x13,
    Monitor = 0x16,
    Abort = 0x17,
    Undefined = 0x1b,
};

const ID_MMFR4 = struct { // Memory Model Feature Register 4: G8-12089[1]
    const common = CP15Reg(0, 0, 2, 6, .ReadOnly);
    const Field = common.Field;
    pub const CnP = ID_MMFR4.Field(12, 4, enum(u4) { Supported = 0b001 }); // Common not Private
};

const TLBIALL = CP15Reg(0, 8, 7, 0, .WriteOnly); // TLB Invalidate All: G8-12240

const MemoryAttribute = enum(u8) {
    Device = Device_nGnRE,
    NormalMemory = NonTransientMemWbWaRa + (NonTransientMemWbWaRa << 4),
    NormalNonCacheableMemory = CacheDisabled + (CacheDisabled << 4),

    const CacheWriteAllocate = 1;
    const CacheReadAllocate = 1 << 1;
    const CacheDisabled = 1 << 2;
    const CacheNonTransient = 1 << 3;
    const TransientMemWbWaRa = CacheDisabled + CacheWriteAllocate + CacheReadAllocate;
    const NonCachable = CacheDisabled;
    const TransientMemWbWa = CacheDisabled + CacheWriteAllocate;
    const TransientMemWbRa = CacheDisabled + CacheReadAllocate;
    const NonTransientMemWbWa = CacheDisabled + CacheWriteAllocate + CacheNonTransient;
    const NonTransientMemWbWaRa = CacheDisabled + CacheWriteAllocate + CacheReadAllocate + CacheNonTransient;
    const NonTransientMemWbRa = CacheDisabled + CacheReadAllocate + CacheNonTransient;
    const TransientMemWtWa = CacheWriteAllocate;
    const TransientMemWtWaRa = CacheWriteAllocate + CacheReadAllocate;
    const TransientMemWtRa = CacheReadAllocate;
    const NonTransientMemWtWa = CacheWriteAllocate + CacheNonTransient;
    const NonTransientMemWtWaRa = CacheWriteAllocate + CacheReadAllocate + CacheNonTransient;
    const NonTransientMemWtRa = CacheReadAllocate + CacheNonTransient;
    const Device_nGnRnE = 0;
    const Device_nGnRE = CacheDisabled;
    const Device_nGRE = CacheNonTransient;
    const Device_GRE = CacheDisabled + CacheNonTransient;
    pub fn asLongDescriptorField(comptime self: MemoryAttribute) u64 {
        return @as(u64, MAIR.GetIndex(self)) << 2;
    }
};

const MAIR = struct {
    pub const AttrIndexIntType = u3;
    const MAIR0 = struct { // Memory Attribute Indirection Register 0: G8-12129[1]
        const common = CP15Reg(0, 10, 2, 0, .ReadWrite);
        const Field = common.Field;
        const writeFrom = common.writeFrom;
    };

    const MAIR1 = struct { // Memory Attribute Indirection Register 1: G8-12133[1]
        const common = CP15Reg(0, 10, 2, 1, .ReadWrite);
        const Field = common.Field;
        const writeFrom = common.writeFrom;
    };
    const Attr0 = MAIR0.Field(0, 8, MemoryAttribute); // Attribute 0
    const Attr1 = MAIR0.Field(8, 8, MemoryAttribute); // ...
    const Attr2 = MAIR0.Field(16, 8, MemoryAttribute);
    const Attr3 = MAIR0.Field(24, 8, MemoryAttribute);
    const Attr4 = MAIR1.Field(0, 8, MemoryAttribute);
    const Attr5 = MAIR1.Field(8, 8, MemoryAttribute);
    const Attr6 = MAIR1.Field(16, 8, MemoryAttribute);
    const Attr7 = MAIR1.Field(24, 8, MemoryAttribute);
    pub inline fn Reset() void {
        const reset = SET(.r0, 0);
        MAIR0.writeFrom(reset);
        MAIR1.writeFrom(reset);
    }

    fn isMatch(comptime field: anytype, comptime attr: MemoryAttribute) bool {
        return switch (field.Read()) {
            0 => brk: {
                field.Select(attr);
                break :brk true;
            },
            @intFromEnum(attr) => true,
            else => false,
        };
    }

    pub fn GetIndex(comptime attr: MemoryAttribute) AttrIndexIntType {
        if (@intFromEnum(attr) == 0) {
            Attr0.Set(0);
            return 0;
        }
        if (isMatch(Attr1, attr)) return 1;
        if (isMatch(Attr2, attr)) return 2;
        if (isMatch(Attr3, attr)) return 3;
        if (isMatch(Attr4, attr)) return 4;
        if (isMatch(Attr5, attr)) return 5;
        if (isMatch(Attr6, attr)) return 6;
        if (isMatch(Attr7, attr)) return 7;
        unreachable; // FIXME?
    }
};

fn TTBR(common: anytype) type {
    return struct {
        const writeFrom = common.writeFrom;
        const ExtendedField = common.ExtendedField;
        const ExtendedBit = common.ExtendedBit;
        const self = @This();
        // When TTBCR.EAE = 0. Unused alternative
        const TTB = undefined; // Translation table base address.
        const IRGN0 = undefined; // Inner region cacheability
        const IRGN1 = undefined; // Inner region cacheability
        const NOS = undefined; // Not Outer Shareable
        const RGN = undefined; // Outer cacheability
        const S = undefined; // Shareable
        // When TTBCR.EAE = 1
        pub const ASID = self.ExtendedField(48, 8, enum {}); // Address Space Identifier
        pub const BADDR = self.ExtendedField(1, 47, enum {}); // Translation table base address. NOTE: meaningful bits are tuneable!
        pub const CnP = self.ExtendedBit(0); // Common not Private

        pub inline fn Reset() void {
            _ = SET(.r0, 0);
            _ = SET(.r1, 0);
            self.writeFrom(.{ .r0, .r1 });
        }
    };
}

const TTBR0 = TTBR(CP15Reg(0, 2, 0, 0, .ReadWrite)); // Translation Table Base Register 0: G8-12307[1]
const TTBR1 = undefined; // Translation Table Base Register 1: G8-12314[1]

const TTBCR = struct { // Translation Table Base Control Register: G8-12293[1]
    const common = CP15Reg(0, 2, 0, 2, .ReadWrite);
    const writeFrom = common.writeFrom;
    const Bit = common.Bit;
    const Field = common.Field;
    pub const EAE = TTBCR.Bit(31); // Extended Address Enable
    // When TTBCR.EAE = 0. Unused alternative
    const PD1 = undefined; // Translation table walk disable for translating using TTBR1
    const PD0 = undefined; // Translation table walk disable for translating using TTBR0
    const N = undefined; // Width of base address in TTBR0 & TTBR1
    // When TTBCR.EAE = 1
    const SH1 = undefined; // Shareability attribute for memory associated with translation table walks using TTBR1
    const ORGN1 = undefined; // Outer cacheability attribute for memory associated with translation table walks using TTBR1
    const IRGN1 = undefined; // Inner cacheability attribute for memory associated with translation table walks using TTBR1
    pub const EPD1 = TTBCR.Bit(23); // Translation table walk disable for translations using TTBR1
    pub const A1 = TTBCR.Field(22, 1, enum(u1) { ASIDfromTTBR0 = 0, ASIDfromTTBR1 = 1 }); // Selects whether TTBR0 or TTBR1 defines the ASID
    pub const TSZ = enum(u6) { // address ranges. See G5.5.5 (G5-11652[1])
        TTBR0_4GB = 0,
        TTBR0_2GB = 1,
        TTBR0_1GB = 2,
        TTBR0_512MB = 3,
        TTBR0_256MB = 4,
        TTBR0_128MB = 5,
        TTBR0_64MB = 6,
        TTBR0_32MB = 7,
        // Input address ranges & sizes, translated using TTBR0 and TTBR1
        const T1SZ = TTBCR.Field(16, 3, enum {});
        const T0SZ = TTBCR.Field(0, 3, enum {});
        pub inline fn Select(comptime range: @This()) void {
            T0SZ.Set(@intFromEnum(range) & 0b111);
            T1SZ.Set(@intFromEnum(range) >> 3);
        }
        pub inline fn asU32(comptime range: @This()) u32 {
            return (@as(u32, @intFromEnum(range)) & 0b111) | ((@as(u32, @intFromEnum(range)) >> 3) << 16);
        }
    };
    pub const SH0 = TTBCR.Field(12, 3, enum(u3) { NonShareable = 0, OuterShareable = 1, InnerShareable = 3 }); // Shareability attribute for memory associated with translation table walks using TTBR0
    const CacheAbility = enum(u2) { NonCacheable = 0, WbRaWaCacheable = 1, WtRaCacheable = 2, WbRaCacheable = 3 };
    pub const ORGN0 = TTBCR.Field(10, 2, CacheAbility); // Outer cacheability attribute for memory associated with translation table walks using TTBR0
    pub const IRGN0 = TTBCR.Field(8, 2, CacheAbility); // Inner cacheability attribute for memory associated with translation table walks using TTBR0
    pub const EPD0 = TTBCR.Bit(7); // Translation table walk disable for translations using TTBR0
    pub const T2E = TTBCR.Bit(6); // TTBCR2 Enable

    pub inline fn Reset() void {
        const reset = SET(.r0, 0);
        TTBCR.writeFrom(reset);
    }
};

const ACTLR = struct { // Auxilary Control Register: G8-11795[1], 4-59[2]
    const common = CP15Reg(0, 1, 0, 1, .ReadWrite);
    const Bit = common.Bit;
    pub const SMP = ACTLR.Bit(6); // Coherent requests to the processor
};

const CTR = struct { // Cache Type Register: G8-11868[1]
    const common = CP15Reg(0, 0, 0, 1, .ReadOnly);
    const Field = common.Field;
    pub const DminLine = CTR.Field(16, 4, enum {}); // Log2 of the number of words in the smallest cache line

};

const DCIMVAC = struct { // Data Cache line Invalidate by VA to Point of Coherency: G8-11883[1]
    const common = CP15Reg(0, 7, 6, 1, .WriteOnly);
    const write = common.write;
};

const MPIDR = struct { // Multiprocessor Affinity Register: G8-12140[1]
    const common = CP15Reg(0, 0, 0, 5, .ReadOnly);
    const Field = common.Field;
    pub const Aff1 = MPIDR.Field(8, 8, enum(u8) { CLUSTER0 = 0 });
    pub const Aff0 = MPIDR.Field(0, 8, enum(u8) { CPU0 = 0 });
};

const CPSR = struct { // Current Program Status Register: G8-11861[1]
    const common = SysReg(.cpsr, .mrs, .msr);
    const Bit = common.Bit;
    pub const DIT = CPSR.Bit(21); // Data Independent Timing
};

const ID_PFR0 = struct { // Processor Feature Register 0: G8-12095[1]
    const common = CP15Reg(0, 0, 1, 0, .ReadOnly);
    const Field = common.Field;
    pub const DIT = Field(24, 4, enum(u4) { Implemented = 0b001 }); // Data Independent Timing
};

const PMCR = struct { // Performance Monitor Control Register: G8-12525[1]
    const common = CP15Reg(0, 9, 12, 0, .ReadWrite);
    const Bit = common.Bit;
    const writeFrom = common.writeFrom;
    pub const DP = PMCR.Bit(5); // Disable cycle counter when event counting Prohibited
    pub const ResetValue = 0;
};

const SCR = struct { // Secure Configuration Register: G8-12190[1]
    const common = CP15Reg(0, 1, 1, 0, .ReadWrite);
    const writeFrom = common.writeFrom;
    const Bit = common.Bit;
    pub const SIF = SCR.Bit(9); // Secure Instruction Fetch
    pub const ResetValue = 0;
};

const VBAR = struct { // Vector Base Address Register: G8-12321[1]
    const common = CP15Reg(0, 12, 0, 0, .ReadWrite);
    const writeFrom = common.writeFrom;
};

const MVBAR = struct { // Monitor Vector Base Register: G8-12143[1]
    const common = CP15Reg(0, 12, 0, 1, .ReadWrite);
    const writeFrom = common.writeFrom;
};

const SCTLR = struct { // System Control Register: G8-12196[1]
    const common = CP15Reg(0, 1, 0, 0, .ReadWrite);
    const Field = common.Field;
    const Bit = common.Bit;
    const ReservedBit = common.ReservedBit;
    const writeFrom = common.writeFrom;
    pub const DSSBS = Field(31, 1, enum(u1) { DisableMitigation = 0, EnableMitigation = 1 }); // Default Speculative Store Bypass Safe value on exception
    pub const TE = Field(30, 1, enum(u1) { ARM = 0, Thumb = 1 }); // T32 Exception Enable
    pub const EE = Field(25, 1, enum(u1) { LittleEndian = 0, BigEndian = 1 }); // Endianess on Exception
    pub const WXN = Bit(19); // Write permission implies XN (Execute-never)
    pub const NTWE = Field(18, 1, enum(u1) { Trapped = 0, NotTrapped = 1 }); // Trap execution of WFE at EL0
    pub const NTWI = Field(16, 1, enum(u1) { Trapped = 0, NotTrapped = 1 }); // Trap execution of WFI at EL0
    pub const V = Field(13, 1, enum(u1) { LowVectors = 0, HiVectors = 1 }); // Vectors bit
    pub const I = Bit(12); // Instruction access Cacheability control, for accesses from EL1 and EL0
    pub const CP15BEN = Bit(5); // System instruction memory barrier enable
    pub const C = Bit(2); // Cecheability control, for data accesses at EL1 and EL0
    pub const A = Bit(1); // Alignment ckeck enable
    pub const M = Bit(0); // MMU enablefor EL1 ad EL0 stage 1 translation
    const ReservedBit23 = ReservedBit(23, 1); // aka SPAN
    const ReservedBit22 = ReservedBit(22, 1); // RES1
    const ReservedBit04 = ReservedBit(4, 1); // aka LSMAOE
    const ReservedBit03 = ReservedBit(3, 1); // aka nTLSMD
    pub const ResetValue = ReservedBit23.asU32() | ReservedBit22.asU32() | ReservedBit04.asU32() | ReservedBit03.asU32();
};

const CPACR = struct { // Architectural Feature Access Control Register: G8-11852[1]
    const common = CP15Reg(0, 1, 0, 2, .ReadWrite);
    const Field = common.Field;
    const Bit = common.Bit;
    const writeFrom = common.writeFrom;
    pub const CP10 = CPACR.Field(20, 2, enum(u2) { Disabled = 0b0, PL1Only = 0b1, Enabled = 0b11 });
    pub const CP11 = CPACR.Field(22, 2, enum(u2) { Disabled = 0b0, PL1Only = 0b1, Enabled = 0b11 });
    pub const TRCDIS = CPACR.Bit(28); // Traps PL0 and PL1 access to all trace registers to Undefined Mode
    pub const ResetValue = 0;
};

const NSACR = struct { // Non-Secure Access Control Register: G8-12162[1]
    const common = CP15Reg(0, 1, 1, 2, .ReadWrite);
    const writeFrom = common.writeFrom;
    const Field = common.Field;
    const Bit = common.Bit;
    pub const AllCPAccessInNonSecureState = NSACR.Field(0, 14, enum(u1) { Disabled = 0 });
    pub const CP10 = NSACR.Field(10, 1, enum(u1) { SecureAccessOnly = 0, AccessFromAnySecureState = 1 }); //Defines access rights for SIMD and FP functionality
    pub const CP11 = NSACR.Field(11, 1, enum(u1) { SecureAccessOnly = 0, AccessFromAnySecureState = 1 }); // unused, must follow CP10
    const NSTRCDIS = NSACR.Bit(20);
};

const ID_DFR0 = struct { // Debug Feature Register: G8-12037[1]
    const common = CP15Reg(0, 0, 1, 2, .ReadOnly);
    const Field = common.Field;
    pub const CopTrc = ID_DFR0.Field(12, 4, enum(u4) { Implemented = 0b0001 }); // Support for System registers-based model, using registers in the coproc == 0b1110 encoding space
};

const FPEXC = struct { // Floating Point Excepion Register: G8-11910[1]
    const common = SysReg(.fpexc, .vmrs, .vmsr);
    const ReservedField = common.ReservedField;
    const Bit = common.Bit;
    const writeFrom = common.writeFrom;
    pub const VECITR = FPEXC.ReservedField(8, 3, 0b111); // Vector Iteration count
    pub const EN = FPEXC.Bit(30); // Enable Access to SIMD and FP functionality
    pub const ResetValue = VECITR.asU32();
};

const BSEC_DENABLE = struct {
    const common = PlatformReg(0x5C005014);
    const writeFrom = common.writeFrom;
};

fn CP15Reg(comptime op1: u3, comptime crn: u4, comptime crm: u4, comptime op2: u3, comptime rw: enum { ReadOnly, ReadWrite, WriteOnly }) type {
    return struct {
        const generic = GenericAccessors(@This());
        pub const Field = generic.Field;
        pub const Bit = generic.Bit;
        pub const ExtendedField = generic.ExtendedField;
        pub const ExtendedBit = generic.ExtendedBit;
        pub const ReservedField = generic.ReservedField;
        pub const ReservedBit = generic.ReservedBit;
        // pub const common = GenericAccessors(@This());
        inline fn readTo(comptime access_reg: anytype) void {
            switch (rw) {
                .WriteOnly => {
                    @compileError("Register is ReadOnly");
                },
                else => {
                    const instruction = switch (@TypeOf(access_reg)) {
                        armv7_general_register => .mrc,
                        else => .mrrc,
                    };
                    access(access_reg, instruction);
                },
            }
        }
        inline fn writeFrom(access_reg: anytype) void {
            switch (rw) {
                .ReadOnly => @compileError("Register is ReadOnly"),
                else => {
                    const instruction = switch (@TypeOf(access_reg)) {
                        armv7_general_register => .mcr,
                        else => .mcrr,
                    };
                    access(access_reg, instruction);
                    asm volatile ("isb");
                },
            }
        }
        pub fn write(value: u32) void {
            comptime if (rw == .ReadOnly) @compileError("Register is ReadOnly");
            writeFrom(LOAD(.Word, .r1, SET(.r0, &value)));
        }

        inline fn access(comptime access_reg: anytype, comptime instruction: @TypeOf(.@"enum")) void {
            switch (@TypeOf(access_reg)) {
                armv7_general_register => {
                    asm volatile (print("{s} p15, {d}, {s}, c{d}, c{d}, {d}", .{ @tagName(instruction), op1, @tagName(access_reg), crn, crm, op2 }));
                    addRegClobber(access_reg);
                },
                else => {
                    comptime if ((@intFromEnum(@as(armv7_general_register, access_reg[0])) == @intFromEnum(@as(armv7_general_register, access_reg[1]))) or
                        (@import("std").mem.eql(u8, @tagName(access_reg[0]), "r15")) or
                        (@import("std").mem.eql(u8, @tagName(access_reg[1]), "r15")))
                    {
                        @compileLog(print("Unsupported registers as arguments: {s}, {s}", .{ @tagName(access_reg[0]), @tagName(access_reg[1]) }));
                    };
                    // BUG: parameters untested for the whole range of 64 bit registers for aarch32
                    asm volatile (print("{s} p15, {d}, {s}, {s}, c{d}", .{ @tagName(instruction), op2, @tagName(access_reg[0]), @tagName(access_reg[1]), crn }));
                    addRegClobber(access_reg[0]);
                    addRegClobber(access_reg[1]);
                },
            }
        }
    };
}

fn SysReg(comptime register: @TypeOf(.@"enum"), comptime read_instruction: @TypeOf(.@"enum"), comptime write_instruction: @TypeOf(.@"enum")) type {
    return struct {
        const common = GenericAccessors(@This());
        const ReservedField = common.ReservedField;
        const Bit = common.Bit;
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
        pub const common = GenericAccessors(@This());
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

        pub fn ExtendedBit(comptime bit: u6) type {
            return ExtendedField(bit, 1, enum(u1) { Disabled = 0, Enabled = 1 });
        }

        fn ExtendedField(comptime shift: u8, comptime width: u8, comptime values: @TypeOf(enum {})) type {
            const max = (1 << width) - 1;
            const mask: u64 = max << shift;
            const register_max_bits = 64;
            comptime if (shift + width > register_max_bits) {
                @compileError("field declaration error");
            };
            return struct {
                pub const IntType = @Type(@import("std").builtin.Type{ .int = .{ .bits = width, .signedness = .unsigned } });
                pub inline fn Select(comptime v: values) void {
                    Write(v);
                }
                pub fn Write(value: anytype) void {
                    var u64_value: u64 = switch (@TypeOf(value)) {
                        comptime_int => brk: {
                            if (value > max) {
                                @compileError(print("value {d} does not fit field space", .{value}));
                            }
                            break :brk value;
                        },
                        IntType => brk: {
                            break :brk value;
                        },
                        @TypeOf(.enum_enternal) => @intFromEnum(@as(values, value)),
                        values => @intFromEnum(value),
                        else => @compileError(print("value type unsupported: {s}. Expected: comptime_int, {s} or {s}", .{ @typeName(@TypeOf(value)), @typeName(values), @typeName(IntType) })),
                    };
                    u64_value <<= shift;
                    var reg = readRegister();
                    reg &= ~mask;
                    reg |= u64_value;
                    asm volatile (
                        \\ push {r0, r1}
                        \\ mov r0, %[low]
                        \\ mov r1, %[high]
                        :
                        : [low] "r" (@as(u32, @truncate(reg))),
                          [high] "r" (@as(u32, @intCast(reg >> 32))),
                        : .{ .r0 = true, .r1 = true });
                    self.writeFrom(.{ .r0, .r1 });
                    asm volatile ("pop {r0, r1}");
                }
                pub fn Read() IntType {
                    return @truncate((readRegister() & mask) >> shift);
                }
                pub inline fn asU64(comptime value: values) u64 {
                    return @as(u64, @intFromEnum(value)) << shift;
                }
                fn readRegister() u64 {
                    var lo: u32 = undefined;
                    var hi: u32 = undefined;
                    asm volatile ("push {r0, r1}");
                    self.readTo(.{ .r0, .r1 });
                    asm volatile (
                        \\ mov %[low], r0
                        \\ mov %[high], r1
                        : [low] "=r" (lo),
                          [high] "=r" (hi),
                        :
                        : .{ .r0 = true, .r1 = true });
                    asm volatile ("pop {r0, r1}");
                    return (@as(u64, @intCast(hi)) << 32) + lo;
                }
            };
        }

        pub fn Bit(comptime bit: u5) type {
            return Field(bit, 1, enum(u1) { Disabled = 0, Enabled = 1 });
        }

        const r0 = armv7_general_register.r0;
        fn Field(comptime shift: u5, comptime width: u5, comptime values: @TypeOf(enum {})) type {
            _ = comptime (shift + (width - 1)); // check Field declaration sanity
            return struct {
                pub const IntType = @Type(@import("std").builtin.Type{ .int = .{ .bits = width, .signedness = .unsigned } });
                const max = (1 << width) - 1;
                const mask: u32 = max << shift;
                pub fn Read() IntType {
                    var value: u32 = undefined;
                    self.readTo(r0);
                    asm volatile (
                        \\ mov %[value], r0
                        : [value] "=r" (value),
                        :
                        : .{ .r0 = true });
                    return @truncate((value & mask) >> shift);
                }
                pub inline fn Select(comptime v: values) void {
                    Set(@intFromEnum(v));
                }
                pub inline fn Set(comptime v: u32) void {
                    if (v == 0) {
                        Clear();
                    } else {
                        self.readTo(r0);
                        asm volatile (print("bfc {s}, #{d}, #{d}", .{ @tagName(r0), shift, width }));
                        asm volatile (print("orr {s}, {s}, #{d}", .{ @tagName(r0), @tagName(r0), v << shift }));
                        self.writeFrom(r0);
                    }
                }
                pub inline fn Clear() void {
                    self.readTo(r0);
                    asm volatile (print("bfc {s}, #{d}, #{d} ", .{ @tagName(r0), shift, width }));
                    self.writeFrom(r0);
                }
                pub inline fn readTo(comptime reg: armv7_general_register) armv7_general_register {
                    self.readTo(reg);
                    asm volatile (print("ubfx {s}, {s}, {d}, {d}", .{ @tagName(reg), @tagName(reg), shift, width }));
                    return reg;
                }
                pub inline fn asU32(comptime value: values) u32 {
                    return @as(u32, @intFromEnum(value)) << shift;
                }
                pub inline fn If(comptime condition: anytype, comptime cmp_value: anytype, action: anytype, comptime action_argument: anytype) void {
                    const reg = readTo(r0);
                    IF(reg, condition, switch (@TypeOf(cmp_value)) {
                        @TypeOf(.@"enum") => @intFromEnum(@as(values, cmp_value)),
                        else => cmp_value,
                    }, action, action_argument);
                }
            };
        }
    };
}

inline fn addRegClobber(reg: armv7_general_register) void {
    switch (reg) {
        .r0 => asm volatile ("" ::: .{ .r0 = true }),
        .r1 => asm volatile ("" ::: .{ .r1 = true }),
        .r2 => asm volatile ("" ::: .{ .r2 = true }),
        .r3 => asm volatile ("" ::: .{ .r3 = true }),
        .r4 => asm volatile ("" ::: .{ .r4 = true }),
        .r5 => asm volatile ("" ::: .{ .r5 = true }),
        .sp => {},
    }
}

pub inline fn SET(reg: armv7_general_register, value: anytype) armv7_general_register {
    switch (@typeInfo(@TypeOf(value))) {
        .pointer => {
            asm volatile (print("mov {s}, %[addr]", .{@tagName(reg)})
                :
                : [addr] "r" (value),
            );
        },
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
    addRegClobber(reg);
    return reg;
}

pub inline fn LOAD(comptime size: enum { Byte, HalfWord, Word }, comptime reg: armv7_general_register, address: armv7_general_register) armv7_general_register {
    asm volatile (print("{s} {s}, [{s}]", .{ switch (size) {
            .Byte => "ldrb",
            .HalfWord => "ldrh",
            .Word => "ldr",
        }, @tagName(reg), @tagName(address) }) ::: .{ .memory = true });
    addRegClobber(reg);
    return reg;
}

inline fn SAVE(comptime size: enum { Byte, HalfWord, Word }, comptime reg: armv7_general_register, address: armv7_general_register) void {
    asm volatile (print("{s} {s}, [{s}]", .{ switch (size) {
            .Byte => "strb",
            .HalfWord => "strh",
            .Word => "str",
        }, @tagName(reg), @tagName(address) }) ::: .{ .memory = true });
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
    @setEvalBranchQuota(100_000);
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
    addRegClobber(reg);
    addRegClobber(tmp_reg);
}

pub inline fn EndlessLoop() void {
    asm volatile ("bl parkingLoop");
    asm volatile ("nop");
}

export fn parkingLoop() callconv(.naked) void {
    @at(.label);
    asm volatile("label:");
    asm volatile ("b .");

}

pub inline fn goto(comptime func: anytype) void {
    asm volatile ("b %[addr]"
        :
        : [addr] "i" (func),
    );
}

pub inline fn call(comptime func: anytype) void {
    asm volatile ("bl %[addr]"
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

fn UpdateDataCache(comptime action: enum { Invalidate }, base: u32, size: u32) void {
    const sys_reg = switch (action) {
        .Invalidate => DCIMVAC,
    };
    if (size > 0) {
        const data_cache_line_size = @as(u32, CTR.DminLine.Read()) << cpu_word_size;
        var data_addr: u32 = base & ~(data_cache_line_size - 1);
        const data_end_addr: u32 = base + size;
        while (data_addr < data_end_addr) {
            sys_reg.write(data_addr);
            data_addr += data_cache_line_size;
        }
    }
}

pub export fn ResetMemory() void {
    UpdateDataCache(.Invalidate, @intFromPtr(&rw_data_start), @intFromPtr(&rw_data_size));
    var bss: []volatile u8 = undefined;
    bss.len = @intFromPtr(&bss_data_size);
    bss.ptr = @ptrFromInt(@intFromPtr(&bss_data_start));
    @memset(bss, 0);
}

pub inline fn DebugMode() void {
    BSEC_DENABLE.writeFrom(SET(.r0, 0x47f)); // enable full debug access
}

pub inline fn InitializeStacks() void {
    const cpus_per_cluster = SET(.r0, platform_cpus_per_cluster);
    const cpu_nr = MPIDR.Aff0.readTo(.r1);
    const cluster_nr = MPIDR.Aff1.readTo(.r2);
    const per_cpu_stack = SET(.r3, per_cpu_stacks_size);
    const shift = MUL(ADD(cpu_nr, MUL(cpus_per_cluster, cluster_nr)), per_cpu_stack);
    const stack_addr = ADD(SET(.r4, &platform_stacks), SET(.r5, @sizeOf(@TypeOf(platform_stacks))));

    SetMode(.Supervisor); // TODO: implement PreserveMode and reimplement SetMode accouning a register as argument
    _ = SET(.sp, SUB(stack_addr, shift));

    SetMode(.Monitor);
    _ = SET(.sp, SUB(stack_addr, svc_stack_size));

    SetMode(.Irq);
    _ = SET(.sp, SUB(stack_addr, mon_stack_size));

    SetMode(.Fiq);
    _ = SET(.sp, SUB(stack_addr, irq_stack_size));

    SetMode(.Abort);
    _ = SET(.sp, SUB(stack_addr, fiq_stack_size));

    SetMode(.Undefined);
    _ = SET(.sp, SUB(stack_addr, abt_stack_size));

    SetMode(.Monitor);
    asm volatile ("" ::: .{ .r0 = true, .r1 = true, .r2 = true, .r3 = true, .r4 = true, .r5 = true });
}

const fiq_stack_size = 512;
const irq_stack_size = 2048;
const abt_stack_size = 1024;
const und_stack_size = 1024;
const mon_stack_size = 2048;
const svc_stack_size = 8192;
const per_cpu_stacks_size = fiq_stack_size + irq_stack_size + abt_stack_size + und_stack_size + mon_stack_size + svc_stack_size;

const platform_stacks: [platform_clusters * platform_cpus_per_cluster * per_cpu_stacks_size]u8 align(8) linksection(".stack") = undefined;

const TranslationTableGranularity = enum(u5) {
    Block2M = 21,
    Page4K = 12,
    fn factor(self: TranslationTableGranularity) u32 {
        return @as(u32, 1) << @intFromEnum(self);
    }
    fn alignmentMask(self: TranslationTableGranularity) u32 {
        return ~(self.factor() - 1);
    }
    pub fn isAligned(self: TranslationTableGranularity, pa: u32) bool {
        return (pa & (self.factor() - 1)) == 0;
    }
    pub fn tableIndex(self: TranslationTableGranularity, table_base_va: u32, addr: u32) usize {
        return (addr & self.alignmentMask() - table_base_va) / self.factor();
    }
    pub fn va(self: TranslationTableGranularity, table_base_va: u32, table_idx: u32) u32 {
        return table_base_va + table_idx * self.factor();
    }
};

const DescriptorType = enum {
    Block,
    Page,
    Table,
    pub fn asLongDescriptorField(self: DescriptorType) u64 {
        return switch (self) {
            .Table, .Page => 0b11,
            .Block => 0b1,
        };
    }
};

const SecurityAccess = enum(u1) {
    SecureAccess = 0,
    NonSecureAccess = 1,
    pub fn asLongDescriptorField(self: SecurityAccess) u64 {
        return @as(u64, @intFromEnum(self)) << 5;
    }
};

const ReadWriteAccess = enum(u1) {
    ReadWrite = 0,
    ReadOnly = 1,
    pub fn asLongDescriptorField(self: ReadWriteAccess) u64 {
        return @as(u64, @intFromEnum(self)) << 7;
    }
};

const Shareability = enum(u2) {
    NonShareable = 0,
    OuterShareable = 0b10,
    InnerShareable = 0b11,
    pub fn asLongDescriptorField(self: Shareability) u64 {
        return @as(u64, @intFromEnum(self)) << 8;
    }
};

const ExecutePermission = enum(u2) {
    ExecutionPermitted = 0,
    ExecutionProhibited = 0b11,
    pub fn asLongDescriptorField(self: ExecutePermission) u64 {
        return @as(u64, @intFromEnum(self)) << 53;
    }
};

fn ttLongDescriptor(
    pa: u32,
    comptime desc_type: DescriptorType,
    comptime mem_attr: MemoryAttribute,
    comptime exec_perm: ExecutePermission,
    comptime secur_level: SecurityAccess,
    comptime rw_access: ReadWriteAccess,
    comptime shareability: Shareability,
) u64 {
    comptime if (exec_perm == .ExecutionPermitted) {
        if (rw_access == .ReadWrite)
            @compileError(print("prohibited attributes multiplex: {s}, {s}", .{ @tagName(rw_access), @tagName(exec_perm) }));
        if (mem_attr == .Device)
            @compileError(print("prohibited attributes multiplex: {s}, {s}", .{ @tagName(mem_attr), @tagName(exec_perm) }));
    };
    var ret: u64 = pa;
    ret |= desc_type.asLongDescriptorField();
    ret |= rw_access.asLongDescriptorField();
    ret |= exec_perm.asLongDescriptorField();
    ret |= mem_attr.asLongDescriptorField();
    ret |= secur_level.asLongDescriptorField();
    ret |= shareability.asLongDescriptorField();
    ret |= 1 << 10; // Access flag

    return ret;
}

fn ttTableLongDescriptor(pa: u32) u64 {
    return (@as(u64, pa) | DescriptorType.Table.asLongDescriptorField());
}

extern const sysram_start: u32;
extern const text_start: u32;
extern const ro_data_start: u32;
extern const mbox_start: u32;
extern const mbox_end: u32;
const page_size = 0x1000;
const master_tt = tt[0..512]; // Translation Table for first Gigabyte VA with 2MB granularity
const sysram_tt = tt[512..1024]; // Translation Table for Sysram & some periphery VAs with 4K granularity

fn invalidateTTCache() void {
    TLBIALL.writeFrom(armv7_general_register.r0);
}

pub export fn InitializeMMU() void { // G5.5 G5-11643[1]
    // NOTE: ad-hoc implementation for particular use case using Long-descriptor table format
    MAIR.Reset();
    @memset(&tt, 0);

    const sysram_idx = TranslationTableGranularity.Block2M.tableIndex(0, @intFromPtr(&sysram_start));
    master_tt[sysram_idx] = ttTableLongDescriptor(@intFromPtr(sysram_tt));
    const sysram_block_start_address = TranslationTableGranularity.Block2M.va(0, sysram_idx);

    // map sysram pages:
    var current_pa = @intFromPtr(&text_start);
    while (current_pa < @intFromPtr(&ro_data_start)) {
        const idx = TranslationTableGranularity.Page4K.tableIndex(sysram_block_start_address, current_pa);
        sysram_tt[idx] = ttLongDescriptor(current_pa, .Page, .NormalMemory, .ExecutionPermitted, .SecureAccess, .ReadOnly, .InnerShareable);
        current_pa +|= page_size;
    }

    current_pa = @intFromPtr(&ro_data_start);
    while (current_pa < @intFromPtr(&rw_data_start)) {
        const idx = TranslationTableGranularity.Page4K.tableIndex(sysram_block_start_address, current_pa);
        sysram_tt[idx] = ttLongDescriptor(current_pa, .Page, .NormalMemory, .ExecutionProhibited, .SecureAccess, .ReadOnly, .InnerShareable);
        current_pa +|= page_size;
    }

    current_pa = @intFromPtr(&rw_data_start);
    while (current_pa < @intFromPtr(&mbox_start)) {
        const idx = TranslationTableGranularity.Page4K.tableIndex(sysram_block_start_address, current_pa);
        sysram_tt[idx] = ttLongDescriptor(current_pa, .Page, .NormalMemory, .ExecutionProhibited, .SecureAccess, .ReadWrite, .InnerShareable);
        current_pa +|= page_size;
    }

    current_pa = @intFromPtr(&mbox_start);
    while (current_pa < @intFromPtr(&mbox_end)) {
        const idx = TranslationTableGranularity.Page4K.tableIndex(sysram_block_start_address, current_pa);
        sysram_tt[idx] = ttLongDescriptor(current_pa, .Page, .NormalNonCacheableMemory, .ExecutionProhibited, .SecureAccess, .ReadWrite, .NonShareable);
        current_pa +|= page_size;
    }

    // map pages, occupied by translation tables
    for ([2][]u64{ master_tt, sysram_tt }) |t| {
        current_pa = @intFromPtr(t.ptr);
        const tt_idx = TranslationTableGranularity.Page4K.tableIndex(sysram_block_start_address, current_pa);
        sysram_tt[tt_idx] = ttLongDescriptor(current_pa, .Page, .NormalMemory, .ExecutionProhibited, .SecureAccess, .ReadWrite, .InnerShareable);
    }

    TTBCR.Reset();
    TTBR0.Reset();
    TTBCR.EAE.Select(.Enabled);
    TTBCR.TSZ.Select(.TTBR0_1GB);
    TTBCR.EPD0.Select(.Disabled);
    TTBCR.EPD1.Select(.Enabled);
    TTBCR.A1.Select(.ASIDfromTTBR0);
    TTBCR.SH0.Select(.InnerShareable);
    TTBCR.IRGN0.Select(.WbRaWaCacheable);
    TTBCR.ORGN0.Select(.WbRaWaCacheable);
    TTBCR.T2E.Select(.Disabled);
    TTBR0.BADDR.Write(@as(TTBR0.BADDR.IntType, @intCast(@intFromPtr(master_tt) >> 1)));
    TTBR0.ASID.Write(@as(TTBR0.ASID.IntType, 0));
    ID_MMFR4.CnP.If(.Equal, .Supported, TTBR0.CnP.Select, .{.Enabled});
    invalidateTTCache();
    SCTLR.WXN.Select(.Enabled);
    SCTLR.C.Select(.Enabled);
    asm volatile ("dsb ish");
    SCTLR.M.Select(.Enabled);
}

var tt: [512 * 2]u64 align(4096) linksection(".tt") = undefined;

fn avaliablePages() struct { start_idx: usize, pages: usize } {
    var start_idx: usize = 0;
    for (sysram_tt[start_idx..]) |d| {
        if (d == 0) break;
        start_idx +|= 1;
    }
    var end_idx: usize = start_idx;
    for (sysram_tt[end_idx..]) |d| {
        if (d != 0) break;
        end_idx +|= 1;
    }
    return .{ .start_idx = start_idx, .pages = end_idx - start_idx };
}

pub fn mapPeriphery(pa: u32, size: usize) error{ NotAvailable, NotAligned }!u32 {
    if (!TranslationTableGranularity.Page4K.isAligned(pa)) return error.NotAligned;
    invalidateTTCache();
    defer invalidateTTCache();
    const required_pages = @max(1, size / page_size);
    const available = avaliablePages();
    if (required_pages > available.pages)
        return error.NotAvailable;

    const sysram_idx = TranslationTableGranularity.Block2M.tableIndex(0, @intFromPtr(&sysram_start));
    const sysram_block_start_address = TranslationTableGranularity.Block2M.va(0, sysram_idx);

    var current_pa = pa;
    const end_pa = current_pa + size;
    var current_idx = available.start_idx;
    while (current_pa < end_pa) {
        sysram_tt[current_idx] = ttLongDescriptor(current_pa, .Page, .Device, .ExecutionProhibited, .SecureAccess, .ReadWrite, .NonShareable);
        current_pa +|= page_size;
        current_idx +|= 1;
    }
    return TranslationTableGranularity.Page4K.va(sysram_block_start_address, available.start_idx);
}
