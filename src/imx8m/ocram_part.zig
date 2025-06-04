extern const stack_bottom_addr: u64;
export fn _start() callconv(.naked) void {
    asm volatile ("mov sp, %[sp_value]"
        :
        : [sp_value] "r" (&stack_bottom_addr),
    );
    asm volatile (
    // Enable SIMD and Floating Point (coprocessor equivalent in AArch64)
    // Load the necessary value to enable VFP&NEON
        \\ mov x1, #0xF00000           // Move immediate value into x1
        \\ orr x1, x1, #(0b11 << 20)   // Enable access to SIMD/VFP (bits 21:20 to 0b11)
        \\ msr cpacr_el1, x1           // Write to CPACR_EL1 to enable coprocessor access (replaces MCR in AArch64)
        // Enable VFP (Floating Point Extension) instructions by setting FPEXC (replaced in AArch64)
        \\ mov x0, #0x40000000
        \\ msr fpcr, x0                // In AArch64, use FPCR for floating-point configuration
    );

    asm volatile ("bl main");
    asm volatile ("nop");
    asm volatile ("b .-4");
}

export fn main() void {}
