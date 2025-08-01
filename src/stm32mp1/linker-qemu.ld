OUTPUT_ARCH(arm)

SYSRAM_LENGTH = 256K;
SYSRAM_START = 0x0;
TRO_START = SYSRAM_START;
TRO_LENGTH = 64K;
MBOX_LENGTH = 4K;
MBOX_START = SYSRAM_START + SYSRAM_LENGTH - MBOX_LENGTH;
TT_LENGTH = 8K;
TT_START = MBOX_START - TT_LENGTH;
RW_START = TRO_START + TRO_LENGTH;
RW_LENGTH = MBOX_START - TRO_START - TRO_LENGTH;

PROVIDE(sysram_start = SYSRAM_START);
PROVIDE(mbox_start = MBOX_START);
PROVIDE(mbox_end = MBOX_START + MBOX_LENGTH);

MEMORY
{
  TRO (xr)          : ORIGIN = TRO_START, LENGTH = TRO_LENGTH
  RW (xw)           : ORIGIN = RW_START, LENGTH = RW_LENGTH
  TT  (rw)          : ORIGIN = TT_START, LENGTH = TT_LENGTH
  MBOX (rw)         : ORIGIN = MBOX_START, LENGTH = MBOX_LENGTH
}

SECTIONS
{
    .tt (NOLOAD) : ALIGN(4K) {
        *(.tt);
    } > TT

    .entry : ALIGN(4) {
        KEEP(*(.text.EntryPoint));
    } > TRO

    .text : ALIGN(4K) {
        PROVIDE(text_start = .);
        KEEP(*(.text.ExceptionVectors))
        *(.text)
    } > TRO

    .rodata : ALIGN(4K) {
        PROVIDE(ro_data_start = .);
        *(.rodata)
        *(.rodata.*)
    } > TRO

    .tt (NOLOAD) : ALIGN(4K) {
        *(.tt);
    } > RW

    .data (NOLOAD) : ALIGN(4K) {
        PROVIDE(rw_data_start = .);
        *(.stack);
        *(.data)
        PROVIDE(rw_data_size = . - rw_data_start);
    } > RW

    .bss (NOLOAD) : ALIGN(4) {
        PROVIDE(bss_data_start = .);
        *(COMMON)
        *(.bss)
        PROVIDE(bss_data_size = . - bss_data_start);
    } > RW

    .mbox (NOLOAD) : ALIGN(4K) {
    } > MBOX

    /DISCARD/ : { *(.ARM.exidx*) }
}
