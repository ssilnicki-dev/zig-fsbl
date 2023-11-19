OUTPUT_ARCH(arm)
ENTRY(_start)

SECTIONS
{
  . = 0xC0000200;

  .text :
  {
    *(.text)
  }

  .data :
  {
    *(.data)
  }

  .bss :
  {
    *(.bss)
  }
}
