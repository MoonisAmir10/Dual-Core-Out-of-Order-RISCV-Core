/* ============================================================================
 * Bare-Metal Linker Script for Dual-Core RV32IMA OoO Processor
 * Unified 256 KB Physical RAM: 0x0000_0000 .. 0x0003_FFFF
 *   - 0x0000_0000 : _start (.text.init) — reset vector for Core 0 & Core 1
 *   - .text/.rodata/.data/.bss : shared program & global state
 *   - 0x0003_FFF0 : Core 0 Stack Top (16 KB)
 *   - 0x0003_BFF0 : Core 1 Stack Top (16 KB)
 * ============================================================================ */

ENTRY(_start)

MEMORY
{
    RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 256K
}

SECTIONS
{
    . = 0x00000000;

    .text : {
        KEEP(*(.text.init))
        *(.text .text.*)
    } > RAM

    .rodata : ALIGN(16) {
        *(.rodata .rodata.* .srodata .srodata.*)
    } > RAM

    .data : ALIGN(16) {
        *(.data .data.* .sdata .sdata.*)
    } > RAM

    .bss : ALIGN(16) {
        __bss_start = .;
        *(.bss .bss.* .sbss .sbss.* COMMON)
        . = ALIGN(16);
        __bss_end = .;
    } > RAM

    /DISCARD/ : {
        *(.comment)
        *(.riscv.attributes)
        *(.eh_frame)
    }
}
