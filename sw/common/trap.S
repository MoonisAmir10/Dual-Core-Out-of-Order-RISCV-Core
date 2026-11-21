/* ============================================================================
 * Bare-Metal M-Mode Trap & Interrupt Trampoline (Direct & Vectored mtvec)
 * (Phase U3 — RV32IMA_Zicsr_Zifencei Dual-Core 2W OoO SoC)
 *
 * Stack Frame Layout (80 bytes, 16-byte aligned per RV32 ILP32 ABI):
 *   0(sp)  .. 60(sp) : 16 caller-saved GPRs (ra, t0..t6, a0..a7)
 *   64(sp)           : mepc   (mutable by C handler, e.g. tf->mepc += 4)
 *   68(sp)           : mstatus
 *   72(sp)           : mcause
 *   76(sp)           : mtval
 *
 * Exports:
 *   - _trap_entry_direct  (4B aligned)  : Direct mode  (mtvec = BASE | 0)
 *   - _trap_vector_table  (256B aligned): Vectored mode (mtvec = BASE | 1)
 *
 * Weakly-aliased C hooks (default to soc_trap_handler):
 *   - soc_trap_handler(mcause, mepc, mtval, trap_frame_t *tf)
 *   - soc_vec_exc_handler(...)   [Vector slot  0: +0x00]
 *   - soc_vec_msi_handler(...)   [Vector slot  3: +0x0C]
 *   - soc_vec_mti_handler(...)   [Vector slot  7: +0x1C]
 *   - soc_vec_mei_handler(...)   [Vector slot 11: +0x2C]
 * ============================================================================ */

.section .text.trap, "ax"

.macro SAVE_TRAP_FRAME
    addi    sp, sp, -80
    sw      ra,   0(sp)
    sw      t0,   4(sp)
    sw      t1,   8(sp)
    sw      t2,  12(sp)
    sw      a0,  16(sp)
    sw      a1,  20(sp)
    sw      a2,  24(sp)
    sw      a3,  28(sp)
    sw      a4,  32(sp)
    sw      a5,  36(sp)
    sw      a6,  40(sp)
    sw      a7,  44(sp)
    sw      t3,  48(sp)
    sw      t4,  52(sp)
    sw      t5,  56(sp)
    sw      t6,  60(sp)

    csrr    a1, mepc
    csrr    t0, mstatus
    csrr    a0, mcause
    csrr    a2, mtval
    sw      a1,  64(sp)
    sw      t0,  68(sp)
    sw      a0,  72(sp)
    sw      a2,  76(sp)
    mv      a3, sp
#ifdef __riscv_flen
    /* Lazy FP caller-saved context save (only when mstatus.FS == Dirty 2'b11).
     * Keeps a3 pointing at the 80-byte trap_frame_t and preserves 16B sp alignment! */
    li      t1, 0x6000
    and     t2, t0, t1
    bne     t2, t1, 1f
    addi    sp, sp, -96
    addi    t2, zero, 1
    sw      t2,   0(sp)
    frcsr   t1
    sw      t1,   4(sp)
    fsw     ft0,  8(sp)
    fsw     ft1, 12(sp)
    fsw     ft2, 16(sp)
    fsw     ft3, 20(sp)
    fsw     ft4, 24(sp)
    fsw     ft5, 28(sp)
    fsw     ft6, 32(sp)
    fsw     ft7, 36(sp)
    fsw     ft8, 40(sp)
    fsw     ft9, 44(sp)
    fsw     ft10, 48(sp)
    fsw     ft11, 52(sp)
    fsw     fa0, 56(sp)
    fsw     fa1, 60(sp)
    fsw     fa2, 64(sp)
    fsw     fa3, 68(sp)
    fsw     fa4, 72(sp)
    fsw     fa5, 76(sp)
    fsw     fa6, 80(sp)
    fsw     fa7, 84(sp)
    j       2f
1:
    addi    sp, sp, -16
    sw      zero, 0(sp)
2:
#endif
.endm

/* ----------------------------------------------------------------------------
 * Direct-Mode Trap Entry Point (4-byte aligned)
 * ---------------------------------------------------------------------------- */
.balign 4
.global _trap_entry_direct
.type _trap_entry_direct, @function
_trap_entry_direct:
    SAVE_TRAP_FRAME
    call    soc_trap_handler
    j       _trap_restore_and_mret

/* ----------------------------------------------------------------------------
 * Vectored-Mode Jump Table (256-byte aligned; mtvec = &_trap_vector_table | 1)
 * ---------------------------------------------------------------------------- */
.balign 256
.global _trap_vector_table
.type _trap_vector_table, @function
_trap_vector_table:
    j       _trap_vec_exc          /*  0 (+0x00): Synchronous Exceptions      */
    j       _trap_entry_direct     /*  1 (+0x04): Reserved                    */
    j       _trap_entry_direct     /*  2 (+0x08): Reserved                    */
    j       _trap_vec_msi          /*  3 (+0x0C): Machine Software Interrupt  */
    j       _trap_entry_direct     /*  4 (+0x10): Reserved                    */
    j       _trap_entry_direct     /*  5 (+0x14): Reserved                    */
    j       _trap_entry_direct     /*  6 (+0x18): Reserved                    */
    j       _trap_vec_mti          /*  7 (+0x1C): Machine Timer Interrupt     */
    j       _trap_entry_direct     /*  8 (+0x20): Reserved                    */
    j       _trap_entry_direct     /*  9 (+0x24): Reserved                    */
    j       _trap_entry_direct     /* 10 (+0x28): Reserved                    */
    j       _trap_vec_mei          /* 11 (+0x2C): Machine External Interrupt  */
    j       _trap_entry_direct     /* 12 (+0x30): Reserved                    */
    j       _trap_entry_direct     /* 13 (+0x34): Reserved                    */
    j       _trap_entry_direct     /* 14 (+0x38): Reserved                    */
    j       _trap_entry_direct     /* 15 (+0x3C): Reserved                    */

.balign 4
_trap_vec_exc:
    SAVE_TRAP_FRAME
    call    soc_vec_exc_handler
    j       _trap_restore_and_mret

.balign 4
_trap_vec_msi:
    SAVE_TRAP_FRAME
    call    soc_vec_msi_handler
    j       _trap_restore_and_mret

.balign 4
_trap_vec_mti:
    SAVE_TRAP_FRAME
    call    soc_vec_mti_handler
    j       _trap_restore_and_mret

.balign 4
_trap_vec_mei:
    SAVE_TRAP_FRAME
    call    soc_vec_mei_handler
    /* Fall through to shared epilogue */

/* ----------------------------------------------------------------------------
 * Shared Epilogue: Write Back Potentially Updated mepc, Restore 16 GPRs & MRET
 * ---------------------------------------------------------------------------- */
_trap_restore_and_mret:
#ifdef __riscv_flen
    lw      t0,   0(sp)
    beqz    t0,   1f
    flw     ft0,  8(sp)
    flw     ft1, 12(sp)
    flw     ft2, 16(sp)
    flw     ft3, 20(sp)
    flw     ft4, 24(sp)
    flw     ft5, 28(sp)
    flw     ft6, 32(sp)
    flw     ft7, 36(sp)
    flw     ft8, 40(sp)
    flw     ft9, 44(sp)
    flw     ft10, 48(sp)
    flw     ft11, 52(sp)
    flw     fa0, 56(sp)
    flw     fa1, 60(sp)
    flw     fa2, 64(sp)
    flw     fa3, 68(sp)
    flw     fa4, 72(sp)
    flw     fa5, 76(sp)
    flw     fa6, 80(sp)
    flw     fa7, 84(sp)
    lw      t1,   4(sp)
    fscsr   t1
    addi    sp, sp, 96
    j       2f
1:
    addi    sp, sp, 16
2:
#endif
    lw      t0,  64(sp)
    csrw    mepc, t0

    lw      ra,   0(sp)
    lw      t0,   4(sp)
    lw      t1,   8(sp)
    lw      t2,  12(sp)
    lw      a0,  16(sp)
    lw      a1,  20(sp)
    lw      a2,  24(sp)
    lw      a3,  28(sp)
    lw      a4,  32(sp)
    lw      a5,  36(sp)
    lw      a6,  40(sp)
    lw      a7,  44(sp)
    lw      t3,  48(sp)
    lw      t4,  52(sp)
    lw      t5,  56(sp)
    lw      t6,  60(sp)
    addi    sp, sp, 80
    mret

