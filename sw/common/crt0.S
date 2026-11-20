/* ============================================================================
 * Bare-Metal Dual-Core / Single-Core Startup (crt0.S)
 *   - Reset vector at 0x0000_0000
 *   - In dual-core mode (default): reads mhartid -> disjoint 16 KB stacks:
 *       Core 0 SP = 0x0003_FFF0, Core 1 SP = 0x0003_BFF0
 *   - In SINGLE_CORE_BASELINE mode (Phase 0 smoke on baseline RV32IM core):
 *       Sets a0 = 0, SP = 0x0000_3FF0 (fits 16 KB baseline DMEM)
 *   - Calls main(uint32_t hartid), stores return value in a4 (x14), halts via EBREAK
 * ============================================================================ */

.section .text.init, "ax"
.global _start
.type _start, @function

_start:
#ifdef SINGLE_CORE_BASELINE
    li      sp, 0x00003FF0
    li      a0, 0
#else
    csrr    a0, mhartid
    li      t0, 0x0003FFF0
    slli    t1, a0, 14          /* hartid * 16 KB */
    sub     sp, t0, t1
#endif
    call    main
    mv      a4, a0              /* Expose return checksum on hardware x14 port */
    ebreak
1:
    j       1b
