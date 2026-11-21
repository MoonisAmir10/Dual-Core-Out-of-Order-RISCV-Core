# Peripherals Register Reference & Bare-Metal C HAL Programmer's Guide

**Document ID:** `SOC-REF-03`
**HAL Headers:** [`sw/hal/`](../sw/hal/) (`soc_regs.h`, `hal_irq.h`, `hal_clint.h`, `hal_plic.h`, `hal_gpio.h`, `hal_uart.h`, `hal_dma.h`, `hal_timer.h`)
**Runtime Startup & Trap Trampoline:** [`sw/common/crt0.S`](../sw/common/crt0.S), [`sw/common/trap.S`](../sw/common/trap.S), [`sw/common/link.ld`](../sw/common/link.ld), [`sw/common/sync.h`](../sw/common/sync.h)

---

## 1. PLIC External Interrupt Source Map

| PLIC Source ID | Signal | Peripheral | Trigger Default | Clear Mechanism |
| :---: | :--- | :--- | :---: | :--- |
| `0` | Reserved | — | — | Hardwired non-existent per RISC-V PLIC spec |
| `1` | `GPIO_IRQ` | `apb4_gpio` (`S2`) | Level | Write 1 to clear bit(s) in `GPIO_IRQ_STAT` (`0x1000_0024`) |
| `2` | `UART_RX_IRQ` | `apb4_uart` (`S3`) | Level | Drain RX FIFO via `UART_RX_DATA` (`0x1001_0004`) |
| `3` | `UART_TX_IRQ` | `apb4_uart` (`S3`) | Level | Push byte to `UART_TX_DATA` or clear `CTRL.TX_IRQ_EN` |
| `4` | `DMA_CH0_IRQ` | `dma_axi4_coh` (`S4`) | Level | Write `1 << 0` (W1C) to `DMA_IRQ_STAT` (`0x1002_0004`) |
| `5` | `DMA_CH1_IRQ` | `dma_axi4_coh` (`S4`) | Level | Write `1 << 1` (W1C) to `DMA_IRQ_STAT` (`0x1002_0004`) |
| `6` | `TIMER0_IRQ` | `apb4_timer_pwm` (`S5`) | Level | Write `1` (W1C) to `TIMER0_STAT` (`0x1003_0014`) |
| `7` | `TIMER1_IRQ` | `apb4_timer_pwm` (`S5`) | Level | Write `1` (W1C) to `TIMER1_STAT` (`0x1003_0034`) |
| `8`–`15` | Expansion / `IRQ_INJECT` | Expansion slots / `SOC_CTRL` | Configurable | Clear source or `SOC_CTRL.IRQ_INJECT` (`0x1004_0018`) |

---

## 2. Peripheral Register Maps (Bit-by-Bit Reference)

### 2.1 S0: CLINT (`0x0200_0000` — `apb4_clint.sv`, `hal_clint.h`)

| Offset | Register | Access | Reset | Bitfields & Function |
| :---: | :--- | :---: | :---: | :--- |
| `0x0000` | `MSIP0` | RW | `0x0` | `[0]` Core 0 Machine Software Interrupt (write `1` to send IPI to Core 0, `0` to clear) |
| `0x0004` | `MSIP1` | RW | `0x0` | `[0]` Core 1 Machine Software Interrupt (write `1` to send IPI to Core 1, `0` to clear) |
| `0x4000` | `MTIMECMP0_LO` | RW | `0xFFFF_FFFF` | Lower 32 bits of Core 0 timer compare (`MTIP0 = (mtime >= mtimecmp0)`) |
| `0x4004` | `MTIMECMP0_HI` | RW | `0xFFFF_FFFF` | Upper 32 bits of Core 0 timer compare |
| `0x4008` | `MTIMECMP1_LO` | RW | `0xFFFF_FFFF` | Lower 32 bits of Core 1 timer compare (`MTIP1 = (mtime >= mtimecmp1)`) |
| `0x400C` | `MTIMECMP1_HI` | RW | `0xFFFF_FFFF` | Upper 32 bits of Core 1 timer compare |
| `0xBFF8` | `MTIME_LO` | RW | `0x0000_0000` | Lower 32 bits of shared 64-bit cycle-accurate timebase (increments +1/clk) |
| `0xBFFC` | `MTIME_HI` | RW | `0x0000_0000` | Upper 32 bits of shared 64-bit timebase |
| `0xC000`–`0xC00C` | `SCRATCH[0..3]` | RW | `0x0000_0000` | Four 32-bit uncached hardware mailbox registers |

---

### 2.2 S1: RISC-V PLIC (`0x0C00_0000` — `apb4_plic.sv`, `hal_plic.h`)

Compliant with the RISC-V PLIC v1.0.0 specification (16 sources, 3-bit priorities `0..7`, 2 independent hart M-mode contexts). Ties at equal priority resolve to the **lowest source ID**.

| Offset | Register | Access | Reset | Bitfields & Function |
| :---: | :--- | :---: | :---: | :--- |
| `0x000000` | `PRIO[0]` | RO | `0x0` | Source 0 priority (hardwired `0`) |
| `0x000004`–`0x00003C` | `PRIO[1..15]` | RW | `0x0` | `[2:0]` Source priority (`0` = disabled, `1` = lowest, `7` = highest) |
| `0x001000` | `PENDING` | RO | `0x0` | `[15:1]` Live gateway pending bitmap (`IP[src]`); bit `0` always `0` |
| `0x001080` | `TRIG_EDGE` | RW | `0x0` | `[15:1]` Per-source gateway trigger mode (`0` = level-sensitive, `1` = rising-edge) |
| `0x002000` | `ENABLE_C0` | RW | `0x0` | `[15:1]` Core 0 (Context 0) per-source interrupt enable mask |
| `0x002080` | `ENABLE_C1` | RW | `0x0` | `[15:1]` Core 1 (Context 1) per-source interrupt enable mask |
| `0x200000` | `THRESH_C0` | RW | `0x0` | `[2:0]` Core 0 priority threshold (only sources with `prio > thresh` fire `MEIP0`) |
| `0x200004` | `CLAIM_C0` | RW | `0x0` | **Read:** Atomically claims highest-prio pending enabled ID (`0` if none), clears `IP[id]`, locks gateway.<br/>**Write:** Completes `id[3:0]`, unlocking gateway. |
| `0x201000` | `THRESH_C1` | RW | `0x0` | `[2:0]` Core 1 priority threshold |
| `0x201004` | `CLAIM_C1` | RW | `0x0` | Core 1 Atomic Claim (Read) / Complete (Write) |

---

### 2.3 S2: 32-Pin GPIO Controller (`0x1000_0000` — `apb4_gpio.sv`, `hal_gpio.h`)

| Offset | Register | Access | Reset | Bitfields & Function |
| :---: | :--- | :---: | :---: | :--- |
| `0x000` | `DATA_IN` | RO | Sampled | `[31:0]` Synchronized input pin values (output pins reflect `DATA_OUT`) |
| `0x004` | `DATA_OUT` | RW | `0x0000_0000` | `[31:0]` Output latch values (driven on `gpio_out[31:0]`) |
| `0x008` | `DIR` | RW | `0x0000_0000` | `[31:0]` Pin direction (`1` = output / `gpio_oe=1`, `0` = input) |
| `0x00C` | `OUT_SET` | WO | `0` | `[31:0]` Atomic bitwise set: `DATA_OUT <= DATA_OUT \| wdata` (dual-core safe!) |
| `0x010` | `OUT_CLR` | WO | `0` | `[31:0]` Atomic bitwise clear: `DATA_OUT <= DATA_OUT & ~wdata` |
| `0x014` | `OUT_TGL` | WO | `0` | `[31:0]` Atomic bitwise toggle: `DATA_OUT <= DATA_OUT ^ wdata` |
| `0x018` | `IRQ_EN` | RW | `0x0000_0000` | `[31:0]` Per-pin interrupt enable mask (`GPIO_IRQ = \|(IRQ_STAT & IRQ_EN)`) |
| `0x01C` | `IRQ_TYPE` | RW | `0x0000_0000` | `[31:0]` Trigger mode (`0` = level-sensitive, `1` = edge-sensitive) |
| `0x020` | `IRQ_POL` | RW | `0xFFFF_FFFF` | `[31:0]` Polarity (`1` = active-high / rising-edge, `0` = active-low / falling-edge) |
| `0x024` | `IRQ_STAT` | W1C | `0x0000_0000` | `[31:0]` Latched per-pin IRQ status (write `1` to clear acknowledged pins) |

---

### 2.4 S3: 16-FIFO Full-Duplex 8N1 UART (`0x1001_0000` — `apb4_uart.sv`, `hal_uart.h`)

| Offset | Register | Access | Reset | Bitfields & Function |
| :---: | :--- | :---: | :---: | :--- |
| `0x000` | `TX_DATA` | WO | `0` | `[7:0]` Push byte into 16-entry TX FIFO (dropped if `TX_FULL=1`) |
| `0x004` | `RX_DATA` | RO | `0` | `[7:0]` Pop oldest byte from 16-entry RX FIFO (`0` if `RX_EMPTY=1`) |
| `0x008` | `STATUS` | RO | `0x0000_0006` | `[0] TX_FULL`, `[1] TX_EMPTY`, `[2] RX_EMPTY`, `[3] RX_FULL`, `[4] TX_BUSY`, `[12:8] TX_LEVEL` (`0..16`), `[20:16] RX_LEVEL` (`0..16`) |
| `0x00C` | `CTRL` | RW | `0x0000_0003` | `[0] TX_EN (1)`, `[1] RX_EN (1)`, `[2] LOOPBACK` (`uart_tx → rx_line`), `[3] FAST_SIM` (1-cyc console drain), `[4] RX_IRQ_EN` (PLIC Src 2), `[5] TX_IRQ_EN` (PLIC Src 3) |
| `0x010` | `BAUD_DIV` | RW | `0x0000_0008` | `[15:0]` Clocks per serial bit (clamped to `>= 2`; 8N1 frame = `10 × BAUD_DIV` cycles) |

---

### 2.5 S4: 2-Channel I/O-Coherent AXI4 DMA Engine (`0x1002_0000` — `dma_axi4_coh.sv`, `hal_dma.h`)

**Write Protection (`PSLVERR=1`):** Writing any RO register (`DMA_STATUS`, `DMA_VERSION`, `CH_STATUS`), accessing any unmapped offset, or writing channel config registers (`SRC/DST/LEN/STRIDE/ROW_CNT`) while `CH_STATUS.BUSY == 1` asserts `PSLVERR=1` (`mcause=7`). Writing `CH_CTRL` (`ABORT`) and `DMA_IRQ_STAT` while busy is always permitted.

| Offset | Register | Access | Reset | Bitfields & Function |
| :---: | :--- | :---: | :---: | :--- |
| `0x000` | `DMA_STATUS` | RO | `0x0` | `[0] CH0_BUSY`, `[1] CH1_BUSY`, `[8] CH0_DONE`, `[9] CH1_DONE`, `[16] CH0_ERR`, `[17] CH1_ERR` |
| `0x004` | `DMA_IRQ_STAT` | W1C | `0x0` | `[0] CH0_IRQ` (PLIC Src 4), `[1] CH1_IRQ` (PLIC Src 5); write `1` to clear |
| `0x008` | `DMA_VERSION` | RO | `0x0002_0100` | `[31:16]=0x0002` (2 Channels), `[15:8]=0x01` (v1), `[7:0]=0x00` (128-bit AXI4) |
| `0x040` / `0x080` | `CH0/1_SRC` | RW | `0x0` | 32-bit source byte address (any byte alignment!) |
| `0x044` / `0x084` | `CH0/1_DST` | RW | `0x0` | 32-bit destination byte address (any byte alignment!) |
| `0x048` / `0x088` | `CH0/1_LEN` | RW | `0x0` | Transfer length in bytes (1D: total bytes; 2D: bytes per row) |
| `0x04C` / `0x08C` | `CH0/1_SRC_STRIDE` | RW | `0x0` | Row-to-row source byte stride (`MODE_2D = 1`) |
| `0x050` / `0x090` | `CH0/1_DST_STRIDE` | RW | `0x0` | Row-to-row destination byte stride (`MODE_2D = 1`) |
| `0x054` / `0x094` | `CH0/1_ROW_CNT` | RW | `0x1` | Number of rows in 2D strided transfer (`MODE_2D = 1`) |
| `0x058` / `0x098` | `CH0/1_CTRL` | RW | `0x0` | `[0] START`, `[1] IRQ_EN`, `[2] MODE_2D`, `[3] ABORT (W1P)`, `[7:4] BURST_MAX` (`0` = 16 beats / 256 B default; `1..15` = 1..15 beats) |
| `0x05C` / `0x09C` | `CH0/1_STATUS` | RO | `0x0` | `[0] BUSY`, `[1] DONE`, `[2] ERR`, `[5:4] ERR_CODE` (`2=SLVERR, 3=DECERR`), `[31:8] BYTES_DONE` (24-bit live counter) |

---

### 2.6 S5: 2-Channel 32-Bit Timer & PWM (`0x1003_0000` — `apb4_timer_pwm.sv`, `hal_timer.h`)

Channel 0 base = `+0x00`, Channel 1 base = `+0x20` (`CH0` → PLIC Src 6 & `pwm_out[0]`; `CH1` → PLIC Src 7 & `pwm_out[1]`):

| Offset (CH0 / CH1) | Register | Access | Reset | Bitfields & Function |
| :---: | :--- | :---: | :---: | :--- |
| `0x00` / `0x20` | `CTRL` | RW | `0x0` | `[0] EN`, `[1] AUTO_RELOAD` (`1`=periodic, `0`=one-shot auto-stop), `[2] PWM_EN`, `[3] IRQ_EN`, `[4] PWM_INV` |
| `0x04` / `0x24` | `PRESCALE` | RW | `0x0` | `[15:0]` Clock divider (counter ticks every `PRESCALE + 1` clocks) |
| `0x08` / `0x28` | `PERIOD` | RW | `0xFFFF_FFFF` | `[31:0]` Wrap period (`0..PERIOD` inclusive; total period = `(PERIOD+1)*(PRESCALE+1)` clocks) |
| `0x0C` / `0x2C` | `CMP` | RW | `0x0` | `[31:0]` PWM compare threshold (`pwm_out` high while `COUNT < CMP`) |
| `0x10` / `0x30` | `COUNT` | RW | `0x0` | `[31:0]` Live timer counter (writing resets prescaler phase) |
| `0x14` / `0x34` | `IRQ_STAT` | W1C | `0x0` | `[0]` Period-match flag (write `1` to clear) |

---

### 2.7 S6: SoC Control, `TOHOST`/`SYS_HALT` & Live PMU (`0x1004_0000` — `apb4_soc_ctrl.sv`, `soc_regs.h`)

Writing any RO register or unimplemented offset asserts `PSLVERR=1` (`mcause=7`).

| Offset | Register | Access | Reset / Value | Function |
| :---: | :--- | :---: | :---: | :--- |
| `0x000` | `SOC_ID` | RO | `0x2002_0002` (2C) / `0x2002_0001` (1C) | Hardware SoC version identifier |
| `0x004` | `CORE_COUNT` | RO | `2` (2C) / `1` (1C) | Number of active harts |
| `0x008` | `FEATURE_MASK` | RO | `0x0000_003F` (2C) / `0x0000_001F` (1C) | `[0] TRAPS`, `[1] APB4`, `[2] PLIC`, `[3] PERIPH`, `[4] FPU`, `[5] DMA` |
| `0x00C` | `SYS_HALT` | RW | `0x0` | Writing `[0]=1` asserts chip `all_halted` cleanly (works even when `mtvec != 0`) |
| `0x010` | `TOHOST` | RW | `0x0` | 32-bit software exit code / status mailbox |
| `0x014` | `SCRATCH` | RW | `0x0` | General-purpose 32-bit scratch register |
| `0x018` | `IRQ_INJECT` | RW | `0x0` | `[15:1]` Software-injectable PLIC test interrupt lines (OR'd with hardware IRQs) |
| `0x020`–`0x03C` | `PMU_*` | RO | Live | `0x20` Cycles, `0x24` C0 Instret, `0x28` C1 Instret, `0x2C` Dirty C2C, `0x30` L2 Hits, `0x34` L2 Misses, `0x38` DRAM Rds, `0x3C` DRAM Wrs |

---

## 3. Bare-Metal Software Startup, Trap Trampoline & C HAL API

### 3.1 Memory Layout & Per-Hart Stacks (`sw/common/link.ld`, `crt0.S`)
- `0x0000_0000`: Reset vector `_start` (both harts boot here, read `mhartid`, set disjoint 16 KB stacks: Core 0 `SP = 0x0003_FFF0`, Core 1 `SP = 0x0003_BFF0`, call `main(uint32_t hartid)`).
- **M-Mode Trap Trampoline (`sw/common/trap.S`):**
  - Supports both **Direct** (`_trap_entry_direct`) and **Vectored** (`_trap_vector_table`, 256 B aligned) `mtvec` modes.
  - Saves an 80-byte integer `trap_frame_t` (`ra, t0..t6, a0..a7, mepc, mstatus, mcause, mtval`).
  - **Lazy FP Context Save (`#ifdef __riscv_flen`):** Inspects `mstatus.FS` (`bits [14:13]`). Only if `FS == Dirty (2'b11)` does it push an additional 96-byte frame saving `fcsr` + all 20 caller-saved FP registers (`ft0`–`ft11`, `fa0`–`fa7`) — zero FP save/restore overhead for pure integer tasks!

### 3.2 Quick C HAL API Summary

| Header | Core Functions |
| :--- | :--- |
| [`soc_regs.h`](../sw/hal/soc_regs.h) | `mmio_read32()`, `mmio_write32()`, `soc_halt(code)`, `soc_park_secondary(code, &flag)`, base addresses & `SOC_FEAT_*` |
| [`hal_irq.h`](../sw/hal/hal_irq.h) | `hal_irq_init(hart)`, `hal_irq_set_mtvec_direct()`, `hal_irq_set_mtvec_vectored()`, `hal_irq_enable_mie(mask)`, `hal_irq_global_enable()`, `hal_irq_register_timer/ipi/ext/exc()` |
| [`hal_clint.h`](../sw/hal/hal_clint.h) | `hal_clint_mtime64()`, `hal_clint_set_mtimecmp(hart, cmp)`, `hal_clint_set_alarm_rel(hart, delta)`, `hal_clint_send_ipi(hart)`, `hal_clint_clear_ipi(hart)` |
| [`hal_plic.h`](../sw/hal/hal_plic.h) | `hal_plic_set_priority(src, prio)`, `hal_plic_set_trigger_edge(src, edge)`, `hal_plic_enable_irq(hart, src)`, `hal_plic_set_threshold(hart, th)`, `hal_plic_claim(hart)`, `hal_plic_complete(hart, src)` |
| [`hal_gpio.h`](../sw/hal/hal_gpio.h) | `hal_gpio_set_dir()`, `hal_gpio_read()`, `hal_gpio_write()`, `hal_gpio_set_bits()`, `hal_gpio_clear_bits()`, `hal_gpio_toggle_bits()`, `hal_gpio_irq_config()`, `hal_gpio_irq_enable()`, `hal_gpio_irq_clear()` |
| [`hal_uart.h`](../sw/hal/hal_uart.h) | `hal_uart_init(div, fast_sim, lb)`, `hal_uart_putc()`, `hal_uart_puts()`, `hal_uart_getc_nonblock()`, `hal_uart_tx_flush()`, `hal_uart_printf(fmt, ...)` |
| [`hal_dma.h`](../sw/hal/hal_dma.h) | `hal_dma_start()`, `hal_dma_start_ex()`, `hal_dma_start_2d()`, `hal_dma_memcpy_async()`, `hal_dma_wait()`, `hal_dma_abort()`, `hal_dma_irq_clear()`, `hal_dma_ch_has_err()`, `hal_dma_ch_err_code()` |
| [`hal_timer.h`](../sw/hal/hal_timer.h) | `hal_timer_init(ch, pre, per, cmp, flags)`, `hal_timer_stop(ch)`, `hal_timer_set_duty(ch, cmp)`, `hal_timer_irq_clear(ch)` |
| [`sync.h`](../sw/common/sync.h) | `amo_add_w()`, `amo_swap_w()`, `spin_lock()`, `spin_unlock()`, `barrier_wait(&bar, ncores, &sense)`, `fence_rw()` |

---

## 4. Ready-to-Use Bare-Metal C Examples

### Example 1: Zero-Flush Async Coherent DMA + Cross-Core PLIC ISR + Hard-Float Compute
```c
#include "hal_irq.h"
#include "hal_dma.h"
#include "hal_uart.h"

static float src_buf[1024] __attribute__((aligned(64)));
static float dst_buf[1024] __attribute__((aligned(64)));
static volatile uint32_t dma_done = 0u;

static void on_dma_ch0_done(uint32_t hartid, uint32_t src_id) {
    (void)hartid; (void)src_id;
    hal_dma_irq_clear(1u << 0);   /* Clear level-sensitive DMA IRQ */
    dma_done = 1u;
}

void run_coherent_dma_example(uint32_t hartid) {
    hal_irq_init(hartid);
    hal_irq_set_mtvec_direct();
    hal_irq_enable_mie(MIE_MEIE);
    hal_plic_set_threshold(hartid, 0u);

    if (hartid == 1u) {
        /* Route DMA Ch0 completion IRQ (PLIC Source 4) to Core 1 */
        hal_plic_set_priority(PLIC_IRQ_DMA_CH0, 5u);
        hal_irq_register_ext(1u, PLIC_IRQ_DMA_CH0, on_dma_ch0_done);
        hal_plic_enable_irq(1u, PLIC_IRQ_DMA_CH0);
        hal_irq_global_enable();

        while (!dma_done) { /* Core 1 executes hard-float RV32IMAF work concurrently */ }
        __asm__ volatile ("fence i, r" ::: "memory");
        /* dst_buf is immediately coherent — 0 cache invalidations needed! */
    } else {
        /* Core 0 writes dirty floats into L1D (M state) — NO manual cache flush! */
        for (uint32_t i = 0; i < 1024u; i++) src_buf[i] = (float)i * 0.5f + 1.0f;
        hal_dma_memcpy_async(0u, dst_buf, src_buf, sizeof(src_buf), 1u);
    }
}
```

### Example 2: Custom Synchronous Exception Recovery (`mcause = 2, 5, 7, 11`)
```c
static void my_exc_handler(uint32_t hartid, uint32_t cause, uint32_t mtval, trap_frame_t *tf) {
    (void)hartid; (void)mtval;
    if (cause == EXC_ILLEGAL_INSTR) {
        tf->a0 += 7u;           /* Emulate custom instruction */
    } else if (cause == EXC_LOAD_FAULT || cause == EXC_STORE_FAULT) {
        /* Recover cleanly from unmapped APB4 PSLVERR bus fault */
    }
    tf->mepc += 4u;             /* Advance past faulting 32-bit instruction */
}
```

### Example 3: Compiling & Simulating a Dual-Core `RV32IMAF` SoC Application
```bash
riscv64-unknown-elf-gcc -march=rv32imaf -mabi=ilp32f -O2 \
  -ffreestanding -nostdlib -nostartfiles -Wall -Wextra -Werror \
  -Isw/common -Isw/hal -T sw/common/link.ld \
  sw/common/crt0.S sw/common/trap.S sw/hal/hal_irq.c sw/hal/hal_uart.c \
  my_app.c -o build/my_app.elf

riscv64-unknown-elf-objcopy -O binary build/my_app.elf build/my_app.bin
python3 scripts/elf2hex.py build/my_app.bin build/my_app.hex --pad-words 4096

# Execute on Dual-Core MOESI + APB4 + Coherent AXI4 DMA Verilator SoC:
./build/v_dual_soc/Vdual_soc +HEX=build/my_app.hex +JITTER +UART_OUT=build/console.log
```
