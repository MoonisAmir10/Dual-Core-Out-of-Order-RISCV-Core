/* ============================================================================
 * Phase U4 Live Bare-Metal C HAL & AMBA APB4 Peripheral Suite (1C & 2C!)
 *
 * Exercises the complete Phase U4 SoC Peripheral Suite + C HAL + PLIC ISRs:
 *   1. 32-Pin AMBA APB4 GPIO (0x1000_0000, PLIC Source 1):
 *        - Configures pins [15:8] output, [7:0] input, rising-edge IRQs on [2:0]
 *        - Raises gpio_out[15]=1 to trigger external +GPIO_STIM pin stimulus
 *        - Services all 3 external rising-edge interrupts via C PLIC ISR,
 *          atomically toggling gpio_out[10:8] and logging via hal_uart_printf()!
 *   2. 2-Channel AMBA APB4 Timer & PWM (0x1003_0000, PLIC Sources 6 & 7):
 *        - CH0: Periodic 200-cycle 50%-duty PWM (CMP=100) + 4 C PLIC ISR ticks
 *        - CH1: One-shot 250-cycle 50%-duty PWM (CMP=125) + auto-stop verification
 *        - Runs concurrently with GPIO + foreground deterministic ALU/MUL compute!
 *   3. 16-FIFO 8N1 AMBA APB4 UART (0x1001_0000, PLIC Sources 2 & 3):
 *        - Console output via zero-libc hal_uart_printf() (%u, %d, %02x, %04X, %s, %%)
 *        - Real bit-by-bit 8N1 serial loopback (BAUD_DIV=4, FAST_SIM=0, LOOPBACK=1)
 *          delivering "OK!" into the RX FIFO -> PLIC Source 2 C RX ISR!
 * ============================================================================ */
#include <stdint.h>
#include "sync.h"
#include "hal_irq.h"
#include "hal_gpio.h"
#include "hal_uart.h"
#include "hal_timer.h"

#define TARGET_GPIO_IRQS    3u
#define TARGET_T0_TICKS     4u
#define TARGET_T1_TICKS     1u
#define TARGET_UART_RX      3u

static volatile uint32_t g_gpio_cnt;
static volatile uint32_t g_t0_cnt;
static volatile uint32_t g_t1_cnt;
static volatile uint32_t g_rx_cnt;
static volatile char     g_rx_buf[8];
static volatile uint32_t g_c1_done;
static barrier_t         g_bar;

static void gpio_isr(uint32_t hartid, uint32_t src_id) {
    (void)src_id;
    uint32_t st = hal_gpio_irq_status();
    uint32_t in_val = hal_gpio_read() & 0xFFu;
    hal_gpio_irq_clear(st);

    uint32_t masked = st & 0x07u;
    if (masked != 0u) {
        /* Atomically toggle corresponding output LEDs gpio_out[10:8] */
        hal_gpio_toggle_bits(masked << 8);
        for (uint32_t b = 0u; b < 3u; b++) {
            if (masked & (1u << b)) {
                g_gpio_cnt += 1u;
            }
        }
        hal_uart_printf("GPIO_ISR core=%u st=0x%02x in=0x%02x\n", hartid, masked, in_val);
    }
}

static void timer0_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid;
    (void)src_id;
    hal_timer_irq_clear(0u);
    uint32_t c = g_t0_cnt + 1u;
    g_t0_cnt = c;
    if (c >= TARGET_T0_TICKS) {
        hal_timer_stop(0u);
    }
}

static void timer1_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid;
    (void)src_id;
    hal_timer_irq_clear(1u);
    g_t1_cnt += 1u;
}

static void uart_rx_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid;
    (void)src_id;
    uint8_t ch = 0u;
    while (hal_uart_getc_nonblock(&ch)) {
        uint32_t idx = g_rx_cnt;
        if (idx < 7u) {
            g_rx_buf[idx] = (char)ch;
            g_rx_cnt = idx + 1u;
        }
    }
}

/* Deterministic foreground ALU/MUL compute (verifies zero state corruption under live IRQs) */
static uint32_t compute_workload(uint32_t seed, uint32_t iters) {
    uint32_t acc = 0x24681357u ^ seed;
    for (uint32_t i = 1u; i <= iters; i++) {
        uint32_t a = (i * 2654435761u) ^ acc;
        acc = ((acc << 7) | (acc >> 25)) + (a ^ (i * 31u));
    }
    return acc;
}

uint32_t main(uint32_t hartid) {
    uint32_t ncores = mmio_read32(SOC_CTRL_BASE + SOC_CTRL_CORES_OFF);
    uint32_t local_sense = 0u;

    /* 1. Verify SoC FEATURE_MASK advertises Traps + APB4 + PLIC + Peripherals */
    uint32_t feat = mmio_read32(SOC_CTRL_BASE + SOC_CTRL_FEATURES_OFF);
    uint32_t need = SOC_FEAT_TRAPS | SOC_FEAT_APB4 | SOC_FEAT_PLIC | SOC_FEAT_PERIPH;
    if ((feat & need) != need) {
        soc_halt(0xBAD40001u);
    }

    /* 2. Per-hart trap & PLIC initialization */
    hal_irq_init(hartid);
    hal_irq_set_mtvec_direct();
    hal_irq_enable_mie(MIE_MEIE);
    hal_plic_set_threshold(hartid, 0u);

    uint32_t gpio_owner = (ncores == 2u) ? 1u : 0u;

    if (hartid == 0u) {
        g_gpio_cnt = 0u;
        g_t0_cnt   = 0u;
        g_t1_cnt   = 0u;
        g_rx_cnt   = 0u;
        g_c1_done  = 0u;
        for (uint32_t i = 0u; i < 8u; i++) {
            g_rx_buf[i] = '\0';
        }

        /* Initialize UART in fast-sim console mode for printf logging */
        hal_uart_init(4u, 1u, 0u);
        hal_uart_irq_enable(0u, 0u);
        hal_uart_printf("BOOT cores=%u\n", ncores);

        /* Register & enable Timer0 + Timer1 PLIC interrupts on Core 0 */
        hal_plic_set_trigger_edge(PLIC_IRQ_TIMER0, 0u);
        hal_plic_set_priority(PLIC_IRQ_TIMER0, 4u);
        hal_irq_register_ext(0u, PLIC_IRQ_TIMER0, timer0_isr);
        hal_plic_enable_irq(0u, PLIC_IRQ_TIMER0);

        hal_plic_set_trigger_edge(PLIC_IRQ_TIMER1, 0u);
        hal_plic_set_priority(PLIC_IRQ_TIMER1, 6u);
        hal_irq_register_ext(0u, PLIC_IRQ_TIMER1, timer1_isr);
        hal_plic_enable_irq(0u, PLIC_IRQ_TIMER1);
    }

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    if (hartid == gpio_owner) {
        /* Configure GPIO: [15:8]=output, [7:0]=input, rising-edge IRQ on [2:0] */
        hal_gpio_set_dir(0x0000FF00u);
        hal_gpio_write(0u);
        hal_gpio_irq_clear(0xFFFFFFFFu);
        hal_gpio_irq_config(0x00000007u, 1u, 1u);
        hal_gpio_irq_enable(0x00000007u);

        hal_plic_set_trigger_edge(PLIC_IRQ_GPIO, 0u);
        hal_plic_set_priority(PLIC_IRQ_GPIO, 5u);
        hal_irq_register_ext(gpio_owner, PLIC_IRQ_GPIO, gpio_isr);
        hal_plic_enable_irq(gpio_owner, PLIC_IRQ_GPIO);
    }

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* 3. Phase A: Concurrent GPIO Pin ISRs + Timer0 Periodic PWM + Timer1 One-Shot */
    uint32_t expected_work = compute_workload(hartid + 10u, 80u);
    uint32_t actual_work   = expected_work;

    hal_irq_global_enable();

    if (hartid == 0u) {
        /* CH0: 500-cycle period, 50% duty (CMP=250), periodic IRQ */
        hal_timer_init(0u, 0u, 499u, 250u,
                       TIMER_CTRL_EN | TIMER_CTRL_AUTO_RELOAD | TIMER_CTRL_PWM_EN | TIMER_CTRL_IRQ_EN);
        /* CH1: 600-cycle one-shot, 50% duty (CMP=300), auto-stops after 1 expiration */
        hal_timer_init(1u, 0u, 599u, 300u,
                       TIMER_CTRL_EN | TIMER_CTRL_PWM_EN | TIMER_CTRL_IRQ_EN);
    }

    if (hartid == gpio_owner) {
        /* Handshake bit gpio_out[15] starts the external +GPIO_STIM stimulus generator! */
        hal_gpio_set_bits(1u << 15);
    }

    if (hartid == 0u) {
        while (g_t0_cnt < TARGET_T0_TICKS || g_t1_cnt < TARGET_T1_TICKS ||
               (ncores == 1u && g_gpio_cnt < TARGET_GPIO_IRQS)) {
            actual_work = compute_workload(hartid + 10u, 80u);
        }
        /* Verify Timer1 auto-stopped its EN bit in one-shot mode */
        if ((mmio_read32(SOC_TIMER_BASE + TIMER_CTRL_OFF(1u)) & TIMER_CTRL_EN) != 0u) {
            soc_halt(0xBAD40002u);
        }
        hal_plic_disable_irq(0u, PLIC_IRQ_TIMER0);
        hal_plic_disable_irq(0u, PLIC_IRQ_TIMER1);
    } else {
        while (g_gpio_cnt < TARGET_GPIO_IRQS) {
            actual_work = compute_workload(hartid + 10u, 80u);
        }
    }

    if (actual_work != expected_work) {
        soc_halt(0xBAD40003u | (hartid << 8));
    }

    if (hartid == gpio_owner) {
        /* Verify gpio_out[15] + toggled LEDs gpio_out[10:8] == 0x8700 */
        uint32_t gout = mmio_read32(SOC_GPIO_BASE + GPIO_DATA_OUT_OFF);
        if ((gout & 0x0000FFFFu) != 0x00008700u) {
            soc_halt(0xBAD40004u);
        }
        hal_plic_disable_irq(gpio_owner, PLIC_IRQ_GPIO);
    }

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* 4. Phase B: Real 8N1 Serial UART Loopback + RX PLIC ISR (Source 2) */
    if (hartid == 0u) {
        hal_uart_tx_flush();

        hal_plic_set_trigger_edge(PLIC_IRQ_UART_RX, 0u);
        hal_plic_set_priority(PLIC_IRQ_UART_RX, 5u);
        hal_irq_register_ext(0u, PLIC_IRQ_UART_RX, uart_rx_isr);
        hal_plic_enable_irq(0u, PLIC_IRQ_UART_RX);

        /* Real bit-by-bit 8N1 framing (BAUD_DIV=4, FAST_SIM=0, LOOPBACK=1) */
        hal_uart_init(4u, 0u, 1u);
        hal_uart_irq_enable(1u, 0u);

        hal_uart_puts("OK!");

        while (g_rx_cnt < TARGET_UART_RX) {
            /* Service incoming 8N1 bytes asynchronously via PLIC Source 2 C ISR! */
        }
        hal_uart_tx_flush();

        hal_uart_irq_enable(0u, 0u);
        hal_plic_disable_irq(0u, PLIC_IRQ_UART_RX);

        if (g_rx_buf[0] != 'O' || g_rx_buf[1] != 'K' || g_rx_buf[2] != '!') {
            soc_halt(0xBAD40005u);
        }
        g_rx_buf[3] = '\0';

        /* Switch back to fast-sim console mode and emit formatted summary */
        hal_uart_init(4u, 1u, 0u);
        char rx_str[4];
        rx_str[0] = g_rx_buf[0];
        rx_str[1] = g_rx_buf[1];
        rx_str[2] = g_rx_buf[2];
        rx_str[3] = '\0';
        hal_uart_printf("SOC_U4_DONE: cores=%u gpio=%u t0=%u t1=%u rx=%s neg=%d hex=0x%04X 100%%\n",
                        ncores, g_gpio_cnt, g_t0_cnt, g_t1_cnt, rx_str, -42, 0xCAFEu);
        hal_uart_tx_flush();
    }

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* 5. Clean Halt with Deterministic Checksum */
    hal_irq_global_disable();
    uint32_t checksum = (ncores << 16) |
                        (g_gpio_cnt << 12) |
                        (g_t0_cnt << 8) |
                        (g_t1_cnt << 4) |
                        g_rx_cnt;

    if (ncores == 2u) {
        if (hartid == 1u) {
            soc_park_secondary(checksum | 0x00100000u, &g_c1_done);
        } else {
            while (!g_c1_done) {}
            soc_halt(checksum);
        }
    } else {
        soc_halt(checksum);
    }
    return checksum;
}
