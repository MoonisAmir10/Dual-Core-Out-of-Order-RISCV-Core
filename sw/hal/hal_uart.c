/* ============================================================================
 * Bare-Metal Lightweight hal_uart_printf() Implementation (0 libc deps!)
 * Supports: %s, %c, %d, %u, %x, %X, %0<N>x / %0<N>u / %0<N>d, %%
 * ============================================================================ */
#include <stdarg.h>
#include <stdint.h>
#include "hal_uart.h"

static void put_uint_base(uint32_t v, uint32_t base, uint32_t width, char pad, int upper) {
    char buf[16];
    uint32_t n = 0u;
    if (v == 0u) {
        buf[n++] = '0';
    } else {
        while (v != 0u && n < sizeof(buf)) {
            uint32_t d = v % base;
            v /= base;
            if (d < 10u) buf[n++] = (char)('0' + d);
            else         buf[n++] = (char)((upper ? 'A' : 'a') + (d - 10u));
        }
    }
    while (n < width) {
        hal_uart_putc(pad);
        width--;
    }
    while (n > 0u) {
        hal_uart_putc(buf[--n]);
    }
}

void hal_uart_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    while (*fmt) {
        if (*fmt != '%') {
            hal_uart_putc(*fmt++);
            continue;
        }
        fmt++;
        char pad = ' ';
        uint32_t width = 0u;
        if (*fmt == '0') {
            pad = '0';
            fmt++;
        }
        while (*fmt >= '0' && *fmt <= '9') {
            width = width * 10u + (uint32_t)(*fmt - '0');
            fmt++;
        }
        char spec = *fmt;
        if (!spec) break;
        fmt++;

        switch (spec) {
            case 's': {
                const char *s = va_arg(ap, const char *);
                hal_uart_puts(s ? s : "(null)");
                break;
            }
            case 'c': {
                char c = (char)va_arg(ap, int);
                hal_uart_putc(c);
                break;
            }
            case 'd': {
                int32_t sv = va_arg(ap, int32_t);
                if (sv < 0) {
                    hal_uart_putc('-');
                    put_uint_base((uint32_t)(-(int64_t)sv), 10u, (width > 0u) ? (width - 1u) : 0u, pad, 0);
                } else {
                    put_uint_base((uint32_t)sv, 10u, width, pad, 0);
                }
                break;
            }
            case 'u': {
                uint32_t uv = va_arg(ap, uint32_t);
                put_uint_base(uv, 10u, width, pad, 0);
                break;
            }
            case 'x': {
                uint32_t hv = va_arg(ap, uint32_t);
                put_uint_base(hv, 16u, width, pad, 0);
                break;
            }
            case 'X': {
                uint32_t hv = va_arg(ap, uint32_t);
                put_uint_base(hv, 16u, width, pad, 1);
                break;
            }
            case '%':
                hal_uart_putc('%');
                break;
            default:
                hal_uart_putc('%');
                hal_uart_putc(spec);
                break;
        }
    }

    va_end(ap);
}
