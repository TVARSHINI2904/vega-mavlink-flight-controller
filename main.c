#include <stdint.h>
#include "uart.h"
#include "scheduler.h"
#include "mavlink_tx.h"
#include "mavlink_rx.h"
#include "mission.h"

/* ── timer ── */
#define MTIME_BASE    0x0200BFF8UL
#define MTIMECMP_BASE 0x02004000UL
#define MTIME         (*(volatile uint64_t*)MTIME_BASE)
#define MTIMECMP      (*(volatile uint64_t*)MTIMECMP_BASE)
#define TICK_HZ       10000000UL
#define TICK_INTERVAL (TICK_HZ / 1000)   /* 1ms */

/* ── bare-metal stdlib ── */
void *memcpy(void *dest, const void *src, unsigned int n)
{
    unsigned char *d = dest;
    const unsigned char *s = src;
    while (n--) *d++ = *s++;
    return dest;
}

void *memset(void *s, int c, unsigned int n)
{
    unsigned char *p = s;
    while (n--) *p++ = (unsigned char)c;
    return s;
}

int memcmp(const void *s1, const void *s2, unsigned int n)
{
    const unsigned char *p1 = s1;
    const unsigned char *p2 = s2;

    while (n--) {
        if (*p1 != *p2)
            return (int)*p1 - (int)*p2;
        p1++;
        p2++;
    }

    return 0;
}

/* ── timer ISR ── */
void timer_isr(void)
{
    MTIMECMP = MTIME + TICK_INTERVAL;
    sys_tick++;
}

/* ── entry point ── */
void main(void)
{
    MTIMECMP = MTIME + TICK_INTERVAL;
    /* generate a session-specific command auth token and publish it
       as the `CMD_AUTH` parameter so GCS can read it */
    {
        uint32_t seed = (uint32_t)(MTIME & 0xFFFFFFFFUL) ^ (uint32_t)sys_tick ^ (uint32_t)(uintptr_t)&seed;
        /* xorshift32 */
        uint32_t x = seed;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        uint32_t token = (x % 10000); /* small 0..9999 token for safe float representation */
        current_cmd_auth_token = token;
        set_param_value(PARAM_CMD_AUTH, (float)current_cmd_auth_token);
        uart_print("-------------------------------------\n");
        uart_print("SESSION TOKEN GENERATED : ");
        uart_print_int(current_cmd_auth_token);
        uart_putchar('\n');
        uart_print("-------------------------------------\n\n");
    }

    /* generate a session-specific mission challenge and publish it
       as the `MISSION_CHALLENGE` parameter so GCS can read it */
    {
        uint32_t seed = (uint32_t)((MTIME >> 32) & 0xFFFFFFFFUL) ^ (uint32_t)sys_tick ^ (uint32_t)(uintptr_t)&seed;
        /* xorshift32 */
        uint32_t x = seed ? seed : 0xA5A5A5A5UL;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        /* produce a 7-digit challenge (1,000,000..9,999,999) for readability */
        uint32_t chal = (x % 9000000UL) + 1000000UL;
        current_mission_challenge = chal;
        set_param_value(PARAM_MISSION_CHALLENGE, (float)current_mission_challenge);
        uart_print("-------------------------------------\n");
        uart_print("SESSION MISSION CHALLENGE GENERATED : ");
        uart_print_int(current_mission_challenge);
        uart_putchar('\n');
        uart_print("-------------------------------------\n\n");
    }

    /* restore mission from non-volatile memory (if available) */
    if (load_mission_from_nvm()) {
        send_statustext(MAV_SEVERITY_INFO, "NVM RESTORED");
    } else {
        mission_reset();
    }

    while (1) {
        __asm__ volatile("wfi");
        scheduler_run();
        mavlink_rx_poll();
    }
}
