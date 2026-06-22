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

    /* restore mission from non-volatile memory (if available) */
    if (load_mission_from_nvm())
        send_statustext(MAV_SEVERITY_INFO, "NVM RESTORED");

    while (1) {
        __asm__ volatile("wfi");
        scheduler_run();
        mavlink_rx_poll();
    }
}
