#ifndef SCHEDULER_METRICS_H
#define SCHEDULER_METRICS_H

#include <stdint.h>

/* ── compile-time enable ── */
#define ENABLE_SCHEDULER_TIMING_TEST  1   /* set to 0 to disable */

/* ── experiment duration: 60 seconds = 60000 ticks ── */
#define METRICS_DURATION_TICKS  60000UL

/* ── per-task timing statistics ── */
typedef struct {
    uint32_t period_ms;          /* expected period */
    uint32_t sample_count;       /* number of times task executed */
    uint32_t total_interval;     /* sum of all actual intervals (for avg) */
    uint32_t max_jitter;         /* maximum observed jitter */
    uint32_t deadline_misses;    /* count of deadline misses */
    uint32_t prev_tick;          /* previous execution tick (for interval calc) */
    uint8_t  initialized;        /* 1 after first execution recorded */
} task_metrics_t;

/* ── public API ── */
void scheduler_metrics_init(uint32_t tick_now);
void scheduler_metrics_record(uint8_t task_index, uint32_t period_ms, uint32_t tick_now);
void scheduler_metrics_report(void);
uint8_t scheduler_metrics_is_complete(void);

#endif /* SCHEDULER_METRICS_H */