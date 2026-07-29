#include "scheduler_metrics.h"
#include "uart.h"

#if ENABLE_SCHEDULER_TIMING_TEST

/* ── 5 monitored tasks ── */
#define METRICS_TASK_COUNT  5

/* indexes into the task metrics array (matches order in scheduler.c) */
#define METRICS_IDX_HEARTBEAT  0   /* send_heartbeat,       500ms */
#define METRICS_IDX_ATTITUDE   1   /* send_attitude,        100ms */
#define METRICS_IDX_MISSION    2   /* mission_update,       100ms */
#define METRICS_IDX_BATTERY    3   /* sim_battery_update,   100ms */
#define METRICS_IDX_GPS        4   /* send_gps_raw_int,    1000ms */

/* ── static metrics state ── */
static task_metrics_t metrics[METRICS_TASK_COUNT];
static uint8_t        experiment_complete = 0;
static uint32_t       experiment_start_tick = 0;

/* ── task label strings ── */
static const char * const task_labels[METRICS_TASK_COUNT] = {
    "Heartbeat",
    "Attitude",
    "Mission ",
    "Battery ",
    "GPS     "
};

void scheduler_metrics_init(uint32_t tick_now)
{
    uint8_t i;
    for (i = 0; i < METRICS_TASK_COUNT; i++) {
        metrics[i].period_ms       = 0;
        metrics[i].sample_count    = 0;
        metrics[i].total_interval  = 0;
        metrics[i].max_jitter      = 0;
        metrics[i].deadline_misses = 0;
        metrics[i].prev_tick       = 0;
        metrics[i].initialized     = 0;
    }
    experiment_start_tick = tick_now;
    experiment_complete = 0;
}

void scheduler_metrics_record(uint8_t task_index, uint32_t period_ms, uint32_t tick_now)
{
    task_metrics_t *m;

    if (task_index >= METRICS_TASK_COUNT)
        return;

    m = &metrics[task_index];
    m->period_ms = period_ms;

    if (!m->initialized) {
        /* first execution: store tick only, no interval or sample counted */
        m->prev_tick = tick_now;
        m->initialized = 1;
        /* sample_count stays 0 — first call contributes no interval */
        return;
    }

    /* compute actual interval since last execution */
    uint32_t actual_interval = tick_now - m->prev_tick;
    m->sample_count++;
    m->total_interval += actual_interval;
    m->prev_tick = tick_now;

    /* jitter = |actual_interval - expected_period| */
    uint32_t jitter;
    if (actual_interval >= period_ms)
        jitter = actual_interval - period_ms;
    else
        jitter = period_ms - actual_interval;

    if (jitter > m->max_jitter)
        m->max_jitter = jitter;

    /* deadline miss: actual_interval > period_ms + 1 */
    if (actual_interval > (period_ms + 1))
        m->deadline_misses++;
}

uint8_t scheduler_metrics_is_complete(void)
{
    return experiment_complete;
}

void scheduler_metrics_report(void)
{
    uint8_t i;

    experiment_complete = 1;

    uart_print("\n");
    uart_print("------------------------------------------------------------\n");
    uart_print("Scheduler Timing Accuracy Report\n");
    uart_print("------------------------------------------------------------\n");
    uart_print("Task        Period Samples Avg(ms) MaxJitter DeadlineMiss\n");
    uart_print("------------------------------------------------------------\n");

    for (i = 0; i < METRICS_TASK_COUNT; i++) {
        task_metrics_t *m = &metrics[i];
        uint32_t avg_ms = 0;
        if (m->sample_count > 0)
            avg_ms = m->total_interval / m->sample_count;

        /* print label (fixed width) */
        uart_print(task_labels[i]);

        /* period */
        uart_print(" ");
        uart_print_int(m->period_ms);
        /* pad to align */
        if (m->period_ms < 1000) uart_print(" ");
        if (m->period_ms < 100)  uart_print(" ");
        if (m->period_ms < 10)   uart_print(" ");

        uart_print("   ");
        uart_print_int(m->sample_count);
        if (m->sample_count < 10000) uart_print(" ");
        if (m->sample_count < 1000)  uart_print(" ");
        if (m->sample_count < 100)   uart_print(" ");
        if (m->sample_count < 10)    uart_print(" ");

        uart_print("  ");
        uart_print_int(avg_ms);
        if (avg_ms < 1000) uart_print(" ");
        if (avg_ms < 100)  uart_print(" ");
        if (avg_ms < 10)   uart_print(" ");

        uart_print("      ");
        uart_print_int(m->max_jitter);
        if (m->max_jitter < 100) uart_print(" ");
        if (m->max_jitter < 10)  uart_print(" ");

        uart_print("         ");
        uart_print_int(m->deadline_misses);
        uart_print("\n");
    }

    uart_print("------------------------------------------------------------\n");
    uart_print("Experiment duration: 60 seconds\n");
    uart_print("Deadline miss definition: actual_interval > period + 1 tick\n");
    uart_print("------------------------------------------------------------\n\n");
}

#else  /* ENABLE_SCHEDULER_TIMING_TEST = 0 — stubs */

void scheduler_metrics_init(uint32_t tick_now)            { (void)tick_now; }
void scheduler_metrics_record(uint8_t ti, uint32_t p, uint32_t t) { (void)ti; (void)p; (void)t; }
void scheduler_metrics_report(void)                        {}
uint8_t scheduler_metrics_is_complete(void)                 { return 0; }

#endif /* ENABLE_SCHEDULER_TIMING_TEST */