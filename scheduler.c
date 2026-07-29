#include "scheduler.h"
#include "scheduler_metrics.h"
#include "mavlink_tx.h"
#include "mission.h"

volatile uint32_t sys_tick = 0;

static task_t tasks[] = {
    {send_heartbeat,           500,  0},
    {send_attitude,            100,  0},
    {mission_update,           100,  0},
    {sim_battery_update,       100,  0},   /* battery drain simulation */
    {failsafe_check,           100,  0},   /* failsafe monitoring */
    {send_vfr_hud,             200,  0},
    {send_sys_status,         1000,  0},
    {send_battery_status,     1000,  0},
    {send_gps_raw_int,        1000,  0},
    {send_global_position_int, 1000,  0},
    {send_nav_controller_output, 200, 0},
};

#define NUM_TASKS (sizeof(tasks)/sizeof(tasks[0]))

/* ── scheduler timing experiment hook ── */
#if ENABLE_SCHEDULER_TIMING_TEST
/* map task index → metrics index. -1 = not monitored */
static const int8_t task_to_metrics[] = {
    0,   /* send_heartbeat        → METRICS_IDX_HEARTBEAT */
    1,   /* send_attitude         → METRICS_IDX_ATTITUDE  */
    2,   /* mission_update        → METRICS_IDX_MISSION   */
    3,   /* sim_battery_update    → METRICS_IDX_BATTERY   */
    -1,  /* failsafe_check        → not monitored */
    -1,  /* send_vfr_hud          → not monitored */
    -1,  /* send_sys_status       → not monitored */
    -1,  /* send_battery_status   → not monitored */
    4,   /* send_gps_raw_int      → METRICS_IDX_GPS       */
    -1,  /* send_global_position_int → not monitored */
    -1,  /* send_nav_controller_output → not monitored */
};
#endif

void scheduler_run(void)
{
    uint32_t now = sys_tick;

#if ENABLE_SCHEDULER_TIMING_TEST
    /* auto-start metrics on first scheduler call */
    {
        static uint8_t metrics_inited = 0;
        if (!metrics_inited) {
            scheduler_metrics_init(now);
            metrics_inited = 1;
        }
    }
#endif

    for (uint32_t i = 0; i < NUM_TASKS; i++) {
        if ((now - tasks[i].last_run) >= tasks[i].period_ms) {
#if ENABLE_SCHEDULER_TIMING_TEST
            int8_t midx = task_to_metrics[i];
            if (midx >= 0)
                scheduler_metrics_record((uint8_t)midx, tasks[i].period_ms, now);
#endif
            tasks[i].task();
            tasks[i].last_run = now;
        }
    }

#if ENABLE_SCHEDULER_TIMING_TEST
    /* check if experiment duration has elapsed */
    if (!scheduler_metrics_is_complete() && (now - 0) >= METRICS_DURATION_TICKS) {
        scheduler_metrics_report();
    }
#endif
}
