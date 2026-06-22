#include "scheduler.h"
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

void scheduler_run(void)
{
    uint32_t now = sys_tick;
    for (uint32_t i = 0; i < NUM_TASKS; i++) {
        if ((now - tasks[i].last_run) >= tasks[i].period_ms) {
            tasks[i].task();
            tasks[i].last_run = now;
        }
    }
}
