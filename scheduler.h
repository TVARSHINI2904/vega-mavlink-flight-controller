#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

typedef struct {
    void     (*task)(void);
    uint32_t  period_ms;
    uint32_t  last_run;
} task_t;

extern volatile uint32_t sys_tick;

void scheduler_run(void);

#endif

